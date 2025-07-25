#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>  // For abs() function

#define DT_DRV_COMPAT zmk_input_processor_acceleration
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS
#define CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS 5
#endif

#define MY_EVENT_QUEUE_SIZE 16
K_MSGQ_DEFINE(my_input_event_queue, sizeof(struct input_event), MY_EVENT_QUEUE_SIZE, 4);

/* Forward declaration of the event handler */
static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state);

/* Maximum number of event codes this processor can handle (e.g. REL_X, REL_Y). */
#define ACCEL_MAX_CODES 4

extern struct k_msgq my_input_event_queue;

void event_consumer_thread(void)
{
    struct input_event evt;
    while (1) {
        int ret = k_msgq_get(&my_input_event_queue, &evt, K_FOREVER);
        if (ret == 0) {
            process_input_event(&evt); // 下で定義
        }
    }
}

K_THREAD_DEFINE(event_consumer_tid, 1024, event_consumer_thread, NULL, NULL, NULL, 5, 0, 0);

// 仮想カーソル座標
static int32_t cursor_x = 0;
static int32_t cursor_y = 0;

void process_input_event(const struct input_event *evt)
{
    if (evt->type == INPUT_EV_REL) {
        if (evt->code == INPUT_REL_X) {
            cursor_x += evt->value;
        } else if (evt->code == INPUT_REL_Y) {
            cursor_y += evt->value;
        }
#ifdef DEBUG_CURSOR
        printk("Cursor: x=%d y=%d (delta: code=%d value=%d)\n", cursor_x, cursor_y, evt->code, evt->value);
#endif
    }
}

/* Configuration from devicetree (constant for each instance) */
struct accel_config {
    uint8_t input_type;                  /* Event type to process (e.g. INPUT_EV_REL) */
    const uint16_t *codes;               /* Array of event code values to accelerate (e.g. REL_X, REL_Y) */
    uint32_t codes_count;                /* Number of codes in the array */
    bool track_remainders;               /* Whether to accumulate fractional movement remainders */
    uint16_t min_factor;                 /* Minimum acceleration factor (scaled by 1000, e.g. 500 = 0.5x) */
    uint16_t max_factor;                 /* Maximum acceleration factor (scaled by 1000, e.g. 3500 = 3.5x) */
    uint32_t speed_threshold;            /* Speed (counts per second) at which factor reaches 1.0 */
    uint32_t speed_max;                  /* Speed (counts per second) at which factor reaches max_factor */
    uint8_t  acceleration_exponent;      /* Exponent for acceleration curve (1=linear, 2=quadratic, etc.) */
    uint8_t  pair_window_ms;
};

/* Runtime state for each instance (mutable data) */
struct accel_data {
    int64_t last_time;                   /* Timestamp of last processed event (ms) */
    int32_t last_phys_dx;                /* Last physical X delta (for direction check) */
    int32_t last_phys_dy;                /* Last physical Y delta (for direction check) */
    uint16_t last_code;                  /* Last event code processed (e.g. REL_X or REL_Y) */
    int16_t remainders[ACCEL_MAX_CODES]; /* Remainder values for fractional movements per code */
    
    int32_t pending_x;                   /* Pending X movement */
    int32_t pending_y;                   /* Pending Y movement */
    int64_t pending_x_time;              /* Timestamp of pending X */
    int64_t pending_y_time;              /* Timestamp of pending Y */
    bool has_pending_x;                  /* Has pending X movement */
    bool has_pending_y;                  /* Has pending Y movement */
    uint16_t shared_factor;              /* Shared acceleration factor for paired events */
    bool factor_ready;                   /* Factor calculated and ready to use */
};

/* Populate config and data for each instance from devicetree */
#define ACCEL_INST_INIT(inst)                                                  \
static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y };     \
static const struct accel_config accel_config_##inst = {                       \
    .input_type = DT_INST_PROP_OR(inst, input_type, INPUT_EV_REL),             \
    .codes = accel_codes_##inst,                                               \
    .codes_count = ARRAY_SIZE(accel_codes_##inst),                                                          \
    .track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),         \
    .min_factor = DT_INST_PROP_OR(inst, min_factor, 1000),                     \
    .max_factor = DT_INST_PROP_OR(inst, max_factor, 3500),                     \
    .speed_threshold = DT_INST_PROP_OR(inst, speed_threshold, 1000),           \
    .speed_max = DT_INST_PROP_OR(inst, speed_max, 6000),                       \
    .acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 1),  \
    .pair_window_ms = CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS              \
};                                                                             \
static struct accel_data accel_data_##inst = {0};                              \
DEVICE_DT_INST_DEFINE(inst,                                                    \
                      NULL,                                                    \
                      NULL,                                                    \
                      &accel_data_##inst,                                      \
                      &accel_config_##inst,                                    \
                      POST_KERNEL,                                             \
                      CONFIG_INPUT_PROCESSOR_ACCELERATION_INIT_PRIORITY,       \
                      &(const struct zmk_input_processor_driver_api){          \
                          .handle_event = accel_handle_event                   \
                      });

/* Instantiate for each DT node matching our compatible */
DT_INST_FOREACH_STATUS_OKAY(ACCEL_INST_INIT)

int input_processor_forward_event(const struct device *dev,
                                 struct input_event *event,
                                 uint32_t param1,
                                 uint32_t param2,
                                 struct zmk_input_processor_state *state) {
    ARG_UNUSED(dev);
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    // Zephyrのメッセージキューにイベントをput
    return k_msgq_put(&my_input_event_queue, event, K_FOREVER);
}

/* Event handler implementation */
static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    if (event->type != cfg->input_type) return 0;

    // コード判定
    bool code_matched = false;
    uint32_t code_index = 0;
    for (uint32_t i = 0; i < cfg->codes_count; ++i) {
        if (event->code == cfg->codes[i]) {
            code_index = i;
            code_matched = true;
            break;
        }
    }
    if (!code_matched) return 0;
    if (event->value == 0) return 0;
    if (code_index >= ACCEL_MAX_CODES) return 0;

    int64_t current_time = k_uptime_get();

    // ペンディングに格納
    if (event->code == INPUT_REL_X) {
        data->pending_x = event->value;
        data->pending_x_time = current_time;
        data->has_pending_x = true;
    } else if (event->code == INPUT_REL_Y) {
        data->pending_y = event->value;
        data->pending_y_time = current_time;
        data->has_pending_y = true;
    }

    // ペア判定
    bool has_pair = false;
    int32_t dx = 0, dy = 0;
    if (data->has_pending_x && data->has_pending_y) {
        int64_t time_diff = abs(data->pending_x_time - data->pending_y_time);
        if (time_diff <= cfg->pair_window_ms) {
            has_pair = true;
            dx = data->pending_x;
            dy = data->pending_y;
        }
    }

    if (has_pair) {
        // ペア時の加速度計算
        int64_t time_delta = current_time - data->last_time;
        if (time_delta <= 0) time_delta = 1;

        uint32_t magnitude_squared = (uint32_t)(dx * dx + dy * dy);
        uint32_t magnitude = 0;
        if (magnitude_squared > 0) {
            uint32_t x = magnitude_squared;
            uint32_t y = (x + 1) / 2;
            while (y < x) {
                x = y;
                y = (x + magnitude_squared / x) / 2;
            }
            magnitude = x;
        }
        uint32_t speed = (magnitude * 1000) / time_delta;

        uint16_t factor = cfg->min_factor;
        if (speed > cfg->speed_threshold) {
            if (speed >= cfg->speed_max) {
                factor = cfg->max_factor;
            } else {
                uint32_t speed_range = cfg->speed_max - cfg->speed_threshold;
                uint32_t factor_range = cfg->max_factor - cfg->min_factor;
                uint32_t speed_offset = speed - cfg->speed_threshold;
                uint32_t normalized_speed = (speed_offset * 1000) / speed_range;
                uint32_t accelerated_speed = normalized_speed;
                if (cfg->acceleration_exponent == 2) {
                    accelerated_speed = (normalized_speed * normalized_speed) / 1000;
                } else if (cfg->acceleration_exponent == 3) {
                    accelerated_speed = (normalized_speed * normalized_speed * normalized_speed) / (1000 * 1000);
                }
                factor = cfg->min_factor + ((factor_range * accelerated_speed) / 1000);
                if (factor > cfg->max_factor) factor = cfg->max_factor;
            }
        }

        // X/Y両方の加速値を計算
        int32_t accelerated_x = (dx * factor) / 1000;
        int32_t accelerated_y = (dy * factor) / 1000;

        // 端数処理（必要なら）
        if (cfg->track_remainders) {
            int32_t rem_x = ((dx * factor) % 1000) / 100;
            int32_t rem_y = ((dy * factor) % 1000) / 100;
            data->remainders[0] += rem_x;
            data->remainders[1] += rem_y;
            if (abs(data->remainders[0]) >= 10) {
                int32_t r = data->remainders[0] / 10;
                accelerated_x += r;
                data->remainders[0] -= r * 10;
            }
            if (abs(data->remainders[1]) >= 10) {
                int32_t r = data->remainders[1] / 10;
                accelerated_y += r;
                data->remainders[1] -= r * 10;
            }
        }

        // Xイベント生成
        struct input_event out_x = *event;
        out_x.code = INPUT_REL_X;
        out_x.value = accelerated_x;
        input_processor_forward_event(dev, &out_x, param1, param2, state);

        // Yイベント生成
        struct input_event out_y = *event;
        out_y.code = INPUT_REL_Y;
        out_y.value = accelerated_y;
        input_processor_forward_event(dev, &out_y, param1, param2, state);

        // 状態クリア
        data->has_pending_x = false;
        data->has_pending_y = false;
        data->factor_ready = false;
        data->last_time = current_time;
        data->last_code = 0;
        data->last_phys_dx = dx;
        data->last_phys_dy = dy;

        // 既に両方出力したので、元のeventは処理しない
        return 1;
    }

    // --- ここから単独軸の加速度処理 ---
    // ペアでなければ従来通り単独軸処理
    int64_t time_delta = current_time - data->last_time;
    if (time_delta <= 0) time_delta = 1;

    uint32_t speed = (abs(event->value) * 1000) / time_delta;
    uint16_t factor = cfg->min_factor;
    if (speed > cfg->speed_threshold) {
        if (speed >= cfg->speed_max) {
            factor = cfg->max_factor;
        } else {
            uint32_t speed_range = cfg->speed_max - cfg->speed_threshold;
            uint32_t factor_range = cfg->max_factor - cfg->min_factor;
            uint32_t speed_offset = speed - cfg->speed_threshold;
            uint32_t normalized_speed = (speed_offset * 1000) / speed_range;
            uint32_t accelerated_speed = normalized_speed;
            if (cfg->acceleration_exponent == 2) {
                accelerated_speed = (normalized_speed * normalized_speed) / 1000;
            } else if (cfg->acceleration_exponent == 3) {
                accelerated_speed = (normalized_speed * normalized_speed * normalized_speed) / (1000 * 1000);
            }
            factor = cfg->min_factor + ((factor_range * accelerated_speed) / 1000);
            if (factor > cfg->max_factor) factor = cfg->max_factor;
        }
    }

    int32_t accelerated_value = (event->value * factor) / 1000;

    // 端数処理
    if (cfg->track_remainders && code_index < ACCEL_MAX_CODES) {
        int32_t remainder = ((event->value * factor) % 1000) / 100;
        data->remainders[code_index] += remainder;
        if (abs(data->remainders[code_index]) >= 10) {
            int32_t remainder_contribution = data->remainders[code_index] / 10;
            accelerated_value += remainder_contribution;
            data->remainders[code_index] -= remainder_contribution * 10;
        }
    }

    // 状態更新
    data->last_time = current_time;
    data->last_code = event->code;
    if (event->code == INPUT_REL_X) {
        data->last_phys_dx = event->value;
    } else if (event->code == INPUT_REL_Y) {
        data->last_phys_dy = event->value;
    }

    // 加速値をeventに反映
    event->value = accelerated_value;

    return 0;
}
#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)