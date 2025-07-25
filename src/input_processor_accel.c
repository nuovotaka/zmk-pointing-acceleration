#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <drivers/input_processor.h>


#define DT_DRV_COMPAT zmk_input_processor_acceleration
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ACCEL_MAX_CODES 4

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS
#define CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS 10
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE
#define CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE 1500
#endif

struct accel_config {
    uint8_t input_type;
    const uint16_t *codes;
    uint32_t codes_count;
    bool track_remainders;
    uint16_t min_factor;
    uint16_t max_factor;
    uint32_t speed_threshold;
    uint32_t speed_max;
    uint8_t  acceleration_exponent;
    uint8_t  pair_window_ms;
    uint16_t y_aspect_scale;
};

struct accel_data {
    int64_t last_time;
    int16_t remainders[ACCEL_MAX_CODES];

    int32_t pending_x;
    int32_t pending_y;
    int64_t pending_x_time;
    int64_t pending_y_time;
    bool has_pending_x;
    bool has_pending_y;
};

int input_processor_forward_event(const struct device *dev,
                                 struct input_event *event,
                                 uint32_t param1,
                                 uint32_t param2,
                                 struct zmk_input_processor_state *state);


static int accel_handle_event(const struct device *dev, struct input_event *event,
                             uint32_t param1, uint32_t param2,
                             struct zmk_input_processor_state *state);


#define ACCEL_INST_INIT(inst)                                                  \
static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y };     \
static const struct accel_config accel_config_##inst = {                       \
    .input_type = INPUT_EV_REL,                                                \
    .codes = accel_codes_##inst,                                               \
    .codes_count = 2,                                                          \
    .track_remainders = true,                                                  \
    .min_factor = 1000,                                                        \
    .max_factor = 3500,                                                        \
    .speed_threshold = 1000,                                                   \
    .speed_max = 6000,                                                         \
    .acceleration_exponent = 2,                                                \
    .pair_window_ms = CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS,             \
    .y_aspect_scale = CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE              \
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

// 例: 1インスタンスだけ生成
ACCEL_INST_INIT(0)


int input_processor_forward_event(const struct device *dev,
                                 struct input_event *event,
                                 uint32_t param1,
                                 uint32_t param2,
                                 struct zmk_input_processor_state *state) {
    ARG_UNUSED(dev);
    ARG_UNUSED(event);
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    return 0;
}

static int accel_handle_event(const struct device *dev, struct input_event *event,
                             uint32_t param1, uint32_t param2,
                             struct zmk_input_processor_state *state) {
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    // 指定タイプ・コード以外はスルー
    if (event->type != cfg->input_type) return 0;
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

        uint32_t magnitude = sqrtf((float)dx * dx + (float)dy * dy);
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

        int32_t accelerated_y = (int32_t)((dy * factor * cfg->y_aspect_scale) / (1000 * 1000));

        // 以降、加速度の有無やペア処理の有無に関係なく、Y軸はこの補正値を使う
        // input_report_rel(dev, INPUT_REL_X, accelerated_x, false, K_NO_WAIT);
        // input_report_rel(dev, INPUT_REL_Y, accelerated_y, true, K_NO_WAIT);

        // 端数処理
        if (cfg->track_remainders) {
            int32_t rem_x = ((dx * factor) % 1000) / 100;
            int32_t rem_y = (((dy * factor * cfg->y_aspect_scale) / 1000) % 1000) / 100;
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

        // X/Y両方のイベントを生成してforward
        struct input_event out_x = *event;
        out_x.code = INPUT_REL_X;
        out_x.value = accelerated_x;
        int ret_x = input_processor_forward_event(dev, &out_x, param1, param2, state);
        if (ret_x == 1) {
            // チェーンの最後なので自分でOSに送信
            input_report_rel(dev, INPUT_REL_X, accelerated_x, false, K_NO_WAIT);
        }

        struct input_event out_y = *event;
        out_y.code = INPUT_REL_Y;
        out_y.value = accelerated_y;
        int ret_y = input_processor_forward_event(dev, &out_y, param1, param2, state);
        if (ret_y == 1) {
            input_report_rel(dev, INPUT_REL_Y, accelerated_y, true, K_FOREVER);
        }


        // 状態クリア
        data->has_pending_x = false;
        data->has_pending_y = false;
        data->last_time = current_time;

        // 既に両方出力したので、元のeventは処理しない
        return 1;
    }

    // --- ここから単独軸の加速度処理 ---
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
    
    // Y軸の場合はアスペクト比スケーリングを適用
    if (event->code == INPUT_REL_Y) {
        int32_t original_value = accelerated_value;
        accelerated_value = (accelerated_value * cfg->y_aspect_scale) / 1000;
        // printk("Y-axis scaling: original=%d, scale=%d, result=%d\n", 
        //        original_value, cfg->y_aspect_scale, accelerated_value);
    }

    // 端数処理
    if (cfg->track_remainders && code_index < ACCEL_MAX_CODES) {
        int32_t base_calculation = event->value * factor;
        if (event->code == INPUT_REL_Y) {
            base_calculation = (base_calculation * cfg->y_aspect_scale) / 1000;
        }
        int32_t remainder = (base_calculation % 1000) / 100;
        data->remainders[code_index] += remainder;
        if (abs(data->remainders[code_index]) >= 10) {
            int32_t remainder_contribution = data->remainders[code_index] / 10;
            accelerated_value += remainder_contribution;
            data->remainders[code_index] -= remainder_contribution * 10;
        }
    }

    // 状態更新
    data->last_time = current_time;

    // 加速値をeventに反映
    event->value = accelerated_value;

    int ret = input_processor_forward_event(dev, event, param1, param2, state);
    if (ret == 1) {
        input_report_rel(dev, event->code, event->value, true, K_FOREVER);
    }

    return 0;
}

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)