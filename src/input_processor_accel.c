#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdlib.h>
#include <drivers/input_processor.h>


#define DT_DRV_COMPAT zmk_input_processor_acceleration
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ACCEL_MAX_CODES 4

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS
#define CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS 8
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE
#define CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE 4000
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_MIN_FACTOR
#define CONFIG_INPUT_PROCESSOR_ACCEL_MIN_FACTOR 1000
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_MAX_FACTOR
#define CONFIG_INPUT_PROCESSOR_ACCEL_MAX_FACTOR 1050
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_SPEED_THRESHOLD
#define CONFIG_INPUT_PROCESSOR_ACCEL_SPEED_THRESHOLD 800
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_SPEED_MAX
#define CONFIG_INPUT_PROCESSOR_ACCEL_SPEED_MAX 3000
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_EXPONENT
#define CONFIG_INPUT_PROCESSOR_ACCEL_EXPONENT 2
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_SENSOR_DPI
#define CONFIG_INPUT_PROCESSOR_ACCEL_SENSOR_DPI 1600
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
    uint16_t sensor_dpi;  // センサーDPI設定を追加
};

struct accel_data {
    int64_t last_time;
    int16_t remainders[ACCEL_MAX_CODES];
    
    // ベクトルバッファシステム
    int32_t vector_x;  // 累積X軸移動量
    int32_t vector_y;  // 累積Y軸移動量
    int64_t last_flush_time; // 最後にベクトルを出力した時間
    
    uint16_t last_factor; // 前回の加速度係数を記録
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
static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y, INPUT_REL_WHEEL, INPUT_REL_HWHEEL };     \
static const struct accel_config accel_config_##inst = {                       \
    .input_type = INPUT_EV_REL,                                                \
    .codes = accel_codes_##inst,                                               \
    .codes_count = 4,                                                          \
    .track_remainders = DT_INST_PROP_OR(inst, track_remainders, false),         \
    .min_factor = DT_INST_PROP_OR(inst, min_factor, CONFIG_INPUT_PROCESSOR_ACCEL_MIN_FACTOR),                     \
    .max_factor = DT_INST_PROP_OR(inst, max_factor, CONFIG_INPUT_PROCESSOR_ACCEL_MAX_FACTOR),                     \
    .speed_threshold = DT_INST_PROP_OR(inst, speed_threshold, CONFIG_INPUT_PROCESSOR_ACCEL_SPEED_THRESHOLD),            \
    .speed_max = DT_INST_PROP_OR(inst, speed_max, CONFIG_INPUT_PROCESSOR_ACCEL_SPEED_MAX),                       \
    .acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, CONFIG_INPUT_PROCESSOR_ACCEL_EXPONENT),  \
    .pair_window_ms = DT_INST_PROP_OR(inst, pair_window_ms, CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS), \
    .y_aspect_scale = DT_INST_PROP_OR(inst, y_aspect_scale, CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE), \
    .sensor_dpi = DT_INST_PROP_OR(inst, sensor_dpi, CONFIG_INPUT_PROCESSOR_ACCEL_SENSOR_DPI)  \
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

    // 指定タイプ以外はそのまま通す
    if (event->type != cfg->input_type) {
        int ret = input_processor_forward_event(dev, event, param1, param2, state);
        if (ret == 1) {
            // タイプに応じて適切な報告関数を使用
            if (event->type == INPUT_EV_KEY) {
                input_report_key(dev, event->code, event->value, event->sync, K_FOREVER);
            } else if (event->type == INPUT_EV_REL) {
                input_report_rel(dev, event->code, event->value, event->sync, K_FOREVER);
            }
        }
        return 0;
    }
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

    // 回転イベント（ホイール）は加速度処理せずにそのまま通す
    if (event->code == INPUT_REL_WHEEL || event->code == INPUT_REL_HWHEEL) {
        int ret = input_processor_forward_event(dev, event, param1, param2, state);
        if (ret == 1) {
            input_report_rel(dev, event->code, event->value, true, K_FOREVER);
        }
        return 0;
    }

    int64_t current_time = k_uptime_get();

    // ベクトルバッファに累積
    if (event->code == INPUT_REL_X) {
        data->vector_x += event->value;
    } else if (event->code == INPUT_REL_Y) {
        data->vector_y += event->value;
    }
    
    // フラッシュ判定（一定時間経過または十分な移動量が蓄積）
    bool should_flush = false;
    int64_t time_since_flush = current_time - data->last_flush_time;
    
    if (time_since_flush >= cfg->pair_window_ms || // 時間経過
        abs(data->vector_x) + abs(data->vector_y) >= 5) { // 十分な移動量
        should_flush = true;
    }
    
    if (!should_flush) {
        // まだフラッシュしない場合は、イベントを蓄積して終了
        return 0;
    }

    // ベクトル処理を実行
    int32_t dx = data->vector_x;
    int32_t dy = data->vector_y;
    
    // ベクトルバッファをクリア
    data->vector_x = 0;
    data->vector_y = 0;
    data->last_flush_time = current_time;
    
    // 移動量がない場合は何もしない
    if (dx == 0 && dy == 0) {
        return 0;
    }
    
    // ベクトルベースの加速度計算
    int64_t time_delta = current_time - data->last_time;
    if (time_delta <= 0) time_delta = 1;
    if (time_delta > 100) time_delta = 100;
    
    uint32_t magnitude = sqrtf((float)dx * dx + (float)dy * dy);
    uint32_t speed = (magnitude * 1000) / time_delta;
    
    // 基本的な加速度処理
    uint16_t factor = cfg->min_factor;
    if (speed > cfg->speed_threshold) {
        if (speed >= cfg->speed_max) {
            factor = cfg->max_factor;
        } else {
            uint32_t speed_range = cfg->speed_max - cfg->speed_threshold;
            uint32_t factor_range = cfg->max_factor - cfg->min_factor;
            uint32_t speed_offset = speed - cfg->speed_threshold;
            factor = cfg->min_factor + ((factor_range * speed_offset) / speed_range);
            if (factor > cfg->max_factor) factor = cfg->max_factor;
        }
    }

    // ベクトル成分に加速度を適用
    int32_t accelerated_x = (dx * factor) / 1000;
    int32_t accelerated_y = (int32_t)(((int64_t)dy * factor * cfg->y_aspect_scale) / (1000 * 1000));
    
    // Y軸の最小感度を保証
    if (dy != 0 && abs(accelerated_y) < abs(dy * 2)) {
        accelerated_y = dy * 3; // 最低でも3倍
    }

        // 端数処理を無効化（階段状動作を防ぐ）
        if (false && cfg->track_remainders) {
            int32_t rem_x = ((dx * factor) % 1000) / 100;
            int32_t rem_y = (int32_t)((((int64_t)dy * factor * cfg->y_aspect_scale) / 1000) % 1000) / 100;
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

    // X軸イベントを送信（値がある場合のみ）
    if (accelerated_x != 0) {
        struct input_event out_x = *event;
        out_x.code = INPUT_REL_X;
        out_x.value = accelerated_x;
        int ret_x = input_processor_forward_event(dev, &out_x, param1, param2, state);
        if (ret_x == 1) {
            input_report_rel(dev, INPUT_REL_X, accelerated_x, false, K_NO_WAIT);
        }
    }

    // Y軸イベントを送信（値がある場合のみ）
    if (accelerated_y != 0) {
        struct input_event out_y = *event;
        out_y.code = INPUT_REL_Y;
        out_y.value = accelerated_y;
        int ret_y = input_processor_forward_event(dev, &out_y, param1, param2, state);
        if (ret_y == 1) {
            input_report_rel(dev, INPUT_REL_Y, accelerated_y, true, K_FOREVER);
        }
    }

    // 状態更新
    data->last_time = current_time;

    // ベクトル処理で処理済み
    return 1;

    return 0;
}

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)