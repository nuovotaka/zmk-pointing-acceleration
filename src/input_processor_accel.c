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
#define CONFIG_INPUT_PROCESSOR_ACCEL_PAIR_WINDOW_MS 30
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE
#define CONFIG_INPUT_PROCESSOR_ACCEL_Y_ASPECT_SCALE 4000
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_MIN_FACTOR
#define CONFIG_INPUT_PROCESSOR_ACCEL_MIN_FACTOR 1000
#endif

#ifndef CONFIG_INPUT_PROCESSOR_ACCEL_MAX_FACTOR
#define CONFIG_INPUT_PROCESSOR_ACCEL_MAX_FACTOR 1100
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

    int32_t pending_x;
    int32_t pending_y;
    int64_t pending_x_time;
    int64_t pending_y_time;
    bool has_pending_x;
    bool has_pending_y;
    
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

    // ペア判定 - より柔軟な条件に変更
    bool has_pair = false;
    int32_t dx = 0, dy = 0;
    if (data->has_pending_x && data->has_pending_y) {
        int64_t time_diff = llabs(data->pending_x_time - data->pending_y_time);
        if (time_diff <= cfg->pair_window_ms) {
            has_pair = true;
            dx = data->pending_x;
            dy = data->pending_y;
        }
    }
    
    // ペアが見つからない場合、古いペンディングデータをクリアして単独処理に進む
    if (!has_pair) {
        // 現在のイベントより古いペンディングデータがあればクリア（さらに長い時間待機）
        if (event->code == INPUT_REL_X && data->has_pending_y) {
            int64_t y_age = llabs(current_time - data->pending_y_time);
            if (y_age > cfg->pair_window_ms * 3) { // 3倍の時間待機
                data->has_pending_y = false;
            }
        } else if (event->code == INPUT_REL_Y && data->has_pending_x) {
            int64_t x_age = llabs(current_time - data->pending_x_time);
            if (x_age > cfg->pair_window_ms * 3) { // 3倍の時間待機
                data->has_pending_x = false;
            }
        }
    }

    // ペアが見つからなくても、片方のペンディングデータがあれば強制ペア処理を試行
    if (!has_pair && ((event->code == INPUT_REL_X && data->has_pending_y) || 
                      (event->code == INPUT_REL_Y && data->has_pending_x))) {
        // 強制ペア処理
        has_pair = true;
        if (event->code == INPUT_REL_X) {
            dx = event->value;
            dy = data->pending_y;
        } else {
            dx = data->pending_x;
            dy = event->value;
        }
    }

    if (has_pair) {
        // ペア時の加速度計算
        int64_t time_delta = current_time - data->last_time;
        if (time_delta <= 0) time_delta = 1;
        if (time_delta < 2) time_delta = 2;     // 最小2msに制限（高ポーリングレート対応）
        if (time_delta > 200) time_delta = 200; // 最大200msに制限

        uint32_t magnitude = sqrtf((float)dx * dx + (float)dy * dy);
        uint32_t speed = (magnitude * 1000) / time_delta;
        if (speed > 10000) speed = 10000; // 異常に高い速度を制限（ペア処理時は緩和）
        
        // 斜め動作の検出（X軸とY軸の比率が近い場合）
        bool is_diagonal = false;
        if (abs(dx) > 0 && abs(dy) > 0) {
            float ratio = (float)abs(dx) / abs(dy);
            if (ratio > 0.2 && ratio < 5.0) { // より広い範囲で斜め動作と判定
                is_diagonal = true;
            }
        }

        uint16_t factor = cfg->min_factor;
        
        // 斜め動作時は加速度を抑制して滑らかに
        if (is_diagonal) {
            factor = cfg->min_factor + ((cfg->max_factor - cfg->min_factor) / 3); // より控えめな加速度
        } else if (speed > cfg->speed_threshold) {
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
        
        // X軸の飛びを抑制するため、異常に大きな値を制限（ペア処理時は緩和）
        int32_t max_x_change = abs(dx) * 3; // 元の値の3倍まで（ペア処理時は緩和）
        if (abs(accelerated_x) > max_x_change) {
            if (accelerated_x > 0) {
                accelerated_x = max_x_change;
            } else {
                accelerated_x = -max_x_change;
            }
        }
        
        // X軸専用の最大倍率制限（斜め動作時は制限を緩和）
        if (is_diagonal) {
            // 斜め動作時はX軸とY軸のバランスを重視
            int32_t x_max_factor = factor; // 同じfactorを使用
            int32_t x_limited_value = (dx * x_max_factor) / 1000;
            if (abs(accelerated_x) > abs(x_limited_value)) {
                accelerated_x = x_limited_value;
            }
        } else {
            // 通常時は制限を適用
            int32_t x_max_factor = (cfg->max_factor * 70) / 100;
            int32_t x_limited_value = (dx * x_max_factor) / 1000;
            if (abs(accelerated_x) > abs(x_limited_value)) {
                accelerated_x = x_limited_value;
            }
        }
        
        // さらに絶対的な上限も設定（斜め動作時はさらに緩和）
        int32_t x_abs_limit = is_diagonal ? 40 : 25; // 斜め動作時は40まで
        if (abs(accelerated_x) > x_abs_limit) {
            if (accelerated_x > 0) {
                accelerated_x = x_abs_limit;
            } else {
                accelerated_x = -x_abs_limit;
            }
        }

        int32_t accelerated_y = (int32_t)(((int64_t)dy * factor) / 1000);
        // アスペクト比スケーリングを適用
        accelerated_y = (int32_t)(((int64_t)accelerated_y * cfg->y_aspect_scale) / 1000);
        
        // ペア処理時もY軸の最小感度を保証（斜め動作時は調整）
        if (is_diagonal) {
            // 斜め動作時はX軸とのバランスを重視
            if (abs(accelerated_y) < abs(dy)) {
                accelerated_y = dy * 1.5; // 控えめな補正
            }
        } else {
            // 通常時は強めの補正
            if (abs(accelerated_y) < abs(dy * 1.5)) {
                accelerated_y = dy * 2; // 最低でも2倍
            }
        }
        


        // 以降、加速度の有無やペア処理の有無に関係なく、Y軸はこの補正値を使う
        // input_report_rel(dev, INPUT_REL_X, accelerated_x, false, K_NO_WAIT);
        // input_report_rel(dev, INPUT_REL_Y, accelerated_y, true, K_NO_WAIT);

        // 端数処理
        if (cfg->track_remainders) {
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
    if (time_delta < 2) time_delta = 2;     // 最小2msに制限（高ポーリングレート対応）
    if (time_delta > 200) time_delta = 200; // 最大200msに制限

    uint32_t speed = (abs(event->value) * 1000) / time_delta;
    if (speed > 8000) speed = 8000; // 異常に高い速度を制限（より厳しく）
    // DPIに基づく調整係数を計算（基準DPI: 1600）
    uint32_t dpi_factor = (1600 * 1000) / cfg->sensor_dpi; // 1000倍スケール
    
    // Y軸の場合は低い閾値と高い最小倍率を使用
    uint32_t threshold = (cfg->speed_threshold * dpi_factor) / 1000;
    uint16_t min_factor = cfg->min_factor;
    uint16_t y_scale = (cfg->y_aspect_scale * dpi_factor) / 1000;
    
    if (event->code == INPUT_REL_Y) {
        threshold = threshold / 3; // Y軸は1/3の閾値（緩和）
        min_factor = cfg->min_factor + (300 * dpi_factor) / 1000; // DPIに応じた最小倍率を緩和
    }
    
    uint16_t factor = min_factor;
    if (speed > threshold) {
        if (speed >= cfg->speed_max) {
            factor = cfg->max_factor;
        } else {
            uint32_t speed_range = cfg->speed_max - threshold;
            uint32_t factor_range = cfg->max_factor - min_factor;
            uint32_t speed_offset = speed - threshold;
            uint32_t normalized_speed = (speed_offset * 1000) / speed_range;
            uint32_t accelerated_speed = normalized_speed;
            if (cfg->acceleration_exponent == 2) {
                accelerated_speed = (normalized_speed * normalized_speed) / 1000;
            } else if (cfg->acceleration_exponent == 3) {
                accelerated_speed = (normalized_speed * normalized_speed * normalized_speed) / (1000 * 1000);
            }
            factor = min_factor + ((factor_range * accelerated_speed) / 1000);
            if (factor > cfg->max_factor) factor = cfg->max_factor;
            
            // 加速度の急激な変化を抑制（スムージング）
            if (data->last_factor > 0) {
                int32_t factor_diff = factor - data->last_factor;
                if (abs(factor_diff) > 50) { // 0.05倍以上の変化を制限（さらに滑らか）
                    if (factor_diff > 0) {
                        factor = data->last_factor + 50;
                    } else {
                        factor = data->last_factor - 50;
                    }
                }
            }
        }
    }

    int32_t accelerated_value = (event->value * factor) / 1000;
    
    // Y軸の場合はアスペクト比スケーリングを適用
    if (event->code == INPUT_REL_Y) {
        int32_t before_scale = accelerated_value;
        // アスペクト比スケーリングを適用
        accelerated_value = (int32_t)(((int64_t)accelerated_value * y_scale) / 1000);
        
        // Y軸の最小感度を保証（階段状動作を防ぐため緩和）
        if (abs(accelerated_value) < abs(event->value * 1.5)) {
            accelerated_value = event->value * 2; // 最低でも2倍（緩和）
        }
    } else if (event->code == INPUT_REL_X) {
        // 一時的にX軸の加速度を完全に無効化（テスト用）
        // accelerated_value = event->value; // この行のコメントを外すとX軸加速度無効
        
        // X軸専用の最大倍率制限（階段状動作を防ぐため緩和）
        int32_t x_max_factor = (cfg->max_factor * 70) / 100; // 最大倍率の70%（緩和）
        int32_t x_limited_value = (event->value * x_max_factor) / 1000;
        if (abs(accelerated_value) > abs(x_limited_value)) {
            accelerated_value = x_limited_value;
        }
        
        // X軸は最低でも元の値以下に制限
        if (abs(accelerated_value) > abs(event->value * 2)) {
            if (accelerated_value > 0) {
                accelerated_value = event->value * 2;
            } else {
                accelerated_value = event->value * 2 * -1;
            }
        }
        
        // X軸の飛びを抑制するため、異常に大きな値を制限（階段状動作を防ぐため緩和）
        int32_t max_change = abs(event->value) * 3; // 元の値の3倍まで（緩和）
        if (abs(accelerated_value) > max_change) {
            if (accelerated_value > 0) {
                accelerated_value = max_change;
            } else {
                accelerated_value = -max_change;
            }
        }
        
        // さらに絶対的な上限も設定（階段状動作を防ぐため緩和）
        if (abs(accelerated_value) > 20) { // 絶対値20を上限（緩和）
            if (accelerated_value > 0) {
                accelerated_value = 20;
            } else {
                accelerated_value = -20;
            }
        }
    }

    // 端数処理
    if (cfg->track_remainders && code_index < ACCEL_MAX_CODES) {
        int64_t base_calculation = (int64_t)event->value * factor;
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
    data->last_factor = factor;

    // 加速値をeventに反映
    event->value = accelerated_value;

    int ret = input_processor_forward_event(dev, event, param1, param2, state);
    if (ret == 1) {
        input_report_rel(dev, event->code, event->value, true, K_FOREVER);
    }

    return 0;
}

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)