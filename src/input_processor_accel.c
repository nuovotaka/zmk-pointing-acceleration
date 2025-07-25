#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>  // For abs() function

#define DT_DRV_COMPAT zmk_input_processor_acceleration
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Forward declaration of the event handler */
static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state);

/* Maximum number of event codes this processor can handle (e.g. REL_X, REL_Y). */
#define ACCEL_MAX_CODES 4

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
};

/* Movement buffer for synchronized processing */
#define MOVEMENT_BUFFER_SIZE 8
struct movement_sample {
    int32_t dx;
    int32_t dy;
    int64_t timestamp;
    bool valid;
};

/* Runtime state for each instance (mutable data) */
struct accel_data {
    int64_t last_time;                   /* Timestamp of last processed event (ms) */
    int32_t last_phys_dx;                /* Last physical X delta (for direction check) */
    int32_t last_phys_dy;                /* Last physical Y delta (for direction check) */
    uint16_t last_code;                  /* Last event code processed (e.g. REL_X or REL_Y) */
    int16_t remainders[ACCEL_MAX_CODES]; /* Remainder values for fractional movements per code */
    
    /* Movement accumulation for synchronized processing */
    int32_t accumulated_dx;              /* Accumulated X movement */
    int32_t accumulated_dy;              /* Accumulated Y movement */
    int64_t accumulation_start_time;     /* Start time of current accumulation */
    bool has_accumulated_movement;       /* Flag for accumulated movement */
    
    /* Movement history for better acceleration calculation */
    struct movement_sample samples[MOVEMENT_BUFFER_SIZE];
    uint8_t sample_index;                /* Current sample index */
    uint8_t sample_count;                /* Number of valid samples */
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
    .acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 1)   \
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

/* Event handler implementation */
static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    /* Process only events of the specified type */
    if (event->type != cfg->input_type) {
        return 0;
    }

    /* Process only the specified event codes */
    bool code_matched = false;
    uint32_t code_index = 0;
    for (uint32_t i = 0; i < cfg->codes_count; ++i) {
        if (event->code == cfg->codes[i]) {
            code_index = i;
            code_matched = true;
            break;
        }
    }
    if (!code_matched) {
        return 0;
    }

        /* Skip zero-value events */
    if (event->value == 0) {
        return 0;
    }

    /* Validate code_index bounds */
    if (code_index >= ACCEL_MAX_CODES) {
        return 0;
    }

    /* Get current timestamp */
    int64_t current_time = k_uptime_get();
    
    /* Accumulate movement instead of processing immediately */
    if (event->code == INPUT_REL_X) {
        data->accumulated_dx += event->value;
    } else if (event->code == INPUT_REL_Y) {
        data->accumulated_dy += event->value;
    }
    
    /* Initialize accumulation timing */
    if (!data->has_accumulated_movement) {
        data->accumulation_start_time = current_time;
        data->has_accumulated_movement = true;
    }
    
    /* Check if we should process accumulated movement */
    int64_t accumulation_time = current_time - data->accumulation_start_time;
    bool should_process = false;
    
    /* Process if accumulation timeout (2ms) or significant movement */
    if (accumulation_time >= 2 || 
        (abs(data->accumulated_dx) + abs(data->accumulated_dy)) > 10) {
        should_process = true;
    }
    
    if (!should_process) {
        /* Continue accumulating, don't process this event yet */
        event->value = 0; /* Suppress this event */
        return 0;
    }
    
    /* Process accumulated movement */
    int32_t dx = data->accumulated_dx;
    int32_t dy = data->accumulated_dy;
    
    /* Calculate movement magnitude */
    uint32_t magnitude_squared = (uint32_t)(dx * dx + dy * dy);
    uint32_t magnitude = 0;
    
    /* Simple integer square root approximation */
    if (magnitude_squared > 0) {
        uint32_t x = magnitude_squared;
        uint32_t y = (x + 1) / 2;
        while (y < x) {
            x = y;
            y = (x + magnitude_squared / x) / 2;
        }
        magnitude = x;
    }
    
    /* Calculate time delta for speed calculation */
    int64_t time_delta = current_time - data->last_time;
    if (time_delta <= 0) {
        time_delta = 1; /* Avoid division by zero */
    }
    
    /* Calculate speed based on accumulated movement */
    uint32_t speed = (magnitude * 1000) / time_delta;
    
    /* Calculate acceleration factor based on speed */
    uint16_t factor = cfg->min_factor;
    
    if (speed > cfg->speed_threshold) {
        if (speed >= cfg->speed_max) {
            factor = cfg->max_factor;
        } else {
            /* Interpolate between min and max factor based on speed */
            uint32_t speed_range = cfg->speed_max - cfg->speed_threshold;
            uint32_t factor_range = cfg->max_factor - cfg->min_factor;
            uint32_t speed_offset = speed - cfg->speed_threshold;
            
            /* Apply acceleration exponent */
            uint32_t normalized_speed = (speed_offset * 1000) / speed_range;
            uint32_t accelerated_speed = normalized_speed;
            
            /* Simple exponent implementation for common cases */
            if (cfg->acceleration_exponent == 2) {
                accelerated_speed = (normalized_speed * normalized_speed) / 1000;
            } else if (cfg->acceleration_exponent == 3) {
                accelerated_speed = (normalized_speed * normalized_speed * normalized_speed) / (1000 * 1000);
            }
            
            factor = cfg->min_factor + ((factor_range * accelerated_speed) / 1000);
            if (factor > cfg->max_factor) {
                factor = cfg->max_factor;
            }
        }
    }
    
    /* Apply acceleration to accumulated movement */
    int32_t accelerated_dx = (dx * factor) / 1000;
    int32_t accelerated_dy = (dy * factor) / 1000;
    
    /* Store in movement history for future reference */
    data->samples[data->sample_index].dx = accelerated_dx;
    data->samples[data->sample_index].dy = accelerated_dy;
    data->samples[data->sample_index].timestamp = current_time;
    data->samples[data->sample_index].valid = true;
    
    data->sample_index = (data->sample_index + 1) % MOVEMENT_BUFFER_SIZE;
    if (data->sample_count < MOVEMENT_BUFFER_SIZE) {
        data->sample_count++;
    }
    
    /* Handle remainders for accumulated movement */
    if (cfg->track_remainders) {
        /* Apply remainders to X movement */
        if (code_index == 0 && accelerated_dx != 0) { /* X axis */
            int32_t remainder_x = ((dx * factor) % 1000) / 100;
            data->remainders[0] += remainder_x;
            
            if (abs(data->remainders[0]) >= 10) {
                int32_t remainder_contribution = data->remainders[0] / 10;
                accelerated_dx += remainder_contribution;
                data->remainders[0] -= remainder_contribution * 10;
            }
        }
        
        /* Apply remainders to Y movement */
        if (code_index == 1 && accelerated_dy != 0) { /* Y axis */
            int32_t remainder_y = ((dy * factor) % 1000) / 100;
            data->remainders[1] += remainder_y;
            
            if (abs(data->remainders[1]) >= 10) {
                int32_t remainder_contribution = data->remainders[1] / 10;
                accelerated_dy += remainder_contribution;
                data->remainders[1] -= remainder_contribution * 10;
            }
        }
    }
    
    /* Set output value based on current event axis */
    int32_t accelerated_value = 0;
    if (event->code == INPUT_REL_X) {
        accelerated_value = accelerated_dx;
    } else if (event->code == INPUT_REL_Y) {
        accelerated_value = accelerated_dy;
    }
    
    /* Update tracking data */
    data->last_time = current_time;
    data->last_code = event->code;
    data->last_phys_dx = dx;
    data->last_phys_dy = dy;
    
    /* Reset accumulation */
    data->accumulated_dx = 0;
    data->accumulated_dy = 0;
    data->has_accumulated_movement = false;
    
    /* Update event value with accelerated result */
    event->value = accelerated_value;

    return 0;
}
#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)