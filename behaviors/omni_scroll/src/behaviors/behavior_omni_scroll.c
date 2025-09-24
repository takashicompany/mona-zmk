/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_omni_scroll

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/mouse_scroll.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define ABS(x) ((x) < 0 ? -(x) : (x))

struct behavior_omni_scroll_config {
    int threshold;
    int vertical_bias;
    int horizontal_bias;
    int smoothing;
    int diagonal_threshold;
};

struct behavior_omni_scroll_data {
    int accumulated_x;
    int accumulated_y;
    int last_direction;  // 0: none, 1: vertical, 2: horizontal
    int sample_count;
    int history_x[5];
    int history_y[5];
    int history_idx;
};

static int omni_scroll_binding_pressed(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    struct behavior_omni_scroll_data *data = dev->data;
    const struct behavior_omni_scroll_config *config = dev->config;

    // This is for sensor rotation, extract delta values
    int16_t delta_x = (int16_t)(binding->param1 & 0xFFFF);
    int16_t delta_y = (int16_t)((binding->param1 >> 16) & 0xFFFF);

    // Accumulate movement
    data->accumulated_x += delta_x;
    data->accumulated_y += delta_y;

    // Add to history for smoothing
    data->history_x[data->history_idx] = delta_x;
    data->history_y[data->history_idx] = delta_y;
    data->history_idx = (data->history_idx + 1) % config->smoothing;

    // Calculate smoothed values
    int smooth_x = 0, smooth_y = 0;
    for (int i = 0; i < config->smoothing; i++) {
        smooth_x += data->history_x[i];
        smooth_y += data->history_y[i];
    }
    smooth_x /= config->smoothing;
    smooth_y /= config->smoothing;

    // Calculate magnitude
    int magnitude = ABS(smooth_x) + ABS(smooth_y);

    if (magnitude < config->threshold) {
        return 0;
    }

    // Determine scroll direction with bias
    int abs_x = ABS(smooth_x);
    int abs_y = ABS(smooth_y);

    // Apply bias factors (stored as x10 to avoid float)
    abs_y = (abs_y * config->vertical_bias) / 10;
    abs_x = (abs_x * config->horizontal_bias) / 10;

    struct zmk_mouse_scroll_state_changed *scroll_ev;

    if (abs_y > abs_x) {
        // Vertical scroll
        if (data->last_direction == 2) {
            // Direction change, apply hysteresis
            if (abs_y < abs_x * 2) {
                return 0;
            }
        }
        data->last_direction = 1;

        scroll_ev = new_zmk_mouse_scroll_state_changed();
        if (scroll_ev) {
            if (smooth_y > 0) {
                scroll_ev->state.v = -1;  // Scroll down
            } else {
                scroll_ev->state.v = 1;   // Scroll up
            }
            scroll_ev->state.h = 0;
            scroll_ev->timestamp = k_uptime_get();
            ZMK_EVENT_RAISE(scroll_ev);
        }
    } else {
        // Horizontal scroll
        if (data->last_direction == 1) {
            // Direction change, apply hysteresis
            if (abs_x < abs_y * 2) {
                return 0;
            }
        }
        data->last_direction = 2;

        scroll_ev = new_zmk_mouse_scroll_state_changed();
        if (scroll_ev) {
            scroll_ev->state.v = 0;
            if (smooth_x > 0) {
                scroll_ev->state.h = 1;   // Scroll right
            } else {
                scroll_ev->state.h = -1;  // Scroll left
            }
            scroll_ev->timestamp = k_uptime_get();
            ZMK_EVENT_RAISE(scroll_ev);
        }
    }

    // Reset accumulation after sending event
    data->accumulated_x = 0;
    data->accumulated_y = 0;

    return 0;
}

static int omni_scroll_binding_released(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    struct behavior_omni_scroll_data *data = dev->data;

    // Reset state on release
    data->last_direction = 0;
    data->accumulated_x = 0;
    data->accumulated_y = 0;
    data->sample_count = 0;

    // Clear history
    for (int i = 0; i < 5; i++) {
        data->history_x[i] = 0;
        data->history_y[i] = 0;
    }
    data->history_idx = 0;

    return 0;
}

static const struct behavior_driver_api behavior_omni_scroll_driver_api = {
    .binding_pressed = omni_scroll_binding_pressed,
    .binding_released = omni_scroll_binding_released,
};

static int behavior_omni_scroll_init(const struct device *dev) {
    struct behavior_omni_scroll_data *data = dev->data;

    data->accumulated_x = 0;
    data->accumulated_y = 0;
    data->last_direction = 0;
    data->sample_count = 0;
    data->history_idx = 0;

    for (int i = 0; i < 5; i++) {
        data->history_x[i] = 0;
        data->history_y[i] = 0;
    }

    return 0;
}

#define OMNI_SCROLL_INST(n)                                                    \
    static struct behavior_omni_scroll_data behavior_omni_scroll_data_##n = {}; \
    static struct behavior_omni_scroll_config behavior_omni_scroll_config_##n = { \
        .threshold = DT_INST_PROP(n, threshold),                               \
        .vertical_bias = DT_INST_PROP(n, vertical_bias),                      \
        .horizontal_bias = DT_INST_PROP(n, horizontal_bias),                  \
        .smoothing = DT_INST_PROP(n, smoothing),                              \
        .diagonal_threshold = DT_INST_PROP(n, diagonal_threshold),            \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(n, behavior_omni_scroll_init, NULL,                 \
                         &behavior_omni_scroll_data_##n,                      \
                         &behavior_omni_scroll_config_##n,                    \
                         APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,    \
                         &behavior_omni_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OMNI_SCROLL_INST)