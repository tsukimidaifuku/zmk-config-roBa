/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_threshold_temp_layer

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/input_processor.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define MAX_LAYERS ZMK_KEYMAP_LAYERS_LEN

struct threshold_temp_layer_config {
    int16_t movement_threshold;
};

struct threshold_temp_layer_state {
    uint8_t toggle_layer;
    bool is_active;
    int16_t pending_x;
    int16_t pending_y;
};

struct threshold_temp_layer_data {
    struct k_mutex lock;
    struct threshold_temp_layer_state state;
    struct k_work_delayable disable_work;
    const struct device *dev;
};

struct layer_state_action {
    const struct device *dev;
    uint8_t layer;
    bool activate;
};

K_MSGQ_DEFINE(threshold_temp_layer_action_msgq, sizeof(struct layer_state_action), 8, 4);

static bool is_xy_event(const struct input_event *event) {
    return event->type == INPUT_EV_REL &&
           (event->code == INPUT_REL_X || event->code == INPUT_REL_Y);
}

static void update_layer_state(struct threshold_temp_layer_state *state, bool activate) {
    if (state->is_active == activate) {
        return;
    }

    state->is_active = activate;
    if (activate) {
        zmk_keymap_layer_activate(state->toggle_layer);
        LOG_DBG("Layer %d activated", state->toggle_layer);
    } else {
        zmk_keymap_layer_deactivate(state->toggle_layer);
        LOG_DBG("Layer %d deactivated", state->toggle_layer);
    }
}

static void layer_action_work_cb(struct k_work *work) {
    (void)work;

    struct layer_state_action action;

    while (k_msgq_get(&threshold_temp_layer_action_msgq, &action, K_NO_WAIT) >= 0) {
        struct threshold_temp_layer_data *data = action.dev->data;

        int ret = k_mutex_lock(&data->lock, K_FOREVER);
        if (ret < 0) {
            LOG_ERR("Error locking for layer action: %d", ret);
            continue;
        }

        data->state.toggle_layer = action.layer;
        if (!action.activate) {
            if (zmk_keymap_layer_active(action.layer)) {
                update_layer_state(&data->state, false);
            }
        } else {
            update_layer_state(&data->state, true);
        }

        k_mutex_unlock(&data->lock);
    }
}

static K_WORK_DEFINE(layer_action_work, layer_action_work_cb);

static void queue_layer_state_action(const struct device *dev, uint8_t layer, bool activate) {
    struct layer_state_action action = {
        .dev = dev,
        .layer = layer,
        .activate = activate,
    };

    int ret = k_msgq_put(&threshold_temp_layer_action_msgq, &action, K_MSEC(10));
    if (ret < 0) {
        LOG_ERR("Error queueing layer action: %d", ret);
        return;
    }

    k_work_submit(&layer_action_work);
}

static void layer_disable_callback(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct threshold_temp_layer_data *data =
        CONTAINER_OF(d_work, struct threshold_temp_layer_data, disable_work);

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        LOG_ERR("Error locking for layer disable: %d", ret);
        return;
    }

    uint8_t layer = data->state.toggle_layer;

    k_mutex_unlock(&data->lock);

    queue_layer_state_action(data->dev, layer, false);
}

static int handle_layer_state_changed(const struct device *dev, const zmk_event_t *eh) {
    (void)eh;

    struct threshold_temp_layer_data *data = dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    if (data->state.is_active &&
        !zmk_keymap_layer_active(zmk_keymap_layer_index_to_id(data->state.toggle_layer))) {
        data->state.is_active = false;
        k_work_cancel_delayable(&data->disable_work);
    }

    k_mutex_unlock(&data->lock);

    return ZMK_EV_EVENT_BUBBLE;
}

#define DISPATCH_EVENT(inst)                                                                       \
    {                                                                                              \
        int err = handle_layer_state_changed(DEVICE_DT_INST_GET(inst), eh);                        \
        if (err < 0) {                                                                             \
            return err;                                                                            \
        }                                                                                          \
    }

static int handle_event_dispatcher(const zmk_event_t *eh) {
    DT_INST_FOREACH_STATUS_OKAY(DISPATCH_EVENT)

    return 0;
}

static bool xy_report_should_trigger(const struct device *dev, const struct input_event *event) {
    const struct threshold_temp_layer_config *cfg = dev->config;
    struct threshold_temp_layer_data *data = dev->data;

    if (event->code == INPUT_REL_X) {
        data->state.pending_x += event->value;
    } else {
        data->state.pending_y += event->value;
    }

    if (!event->sync) {
        return false;
    }

    int movement = abs((int)data->state.pending_x) + abs((int)data->state.pending_y);
    data->state.pending_x = 0;
    data->state.pending_y = 0;

    return data->state.is_active || movement > cfg->movement_threshold;
}

static int threshold_temp_layer_handle_event(const struct device *dev, struct input_event *event,
                                             uint32_t param1, uint32_t param2,
                                             struct zmk_input_processor_state *state) {
    (void)state;

    if (param1 >= MAX_LAYERS) {
        LOG_ERR("Invalid layer index: %d", param1);
        return -EINVAL;
    }

    struct threshold_temp_layer_data *data = dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    data->state.toggle_layer = param1;

    bool should_trigger = is_xy_event(event) ? xy_report_should_trigger(dev, event) : true;
    if (!should_trigger) {
        k_mutex_unlock(&data->lock);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->state.is_active) {
        queue_layer_state_action(dev, param1, true);
    }

    if (param2 > 0) {
        k_work_reschedule(&data->disable_work, K_MSEC(param2));
    }

    k_mutex_unlock(&data->lock);

    return ZMK_INPUT_PROC_CONTINUE;
}

static int threshold_temp_layer_init(const struct device *dev) {
    struct threshold_temp_layer_data *data = dev->data;

    data->dev = dev;
    k_mutex_init(&data->lock);
    k_work_init_delayable(&data->disable_work, layer_disable_callback);

    return 0;
}

static const struct zmk_input_processor_driver_api threshold_temp_layer_driver_api = {
    .handle_event = threshold_temp_layer_handle_event,
};

ZMK_LISTENER(processor_threshold_temp_layer, handle_event_dispatcher);
ZMK_SUBSCRIPTION(processor_threshold_temp_layer, zmk_layer_state_changed);

#define THRESHOLD_TEMP_LAYER_INST(n)                                                               \
    static struct threshold_temp_layer_data threshold_temp_layer_data_##n = {};                     \
    static const struct threshold_temp_layer_config threshold_temp_layer_config_##n = {             \
        .movement_threshold = DT_INST_PROP_OR(n, movement_threshold, 10),                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, threshold_temp_layer_init, NULL, &threshold_temp_layer_data_##n,       \
                          &threshold_temp_layer_config_##n, POST_KERNEL,                           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &threshold_temp_layer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(THRESHOLD_TEMP_LAYER_INST)

#endif
