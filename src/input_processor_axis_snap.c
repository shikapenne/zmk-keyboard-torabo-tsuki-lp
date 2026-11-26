/*
 * Axis snap input processor: lets only the dominant axis (X or Y) through.
 * Place it before the xy->scroll mapper so it sees raw REL_X / REL_Y.
 */
#define DT_DRV_COMPAT zmk_input_processor_axis_snap

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>

struct axis_snap_config {
    int32_t idle_ms;
    int32_t decide_threshold;
    int32_t switch_ratio;
};

struct axis_snap_data {
    int64_t last_ms;
    int32_t acc[2]; /* accumulated |movement|: [0]=X, [1]=Y */
    int8_t locked;  /* -1 = undecided, 0 = X, 1 = Y */
};

static int axis_snap_handle_event(const struct device *dev, struct input_event *event,
                                  uint32_t param1, uint32_t param2,
                                  struct zmk_input_processor_state *state) {
    const struct axis_snap_config *cfg = dev->config;
    struct axis_snap_data *d = dev->data;

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int idx = (event->code == INPUT_REL_X) ? 0 : 1;
    int64_t now = k_uptime_get();

    if (now - d->last_ms > cfg->idle_ms) {
        d->acc[0] = d->acc[1] = 0;
        d->locked = -1;
    }
    d->last_ms = now;

    d->acc[idx] += abs(event->value);

    if (d->locked < 0) {
        if (d->acc[0] + d->acc[1] >= cfg->decide_threshold) {
            d->locked = (d->acc[0] >= d->acc[1]) ? 0 : 1;
        }
    } else {
        int other = 1 - d->locked;
        if (d->acc[other] > d->acc[d->locked] * cfg->switch_ratio) {
            d->locked = other;
            d->acc[0] = d->acc[1] = 0;
            d->acc[idx] = abs(event->value);
        }
    }

    if (d->locked != idx) {
        /* keep the event (and its sync flag) but drop its movement */
        event->value = 0;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api axis_snap_driver_api = {
    .handle_event = axis_snap_handle_event,
};

#define AXIS_SNAP_INST(n)                                                                          \
    static struct axis_snap_data axis_snap_data_##n = {.locked = -1};                              \
    static const struct axis_snap_config axis_snap_config_##n = {                                  \
        .idle_ms = DT_INST_PROP(n, idle_ms),                                                       \
        .decide_threshold = DT_INST_PROP(n, decide_threshold),                                     \
        .switch_ratio = DT_INST_PROP(n, switch_ratio),                                             \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &axis_snap_data_##n, &axis_snap_config_##n, POST_KERNEL,  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &axis_snap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXIS_SNAP_INST)
