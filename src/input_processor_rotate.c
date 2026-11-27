/*
 * Rotate input processor: rotates pointer motion by a fixed angle (degrees).
 * Positive angle = counter-clockwise as seen on screen. Place it right after the
 * zip_xy_transform in the chain.
 *
 * REL_X and REL_Y arrive as separate events, so each event contributes its own
 * part now and the cross term for the other axis is carried to the next event of
 * that axis (fixed point, 1/1024 units, remainders are kept so nothing is lost).
 */
#define DT_DRV_COMPAT zmk_input_processor_rotate

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>

static const int16_t sin_tab[91] = {0, 18, 36, 54, 71, 89, 107, 125, 143, 160, 178, 195, 213, 230, 248, 265, 282, 299, 316, 333, 350, 367, 384, 400, 416, 433, 449, 465, 481, 496, 512, 527, 543, 558, 573, 587, 602, 616, 630, 644, 658, 672, 685, 698, 711, 724, 737, 749, 761, 773, 784, 796, 807, 818, 828, 839, 849, 859, 868, 878, 887, 896, 904, 912, 920, 928, 935, 943, 949, 956, 962, 968, 974, 979, 984, 989, 994, 998, 1002, 1005, 1008, 1011, 1014, 1016, 1018, 1020, 1022, 1023, 1023, 1024, 1024};

struct rotate_config {
    int32_t angle;
};

struct rotate_data {
    int32_t cos_v, sin_v; /* 1/1024 units */
    int32_t pend[2]; /* [0]=X, [1]=Y, 1/1024 units */
    int64_t last_ms;
};

static int rotate_handle_event(const struct device *dev, struct input_event *event,
                               uint32_t param1, uint32_t param2,
                               struct zmk_input_processor_state *state) {
    struct rotate_data *d = dev->data;

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now = k_uptime_get();
    if (now - d->last_ms > 100) {
        d->pend[0] = d->pend[1] = 0;
    }
    d->last_ms = now;

    int32_t v = event->value;
    int32_t total;

    if (event->code == INPUT_REL_X) {
        /* x' = x*cos + y*sin ; y' = -x*sin + y*cos */
        total = v * d->cos_v + d->pend[0];
        d->pend[1] -= v * d->sin_v;
        int32_t out = total / 1024;
        d->pend[0] = total - out * 1024;
        event->value = out;
    } else {
        total = v * d->cos_v + d->pend[1];
        d->pend[0] += v * d->sin_v;
        int32_t out = total / 1024;
        d->pend[1] = total - out * 1024;
        event->value = out;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api rotate_driver_api = {
    .handle_event = rotate_handle_event,
};

static int rotate_init(const struct device *dev) {
    const struct rotate_config *cfg = dev->config;
    struct rotate_data *d = dev->data;
    int a = ((cfg->angle % 360) + 360) % 360;
    int q = a / 90, r = a % 90;
    int s0 = sin_tab[r], c0 = sin_tab[90 - r];

    switch (q) {
    case 0: d->sin_v = s0;  d->cos_v = c0;  break;
    case 1: d->sin_v = c0;  d->cos_v = -s0; break;
    case 2: d->sin_v = -s0; d->cos_v = -c0; break;
    default: d->sin_v = -c0; d->cos_v = s0; break;
    }
    return 0;
}

#define ROTATE_INST(n)                                                                             \
    static struct rotate_data rotate_data_##n;                                                     \
    static const struct rotate_config rotate_config_##n = {.angle = DT_INST_PROP(n, angle)};       \
    DEVICE_DT_INST_DEFINE(n, rotate_init, NULL, &rotate_data_##n, &rotate_config_##n, POST_KERNEL, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &rotate_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ROTATE_INST)
