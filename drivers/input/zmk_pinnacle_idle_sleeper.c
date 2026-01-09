

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include "input_pinnacle.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pinnacle_sleeper, CONFIG_INPUT_LOG_LEVEL);

#define GET_PINNACLE(node_id) DEVICE_DT_GET(node_id),

static const struct device *pinnacle_devs[] = {
    DT_FOREACH_STATUS_OKAY(cirque_pinnacle, GET_PINNACLE)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    bool idle = state_ev->state != ZMK_ACTIVITY_ACTIVE;
    LOG_DBG("Activity state changed, setting idle: %d", idle);
    for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
        // Use idle mode (feed disable) instead of full sleep for faster wake
        pinnacle_set_idle(pinnacle_devs[i], idle);
    }

    return 0;
}

ZMK_LISTENER(zmk_pinnacle_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pinnacle_idle_sleeper, zmk_activity_state_changed);