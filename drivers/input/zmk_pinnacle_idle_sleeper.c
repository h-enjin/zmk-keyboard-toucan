

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include "input_pinnacle.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pinnacle_sleeper, CONFIG_INPUT_LOG_LEVEL);

#define GET_PINNACLE(node_id) DEVICE_DT_GET(node_id),

static const struct device *pinnacle_devs[] = {
    DT_FOREACH_STATUS_OKAY(cirque_pinnacle, GET_PINNACLE)
};

// Polling interval in milliseconds (lower = faster wake, higher = more power saving)
#define TOUCH_POLL_INTERVAL_MS 50

static bool is_idle = false;
static struct k_work_delayable poll_work;

static void poll_for_touch(struct k_work *work) {
    if (!is_idle) {
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
        // Briefly enable feed to check for touch
        pinnacle_set_idle(pinnacle_devs[i], false);

        // Small delay to let data become available
        k_usleep(100);

        // Check if there's touch data
        if (pinnacle_check_touch(pinnacle_devs[i])) {
            // Touch detected! Process the data and stop polling
            LOG_DBG("Touch detected during idle, waking up");
            pinnacle_trigger_report(pinnacle_devs[i]);
            // Keep feed enabled and let normal interrupt-driven tracking take over
            // Don't re-schedule polling - ZMK activity state change will handle return to idle
            is_idle = false;
            return;
        }

        // No touch, disable feed again to save power
        pinnacle_set_idle(pinnacle_devs[i], true);
    }

    // Schedule next poll
    k_work_schedule(&poll_work, K_MSEC(TOUCH_POLL_INTERVAL_MS));
}

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    bool new_idle = state_ev->state != ZMK_ACTIVITY_ACTIVE;

    if (new_idle == is_idle) {
        return 0; // No state change
    }

    is_idle = new_idle;
    LOG_DBG("Activity state changed, idle: %d", is_idle);

    if (is_idle) {
        // Going idle - disable feed and start polling
        for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
            pinnacle_set_idle(pinnacle_devs[i], true);
        }
        // Start polling for touch
        k_work_schedule(&poll_work, K_MSEC(TOUCH_POLL_INTERVAL_MS));
    } else {
        // Waking up - cancel polling and enable feed
        k_work_cancel_delayable(&poll_work);
        for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
            pinnacle_set_idle(pinnacle_devs[i], false);
        }
    }

    return 0;
}

static int pinnacle_sleeper_init(void) {
    k_work_init_delayable(&poll_work, poll_for_touch);
    return 0;
}

SYS_INIT(pinnacle_sleeper_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(zmk_pinnacle_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pinnacle_idle_sleeper, zmk_activity_state_changed);
