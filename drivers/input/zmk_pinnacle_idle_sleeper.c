

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

// Polling interval when idle (checking for wake)
#define TOUCH_POLL_INTERVAL_MS 50

// Polling interval when active (checking for idle transition)
#define ACTIVE_CHECK_INTERVAL_MS 500

// Time without touch before going idle (in ms)
#define TRACKPAD_IDLE_TIMEOUT_MS 5000

// State tracking
static bool is_trackpad_idle = false;      // Trackpad-specific idle state
static bool is_zmk_idle = false;           // ZMK activity state
static int64_t last_touch_time = 0;        // Last time touch was detected

static struct k_work_delayable poll_work;
static struct k_work_delayable idle_check_work;

// Forward declarations
static void start_idle_polling(void);
static void start_active_checking(void);
static void set_trackpad_idle(bool idle);

static void set_trackpad_idle(bool idle) {
    if (idle == is_trackpad_idle) {
        return;
    }

    is_trackpad_idle = idle;
    LOG_DBG("Trackpad idle: %d", idle);

    for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
        pinnacle_set_idle(pinnacle_devs[i], idle);
    }
}

// Called when trackpad is idle, polling for touch to wake up
static void poll_for_touch(struct k_work *work) {
    if (!is_trackpad_idle) {
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
        // Briefly enable feed to check for touch
        pinnacle_set_idle(pinnacle_devs[i], false);

        // Small delay to let data become available
        k_usleep(100);

        // Check if there's touch data
        if (pinnacle_check_touch(pinnacle_devs[i])) {
            // Touch detected! Wake up trackpad
            LOG_DBG("Touch detected during idle, waking up");
            last_touch_time = k_uptime_get();
            pinnacle_trigger_report(pinnacle_devs[i]);

            // Set trackpad to active
            set_trackpad_idle(false);

            // Start checking for idle transition
            start_active_checking();
            return;
        }

        // No touch, disable feed again
        pinnacle_set_idle(pinnacle_devs[i], true);
    }

    // Continue polling
    k_work_schedule(&poll_work, K_MSEC(TOUCH_POLL_INTERVAL_MS));
}

// Called when trackpad is active, checking if we should go idle
static void check_for_idle(struct k_work *work) {
    if (is_trackpad_idle) {
        return;
    }

    // Check if there's current touch
    bool touch_detected = false;
    for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
        if (pinnacle_check_touch(pinnacle_devs[i])) {
            touch_detected = true;
            last_touch_time = k_uptime_get();
            break;
        }
    }

    // Check if timeout has elapsed
    int64_t elapsed = k_uptime_get() - last_touch_time;

    if (!touch_detected && elapsed >= TRACKPAD_IDLE_TIMEOUT_MS) {
        // No touch for timeout period, go idle
        LOG_DBG("Trackpad timeout, going idle");
        set_trackpad_idle(true);
        start_idle_polling();
        return;
    }

    // Continue checking
    k_work_schedule(&idle_check_work, K_MSEC(ACTIVE_CHECK_INTERVAL_MS));
}

static void start_idle_polling(void) {
    k_work_cancel_delayable(&idle_check_work);
    k_work_schedule(&poll_work, K_MSEC(TOUCH_POLL_INTERVAL_MS));
}

static void start_active_checking(void) {
    k_work_cancel_delayable(&poll_work);
    k_work_schedule(&idle_check_work, K_MSEC(ACTIVE_CHECK_INTERVAL_MS));
}

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    bool new_zmk_idle = state_ev->state != ZMK_ACTIVITY_ACTIVE;

    if (new_zmk_idle == is_zmk_idle) {
        return 0; // No state change
    }

    is_zmk_idle = new_zmk_idle;
    LOG_DBG("ZMK activity state changed, zmk_idle: %d", is_zmk_idle);

    if (is_zmk_idle) {
        // ZMK went idle - force trackpad to idle too
        if (!is_trackpad_idle) {
            set_trackpad_idle(true);
            start_idle_polling();
        }
    }
    // Note: When ZMK becomes active, we don't automatically wake the trackpad
    // Trackpad will wake when user actually touches it

    return 0;
}

static int pinnacle_sleeper_init(void) {
    k_work_init_delayable(&poll_work, poll_for_touch);
    k_work_init_delayable(&idle_check_work, check_for_idle);

    // Start with trackpad active, begin checking for idle
    last_touch_time = k_uptime_get();
    start_active_checking();

    return 0;
}

SYS_INIT(pinnacle_sleeper_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(zmk_pinnacle_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pinnacle_idle_sleeper, zmk_activity_state_changed);
