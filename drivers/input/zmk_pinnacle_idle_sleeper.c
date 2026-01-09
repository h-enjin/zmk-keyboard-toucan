/*
 * Copyright (c) 2022 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * ZMK Activity-based power management for Cirque Pinnacle
 * with Wake-on-Touch optimization
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include "input_pinnacle.h"

LOG_MODULE_DECLARE(pinnacle, CONFIG_INPUT_LOG_LEVEL);

/* Collect all Pinnacle devices */
#define GET_PINNACLE(n) DEVICE_DT_GET(DT_INST(n, cirque_pinnacle)),
static const struct device *pinnacles[] = {
    DT_INST_FOREACH_STATUS_OKAY(GET_PINNACLE)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    for (int i = 0; i < ARRAY_SIZE(pinnacles); i++) {
        if (!device_is_ready(pinnacles[i])) {
            continue;
        }

#if IS_ENABLED(CONFIG_ZMK_INPUT_PINNACLE_WAKE_ON_TOUCH)
        /*
         * Wake-on-Touch optimized power management:
         *
         * ACTIVE -> Full power, feed enabled
         * IDLE   -> Feed disabled only (instant wake on touch)
         * SLEEP  -> Full sleep with wake-on-touch interrupt
         *
         * The key optimization is keeping DR GPIO interrupt enabled
         * even during IDLE/SLEEP states, so any touch immediately
         * triggers wake-up without waiting for ZMK's activity timer.
         */
        switch (ev->state) {
        case ZMK_ACTIVITY_ACTIVE:
            LOG_DBG("Pinnacle[%d]: -> ACTIVE", i);
            pinnacle_set_power_state(pinnacles[i], PINNACLE_POWER_ACTIVE);
            break;

        case ZMK_ACTIVITY_IDLE:
            LOG_DBG("Pinnacle[%d]: -> IDLE (fast wake)", i);
            /* Use IDLE mode for faster wake-up than full SLEEP */
            pinnacle_set_power_state(pinnacles[i], PINNACLE_POWER_IDLE);
            break;

        case ZMK_ACTIVITY_SLEEP:
            LOG_DBG("Pinnacle[%d]: -> SLEEP (wake-on-touch enabled)", i);
            /* Full sleep but with wake-on-touch interrupt still active */
            pinnacle_set_power_state(pinnacles[i], PINNACLE_POWER_SLEEP);
            break;
        }
#else
        /* Original behavior: simple sleep on/off */
        bool sleep = (ev->state != ZMK_ACTIVITY_ACTIVE);
        LOG_DBG("Pinnacle[%d]: sleep=%d", i, sleep);
        pinnacle_set_sleep(pinnacles[i], sleep);
#endif
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(pinnacle_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(pinnacle_idle_sleeper, zmk_activity_state_changed);
