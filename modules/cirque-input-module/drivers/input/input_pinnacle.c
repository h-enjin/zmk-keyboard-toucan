/*
 * Copyright (c) 2022 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Cirque Pinnacle Trackpad Driver with Wake-on-Touch optimization
 */

#define DT_DRV_COMPAT cirque_pinnacle

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include "input_pinnacle.h"

LOG_MODULE_REGISTER(pinnacle, CONFIG_INPUT_LOG_LEVEL);

/* ADC sensitivity tuning values */
static const uint8_t adc_attn_values[] = {
    0x00, /* 1x */
    0x01, /* 2x */
    0x02, /* 3x */
    0x03, /* 4x */
};

/* Forward declarations */
static void set_int(const struct device *dev, bool enable);
static int pinnacle_write(const struct device *dev, uint8_t addr, uint8_t val);
static int pinnacle_read(const struct device *dev, uint8_t addr, uint8_t *val);

/*
 * Bus-specific implementations
 */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int pinnacle_spi_write(const struct device *dev, uint8_t addr, uint8_t val) {
    const struct pinnacle_config *cfg = dev->config;
    uint8_t tx_buf[2] = {(addr | 0x80), val};
    struct spi_buf tx = {.buf = tx_buf, .len = 2};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    return spi_write_dt(&cfg->bus.spi, &tx_set);
}

static int pinnacle_spi_read(const struct device *dev, uint8_t addr, uint8_t *val) {
    const struct pinnacle_config *cfg = dev->config;
    uint8_t tx_buf[3] = {(addr | 0xA0), 0xFC, 0xFC};
    uint8_t rx_buf[3] = {0};
    struct spi_buf tx = {.buf = tx_buf, .len = 3};
    struct spi_buf rx = {.buf = rx_buf, .len = 3};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};
    int ret = spi_transceive_dt(&cfg->bus.spi, &tx_set, &rx_set);
    if (ret == 0) {
        *val = rx_buf[2];
    }
    return ret;
}

static int pinnacle_spi_seq_read(const struct device *dev, uint8_t addr, uint8_t *buf, uint8_t len) {
    const struct pinnacle_config *cfg = dev->config;
    uint8_t tx_buf[len + 3];
    uint8_t rx_buf[len + 3];

    tx_buf[0] = (addr | 0xA0);
    tx_buf[1] = 0xFC;
    tx_buf[2] = 0xFC;
    for (int i = 0; i < len; i++) {
        tx_buf[i + 3] = 0xFC;
    }

    struct spi_buf tx = {.buf = tx_buf, .len = len + 3};
    struct spi_buf rx = {.buf = rx_buf, .len = len + 3};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    int ret = spi_transceive_dt(&cfg->bus.spi, &tx_set, &rx_set);
    if (ret == 0) {
        memcpy(buf, &rx_buf[3], len);
    }
    return ret;
}

#endif /* SPI */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pinnacle_i2c_write(const struct device *dev, uint8_t addr, uint8_t val) {
    const struct pinnacle_config *cfg = dev->config;
    return i2c_reg_write_byte_dt(&cfg->bus.i2c, addr, val);
}

static int pinnacle_i2c_read(const struct device *dev, uint8_t addr, uint8_t *val) {
    const struct pinnacle_config *cfg = dev->config;
    return i2c_reg_read_byte_dt(&cfg->bus.i2c, addr, val);
}

static int pinnacle_i2c_seq_read(const struct device *dev, uint8_t addr, uint8_t *buf, uint8_t len) {
    const struct pinnacle_config *cfg = dev->config;
    return i2c_burst_read_dt(&cfg->bus.i2c, addr, buf, len);
}

#endif /* I2C */

/*
 * Register access helpers
 */

static int pinnacle_write(const struct device *dev, uint8_t addr, uint8_t val) {
    const struct pinnacle_config *cfg = dev->config;
    return cfg->write(dev, addr, val);
}

static int pinnacle_read(const struct device *dev, uint8_t addr, uint8_t *val) {
    const struct pinnacle_config *cfg = dev->config;
    return cfg->read(dev, addr, val);
}

static int pinnacle_seq_read(const struct device *dev, uint8_t addr, uint8_t *buf, uint8_t len) {
    const struct pinnacle_config *cfg = dev->config;
    return cfg->seq_read(dev, addr, buf, len);
}

static int pinnacle_era_write(const struct device *dev, uint16_t addr, uint8_t val) {
    int ret;

    ret = pinnacle_write(dev, PINNACLE_ERA_ADDR_HIGH, (addr >> 8) & 0xFF);
    if (ret) return ret;

    ret = pinnacle_write(dev, PINNACLE_ERA_ADDR_LOW, addr & 0xFF);
    if (ret) return ret;

    ret = pinnacle_write(dev, PINNACLE_ERA_VALUE, val);
    if (ret) return ret;

    ret = pinnacle_write(dev, PINNACLE_ERA_CTRL, PINNACLE_ERA_CTRL_WRITE);
    if (ret) return ret;

    /* Wait for ERA operation to complete */
    uint8_t ctrl;
    do {
        ret = pinnacle_read(dev, PINNACLE_ERA_CTRL, &ctrl);
        if (ret) return ret;
    } while (ctrl != 0);

    return 0;
}

static void pinnacle_clear_status(const struct device *dev) {
    pinnacle_write(dev, PINNACLE_STATUS1, 0x00);
    k_usleep(50);
}

/*
 * Interrupt handling
 */

static void set_int(const struct device *dev, bool enable) {
    const struct pinnacle_config *cfg = dev->config;

    if (!cfg->dr_gpio.port) {
        return;
    }

    gpio_pin_interrupt_configure_dt(&cfg->dr_gpio,
        enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
}

static void pinnacle_gpio_cb(const struct device *port, struct gpio_callback *cb,
                              gpio_port_pins_t pins) {
    struct pinnacle_data *data = CONTAINER_OF(cb, struct pinnacle_data, gpio_cb);

#if IS_ENABLED(CONFIG_ZMK_INPUT_PINNACLE_WAKE_ON_TOUCH)
    /* Wake-on-Touch: If in sleep/idle, immediately start waking up */
    if (data->power_state != PINNACLE_POWER_ACTIVE) {
        LOG_DBG("Wake-on-Touch triggered!");
        /* Fast wake: Just enable feed, skip full initialization */
        pinnacle_write(data->dev, PINNACLE_FEED_CFG1,
                       data->feed_cfg1_cache | PINNACLE_FEED_CFG1_EN);
        data->power_state = PINNACLE_POWER_ACTIVE;
    }
#endif

    data->in_int = true;
    k_work_submit(&data->work);
}

/*
 * Data reporting
 */

static void pinnacle_report_data(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    uint8_t packet[3];
    int ret;

    /* Clear status and read packet */
    pinnacle_clear_status(dev);

    ret = pinnacle_seq_read(dev, PINNACLE_PACKET_BYTE_0, packet, 3);
    if (ret) {
        LOG_ERR("Failed to read packet: %d", ret);
        return;
    }

    /* Extract button and motion data */
    uint8_t btn = packet[0] & 0x07;
    int16_t dx = (int16_t)(int8_t)packet[1];
    int16_t dy = (int16_t)(int8_t)packet[2];

    /* Report motion */
    if (dx != 0 || dy != 0) {
        input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
    }

    /* Report button changes */
    if (btn != data->btn_cache) {
        if ((btn ^ data->btn_cache) & 0x01) {
            input_report_key(dev, INPUT_BTN_LEFT, btn & 0x01, true, K_FOREVER);
        }
        if ((btn ^ data->btn_cache) & 0x02) {
            input_report_key(dev, INPUT_BTN_RIGHT, btn & 0x02, true, K_FOREVER);
        }
        if ((btn ^ data->btn_cache) & 0x04) {
            input_report_key(dev, INPUT_BTN_MIDDLE, btn & 0x04, true, K_FOREVER);
        }
        data->btn_cache = btn;
    }
}

static void pinnacle_work_cb(struct k_work *work) {
    struct pinnacle_data *data = CONTAINER_OF(work, struct pinnacle_data, work);

    if (data->in_int) {
        data->in_int = false;
        pinnacle_report_data(data->dev);
    }
}

/*
 * Power Management API
 */

int pinnacle_set_sleep(const struct device *dev, bool sleep) {
    const struct pinnacle_config *cfg = dev->config;
    struct pinnacle_data *data = dev->data;
    int ret;
    uint8_t val;

    ret = pinnacle_read(dev, PINNACLE_SYS_CFG, &val);
    if (ret) {
        LOG_ERR("Failed to read SYS_CFG: %d", ret);
        return ret;
    }

    if (sleep) {
        val |= PINNACLE_SYS_CFG_SLEEP_EN;
        data->power_state = PINNACLE_POWER_SLEEP;

#if !IS_ENABLED(CONFIG_ZMK_INPUT_PINNACLE_WAKE_ON_TOUCH)
        /* Original behavior: disable interrupt during sleep */
        set_int(dev, false);
#endif
        /* Wake-on-Touch: Keep interrupt enabled! */

    } else {
        val &= ~PINNACLE_SYS_CFG_SLEEP_EN;
        data->power_state = PINNACLE_POWER_ACTIVE;
        set_int(dev, true);

#if IS_ENABLED(CONFIG_ZMK_INPUT_PINNACLE_FAST_WAKE)
        /* Fast wake: restore feed config from cache */
        pinnacle_write(dev, PINNACLE_FEED_CFG1, data->feed_cfg1_cache);
#endif
    }

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, val);
    if (ret) {
        LOG_ERR("Failed to write SYS_CFG: %d", ret);
    }

    LOG_DBG("Sleep mode: %s", sleep ? "enabled" : "disabled");
    return ret;
}

int pinnacle_set_idle(const struct device *dev, bool idle) {
    struct pinnacle_data *data = dev->data;
    int ret;

    if (idle) {
        /* Idle mode: Just disable feed (fast wake possible) */
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, 0x00);
        if (ret == 0) {
            data->power_state = PINNACLE_POWER_IDLE;
        }
        /* Keep interrupt enabled for wake-on-touch */
    } else {
        /* Resume from idle: Just re-enable feed */
#if IS_ENABLED(CONFIG_ZMK_INPUT_PINNACLE_FAST_WAKE)
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, data->feed_cfg1_cache);
#else
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, PINNACLE_FEED_CFG1_EN);
#endif
        if (ret == 0) {
            data->power_state = PINNACLE_POWER_ACTIVE;
        }
    }

    LOG_DBG("Idle mode: %s", idle ? "enabled" : "disabled");
    return ret;
}

int pinnacle_set_power_state(const struct device *dev, enum pinnacle_power_state state) {
    struct pinnacle_data *data = dev->data;

    if (data->power_state == state) {
        return 0;
    }

    switch (state) {
    case PINNACLE_POWER_ACTIVE:
        if (data->power_state == PINNACLE_POWER_SLEEP) {
            return pinnacle_set_sleep(dev, false);
        } else {
            return pinnacle_set_idle(dev, false);
        }

    case PINNACLE_POWER_IDLE:
        if (data->power_state == PINNACLE_POWER_SLEEP) {
            /* Wake from sleep first, then go to idle */
            pinnacle_set_sleep(dev, false);
        }
        return pinnacle_set_idle(dev, true);

    case PINNACLE_POWER_SLEEP:
        return pinnacle_set_sleep(dev, true);

    default:
        return -EINVAL;
    }
}

enum pinnacle_power_state pinnacle_get_power_state(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    return data->power_state;
}

/*
 * Initialization
 */

static int pinnacle_configure_sensitivity(const struct device *dev) {
    const struct pinnacle_config *cfg = dev->config;
    int ret;

    if (cfg->sensitivity > PINNACLE_SENSITIVITY_4X) {
        LOG_WRN("Invalid sensitivity, using 1x");
        return 0;
    }

    uint8_t attn = adc_attn_values[cfg->sensitivity];

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_ADC_ATTN_X, attn);
    if (ret) return ret;

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_ADC_ATTN_Y, attn);
    if (ret) return ret;

    LOG_DBG("Sensitivity set to %dx", cfg->sensitivity + 1);
    return 0;
}

static int pinnacle_configure_edge_sensitivity(const struct device *dev) {
    const struct pinnacle_config *cfg = dev->config;
    int ret;

    if (cfg->x_axis_z_min > 0) {
        ret = pinnacle_era_write(dev, 0x0149, cfg->x_axis_z_min);
        if (ret) return ret;
    }

    if (cfg->y_axis_z_min > 0) {
        ret = pinnacle_era_write(dev, 0x0168, cfg->y_axis_z_min);
        if (ret) return ret;
    }

    return 0;
}

static int pinnacle_init(const struct device *dev) {
    const struct pinnacle_config *cfg = dev->config;
    struct pinnacle_data *data = dev->data;
    int ret;
    uint8_t fw_id;

    data->dev = dev;
    data->power_state = PINNACLE_POWER_ACTIVE;

    /* Initialize work queue */
    k_work_init(&data->work, pinnacle_work_cb);

    /* Check bus readiness */
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
    if (cfg->read == pinnacle_spi_read) {
        if (!spi_is_ready_dt(&cfg->bus.spi)) {
            LOG_ERR("SPI bus not ready");
            return -ENODEV;
        }
    }
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
    if (cfg->read == pinnacle_i2c_read) {
        if (!i2c_is_ready_dt(&cfg->bus.i2c)) {
            LOG_ERR("I2C bus not ready");
            return -ENODEV;
        }
    }
#endif

    /* Read firmware ID */
    ret = pinnacle_read(dev, PINNACLE_FW_ID, &fw_id);
    if (ret) {
        LOG_ERR("Failed to read FW ID: %d", ret);
        return ret;
    }
    LOG_INF("Pinnacle FW ID: 0x%02X", fw_id);

    /* Clear status */
    pinnacle_clear_status(dev);

    /* Reset device */
    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
    if (ret) {
        LOG_ERR("Failed to reset: %d", ret);
        return ret;
    }
    k_msleep(50);
    pinnacle_clear_status(dev);

    /* Configure sensitivity */
    ret = pinnacle_configure_sensitivity(dev);
    if (ret) {
        LOG_ERR("Failed to configure sensitivity: %d", ret);
        return ret;
    }

    /* Configure edge sensitivity */
    ret = pinnacle_configure_edge_sensitivity(dev);
    if (ret) {
        LOG_WRN("Failed to configure edge sensitivity: %d", ret);
    }

    /* Configure curved overlay compensation if enabled */
    if (cfg->curved_overlay) {
        pinnacle_era_write(dev, 0x0187, 0x01);
    }

    /* Force calibration */
    ret = pinnacle_write(dev, PINNACLE_CAL_CFG,
                         PINNACLE_CAL_CFG_CAL |
                         PINNACLE_CAL_CFG_BG_COMP |
                         PINNACLE_CAL_CFG_NRD_COMP |
                         PINNACLE_CAL_CFG_TRACK);
    if (ret) {
        LOG_ERR("Failed to calibrate: %d", ret);
        return ret;
    }
    k_msleep(100);

    /* Build FEED_CFG1 value */
    uint8_t feed_cfg1 = PINNACLE_FEED_CFG1_EN;
    if (cfg->x_invert) feed_cfg1 |= PINNACLE_FEED_CFG1_X_INV;
    if (cfg->y_invert) feed_cfg1 |= PINNACLE_FEED_CFG1_Y_INV;

    ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);
    if (ret) {
        LOG_ERR("Failed to configure FEED_CFG1: %d", ret);
        return ret;
    }

#if IS_ENABLED(CONFIG_ZMK_INPUT_PINNACLE_FAST_WAKE)
    /* Cache for fast wake */
    data->feed_cfg1_cache = feed_cfg1;
#endif

    /* Build FEED_CFG2 value */
    uint8_t feed_cfg2 = 0;
    if (cfg->no_taps) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_ALL_TAP_DIS;
    }
    if (cfg->no_secondary_tap) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_SEC_TAP_DIS;
    }
    if (cfg->rotate_90) {
        feed_cfg2 |= BIT(7); /* 90 degree rotation */
    }

    ret = pinnacle_write(dev, PINNACLE_FEED_CFG2, feed_cfg2);
    if (ret) {
        LOG_ERR("Failed to configure FEED_CFG2: %d", ret);
        return ret;
    }

    /* Configure sleep mode if enabled in DTS */
    if (cfg->sleep_en) {
        ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_SLEEP_EN);
        if (ret) {
            LOG_WRN("Failed to enable sleep: %d", ret);
        }
    }

    /* Setup DR GPIO interrupt */
    if (cfg->dr_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->dr_gpio)) {
            LOG_ERR("DR GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&cfg->dr_gpio, GPIO_INPUT);
        if (ret) {
            LOG_ERR("Failed to configure DR GPIO: %d", ret);
            return ret;
        }

        gpio_init_callback(&data->gpio_cb, pinnacle_gpio_cb, BIT(cfg->dr_gpio.pin));
        ret = gpio_add_callback(cfg->dr_gpio.port, &data->gpio_cb);
        if (ret) {
            LOG_ERR("Failed to add GPIO callback: %d", ret);
            return ret;
        }

        set_int(dev, true);
    }

    LOG_INF("Pinnacle initialized successfully");
    return 0;
}

/*
 * Zephyr PM callbacks
 */

#ifdef CONFIG_PM_DEVICE
static int pinnacle_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        set_int(dev, false);
        return 0;
    case PM_DEVICE_ACTION_RESUME:
        set_int(dev, true);
        return 0;
    default:
        return -ENOTSUP;
    }
}
#endif

/*
 * Device instantiation macros
 */

#define PINNACLE_SENSITIVITY(n) \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, sensitivity), \
        (_CONCAT(PINNACLE_SENSITIVITY_, DT_INST_STRING_UPPER_TOKEN(n, sensitivity))), \
        (PINNACLE_SENSITIVITY_1X))

#define PINNACLE_INIT_SPI(n)                                                    \
    static struct pinnacle_data pinnacle_data_##n;                              \
    static const struct pinnacle_config pinnacle_config_##n = {                 \
        .bus = { .spi = SPI_DT_SPEC_INST_GET(n,                                \
                    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0)},\
        .read = pinnacle_spi_read,                                              \
        .write = pinnacle_spi_write,                                            \
        .seq_read = pinnacle_spi_seq_read,                                      \
        .dr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, dr_gpios, {0}),                  \
        .sensitivity = PINNACLE_SENSITIVITY(n),                                 \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 0),                    \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 0),                    \
        .sleep_en = DT_INST_PROP(n, sleep),                                     \
        .no_taps = DT_INST_PROP(n, no_taps),                                    \
        .no_secondary_tap = DT_INST_PROP(n, no_secondary_tap),                  \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                \
        .x_invert = DT_INST_PROP(n, x_invert),                                  \
        .y_invert = DT_INST_PROP(n, y_invert),                                  \
        .curved_overlay = DT_INST_PROP(n, curved_overlay),                      \
    };                                                                          \
    PM_DEVICE_DT_INST_DEFINE(n, pinnacle_pm_action);                            \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, PM_DEVICE_DT_INST_GET(n),           \
                          &pinnacle_data_##n, &pinnacle_config_##n,             \
                          POST_KERNEL, CONFIG_INPUT_PINNACLE_INIT_PRIORITY, NULL);

#define PINNACLE_INIT_I2C(n)                                                    \
    static struct pinnacle_data pinnacle_data_##n;                              \
    static const struct pinnacle_config pinnacle_config_##n = {                 \
        .bus = { .i2c = I2C_DT_SPEC_INST_GET(n) },                             \
        .read = pinnacle_i2c_read,                                              \
        .write = pinnacle_i2c_write,                                            \
        .seq_read = pinnacle_i2c_seq_read,                                      \
        .dr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, dr_gpios, {0}),                  \
        .sensitivity = PINNACLE_SENSITIVITY(n),                                 \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 0),                    \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 0),                    \
        .sleep_en = DT_INST_PROP(n, sleep),                                     \
        .no_taps = DT_INST_PROP(n, no_taps),                                    \
        .no_secondary_tap = DT_INST_PROP(n, no_secondary_tap),                  \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                \
        .x_invert = DT_INST_PROP(n, x_invert),                                  \
        .y_invert = DT_INST_PROP(n, y_invert),                                  \
        .curved_overlay = DT_INST_PROP(n, curved_overlay),                      \
    };                                                                          \
    PM_DEVICE_DT_INST_DEFINE(n, pinnacle_pm_action);                            \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, PM_DEVICE_DT_INST_GET(n),           \
                          &pinnacle_data_##n, &pinnacle_config_##n,             \
                          POST_KERNEL, CONFIG_INPUT_PINNACLE_INIT_PRIORITY, NULL);

#define PINNACLE_INIT(n)                                              \
    COND_CODE_1(DT_INST_ON_BUS(n, spi),                              \
                (PINNACLE_INIT_SPI(n)),                               \
                (PINNACLE_INIT_I2C(n)))

DT_INST_FOREACH_STATUS_OKAY(PINNACLE_INIT)
