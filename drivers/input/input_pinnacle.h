/*
 * Copyright (c) 2022 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>

/* Pinnacle Register Addresses */
#define PINNACLE_FW_ID          0x00
#define PINNACLE_FW_VER         0x01
#define PINNACLE_STATUS1        0x02
#define PINNACLE_SYS_CFG        0x03
#define PINNACLE_FEED_CFG1      0x04
#define PINNACLE_FEED_CFG2      0x05
#define PINNACLE_FEED_CFG3      0x06
#define PINNACLE_CAL_CFG        0x07
#define PINNACLE_PS2_AUX        0x08
#define PINNACLE_SAMPLE         0x09
#define PINNACLE_Z_IDLE         0x0A
#define PINNACLE_Z_SCALER       0x0B
#define PINNACLE_SLEEP_INTERVAL 0x0C
#define PINNACLE_SLEEP_TIMER    0x0D
#define PINNACLE_EMI_THRESHOLD  0x0E
#define PINNACLE_PACKET_BYTE_0  0x12
#define PINNACLE_PACKET_BYTE_1  0x13
#define PINNACLE_PACKET_BYTE_2  0x14
#define PINNACLE_PACKET_BYTE_3  0x15
#define PINNACLE_PACKET_BYTE_4  0x16
#define PINNACLE_PACKET_BYTE_5  0x17
#define PINNACLE_ERA_VALUE      0x1B
#define PINNACLE_ERA_ADDR_HIGH  0x1C
#define PINNACLE_ERA_ADDR_LOW   0x1D
#define PINNACLE_ERA_CTRL       0x1E
#define PINNACLE_HCO_ID         0x1F

/* SYS_CFG Register Bits */
#define PINNACLE_SYS_CFG_RESET      BIT(0)
#define PINNACLE_SYS_CFG_SHUTDOWN   BIT(1)
#define PINNACLE_SYS_CFG_SLEEP_EN   BIT(2)

/* FEED_CFG1 Register Bits */
#define PINNACLE_FEED_CFG1_EN       BIT(0)
#define PINNACLE_FEED_CFG1_ABS      BIT(1)
#define PINNACLE_FEED_CFG1_FILTER   BIT(2)
#define PINNACLE_FEED_CFG1_X_DIS    BIT(3)
#define PINNACLE_FEED_CFG1_Y_DIS    BIT(4)
#define PINNACLE_FEED_CFG1_X_INV    BIT(6)
#define PINNACLE_FEED_CFG1_Y_INV    BIT(7)

/* FEED_CFG2 Register Bits */
#define PINNACLE_FEED_CFG2_INV_TAP      BIT(0)
#define PINNACLE_FEED_CFG2_INV_SEC_TAP  BIT(1)
#define PINNACLE_FEED_CFG2_INV_SCROLL   BIT(2)
#define PINNACLE_FEED_CFG2_INV_GLIDE    BIT(3)
#define PINNACLE_FEED_CFG2_ALL_TAP_DIS  BIT(4)
#define PINNACLE_FEED_CFG2_SEC_TAP_DIS  BIT(5)
#define PINNACLE_FEED_CFG2_SCROLL_DIS   BIT(6)
#define PINNACLE_FEED_CFG2_GLIDE_DIS    BIT(7)

/* STATUS1 Register Bits */
#define PINNACLE_STATUS1_SW_DR      BIT(2)
#define PINNACLE_STATUS1_SW_CC      BIT(3)

/* CAL_CFG Register Bits */
#define PINNACLE_CAL_CFG_CAL        BIT(0)
#define PINNACLE_CAL_CFG_BG_COMP    BIT(1)
#define PINNACLE_CAL_CFG_NRD_COMP   BIT(2)
#define PINNACLE_CAL_CFG_TRACK      BIT(3)
#define PINNACLE_CAL_CFG_TAP        BIT(4)

/* ERA Control Bits */
#define PINNACLE_ERA_CTRL_READ      BIT(0)
#define PINNACLE_ERA_CTRL_WRITE     BIT(1)
#define PINNACLE_ERA_CTRL_INC_ADDR  BIT(2)

/* Extended Register Addresses for sensitivity tuning */
#define PINNACLE_ERA_REG_ADC_ATTN_X     0x0019
#define PINNACLE_ERA_REG_ADC_ATTN_Y     0x001A
#define PINNACLE_ERA_REG_ADC_WIDTH_X    0x001E
#define PINNACLE_ERA_REG_ADC_WIDTH_Y    0x001F

/* Sensitivity values */
enum pinnacle_sensitivity {
    PINNACLE_SENSITIVITY_1X = 0,
    PINNACLE_SENSITIVITY_2X,
    PINNACLE_SENSITIVITY_3X,
    PINNACLE_SENSITIVITY_4X,
};

/* Power state tracking */
enum pinnacle_power_state {
    PINNACLE_POWER_ACTIVE = 0,
    PINNACLE_POWER_IDLE,      /* Feed disabled, fast wake */
    PINNACLE_POWER_SLEEP,     /* Full sleep, slower wake */
};

/* Device configuration (from devicetree) */
struct pinnacle_config {
    union {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
        struct spi_dt_spec spi;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
        struct i2c_dt_spec i2c;
#endif
    } bus;
    int (*read)(const struct device *dev, uint8_t addr, uint8_t *val);
    int (*write)(const struct device *dev, uint8_t addr, uint8_t val);
    int (*seq_read)(const struct device *dev, uint8_t addr, uint8_t *buf, uint8_t len);
    struct gpio_dt_spec dr_gpio;
    enum pinnacle_sensitivity sensitivity;
    uint8_t x_axis_z_min;
    uint8_t y_axis_z_min;
    bool sleep_en;
    bool no_taps;
    bool no_secondary_tap;
    bool rotate_90;
    bool x_invert;
    bool y_invert;
    bool curved_overlay;
};

/* Device runtime data */
struct pinnacle_data {
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
    bool in_int;
    uint8_t btn_cache;
    enum pinnacle_power_state power_state;
#if IS_ENABLED(CONFIG_ZMK_INPUT_PINNACLE_FAST_WAKE)
    uint8_t feed_cfg1_cache;  /* Cache FEED_CFG1 for fast restore */
#endif
};

/* Public API */
int pinnacle_set_sleep(const struct device *dev, bool sleep);
int pinnacle_set_idle(const struct device *dev, bool idle);
int pinnacle_set_power_state(const struct device *dev, enum pinnacle_power_state state);
enum pinnacle_power_state pinnacle_get_power_state(const struct device *dev);
