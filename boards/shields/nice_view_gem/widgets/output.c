#include <zephyr/kernel.h>
#include <string.h>
#include "output.h"
#include "../assets/custom_fonts.h"

LV_IMG_DECLARE(bt_no_signal);
LV_IMG_DECLARE(bt_unbonded);
LV_IMG_DECLARE(bt);
LV_IMG_DECLARE(usb);

// BLE profile name configuration
static const char *ble_profile_names[] = {
    CONFIG_NICE_VIEW_BLE_PROFILE_0_NAME,
    CONFIG_NICE_VIEW_BLE_PROFILE_1_NAME,
    CONFIG_NICE_VIEW_BLE_PROFILE_2_NAME,
    CONFIG_NICE_VIEW_BLE_PROFILE_3_NAME,
    CONFIG_NICE_VIEW_BLE_PROFILE_4_NAME,
};

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static void draw_usb_connected(lv_obj_t *canvas) {
    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &quinquefive_8, LV_TEXT_ALIGN_LEFT);
    lv_canvas_draw_text(canvas, 12, 140, SCREEN_WIDTH-8, &label_dsc, "USB");
}

static void draw_ble_profile_name(lv_obj_t *canvas, int profile_index) {
    if (profile_index < 0 || profile_index >= 5) {
        return;
    }

    const char *name = ble_profile_names[profile_index];
    if (name == NULL || strlen(name) == 0) {
        return;
    }

    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &quinquefive_8, LV_TEXT_ALIGN_LEFT);
    // Position: after "BLE" text, width allows 4 chars before profile boxes (X=85)
    lv_canvas_draw_text(canvas, 42, 140, 42, &label_dsc, name);
}
#endif

static void draw_ble_disconnected(lv_obj_t *canvas) {
    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &quinquefive_8, LV_TEXT_ALIGN_LEFT);
    lv_canvas_draw_text(canvas, 12, 140, SCREEN_WIDTH-8, &label_dsc, "NULL");
}

static void draw_ble_connected(lv_obj_t *canvas) {
    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &quinquefive_8, LV_TEXT_ALIGN_LEFT);
    lv_canvas_draw_text(canvas, 12, 140, SCREEN_WIDTH-8, &label_dsc, "BLE");
}

void draw_output_status(lv_obj_t *canvas, const struct status_state *state) {
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    switch (state->selected_endpoint.transport) {
        case ZMK_TRANSPORT_USB:
            draw_usb_connected(canvas);
            break;
        case ZMK_TRANSPORT_BLE:
            draw_ble_connected(canvas);
            draw_ble_profile_name(canvas, state->active_profile_index);
            break;
        default:
            draw_ble_disconnected(canvas);
            break;
    }
#else
    draw_ble_disconnected(canvas);
#endif

    /*
    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lcd_phone, LV_TEXT_ALIGN_LEFT);
    lv_canvas_draw_text(canvas, 0, 1, 25, &label_dsc, "SIG");

    lv_draw_rect_dsc_t rect_white_dsc;
    init_rect_dsc(&rect_white_dsc, LVGL_FOREGROUND);
    lv_canvas_draw_rect(canvas, 43, 0, 24, 15, &rect_white_dsc);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    switch (state->selected_endpoint.transport) {
    case ZMK_TRANSPORT_USB:
        draw_usb_connected(canvas);
        break;

    case ZMK_TRANSPORT_BLE:
        if (state->active_profile_bonded) {
            if (state->active_profile_connected) {
                draw_ble_connected(canvas);
            } else {
                draw_ble_disconnected(canvas);
            }
        } else {
            draw_ble_unbonded(canvas);
        }
        break;
    }
#else
    if (state->connected) {
        draw_ble_connected(canvas);
    } else {
        draw_ble_disconnected(canvas);
    }
#endif
*/
}