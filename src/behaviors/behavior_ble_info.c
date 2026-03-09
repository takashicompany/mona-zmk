/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_ble_info

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/ble.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

#include <dt-bindings/zmk/hid_usage.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/modifiers.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define BLE_INFO_BUF_SIZE 128
#define BLE_INFO_KEY_DELAY_MS 20

static uint8_t char_to_hid_keycode(char c, bool *need_shift) {
    *need_shift = false;

    if (c >= 'a' && c <= 'z') {
        return HID_USAGE_KEY_KEYBOARD_A + (c - 'a');
    }
    if (c >= 'A' && c <= 'Z') {
        *need_shift = true;
        return HID_USAGE_KEY_KEYBOARD_A + (c - 'A');
    }
    if (c >= '1' && c <= '9') {
        return HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION + (c - '1');
    }
    if (c == '0') {
        return HID_USAGE_KEY_KEYBOARD_0_AND_RIGHT_PARENTHESIS;
    }
    if (c == ' ') {
        return HID_USAGE_KEY_KEYBOARD_SPACEBAR;
    }
    /* JIS-specific mappings */
    if (c == ':') {
        return HID_USAGE_KEY_KEYBOARD_APOSTROPHE_AND_QUOTE; /* JIS: colon */
    }
    if (c == '[') {
        return HID_USAGE_KEY_KEYBOARD_RIGHT_BRACKET_AND_RIGHT_BRACE; /* JIS: [ */
    }
    if (c == ']') {
        return HID_USAGE_KEY_KEYBOARD_NON_US_HASH_AND_TILDE; /* JIS: ] */
    }

    return 0; /* unsupported character */
}

/* Work item state */
static char output_buf[BLE_INFO_BUF_SIZE];
static int output_len;
static int output_pos;
static bool output_busy;
static bool key_pressed;

static void ble_info_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ble_info_work, ble_info_work_handler);

static void ble_info_work_handler(struct k_work *work) {
    if (key_pressed) {
        /* Release the previous key */
        bool need_shift;
        char c = output_buf[output_pos - 1];
        uint8_t keycode = char_to_hid_keycode(c, &need_shift);

        if (keycode) {
            zmk_hid_keyboard_release(keycode);
            if (need_shift) {
                zmk_hid_unregister_mods(MOD_LSFT);
            }
            zmk_endpoints_send_report(HID_USAGE_KEY);
        }

        key_pressed = false;

        if (output_pos < output_len) {
            /* Schedule next key press after release delay */
            k_work_schedule(&ble_info_work, K_MSEC(BLE_INFO_KEY_DELAY_MS));
        } else {
            /* All done */
            output_busy = false;
            LOG_DBG("BLE info output complete");
        }
        return;
    }

    if (output_pos >= output_len) {
        output_busy = false;
        return;
    }

    /* Press the next key */
    char c = output_buf[output_pos];
    bool need_shift;
    uint8_t keycode = char_to_hid_keycode(c, &need_shift);

    if (keycode) {
        if (need_shift) {
            zmk_hid_register_mods(MOD_LSFT);
        }
        zmk_hid_keyboard_press(keycode);
        zmk_endpoints_send_report(HID_USAGE_KEY);
        key_pressed = true;
    }

    output_pos++;

    /* Schedule release after press delay */
    k_work_schedule(&ble_info_work, K_MSEC(BLE_INFO_KEY_DELAY_MS));
}

static int buf_append_str(char *buf, int pos, int max, const char *str) {
    for (int i = 0; str[i] != '\0' && pos < max; i++) {
        buf[pos++] = str[i];
    }
    return pos;
}

static void build_ble_info_string(void) {
    int pos = 0;
    int active = zmk_ble_active_profile_index();
    int max = BLE_INFO_BUF_SIZE - 1;

    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT && pos < max; i++) {
        if (i > 0) {
            output_buf[pos++] = ' ';
        }

        /* Active profile: [N], others: N */
        if (i == active) {
            output_buf[pos++] = '[';
        }
        if (pos < max) {
            output_buf[pos++] = '1' + i;
        }
        if (i == active && pos < max) {
            output_buf[pos++] = ']';
        }

        /* Colon separator */
        if (pos < max) {
            output_buf[pos++] = ':';
        }

        /* Connection status */
        if (zmk_ble_profile_is_open(i)) {
            pos = buf_append_str(output_buf, pos, max, "Open");
        } else if (zmk_ble_profile_is_connected(i)) {
            pos = buf_append_str(output_buf, pos, max, "Connected");
        } else {
            pos = buf_append_str(output_buf, pos, max, "Paired");
        }
    }

    output_buf[pos] = '\0';
    output_len = pos;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    if (output_busy) {
        LOG_WRN("BLE info output already in progress");
        return ZMK_BEHAVIOR_OPAQUE;
    }

    build_ble_info_string();
    LOG_DBG("BLE info: %s", output_buf);

    if (output_len == 0) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    output_pos = 0;
    key_pressed = false;
    output_busy = true;

    /* Start output immediately */
    k_work_schedule(&ble_info_work, K_NO_WAIT);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_ble_info_driver_api = {
    .locality = BEHAVIOR_LOCALITY_CENTRAL,
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_ble_info_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
