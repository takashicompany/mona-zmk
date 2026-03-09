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

/* HID keycode lookup tables */
static const uint8_t hid_key_a = HID_USAGE_KEY_KEYBOARD_A;                    /* 0x04 */
static const uint8_t hid_key_space = HID_USAGE_KEY_KEYBOARD_SPACEBAR;         /* 0x2C */
static const uint8_t hid_key_1 = HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION;    /* 0x1E */

static uint8_t char_to_hid_keycode(char c, bool *need_shift) {
    *need_shift = false;

    if (c >= 'a' && c <= 'z') {
        return hid_key_a + (c - 'a');
    }
    if (c >= 'A' && c <= 'Z') {
        *need_shift = true;
        return hid_key_a + (c - 'A');
    }
    if (c >= '1' && c <= '9') {
        return hid_key_1 + (c - '1');
    }
    if (c == '0') {
        return HID_USAGE_KEY_KEYBOARD_0_AND_RIGHT_PARENTHESIS; /* 0x27 */
    }
    if (c == ' ') {
        return hid_key_space;
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

static void build_ble_info_string(void) {
    int pos = 0;
    int active = zmk_ble_active_profile_index();

    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT && pos < BLE_INFO_BUF_SIZE - 1; i++) {
        if (i > 0 && pos < BLE_INFO_BUF_SIZE - 1) {
            output_buf[pos++] = ' ';
        }

        /* Profile number (1-based) */
        if (pos < BLE_INFO_BUF_SIZE - 1) {
            output_buf[pos++] = '1' + i;
        }

        /* Active marker */
        if (i == active && pos < BLE_INFO_BUF_SIZE - 1) {
            output_buf[pos++] = 'a';
        }

        /* Space before status */
        if (pos < BLE_INFO_BUF_SIZE - 1) {
            output_buf[pos++] = ' ';
        }

        /* Connection status: C=Connected, P=Paired, O=Open */
        if (zmk_ble_profile_is_open(i)) {
            if (pos < BLE_INFO_BUF_SIZE - 1) {
                output_buf[pos++] = 'O';
            }
        } else if (zmk_ble_profile_is_connected(i)) {
            if (pos < BLE_INFO_BUF_SIZE - 1) {
                output_buf[pos++] = 'C';
            }

            /* Show device name for connected active profile */
            if (i == active) {
                char *name = zmk_ble_active_profile_name();
                if (name && name[0] != '\0') {
                    /* Save position to rollback if no printable chars */
                    int name_start = pos;
                    if (pos < BLE_INFO_BUF_SIZE - 1) {
                        output_buf[pos++] = ' ';
                    }
                    int chars_written = 0;
                    for (int j = 0; name[j] != '\0' && pos < BLE_INFO_BUF_SIZE - 1; j++) {
                        char ch = name[j];
                        /* Only output alphanumeric and space (JIS-safe) */
                        if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                            (ch >= '0' && ch <= '9') || ch == ' ') {
                            output_buf[pos++] = ch;
                            chars_written++;
                        }
                    }
                    if (chars_written == 0) {
                        /* No printable chars, rollback the space */
                        pos = name_start;
                    }
                }
            }
        } else {
            /* Paired but not connected */
            if (pos < BLE_INFO_BUF_SIZE - 1) {
                output_buf[pos++] = 'P';
            }
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
