/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_ble_info

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

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

#define BLE_INFO_BUF_SIZE 256
#define BLE_INFO_KEY_DELAY_MS 20
#define DEVICE_NAME_MAX_LEN 32
#define DEVICE_NAME_READ_DELAY_MS 1000

/* ========== Device name storage ========== */

static char device_names[ZMK_BLE_PROFILE_COUNT][DEVICE_NAME_MAX_LEN];

/* ========== GATT device name read ========== */

static struct bt_conn *pending_name_conn;
static struct bt_gatt_read_params name_read_params;
static bool name_read_in_progress;

static void name_read_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(name_read_work, name_read_work_handler);

static uint8_t device_name_read_cb(struct bt_conn *conn, uint8_t err,
                                   struct bt_gatt_read_params *params,
                                   const void *data, uint16_t length) {
    if (err || !data || length == 0) {
        LOG_DBG("Device name read done (err=%d)", err);
        name_read_in_progress = false;
        if (pending_name_conn) {
            bt_conn_unref(pending_name_conn);
            pending_name_conn = NULL;
        }
        return BT_GATT_ITER_STOP;
    }

    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    int idx = zmk_ble_profile_index(addr);

    if (idx >= 0 && idx < ZMK_BLE_PROFILE_COUNT) {
        int copy_len = MIN(length, DEVICE_NAME_MAX_LEN - 1);
        memcpy(device_names[idx], data, copy_len);
        device_names[idx][copy_len] = '\0';
        LOG_INF("Profile %d device name: %s", idx, device_names[idx]);
    }

    name_read_in_progress = false;
    if (pending_name_conn) {
        bt_conn_unref(pending_name_conn);
        pending_name_conn = NULL;
    }

    return BT_GATT_ITER_STOP;
}

static void name_read_work_handler(struct k_work *work) {
    if (!pending_name_conn || name_read_in_progress) {
        return;
    }

    memset(&name_read_params, 0, sizeof(name_read_params));
    name_read_params.func = device_name_read_cb;
    name_read_params.handle_count = 0;
    name_read_params.by_uuid.uuid = BT_UUID_GAP_DEVICE_NAME;
    name_read_params.by_uuid.start_handle = 0x0001;
    name_read_params.by_uuid.end_handle = 0xffff;

    name_read_in_progress = true;
    int err = bt_gatt_read(pending_name_conn, &name_read_params);
    if (err) {
        LOG_WRN("Failed to read device name (err %d)", err);
        name_read_in_progress = false;
        bt_conn_unref(pending_name_conn);
        pending_name_conn = NULL;
    }
}

static void ble_info_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        return;
    }

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info)) {
        return;
    }

    /* Only read device name for host connections (keyboard is peripheral) */
    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        return;
    }

    k_work_cancel_delayable(&name_read_work);

    if (pending_name_conn) {
        bt_conn_unref(pending_name_conn);
    }
    pending_name_conn = bt_conn_ref(conn);

    /* Delay to allow encryption/bonding to complete */
    k_work_schedule(&name_read_work, K_MSEC(DEVICE_NAME_READ_DELAY_MS));
}

static void ble_info_disconnected(struct bt_conn *conn, uint8_t reason) {
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    int idx = zmk_ble_profile_index(addr);

    if (idx >= 0 && idx < ZMK_BLE_PROFILE_COUNT) {
        device_names[idx][0] = '\0';
    }

    if (pending_name_conn == conn) {
        k_work_cancel_delayable(&name_read_work);
        bt_conn_unref(pending_name_conn);
        pending_name_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(ble_info_conn_cb) = {
    .connected = ble_info_connected,
    .disconnected = ble_info_disconnected,
};

/* ========== HID keycode mapping ========== */

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

/* ========== HID keystroke output ========== */

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
            k_work_schedule(&ble_info_work, K_MSEC(BLE_INFO_KEY_DELAY_MS));
        } else {
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

    k_work_schedule(&ble_info_work, K_MSEC(BLE_INFO_KEY_DELAY_MS));
}

/* ========== Output string builder ========== */

static int buf_append_str(char *buf, int pos, int max, const char *str) {
    for (int i = 0; str[i] != '\0' && pos < max; i++) {
        buf[pos++] = str[i];
    }
    return pos;
}

static int buf_append_filtered_str(char *buf, int pos, int max, const char *str) {
    for (int i = 0; str[i] != '\0' && pos < max; i++) {
        char ch = str[i];
        if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') || ch == ' ') {
            buf[pos++] = ch;
        }
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

        /* Connection status and device name */
        if (zmk_ble_profile_is_open(i)) {
            pos = buf_append_str(output_buf, pos, max, "Open");
        } else if (zmk_ble_profile_is_connected(i)) {
            pos = buf_append_str(output_buf, pos, max, "Connected");
            /* Append device name if available */
            if (device_names[i][0] != '\0') {
                if (pos < max) {
                    output_buf[pos++] = ' ';
                }
                int name_start = pos;
                pos = buf_append_filtered_str(output_buf, pos, max, device_names[i]);
                if (pos == name_start) {
                    /* No printable chars, rollback space */
                    pos = name_start - 1;
                }
            }
        } else {
            pos = buf_append_str(output_buf, pos, max, "Paired");
        }
    }

    output_buf[pos] = '\0';
    output_len = pos;
}

/* ========== Behavior callbacks ========== */

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
