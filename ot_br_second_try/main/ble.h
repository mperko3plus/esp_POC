/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_bt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_system.h"

#define GATTC_TAG "GATTC_DEMO"

// #define REMOTE_SERVICE_UUID_128 { 0xFB, 0x34, 0x9B, 0x5F, 0x00, 0x80, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xF6, 0xFF, 0x00, 0x00 }
// #define REMOTE_WRITE_CHAR_UUID_128 { 0xFB, 0xB3, 0x9F, 0x05, 0x80, 0x00, 0x00, 0x80, 0x00, 0x30, 0x00, 0x03, 0xF5, 0x2E, 0xEE, 0x18 }
// #define REMOTE_READ_CHAR_UUID_128 { 0xFB, 0xB3, 0x9F, 0x05, 0x80, 0x00, 0x00, 0x80, 0x00, 0x31, 0x00, 0x03, 0xF5, 0x2E, 0xEE, 0x18 }

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE 0
#define QUEUE_SIZE 10 

typedef struct
{
    uint8_t value[256]; // Adjust size based on your data requirements
    uint16_t length;
} BLEResponse;

struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

void ble_init();
void start_ble_scan(uint32_t duration);
void read_from_char();
void write_to_char(uint8_t *data, uint16_t length);
void enable_notifications(esp_bd_addr_t server_bda);

#endif // BLE_H
