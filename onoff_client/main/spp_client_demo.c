/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This file is for ble spp client demo.
 *
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "driver/uart.h"

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "cJSON.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#define TAG "WIFI_DEMO"
#define GATTC_TAG "GATTC_SPP_DEMO"
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0
#define BT_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#define ESP_GATT_SPP_SERVICE_UUID 0xABF0
#define SCAN_ALL_THE_TIME 0

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

enum
{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

////wifi part
#define EXAMPLE_ESP_WIFI_SSID "Galaxy A53 5G CA17"
#define EXAMPLE_ESP_WIFI_PASS "mperko12"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;

typedef enum
{
    EVENT_ON,
    EVENT_OFF,
    READ_LED,
    EVENT_UNKNOWN,
    ADD_DEVICE,
    SCAN
} EventType;

EventType get_event_type(const char *event)
{
    if (strcmp(event, "on") == 0)
    {
        return EVENT_ON;
    }
    else if (strcmp(event, "off") == 0)
    {
        return EVENT_OFF;
    }
    else if (strcmp(event, "scan") == 0)
    {
        return SCAN;
    }
    else if (strcmp(event, "get") == 0)
    {
        return READ_LED;
    }
    else if (strcmp(event, "add") == 0)
    {
        return ADD_DEVICE;
    }
    else
    {
        return EVENT_UNKNOWN;
    }
}

static EventGroupHandle_t s_wifi_event_group;

/* Constants for IP address and port */
#define REMOTE_IP_ADDR "192.168.199.231"
#define REMOTE_PORT 5002

/// Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

static const char device_name[] = "ESP_SPP_SERVER";
static bool is_connect = false;
static uint16_t spp_conn_id = 0;
static uint16_t spp_mtu_size = 23;
static uint16_t cmd = 0;
static uint16_t spp_srv_start_handle = 0;
static uint16_t spp_srv_end_handle = 0;
static uint16_t spp_gattc_if = 0xff;
static char *notify_value_p = NULL;
static int notify_value_offset = 0;
static int notify_value_count = 0;
static uint16_t count = SPP_IDX_NB;
static esp_gattc_db_elem_t *db = NULL;
static esp_ble_gap_cb_param_t scan_rst;
static QueueHandle_t cmd_reg_queue = NULL;
QueueHandle_t spp_uart_queue = NULL;
uint8_t socket_fd;

#ifdef SUPPORT_HEARTBEAT
static uint8_t heartbeat_s[9] = {'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
static QueueHandle_t cmd_heartbeat_queue = NULL;
#endif

static esp_bt_uuid_t spp_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_SPP_SERVICE_UUID,
    },
};

static void notify_event_handler(esp_ble_gattc_cb_param_t *p_data)
{
    uint8_t handle = 0;

    if (p_data->notify.is_notify == true)
    {
        ESP_LOGI(GATTC_TAG, "+NOTIFY:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
    }
    else
    {
        ESP_LOGI(GATTC_TAG, "+INDICATE:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
    }
    handle = p_data->notify.handle;
    if (db == NULL)
    {
        ESP_LOGE(GATTC_TAG, " %s db is NULL", __func__);
        return;
    }
    if (handle == db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle)
    {
#ifdef SPP_DEBUG_MODE
        ESP_LOG_BUFFER_CHAR(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
#else
        if ((p_data->notify.value[0] == '#') && (p_data->notify.value[1] == '#'))
        {
            if ((++notify_value_count) != p_data->notify.value[3])
            {
                if (notify_value_p != NULL)
                {
                    free(notify_value_p);
                }
                notify_value_count = 0;
                notify_value_p = NULL;
                notify_value_offset = 0;
                ESP_LOGE(GATTC_TAG, "notify value count is not continuous,%s", __func__);
                return;
            }
            if (p_data->notify.value[3] == 1)
            {
                notify_value_p = (char *)malloc(((spp_mtu_size - 7) * (p_data->notify.value[2])) * sizeof(char));
                if (notify_value_p == NULL)
                {
                    ESP_LOGE(GATTC_TAG, "malloc failed,%s L#%d", __func__, __LINE__);
                    notify_value_count = 0;
                    return;
                }
                memcpy((notify_value_p + notify_value_offset), (p_data->notify.value + 4), (p_data->notify.value_len - 4));
                if (p_data->notify.value[2] == p_data->notify.value[3])
                {
                    uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    free(notify_value_p);
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
            }
            else if (p_data->notify.value[3] <= p_data->notify.value[2])
            {
                memcpy((notify_value_p + notify_value_offset), (p_data->notify.value + 4), (p_data->notify.value_len - 4));
                if (p_data->notify.value[3] == p_data->notify.value[2])
                {
                    uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    free(notify_value_p);
                    notify_value_count = 0;
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
            }
        }
        else
        {
            uart_write_bytes(UART_NUM_0, (char *)(p_data->notify.value), p_data->notify.value_len);
        }
#endif
    }
    else if (handle == ((db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle))
    {
        ESP_LOG_BUFFER_CHAR(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
        // TODO:server notify status characteristic
    }
    else
    {
        ESP_LOG_BUFFER_CHAR(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
    }
}

static void free_gattc_srv_db(void)
{
    is_connect = false;
    spp_gattc_if = 0xff;
    spp_conn_id = 0;
    spp_mtu_size = 23;
    cmd = 0;
    spp_srv_start_handle = 0;
    spp_srv_end_handle = 0;
    notify_value_p = NULL;
    notify_value_offset = 0;
    notify_value_count = 0;
    if (db)
    {
        free(db);
        db = NULL;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    esp_err_t err;

    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        if ((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan param set failed: %s", esp_err_to_name(err));
            break;
        }
        // the unit of the duration is second
        uint32_t duration = 0xFFFF;
        ESP_LOGI(GATTC_TAG, "Enable Ble Scan:during time %04" PRIx32 " minutes.", duration);
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan start failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scan start successfully");
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan stop failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scan stop successfully");
        if (is_connect == false)
        {
            ESP_LOGI(GATTC_TAG, "Connect to the remote device.");
            esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, scan_rst.scan_rst.bda, scan_rst.scan_rst.ble_addr_type, true);
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            ESP_LOG_BUFFER_HEX(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "Searched Device Name Len %d", adv_name_len);
            ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
            ESP_LOGI(GATTC_TAG, " ");
            if (adv_name != NULL)
            {
                if (strncmp((char *)adv_name, device_name, adv_name_len) == 0)
                {
                    memcpy(&(scan_rst), scan_result, sizeof(esp_ble_gap_cb_param_t));
                    esp_ble_gap_stop_scanning();
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Stop adv successfully");
        }
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG EVT, set scan params");
        // esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT: conn_id=%d, gatt_if = %d", spp_conn_id, gattc_if);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        ESP_LOG_BUFFER_HEX(GATTC_TAG, gl_profile_tab[PROFILE_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        spp_gattc_if = gattc_if;
        is_connect = true;
        spp_conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_search_service(spp_gattc_if, spp_conn_id, &spp_service_uuid);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "disconnect");
        free_gattc_srv_db();
        esp_ble_gap_start_scanning(SCAN_ALL_THE_TIME);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_RES_EVT: start_handle = %d, end_handle = %d, UUID:0x%04x", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.uuid.uuid.uuid16);
        spp_srv_start_handle = p_data->search_res.start_handle;
        spp_srv_end_handle = p_data->search_res.end_handle;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "SEARCH_CMPL: conn_id = %x, status %d", spp_conn_id, p_data->search_cmpl.status);
        esp_ble_gattc_send_mtu_req(gattc_if, spp_conn_id);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
        ESP_LOGI(GATTC_TAG, "Index = %d,status = %d,handle = %d", cmd, p_data->reg_for_notify.status, p_data->reg_for_notify.handle);
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT, status = %d", p_data->reg_for_notify.status);
            break;
        }
        uint16_t notify_en = 1;
        esp_ble_gattc_write_char_descr(
            spp_gattc_if,
            spp_conn_id,
            (db + cmd + 1)->attribute_handle,
            sizeof(notify_en),
            (uint8_t *)&notify_en,
            ESP_GATT_WRITE_TYPE_RSP,
            ESP_GATT_AUTH_REQ_NONE);

        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT");
        notify_event_handler(p_data);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        if (param->read.status == ESP_GATT_OK)
        {
            uint8_t *value = param->read.value;
            uint16_t value_len = param->read.value_len;
            ESP_LOGI("BLE", "Read response received, value length: %d", value_len);
            if (value_len > 0)
            {
                uint8_t led_state = value[0]; // Assuming the LED state is a single byte
                ESP_LOGI("BLE", "LED State: %d", led_state);
            }
        }
        else
        {
            ESP_LOGE("BLE", "Read characteristic failed, error code: %d", param->read.status);
        }
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_WRITE_CHAR_EVT:status = %d,handle = %d", param->write.status, param->write.handle);
        if (param->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_CHAR_EVT, error status = %d", p_data->write.status);
            break;
        }
        break;
    case ESP_GATTC_PREP_WRITE_EVT:
        break;
    case ESP_GATTC_EXEC_EVT:
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_WRITE_DESCR_EVT: status =%d,handle = %d", p_data->write.status, p_data->write.handle);
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_DESCR_EVT, error status = %d", p_data->write.status);
            break;
        }
        switch (cmd)
        {
        case SPP_IDX_SPP_DATA_NTY_VAL:
            cmd = SPP_IDX_SPP_STATUS_VAL;
            xQueueSend(cmd_reg_queue, &cmd, 10 / portTICK_PERIOD_MS);
            break;
        case SPP_IDX_SPP_STATUS_VAL:
#ifdef SUPPORT_HEARTBEAT
            cmd = SPP_IDX_SPP_HEARTBEAT_VAL;
            xQueueSend(cmd_reg_queue, &cmd, 10 / portTICK_PERIOD_MS);
#endif
            break;
#ifdef SUPPORT_HEARTBEAT
        case SPP_IDX_SPP_HEARTBEAT_VAL:
            xQueueSend(cmd_heartbeat_queue, &cmd, 10 / portTICK_PERIOD_MS);
            break;
#endif
        default:
            break;
        };
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (p_data->cfg_mtu.status != ESP_OK)
        {
            break;
        }
        ESP_LOGI(GATTC_TAG, "+MTU:%d", p_data->cfg_mtu.mtu);
        spp_mtu_size = p_data->cfg_mtu.mtu;

        db = (esp_gattc_db_elem_t *)malloc(count * sizeof(esp_gattc_db_elem_t));
        if (db == NULL)
        {
            ESP_LOGE(GATTC_TAG, "%s:malloc db failed", __func__);
            break;
        }
        if (esp_ble_gattc_get_db(spp_gattc_if, spp_conn_id, spp_srv_start_handle, spp_srv_end_handle, db, &count) != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "%s:get db failed", __func__);
            break;
        }
        if (count != SPP_IDX_NB)
        {
            ESP_LOGE(GATTC_TAG, "%s:get db count != SPP_IDX_NB, count = %d, SPP_IDX_NB = %d", __func__, count, SPP_IDX_NB);
            break;
        }
        for (int i = 0; i < SPP_IDX_NB; i++)
        {
            switch ((db + i)->type)
            {
            case ESP_GATT_DB_PRIMARY_SERVICE:
                ESP_LOGI(GATTC_TAG, "attr_type = PRIMARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_SECONDARY_SERVICE:
                ESP_LOGI(GATTC_TAG, "attr_type = SECONDARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_CHARACTERISTIC:
                ESP_LOGI(GATTC_TAG, "attr_type = CHARACTERISTIC,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_DESCRIPTOR:
                ESP_LOGI(GATTC_TAG, "attr_type = DESCRIPTOR,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_INCLUDED_SERVICE:
                ESP_LOGI(GATTC_TAG, "attr_type = INCLUDED_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_ALL:
                ESP_LOGI(GATTC_TAG, "attr_type = ESP_GATT_DB_ALL,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            default:
                break;
            }
        }
        cmd = SPP_IDX_SPP_DATA_NTY_VAL;
        xQueueSend(cmd_reg_queue, &cmd, 10 / portTICK_PERIOD_MS);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
        break;
    default:
        break;
    }
}

void spp_client_reg_task(void *arg)
{
    uint16_t cmd_id;
    for (;;)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_reg_queue, &cmd_id, portMAX_DELAY))
        {
            if (db != NULL)
            {
                if (cmd_id == SPP_IDX_SPP_DATA_NTY_VAL)
                {
                    ESP_LOGI(GATTC_TAG, "Index = %d,UUID = 0x%04x, handle = %d", cmd_id, (db + SPP_IDX_SPP_DATA_NTY_VAL)->uuid.uuid.uuid16, (db + SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db + SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                }
                else if (cmd_id == SPP_IDX_SPP_STATUS_VAL)
                {
                    ESP_LOGI(GATTC_TAG, "Index = %d,UUID = 0x%04x, handle = %d", cmd_id, (db + SPP_IDX_SPP_STATUS_VAL)->uuid.uuid.uuid16, (db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                }
#ifdef SUPPORT_HEARTBEAT
                else if (cmd_id == SPP_IDX_SPP_HEARTBEAT_VAL)
                {
                    ESP_LOGI(GATTC_TAG, "Index = %d,UUID = 0x%04x, handle = %d", cmd_id, (db + SPP_IDX_SPP_HEARTBEAT_VAL)->uuid.uuid.uuid16, (db + SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db + SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle);
                }
#endif
            }
        }
    }
}

#ifdef SUPPORT_HEARTBEAT
void spp_heart_beat_task(void *arg)
{
    uint16_t cmd_id;

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY))
        {
            while (1)
            {
                if ((is_connect == true) && (db != NULL) && ((db + SPP_IDX_SPP_HEARTBEAT_VAL)->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE)))
                {
                    esp_ble_gattc_write_char(spp_gattc_if,
                                             spp_conn_id,
                                             (db + SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle,
                                             sizeof(heartbeat_s),
                                             (uint8_t *)heartbeat_s,
                                             ESP_GATT_WRITE_TYPE_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                }
                else
                {
                    ESP_LOGI(GATTC_TAG, "disconnect");
                    break;
                }
            }
        }
    }
}
#endif

void ble_client_appRegister(void)
{
    esp_err_t status;
    char err_msg[20];

    ESP_LOGI(GATTC_TAG, "register callback");

    // register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG, "gap register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    // register the callback function to the gattc module
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG, "gattc register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    esp_ble_gattc_app_register(PROFILE_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(200);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTC_TAG, "set local  MTU failed: %s", esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
    }

    cmd_reg_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_client_reg_task, "spp_client_reg_task", 2048, NULL, 10, NULL);

#ifdef SUPPORT_HEARTBEAT
    cmd_heartbeat_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_heart_beat_task, "spp_heart_beat_task", 2048, NULL, 10, NULL);
#endif
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            // Event of UART receiving data
            case UART_DATA:
                if (event.size && (is_connect == true) && (db != NULL) && ((db + SPP_IDX_SPP_DATA_RECV_VAL)->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE)))
                {
                    uint8_t *temp = NULL;
                    temp = (uint8_t *)malloc(sizeof(uint8_t) * event.size);
                    if (temp == NULL)
                    {
                        ESP_LOGE(GATTC_TAG, "malloc failed,%s L#%d", __func__, __LINE__);
                        break;
                    }
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0, temp, event.size, portMAX_DELAY);
                    esp_ble_gattc_write_char(spp_gattc_if,
                                             spp_conn_id,
                                             (db + SPP_IDX_SPP_DATA_RECV_VAL)->attribute_handle,
                                             event.size,
                                             temp,
                                             ESP_GATT_WRITE_TYPE_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);
    // Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    // Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void *)UART_NUM_0, 8, NULL);
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            // .sae_pwe_h2e and .sae_h2e_identifier are optional depending on your configuration
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* Function to connect to a remote server */
/* Function to connect to a remote server */
int connect_to_server(const char *ip, int port)
{
    struct sockaddr_in dest_addr;
    int socket_fd;

    socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (socket_fd == -1)
    {
        ESP_LOGE(TAG, "Failed to create socket");
        return -1;
    }

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &dest_addr.sin_addr);

    if (connect(socket_fd, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
    {
        ESP_LOGE(TAG, "Socket connect failed");
        close(socket_fd);
        return -1;
    }

    ESP_LOGI(TAG, "Successfully connected to server");
    return socket_fd;
}

/* Function to send a JSON message over a connected socket */
void send_json_init_message(int socket_fd, const char *type, const char *content)
{
    // Create a JSON object
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to create cJSON object");
        return;
    }

    // Add data to the JSON object
    cJSON_AddStringToObject(root, "event", type);
    cJSON_AddStringToObject(root, "serial", content);

    // Convert JSON object to string
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON object to string");
        cJSON_Delete(root);
        return;
    }

    // Send JSON string to the server
    if (send(socket_fd, json_string, strlen(json_string), 0) < 0)
    {
        ESP_LOGE(TAG, "Failed to send JSON message");
    }
    else
    {
        ESP_LOGI(TAG, "JSON message sent successfully: %s", json_string);
    }

    // Free JSON object and string
    cJSON_Delete(root);
    free(json_string);
}

void send_value_report(int socket_fd, const char *type, const char *content)
{
    // Create a JSON object
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to create cJSON object");
        return;
    }

    // Add data to the JSON object
    cJSON_AddStringToObject(root, "event", type);
    cJSON_AddStringToObject(root, "serial", content);

    // Convert JSON object to string
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON object to string");
        cJSON_Delete(root);
        return;
    }

    // Send JSON string to the server
    if (send(socket_fd, json_string, strlen(json_string), 0) < 0)
    {
        ESP_LOGE(TAG, "Failed to send JSON message");
    }
    else
    {
        ESP_LOGI(TAG, "JSON message sent successfully: %s", json_string);
    }

    // Free JSON object and string
    cJSON_Delete(root);
    free(json_string);
}


void send_add_device_report(int socket_fd, const char *event, const char *uuid)
{
    // Create a JSON object for the entire message
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        fprintf(stderr, "Failed to create cJSON object\n");
        return;
    }

    // Add event type to the JSON object
    cJSON_AddStringToObject(root, "event", event);

    // Create a JSON object for the devices section
    cJSON *devices = cJSON_CreateObject();
    if (devices == NULL)
    {
        cJSON_Delete(root);
        fprintf(stderr, "Failed to create cJSON object for devices\n");
        return;
    }

    // Create a JSON object for the specific device
    cJSON *device = cJSON_CreateObject();
    if (device == NULL)
    {
        cJSON_Delete(root);
        cJSON_Delete(devices);
        fprintf(stderr, "Failed to create cJSON object for device\n");
        return;
    }

    // Add type and cluster to the device object
    cJSON_AddStringToObject(device, "type", "switch");
    cJSON_AddStringToObject(device, "cluster", "cluster-name");

    // Add the device object to the devices object using the uuid as the key
    cJSON_AddItemToObject(devices, uuid, device);

    // Add the devices object to the root object
    cJSON_AddItemToObject(root, "devices", devices);

    // Convert JSON object to string
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL)
    {
        cJSON_Delete(root);
        cJSON_Delete(devices);
        cJSON_Delete(device);
        fprintf(stderr, "Failed to print cJSON object to string\n");
        return;
    }

    // Send JSON string to the server
    if (write(socket_fd, json_string, strlen(json_string)) < 0)
    {
        fprintf(stderr, "Failed to send JSON message\n");
    }
    else
    {
        printf("JSON message sent successfully: %s\n", json_string);
    }

    // Free JSON object and string
    cJSON_Delete(root);
    free(json_string);
}

void send_led_cmd(uint8_t led_value)
{
    if (!is_connect || db == NULL)
    {
        ESP_LOGE("BLE", "Not connected or DB is NULL");
        return;
    }

    printf("sakhen");
    esp_err_t err = esp_ble_gattc_write_char(
        spp_gattc_if,
        spp_conn_id,
        47,
        sizeof(led_value),
        &led_value,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE);

    if (err != ESP_OK)
    {
        ESP_LOGE("BLE", "Failed to write characteristic: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI("BLE", "Successfully wrote LED OFF command");
    }
}

void read_led_state(void)
{
    if (!is_connect || db == NULL)
    {
        ESP_LOGE("BLE", "Not connected or DB is NULL");
        return;
    }

    uint16_t char_handle_to_read = 47; // Replace with your characteristic handle for reading
    esp_err_t err = esp_ble_gattc_read_char(
        spp_gattc_if,
        spp_conn_id,
        char_handle_to_read,
        ESP_GATT_AUTH_REQ_NONE);

    if (err != ESP_OK)
    {
        ESP_LOGE("BLE", "Failed to read characteristic: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI("BLE", "Successfully initiated read characteristic");
    }
}

void handle_on_event(const char *cluster)
{
    switch (atoi(cluster))
    {
    case 11111:
        ESP_LOGI(TAG, "Received 'on' event for cluster '11111'");
        send_led_cmd(1);
        break;
    default:
        ESP_LOGI(TAG, "Received 'on' event for unrecognized cluster '%s'", cluster);
        break;
    }
}

void handle_off_event(const char *cluster)
{
    switch (atoi(cluster))
    {
    case 11111:
        ESP_LOGI(TAG, "Received 'off' event for cluster '11111'");
        send_led_cmd(0);
        break;
    default:
        ESP_LOGI(TAG, "Received 'off' event for unrecognized cluster '%s'", cluster);
        break;
    }
}

void receive_json_messages(void *param)
{
    int socket_fd = (int)param;
    char buffer[1024];
    int len;

    while (1)
    {
        memset(buffer, 0, sizeof(buffer));
        len = recv(socket_fd, buffer, sizeof(buffer) - 1, 0);

        if (len < 0)
        {
            ESP_LOGE(TAG, "Failed to receive message");
            break;
        }
        else if (len == 0)
        {
            ESP_LOGI(TAG, "Connection closed by server");
            break;
        }

        ESP_LOGI(TAG, "Received message: %s", buffer);

        // Parse the JSON message
        cJSON *json = cJSON_Parse(buffer);
        if (json == NULL)
        {
            ESP_LOGE(TAG, "Failed to parse JSON message");
            continue;
        }

        // Handle events based on the event type
        cJSON *event = cJSON_GetObjectItem(json, "event");
        cJSON *cluster = cJSON_GetObjectItem(json, "cluster");
        cJSON *data = cJSON_GetObjectItem(json, "data");

        if (cJSON_IsString(event) && cJSON_IsString(cluster))
        {
            EventType event_type = get_event_type(event->valuestring);
            switch (event_type)
            {
            case EVENT_ON:
                handle_on_event(cluster->valuestring);
                break;
            case EVENT_OFF:
                handle_off_event(cluster->valuestring);
                break;
            case READ_LED:
                read_led_state();
                break;
            default:
                ESP_LOGI(TAG, "Received unrecognized event: %s", event->valuestring);
                break;
            }
        }
        else if (cJSON_IsString(event))
        {
            EventType event_type = get_event_type(event->valuestring);
            switch (event_type)
            {
            case SCAN:
                esp_ble_gap_set_scan_params(&ble_scan_params);
                break;
            case ADD_DEVICE:
                send_add_device_report(socket_fd, "addDevice", "111");
                break;
            default:
                ESP_LOGI(TAG, "Received unrecognized event: %s", event->valuestring);
                break;
            }
        }
        else
        {
            ESP_LOGE(TAG, "Invalid JSON format or missing required fields");
        }

        cJSON_Delete(json);
    }

    // Close the socket when done
    close(socket_fd);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTC_TAG, "%s init bluetooth", __func__);

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ble_client_appRegister();
    spp_uart_init();

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    wifi_init_sta();

    socket_fd = connect_to_server(REMOTE_IP_ADDR, REMOTE_PORT);
    if (socket_fd != -1)
    {
        // Send a JSON message
        send_json_init_message(socket_fd, "init", "213124");
        // Close the socket after sending the message
        xTaskCreate(&receive_json_messages, "receive_json_messages", 4096, (void *)socket_fd, 5, NULL);
    }
}
