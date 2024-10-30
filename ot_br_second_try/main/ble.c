#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "ble.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "network_utils.h"

static const char remote_device_name[] = "SiLabs-Light-2";
static bool connect_device = false;
static bool get_server = false;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;
esp_gatt_if_t gattc_if_global;
uint16_t conn_id;
static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
    },
};
static uint8_t handshakeFrame[] = {0x65, 0x6c, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06};

QueueHandle_t bleResponseQueue;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};
void init_ble_queue()
{
    bleResponseQueue = xQueueCreate(QUEUE_SIZE, sizeof(BLEResponse));
    if (bleResponseQueue == NULL)
    {
        ESP_LOGE(GATTC_TAG, "Failed to create BLE response queue");
    }
}

void process_ble_responses(void *param)
{
    BLEResponse response;

    while (1)
    {
        // Wait for a response to become available
        if (xQueueReceive(bleResponseQueue, &response, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(GATTC_TAG, "Processing BLE response of length: %d", response.length);

            char *hex_string = malloc(response.length * 2 + 1);
            if (hex_string == NULL)
            {
                ESP_LOGE(GATTC_TAG, "Failed to allocate memory for hex string");
                continue;
            }

            for (int i = 0; i < response.length; i++)
            {
                sprintf(&hex_string[i * 2], "%02X", response.value[i]); // Convert each byte to hex
            }

            // Pass the HEX string to the send_json_init_message function
            send_json_message(socket_fd, "msg", hex_string);

            // Free the allocated memory for the hex string
            free(hex_string);
        }
    }
}

void write_to_char(uint8_t *data, uint16_t length)
{
    esp_err_t status;
    uint16_t write_char_handle = 23;
    status = esp_ble_gattc_write_char(gattc_if_global,
                                      conn_id,
                                      write_char_handle,
                                      length,
                                      data,
                                      ESP_GATT_WRITE_TYPE_RSP,
                                      ESP_GATT_AUTH_REQ_NONE);

    if (status != ESP_OK)
    {
        printf("Failed to write to characteristic, error code = %d\n", status);
    }
    else
    {
        printf("Write successful\n");
    }
}
void read_from_char()
{
    esp_err_t status;
    uint16_t read_char_handle = 25;
    status = esp_ble_gattc_read_char(gattc_if_global,
                                     conn_id,
                                     read_char_handle,
                                     ESP_GATT_AUTH_REQ_NONE);
    if (status != ESP_OK)
    {
        printf("Failed to read from characteristic, error code = %d\n", status);
    }
    else
    {
        printf("Read request sent\n");
    }
}

void enable_notifications(esp_bd_addr_t server_bda)
{
    esp_err_t status;
    uint16_t notify_char_handle = 25;
    status = esp_ble_gattc_register_for_notify(gattc_if_global, server_bda, notify_char_handle);
    if (status != ESP_OK)
    {
        printf("Failed to register for notifications, error code = %d\n", status);
    }
    else
    {
        printf("Successfully registered for notifications\n");
    }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        break;
    case ESP_GATTC_CONNECT_EVT:
    {
        gattc_if_global = gattc_if;
        conn_id = param->connect.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        ESP_LOG_BUFFER_HEX(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
        if (mtu_ret)
        {
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }

        write_to_char(handshakeFrame, sizeof(handshakeFrame));
        enable_notifications(gl_profile_tab[PROFILE_A_APP_ID].remote_bda);
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, NULL);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
    {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16)
        {
            // 16-bit UUID
            uint16_t uuid16 = p_data->search_res.srvc_id.uuid.uuid.uuid16;
            ESP_LOGI(GATTC_TAG, "Service UUID16: 0x%04x", uuid16);
        }
        else if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128)
        {
            // 128-bit UUID
            ESP_LOGI(GATTC_TAG, "Service UUID128: ");
            ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->search_res.srvc_id.uuid.uuid.uuid128, 16);
        }
        else
        {
            // Handle other UUID lengths if needed
            ESP_LOGI(GATTC_TAG, "Service UUID length not supported: %d", p_data->search_res.srvc_id.uuid.len);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (param->search_cmpl.status == ESP_GATT_OK)
        {
            printf("Service search complete.\n");
            uint16_t start_handle = 21;
            uint16_t end_handle = 28;
            esp_gattc_char_elem_t char_elem_result[10];
            uint16_t count = 10;
            uint16_t offset = 0;

            esp_gatt_status_t status = esp_ble_gattc_get_all_char(gattc_if, param->search_cmpl.conn_id, start_handle, end_handle, char_elem_result, &count, offset);

            if (status == ESP_OK)
            {
                printf("Found %d characteristics:\n", count);
                for (int i = 0; i < count; i++)
                {
                    printf("Characteristic UUID: ");
                    if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16)
                    {
                        printf("0x%04x\n", char_elem_result[i].uuid.uuid.uuid16);
                    }
                    else if (char_elem_result[i].uuid.len == ESP_UUID_LEN_128)
                    {
                        printf("UUID128: ");
                        for (int j = 0; j < ESP_UUID_LEN_128; j++)
                        {

                            printf("%02x", char_elem_result[i].uuid.uuid.uuid128[j]);
                        }
                        printf("\n");
                    }
                    printf("Characteristic handle: %d\n", char_elem_result[i].char_handle);
                }

                write_to_char(handshakeFrame, sizeof(handshakeFrame));

                enable_notifications(gl_profile_tab[PROFILE_A_APP_ID].remote_bda);
            }
            else
            {
                printf("Failed to get characteristics, error code: %d\n", status);
            }
        }
        else
        {
            printf("Service search failed with status %d\n", param->search_cmpl.status);
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 2;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                        25,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            if (count > 0)
            {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                {
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        25,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        free(descr_elem_result);
                        descr_elem_result = NULL;
                        break;
                    }

                    ESP_LOGI(GATTC_TAG, "Descriptor handle: %d, UUID: %04x", descr_elem_result[0].handle, descr_elem_result[0].uuid.uuid.uuid16);
                    // printf("props are %d", descr_elem_result->properties);
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                    26,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
        {
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        BLEResponse response;
        memset(&response, 0, sizeof(BLEResponse)); // Zero out the structure
        memcpy(response.value, p_data->notify.value, p_data->notify.value_len);
        response.length = p_data->notify.value_len;

        // Enqueue the response
        if (xQueueSend(bleResponseQueue, &response, pdMS_TO_TICKS(10)) != pdTRUE)
        {
            ESP_LOGE(GATTC_TAG, "Failed to enqueue BLE response");
        }
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->reg_for_notify.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success ");

        break;
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        ESP_LOG_BUFFER_HEX(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write char success ");

        break;
    case ESP_GATTC_READ_CHAR_EVT:
        if (param->read.status == ESP_GATT_OK)
        {
            printf("Characteristic value: ");
            for (int i = 0; i < param->read.value_len; i++)
            {
                printf("%02x ", param->read.value[i]);
            }
            printf("\n");
        }
        else
        {
            printf("Failed to read characteristic, error code = %d\n", param->read.status);
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect_device = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        // the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            ESP_LOG_BUFFER_HEX(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0)
            {
                ESP_LOGI(GATTC_TAG, "adv data:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0)
            {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif
            ESP_LOGI(GATTC_TAG, " ");
            if (adv_name != NULL)
            {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0)
                {
                    ESP_LOGI(GATTC_TAG, "searched device %s", remote_device_name);
                    if (connect_device == false)
                    {
                        connect_device = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
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
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "packet length updated: rx = %d, tx = %d, status = %d",
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len,
                 param->pkt_data_length_cmpl.status);
        break;
    default:
        break;
    }
}
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE ||
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
void start_ble_scan(uint32_t duration)
{
    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (scan_ret)
    {
        ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        return;
    }
}
void ble_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
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
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    init_ble_queue();
    xTaskCreate(process_ble_responses, "BLE_Response_Task", 4096, NULL, 5, NULL);
}
