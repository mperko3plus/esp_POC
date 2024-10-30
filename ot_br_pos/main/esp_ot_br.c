/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Border Router Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <string.h>
#include "network_utils.h"
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_border_router.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_cli_extension.h"
#include "esp_ot_config.h"
#include "esp_ot_wifi_cmd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "openthread/error.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"
#include "openthread/cli.h"
#include "ble.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "cJSON.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "mdns.h"

#define TAG "esp_ot_br"

#define MAX_BUFFER_SIZE 1024
static bool isDataReceivedExt = false;
static bool isDataReceivedDataset = false;

// Global buffer to accumulate output
static char cliOutputBuffer[MAX_BUFFER_SIZE];
static size_t cliOutputBufferIndex = 0;

uint8_t socket_fd;

typedef enum
{
    EVENT_ON,
    EVENT_OFF,
    READ_LED,
    EVENT_UNKNOWN,
    ADD_DEVICE,
    SCAN,
    START,
    DATASET_GET,
    EXTPANID,
    GET_IP
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
    else if (strcmp(event, "start") == 0)
    {
        return START;
    }
    else if (strcmp(event, "dataset_get") == 0)
    {
        return DATASET_GET;
    }
    else if (strcmp(event, "extPanId_get") == 0)
    {
        return EXTPANID;
    }
    else if (strcmp(event, "get_ip") == 0)
    {
        return GET_IP;
    }

    else
    {
        return EVENT_UNKNOWN;
    }
}

void mdns_init_service(void)
{
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());

    // Set hostname
    ESP_ERROR_CHECK(mdns_hostname_set("my_esp_device"));

    // Set instance name
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 mDNS Example"));

    ESP_LOGI("mDNS", "mDNS service initialized.");
}

char *search_mdns_service(const char *name)
{
    mdns_result_t *results = NULL;
    const char *target_instance = name; // Your specific instance identifier

    ESP_LOGI("mDNS", "Starting mDNS query for service: %s", target_instance);

    // Query for the _matterc._tcp service
    esp_err_t err = mdns_query_ptr("_matter", "_tcp", 3000, 20, &results);
    if (err != ESP_OK)
    {
        ESP_LOGE("mDNS", "mDNS query failed: %s", esp_err_to_name(err));
        return NULL;
    }

    if (results)
    {
        mdns_result_t *r = results;
        int count = 0;

        while (r != NULL)
        {
            count++;
            ESP_LOGI("mDNS", "Service #%d", count);
            ESP_LOGI("mDNS", "  Instance Name: %s", r->instance_name);
            ESP_LOGI("mDNS", "  Service Type: %s", r->service_type);
            ESP_LOGI("mDNS", "  Protocol: %s", r->proto);

            // Print the addresses found
            mdns_ip_addr_t *addr = r->addr;
            while (addr != NULL)
            {
                char addr_str[INET6_ADDRSTRLEN] = {0};
                if (addr->addr.type == ESP_IPADDR_TYPE_V6)
                {
                    inet_ntop(AF_INET6, &addr->addr.u_addr.ip6, addr_str, sizeof(addr_str));
                    ESP_LOGI("mDNS", "  IPv6 Address: %s", addr_str);
                }
                else if (addr->addr.type == ESP_IPADDR_TYPE_V4)
                {
                    inet_ntop(AF_INET, &addr->addr.u_addr.ip4, addr_str, sizeof(addr_str));
                    ESP_LOGI("mDNS", "  IPv4 Address: %s", addr_str);
                }
                addr = addr->next;
            }

            // Check if the current service instance matches the target instance
            if (strcmp(r->instance_name, target_instance) == 0)
            {
                ESP_LOGI("mDNS", "Found matching service: %s", target_instance);

                // Check for IPv6 addresses
                addr = r->addr;
                while (addr != NULL)
                {
                    if (addr->addr.type == ESP_IPADDR_TYPE_V6)
                    {
                        char ipv6[INET6_ADDRSTRLEN];
                        inet_ntop(AF_INET6, &addr->addr.u_addr.ip6, ipv6, sizeof(ipv6));

                        char *ipv6_str = malloc(INET6_ADDRSTRLEN);
                        if (ipv6_str != NULL)
                        {
                            strcpy(ipv6_str, ipv6);
                            ESP_LOGI("mDNS", "Returning IPv6 address: %s", ipv6_str);
                        }

                        mdns_query_results_free(results); // Free the results
                        return ipv6_str; // Return the IPv6 string
                    }
                    addr = addr->next;
                }
            }
            r = r->next; // Go to the next result in the list
        }

        ESP_LOGI("mDNS", "No matching services found.");
        mdns_query_results_free(results); // Free the results once done
    }
    else
    {
        ESP_LOGI("mDNS", "No mDNS services found.");
    }

    return NULL; // Return NULL if no matching service or IPv6 address found
}


void hexToBytes(const char *hexString, uint8_t *byteArray, size_t byteArrayLen)
{
    size_t hexLen = strlen(hexString);
    for (size_t i = 0; i < hexLen / 2 && i < byteArrayLen; i++)
    {
        sscanf(&hexString[2 * i], "%2hhx", &byteArray[i]);
    }
}

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#if CONFIG_EXTERNAL_COEX_ENABLE
static void ot_br_external_coexist_init(void)
{
    esp_external_coex_gpio_set_t gpio_pin = ESP_OPENTHREAD_DEFAULT_EXTERNAL_COEX_CONFIG();
    esp_external_coex_set_work_mode(EXTERNAL_COEX_LEADER_ROLE);
    ESP_ERROR_CHECK(esp_enable_extern_coex_gpio_pin(CONFIG_EXTERNAL_COEX_WIRE_TYPE, gpio_pin));
}
#endif /* CONFIG_EXTERNAL_COEX_ENABLE */


static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t       *openthread_netif = esp_netif_new(&cfg);
    assert(openthread_netif != NULL);

    // Initialize the OpenThread stack
    ESP_ERROR_CHECK(esp_openthread_init(&config));
    ESP_ERROR_CHECK(esp_netif_attach(openthread_netif, esp_openthread_netif_glue_init(&config)));
    esp_openthread_lock_acquire(portMAX_DELAY);
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
    esp_openthread_cli_init();
    esp_cli_custom_command_init();
    esp_openthread_cli_create_task();
    esp_openthread_lock_release();

    // Run the main loop
    esp_openthread_launch_mainloop();

    // Clean up
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);
    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}


int myCliOutputCallbackDataset(void *aContext, const char *aFormat, va_list aArguments)
{
    if (isDataReceivedDataset)
    {
        return 0; // Skip processing if data has already been received
    }

    char tempBuffer[256];
    int bytesWritten = vsnprintf(tempBuffer, sizeof(tempBuffer), aFormat, aArguments);

    // Append to the global buffer, ensuring no overflow
    if (cliOutputBufferIndex + bytesWritten < MAX_BUFFER_SIZE)
    {
        strncpy(&cliOutputBuffer[cliOutputBufferIndex], tempBuffer, bytesWritten);
        cliOutputBufferIndex += bytesWritten;
    }

    // Check if the output contains the "Done" keyword, indicating completion
    if (strstr(tempBuffer, "Done") != NULL)
    {
        // Clean the buffer by removing the `>` prompt and trimming newlines and "Done"
        char *cleanOutput = cliOutputBuffer;

        // Skip the `>` prompt if present
        if (cleanOutput[0] == '>')
        {
            cleanOutput++; // Skip first character
        }

        // Remove any trailing `\r\n` or "Done" from the buffer
        char *donePtr = strstr(cleanOutput, "Done");
        if (donePtr != NULL)
        {
            *donePtr = '\0'; // Terminate the string before "Done"
        }

        // Trim any trailing newlines or carriage returns
        size_t length = strlen(cleanOutput);
        while (length > 0 && (cleanOutput[length - 1] == '\r' || cleanOutput[length - 1] == '\n'))
        {
            cleanOutput[--length] = '\0';
        }

        // Print the cleaned output
        printf("Complete CLI Output:\n%s\n", cleanOutput);

        // Send the cleaned output via the JSON message
        send_json_otbr_message(socket_fd, "otbr", "dataset", (const char *)cleanOutput);

        // Reset the buffer for the next command
        memset(cliOutputBuffer, 0, sizeof(cliOutputBuffer));
        cliOutputBufferIndex = 0;

        isDataReceivedDataset = true; // Set flag to indicate data has been received
    }

    // Return the number of bytes written
    return bytesWritten;
}

int myCliOutputCallbackEXT(void *aContext, const char *aFormat, va_list aArguments)
{
    if (isDataReceivedExt)
    {
        return 0; // Skip processing if data has already been received
    }

    char tempBuffer[256];
    int bytesWritten = vsnprintf(tempBuffer, sizeof(tempBuffer), aFormat, aArguments);

    // Append to the global buffer, ensuring no overflow
    if (cliOutputBufferIndex + bytesWritten < MAX_BUFFER_SIZE)
    {
        strncpy(&cliOutputBuffer[cliOutputBufferIndex], tempBuffer, bytesWritten);
        cliOutputBufferIndex += bytesWritten;
    }

    // Check if the output contains the "Done" keyword, indicating completion
    if (strstr(tempBuffer, "Done") != NULL)
    {
        // Clean the buffer by removing the `>` prompt and trimming newlines and "Done"
        char *cleanOutput = cliOutputBuffer;

        // Skip the `>` prompt if present
        if (cleanOutput[0] == '>')
        {
            cleanOutput++; // Skip first character
        }

        // Remove any trailing `\r\n` or "Done" from the buffer
        char *donePtr = strstr(cleanOutput, "Done");
        if (donePtr != NULL)
        {
            *donePtr = '\0'; // Terminate the string before "Done"
        }

        // Trim any trailing newlines or carriage returns
        size_t length = strlen(cleanOutput);
        while (length > 0 && (cleanOutput[length - 1] == '\r' || cleanOutput[length - 1] == '\n'))
        {
            cleanOutput[--length] = '\0';
        }

        // Print the cleaned output
        printf("Complete CLI Output:\n%s\n", cleanOutput);

        // Send the cleaned output via the JSON message
        send_json_otbr_message(socket_fd, "otbr", "extPanId", (const char *)cleanOutput);

        // Reset the buffer for the next command
        memset(cliOutputBuffer, 0, sizeof(cliOutputBuffer));
        cliOutputBufferIndex = 0;

        isDataReceivedExt = true; // Set flag to indicate data has been received
    }

    // Return the number of bytes written
    return bytesWritten;
}
void handle_start_event()
{

    esp_openthread_cli_input("dataset init new");
    esp_openthread_cli_input("dataset commit active");
    esp_openthread_cli_input("ifconfig up");
    esp_openthread_cli_input("thread start");
}

void get_dataset()
{
    otInstance *instance = otInstanceInitSingle(); // Replace with your instance initialization

    // Initialize OpenThread CLI and set the output callback
    otCliInit(instance, myCliOutputCallbackDataset, NULL);
    esp_openthread_cli_input("dataset active -x");
  
}

void get_extPanID()
{
    otInstance *instance = otInstanceInitSingle(); // Replace with your instance initialization

    // Initialize OpenThread CLI and set the output callback
    otCliInit(instance, myCliOutputCallbackEXT, NULL);
    esp_openthread_cli_input("extpanid");
    vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as necessary

   
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
        cJSON *msg = cJSON_GetObjectItem(json, "msg");
        cJSON *msg1 = cJSON_GetObjectItem(json, "msg1");
        cJSON *msg2 = cJSON_GetObjectItem(json, "msg2");
        cJSON *type = cJSON_GetObjectItem(json, "type");
        cJSON *name = cJSON_GetObjectItem(json, "name");

        if (cJSON_IsString(event) && cJSON_IsString(cluster))
        {
            EventType event_type = get_event_type(event->valuestring);
            switch (event_type)
            {

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
            case START:
                ESP_LOGI(TAG, "Received 'start' event for cluster '11111'");
                handle_start_event();
                break;
            case SCAN:
                ESP_LOGI(TAG, "Received 'scan' event for cluster '11111'");
                start_ble_scan(30);
                break;
            case DATASET_GET:
                ESP_LOGI(TAG, "Received 'dataset_get' event for cluster '11111'");
                get_dataset();
                break;
            case EXTPANID:
                ESP_LOGI(TAG, "Received 'dataset_get' event for cluster '11111'");
                get_extPanID();
                break;
            case GET_IP:
                ESP_LOGI(TAG, "Received 'get_ip' event for cluster '11111'");
                if (cJSON_IsString(name))
                {
                    char *ipv6_address = search_mdns_service(cJSON_GetStringValue(name));
                    if (ipv6_address != NULL)
                    {
                        ESP_LOGI(TAG, "Found IPv6 address: %s", ipv6_address);
                        free(ipv6_address); // Free the allocated memory after use
                    }
                    else
                    {
                        ESP_LOGI(TAG, "No matching IPv6 address found.");
                    }
                }
                break;
            default:
                ESP_LOGI(TAG, "Received unrecognized event: %s", event->valuestring);
                break;
            }
        }
        else if (cJSON_IsString(msg) && cJSON_IsString(type))
        {
            const char *msgStr = msg->valuestring;
            if (strcmp(type->valuestring, "PASE") == 0)
            {

                size_t msgLen = strlen(msgStr) / 2;
                uint8_t *msgBytes = (uint8_t *)malloc(msgLen);
                printf("Processing PASE message with length: %zu bytes\n", msgLen);
                hexToBytes(msgStr, msgBytes, msgLen);

                // Now write the byte array to the characteristic
                write_to_char(msgBytes, msgLen);
            }
            else if (strcmp(type->valuestring, "CASE") == 0)
            {
                printf("Processing CASE message...\n");
            }
            else
            {
                printf("Unknown type: \n");
            }
        }
        else if (cJSON_IsString(msg1) && cJSON_IsString(msg2) && cJSON_IsString(type))
        {
            const char *msgStr1 = msg1->valuestring;
            const char *msgStr2 = msg2->valuestring;
            if (strcmp(type->valuestring, "PASE") == 0)
            {

                size_t msg1_Len = strlen(msgStr1) / 2;
                uint8_t *msgBytes1 = (uint8_t *)malloc(msg1_Len);
                printf("Processing PASE message with length: %zu bytes\n", msg1_Len);
                hexToBytes(msgStr1, msgBytes1, msg1_Len);

                // Now write the byte array to the characteristic
                write_to_char(msgBytes1, msg1_Len);

                size_t msg2_Len = strlen(msgStr2) / 2;
                uint8_t *msgBytes2 = (uint8_t *)malloc(msg2_Len);
                printf("Processing PASE message with length: %zu bytes\n", msg2_Len);
                hexToBytes(msgStr2, msgBytes2, msg2_Len);

                // Now write the byte array to the characteristic
                write_to_char(msgBytes2, msg2_Len);
            }
            else if (strcmp(type->valuestring, "CASE") == 0)
            {
                printf("Processing CASE message...\n");
            }
            else
            {
                printf("Unknown type: \n");
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

    esp_vfs_eventfd_config_t eventfd_config = {
#if CONFIG_OPENTHREAD_RADIO_NATIVE || CONFIG_OPENTHREAD_RADIO_SPINEL_SPI
        .max_fds = 4,
#else
        .max_fds = 3,
#endif
    };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_EXAMPLE_CONNECT_WIFI
#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(example_connect());
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE && CONFIG_OPENTHREAD_RADIO_NATIVE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_coex_wifi_i154_enable());
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_EXTERNAL_COEX_ENABLE
    ot_br_external_coexist_init();
#endif

#endif
    esp_openthread_set_backbone_netif(get_example_netif());
#else
    esp_ot_wifi_netif_init();
    esp_openthread_set_backbone_netif(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"));
#endif
#elif CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(example_connect());
    esp_openthread_set_backbone_netif(get_example_netif());
#else
    ESP_LOGE(TAG, "ESP-Openthread has not set backbone netif");
#endif

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));
    xTaskCreate(ot_task_worker, "ot_br_main", 20480, xTaskGetCurrentTaskHandle(), 5, NULL);

    // wifi_init_sta();

    ble_init();
    mdns_init_service();
    // search_mdns_service();

    // Search for mDNS services

    socket_fd = connect_to_server(REMOTE_IP_ADDR, REMOTE_PORT);
    if (socket_fd != -1)
    {
        send_json_init_message(socket_fd, "init", "PCmperko");
        xTaskCreate(&receive_json_messages, "receive_json_messages", 4096, (void *)socket_fd, 5, NULL);
    }
}
