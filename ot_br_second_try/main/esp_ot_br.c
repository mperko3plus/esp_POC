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
#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>

#define BUFFER_SIZE 1024 // Define the buffer size
uint8_t socket_fd;
#define TAG "esp_ot_br"
#define MAX_BUFFER_SIZE 1024
#define UDP_PORT 5540
#define BUFFER_SIZE 1024
static bool isDataReceivedExt = false;
static bool isDataReceivedDataset = false;
static char cliOutputBuffer[MAX_BUFFER_SIZE];
static size_t cliOutputBufferIndex = 0;
int socketUDP;
#if CONFIG_EXTERNAL_COEX_ENABLE
static void ot_br_external_coexist_init(void)
{
    esp_external_coex_gpio_set_t gpio_pin = ESP_OPENTHREAD_DEFAULT_EXTERNAL_COEX_CONFIG();
    esp_external_coex_set_work_mode(EXTERNAL_COEX_LEADER_ROLE);
    ESP_ERROR_CHECK(esp_enable_extern_coex_gpio_pin(CONFIG_EXTERNAL_COEX_WIRE_TYPE, gpio_pin));
}
#endif
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
void hexToBytes(const char *hexString, uint8_t *byteArray, size_t byteArrayLen)
{
    size_t hexLen = strlen(hexString);
    for (size_t i = 0; i < hexLen / 2 && i < byteArrayLen; i++)
    {
        sscanf(&hexString[2 * i], "%2hhx", &byteArray[i]);
    }
}
void handle_start_event()
{
    esp_openthread_cli_input("dataset init new");
    esp_openthread_cli_input("dataset commit active");
    esp_openthread_cli_input("ifconfig up");
    esp_openthread_cli_input("thread start");
}
int myCliOutputCallbackDataset(void *aContext, const char *aFormat, va_list aArguments)
{
    if (isDataReceivedDataset)
    {
        return 0;
    }
    char tempBuffer[256];
    int bytesWritten = vsnprintf(tempBuffer, sizeof(tempBuffer), aFormat, aArguments);
    if (cliOutputBufferIndex + bytesWritten < MAX_BUFFER_SIZE)
    {
        strncpy(&cliOutputBuffer[cliOutputBufferIndex], tempBuffer, bytesWritten);
        cliOutputBufferIndex += bytesWritten;
    }
    if (strstr(tempBuffer, "Done") != NULL)
    {
        char *cleanOutput = cliOutputBuffer;
        if (cleanOutput[0] == '>')
        {
            cleanOutput++;
        }
        char *donePtr = strstr(cleanOutput, "Done");
        if (donePtr != NULL)
        {
            *donePtr = '\0';
        }
        size_t length = strlen(cleanOutput);
        while (length > 0 && (cleanOutput[length - 1] == '\r' || cleanOutput[length - 1] == '\n'))
        {
            cleanOutput[--length] = '\0';
        }
        printf("Complete CLI Output:\n%s\n", cleanOutput);
        send_json_otbr_message(socket_fd, "otbr", "dataset", (const char *)cleanOutput);
        memset(cliOutputBuffer, 0, sizeof(cliOutputBuffer));
        cliOutputBufferIndex = 0;
        isDataReceivedDataset = true;
    }
    return bytesWritten;
}
int myCliOutputCallbackEXT(void *aContext, const char *aFormat, va_list aArguments)
{
    if (isDataReceivedExt)
    {
        return 0;
    }

    char tempBuffer[256];
    int bytesWritten = vsnprintf(tempBuffer, sizeof(tempBuffer), aFormat, aArguments);
    if (cliOutputBufferIndex + bytesWritten < MAX_BUFFER_SIZE)
    {
        strncpy(&cliOutputBuffer[cliOutputBufferIndex], tempBuffer, bytesWritten);
        cliOutputBufferIndex += bytesWritten;
    }
    if (strstr(tempBuffer, "Done") != NULL)
    {
        char *cleanOutput = cliOutputBuffer;
        if (cleanOutput[0] == '>')
        {
            cleanOutput++;
        }
        char *donePtr = strstr(cleanOutput, "Done");
        if (donePtr != NULL)
        {
            *donePtr = '\0';
        }
        size_t length = strlen(cleanOutput);
        while (length > 0 && (cleanOutput[length - 1] == '\r' || cleanOutput[length - 1] == '\n'))
        {
            cleanOutput[--length] = '\0';
        }
        printf("Complete CLI Output:\n%s\n", cleanOutput);
        send_json_otbr_message(socket_fd, "otbr", "extPanId", (const char *)cleanOutput);
        memset(cliOutputBuffer, 0, sizeof(cliOutputBuffer));
        cliOutputBufferIndex = 0;
        isDataReceivedExt = true;
    }
    return bytesWritten;
}
void get_dataset()
{
    otInstance *instance = otInstanceInitSingle();
    otCliInit(instance, myCliOutputCallbackDataset, NULL);
    esp_openthread_cli_input("dataset active -x");
}

void get_extPanID()
{
    otInstance *instance = otInstanceInitSingle();
    otCliInit(instance, myCliOutputCallbackEXT, NULL);
    esp_openthread_cli_input("extpanid");
    vTaskDelay(pdMS_TO_TICKS(100));
}
static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *openthread_netif = esp_netif_new(&cfg);
    assert(openthread_netif != NULL);
    ESP_ERROR_CHECK(esp_openthread_init(&config));
    ESP_ERROR_CHECK(esp_netif_attach(openthread_netif, esp_openthread_netif_glue_init(&config)));
    esp_openthread_lock_acquire(portMAX_DELAY);
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
    esp_openthread_cli_init();
    esp_cli_custom_command_init();
    esp_openthread_cli_create_task();
    esp_openthread_lock_release();
    esp_openthread_launch_mainloop();
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);
    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

void ot_br_init(void *ctx)
{
#if CONFIG_EXAMPLE_CONNECT_WIFI
#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE && CONFIG_OPENTHREAD_RADIO_NATIVE
    ESP_ERROR_CHECK(esp_coex_wifi_i154_enable());
#else
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
    esp_openthread_lock_acquire(portMAX_DELAY);
#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif
#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(esp_openthread_border_router_init());
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif
    esp_openthread_lock_release();
    vTaskDelete(NULL);
}
void wait_for_wifi_connection()
{
    wifi_ap_record_t ap_info;
    int retries = 0;
    const int max_retries = 10;
    const int delay_ms = 1000;
    while (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK && retries < max_retries)
    {
        ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
        retries++;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    if (retries >= max_retries)
    {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi after %d retries", retries);
    }
    else
    {
        ESP_LOGI(TAG, "Connected to Wi-Fi, SSID: %s", ap_info.ssid);
    }
}
int hex_to_bin(const char *hex_str, unsigned char *bin_data, size_t bin_len)
{
    size_t hex_len = strlen(hex_str);
    if (hex_len % 2 != 0 || bin_len < hex_len / 2)
    {
        return -1;
    }
    for (size_t i = 0; i < hex_len / 2; i++)
    {
        sscanf(hex_str + 2 * i, "%2hhx", &bin_data[i]);
    }
    return hex_len / 2;
}
int create_and_bind_socket(int port) {
    int sockfd;
    struct sockaddr_in6 server_addr;

    sockfd = socket(AF_INET6, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin6_family = AF_INET6;
    server_addr.sin6_addr = in6addr_any;
    server_addr.sin6_port = htons(port);

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Failed to bind socket");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    return sockfd;
}

void listen_for_responses(void *pvParameters) {
    int sock = (int)(uintptr_t)pvParameters; // Cast back to int

    struct sockaddr_in6 response_addr;
    socklen_t addr_len = sizeof(response_addr);
    unsigned char buffer[1024];
    ssize_t len;
    char hex_string[2048];

    while (1) {
        len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&response_addr, &addr_len);
        if (len < 0) {
            perror("Failed to receive response");
            continue; // Try again on error
        }

        printf("Received response (%d bytes): ", (int)len);
        for (ssize_t i = 0; i < len; i++) {
            printf("%02x ", buffer[i]);
            sprintf(hex_string + i * 2, "%02x", buffer[i]);
        }
        hex_string[len * 2] = '\0';
        printf("\n");

        // Send JSON case message to cloud
        send_json_case_message(socket_fd, "msg", "true", hex_string);
    }

    vTaskDelete(NULL);
}

void send_udp_message(const char *dest_addr_str, const char *hex_msg_str) {
    // Convert hex message string to binary data
    size_t hex_msg_len = strlen(hex_msg_str);
    unsigned char bin_msg[hex_msg_len / 2];
    int bin_msg_len = hex_to_bin(hex_msg_str, bin_msg, sizeof(bin_msg));
    if (bin_msg_len < 0) {
        fprintf(stderr, "Error converting hex message to binary\n");
        return;
    }

    // Prepare the destination address structure
    struct sockaddr_in6 dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(5540);

    // Convert destination address
    if (inet_pton(AF_INET6, dest_addr_str, &dest_addr.sin6_addr) <= 0) {
        fprintf(stderr, "Invalid destination address: %s\n", dest_addr_str);
        return;
    }

    // Send the message
    if (sendto(socketUDP, bin_msg, bin_msg_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        perror("Failed to send message");
    } else {
        printf("Message sent successfully\n");
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
        cJSON *json = cJSON_Parse(buffer);
        if (json == NULL)
        {
            ESP_LOGE(TAG, "Failed to parse JSON message");
            continue;
        }
        cJSON *event = cJSON_GetObjectItem(json, "event");
        cJSON *cluster = cJSON_GetObjectItem(json, "cluster");
        cJSON *msg = cJSON_GetObjectItem(json, "msg");
        cJSON *msg1 = cJSON_GetObjectItem(json, "msg1");
        cJSON *msg2 = cJSON_GetObjectItem(json, "msg2");
        cJSON *type = cJSON_GetObjectItem(json, "type");
        cJSON *name = cJSON_GetObjectItem(json, "name");
        cJSON *add = cJSON_GetObjectItem(json, "add");
        cJSON *port = cJSON_GetObjectItem(json, "port");
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
                write_to_char(msgBytes, msgLen);
            }
            else if (strcmp(type->valuestring, "CASE") == 0)
            {
                const char *addStr = add->valuestring;
                const char *portStr = port->valuestring;

                send_udp_message(addStr, msgStr);
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
                write_to_char(msgBytes1, msg1_Len);
                size_t msg2_Len = strlen(msgStr2) / 2;
                uint8_t *msgBytes2 = (uint8_t *)malloc(msg2_Len);
                printf("Processing PASE message with length: %zu bytes\n", msg2_Len);
                hexToBytes(msgStr2, msgBytes2, msg2_Len);
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
    xTaskCreate(ot_task_worker, "ot_br_main", 8192, xTaskGetCurrentTaskHandle(), 5, NULL);
    xTaskCreate(ot_br_init, "ot_br_init", 6144, NULL, 4, NULL);
    wait_for_wifi_connection();
    vTaskDelay(pdMS_TO_TICKS(5000));
    ble_init();
    socket_fd = connect_to_server(REMOTE_IP_ADDR, REMOTE_PORT);
    socketUDP = create_and_bind_socket(51000);
    if (socket_fd != -1)
    {
        send_json_init_message(socket_fd, "init", "PCmperko");
        xTaskCreate(&receive_json_messages, "receive_json_messages", 8192, (void *)socket_fd, 5, NULL);
        xTaskCreate(&listen_for_responses, "listen_for_responses", 8192, (void *)socketUDP, 5, NULL);

    }
}
