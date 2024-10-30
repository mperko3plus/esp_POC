#include "network_utils.h"
#include "esp_log.h"

#define TAG "wifi"
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
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
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != NULL)
    {
        ESP_LOGI(TAG, "WiFi already initialized by OTBR.");
        return;
    }

    ESP_LOGI(TAG, "Initializing WiFi...");
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

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

    // Add line terminator
    size_t json_length = strlen(json_string);
    char *json_with_terminator = malloc(json_length + 2); // +1 for '\n' and +1 for '\0'
    if (json_with_terminator == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON with terminator");
        free(json_string);
        cJSON_Delete(root);
        return;
    }
    strcpy(json_with_terminator, json_string);
    json_with_terminator[json_length] = '\n';     // Add newline
    json_with_terminator[json_length + 1] = '\0'; // Null-terminate the string

    // Send JSON string with terminator to the server
    if (send(socket_fd, json_with_terminator, strlen(json_with_terminator), 0) < 0)
    {
        ESP_LOGE(TAG, "Failed to send JSON message");
    }
    else
    {
        ESP_LOGI(TAG, "JSON message sent successfully: %s", json_with_terminator);
    }

    // Free JSON object and strings
    cJSON_Delete(root);
    free(json_string);
    free(json_with_terminator);
}

void send_json_message(int socket_fd, const char *type, const char *content)
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
    cJSON_AddStringToObject(root, "msgString", content);

    // Convert JSON object to string
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON object to string");
        cJSON_Delete(root);
        return;
    }

    // Add line terminator
    size_t json_length = strlen(json_string);
    char *json_with_terminator = malloc(json_length + 2); // +1 for '\n' and +1 for '\0'
    if (json_with_terminator == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON with terminator");
        free(json_string);
        cJSON_Delete(root);
        return;
    }
    strcpy(json_with_terminator, json_string);
    json_with_terminator[json_length] = '\n';     // Add newline
    json_with_terminator[json_length + 1] = '\0'; // Null-terminate the string

    // Send JSON string with terminator to the server
    if (send(socket_fd, json_with_terminator, strlen(json_with_terminator), 0) < 0)
    {
        ESP_LOGE(TAG, "Failed to send JSON message");
    }
    else
    {
        ESP_LOGI(TAG, "JSON message sent successfully: %s", json_with_terminator);
    }

    // Free JSON object and strings
    cJSON_Delete(root);
    free(json_string);
    free(json_with_terminator);
}
void send_json_case_message(int socket_fd, const char *type,  const char *caseSess ,const char *content)
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
    cJSON_AddStringToObject(root, "cse", caseSess);
    cJSON_AddStringToObject(root, "msgString", content);

    // Convert JSON object to string
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON object to string");
        cJSON_Delete(root);
        return;
    }

    // Add line terminator
    size_t json_length = strlen(json_string);
    char *json_with_terminator = malloc(json_length + 2); // +1 for '\n' and +1 for '\0'
    if (json_with_terminator == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON with terminator");
        free(json_string);
        cJSON_Delete(root);
        return;
    }
    strcpy(json_with_terminator, json_string);
    json_with_terminator[json_length] = '\n';     // Add newline
    json_with_terminator[json_length + 1] = '\0'; // Null-terminate the string

    // Send JSON string with terminator to the server
    if (send(socket_fd, json_with_terminator, strlen(json_with_terminator), 0) < 0)
    {
        ESP_LOGE(TAG, "Failed to send JSON message");
    }
    else
    {
        ESP_LOGI(TAG, "JSON message sent successfully: %s", json_with_terminator);
    }

    // Free JSON object and strings
    cJSON_Delete(root);
    free(json_string);
    free(json_with_terminator);
}

void send_json_otbr_message(int socket_fd, const char *type, const char *otbrType, const char *content){
        // Create a JSON object
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to create cJSON object");
        return;
    }

    // Add data to the JSON object
    cJSON_AddStringToObject(root, "event", type);
    cJSON_AddStringToObject(root , "otbrType", otbrType);
    cJSON_AddStringToObject(root, "msgString", content);

    // Convert JSON object to string
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON object to string");
        cJSON_Delete(root);
        return;
    }

    // Add line terminator
    size_t json_length = strlen(json_string);
    char *json_with_terminator = malloc(json_length + 2); // +1 for '\n' and +1 for '\0'
    if (json_with_terminator == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON with terminator");
        free(json_string);
        cJSON_Delete(root);
        return;
    }
    strcpy(json_with_terminator, json_string);
    json_with_terminator[json_length] = '\n';     // Add newline
    json_with_terminator[json_length + 1] = '\0'; // Null-terminate the string

    // Send JSON string with terminator to the server
    if (send(socket_fd, json_with_terminator, strlen(json_with_terminator), 0) < 0)
    {
        ESP_LOGE(TAG, "Failed to send JSON message");
    }
    else
    {
        ESP_LOGI(TAG, "JSON message sent successfully: %s", json_with_terminator);
    }

    // Free JSON object and strings
    cJSON_Delete(root);
    free(json_string);
    free(json_with_terminator);
}