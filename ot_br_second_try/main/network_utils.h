#include <stdint.h>
#include <stdbool.h>
#include "esp_ot_wifi_cmd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "cJSON.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"



#define EXAMPLE_ESP_WIFI_SSID "zipato-xxx"
#define EXAMPLE_ESP_WIFI_PASS "87654321"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define REMOTE_IP_ADDR "172.16.16.83"
#define REMOTE_PORT 5540
extern uint8_t socket_fd;
void wifi_init_sta(void);
int connect_to_server(const char *ip, int port);
void send_json_init_message(int socket_fd, const char *type, const char *content);
void send_json_message(int socket_fd, const char *type, const char *content);
void send_json_case_message(int socket_fd, const char *type,  const char *caseSess ,const char *content);
void send_json_otbr_message(int socket_fd, const char *type, const char *otbrType, const char *content);