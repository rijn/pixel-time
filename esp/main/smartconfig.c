#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"

#include "smartconfig.h"

static void smartconfig_example_task(void * parm);
static void persist_wifi_config(wifi_config_t *wifi_config);
static bool read_wifi_config(wifi_config_t *wifi_config);

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;

static const char *TAG = "smartconfig";

static const char *NVS_NAME = "wifi";

static bool isWifiConfigurated = false;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (isWifiConfigurated) {
            esp_wifi_connect();
        } else {
            xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));

        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        persist_wifi_config(&wifi_config);

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK( esp_wifi_connect() );
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

void initialize_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

    // Check if wifi credential is presisted
    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config_t));
    if (read_wifi_config(&wifi_config)) {
        ESP_LOGI(TAG, "Use persisted wifi config.");
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        isWifiConfigurated = true;
    } else {
        ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    }

    ESP_ERROR_CHECK( esp_wifi_start() );

    esp_wifi_set_ps(WIFI_PS_NONE);
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

static void persist_wifi_config(wifi_config_t *wifi_config)
{
    ESP_LOGI(TAG, "Persist wifi config...");
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAME, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error opening NVS handle!");
    } else {
        err = nvs_set_blob(handle, "ssid", wifi_config->sta.ssid, sizeof(wifi_config->sta.ssid));
        err = nvs_set_blob(handle, "password", wifi_config->sta.password, sizeof(wifi_config->sta.password));

        err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Commit Failed.");
        }

        nvs_close(handle);
    }
}

static bool read_wifi_config(wifi_config_t *wifi_config)
{
    ESP_LOGI(TAG, "Read wifi config...");
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAME, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error opening NVS handle!");
    } else {
        size_t size_ssid = sizeof(wifi_config->sta.ssid);
        err = nvs_get_blob(handle, "ssid", wifi_config->sta.ssid, &size_ssid);
        size_t size_password = sizeof(wifi_config->sta.password);
        err = nvs_get_blob(handle, "password", wifi_config->sta.password, &size_password);

        nvs_close(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed %s.", esp_err_to_name(err));
    }
    return err == ESP_OK;
}
