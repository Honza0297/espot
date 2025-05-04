// smart_pot.c
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#define WATERPUMP_PIN       GPIO_NUM_5
#define LIGHT_PIN           GPIO_NUM_6
#define AMBIENT_LIGHT_PIN   GPIO_NUM_3
#define HUMIDITY_PIN        GPIO_NUM_4
#define WATER_NOW_DURATION  pdMS_TO_TICKS(5000)  // 5s one-shot

static const char *TAG = "smart_pot";

// shared state
static bool  watering_enabled = false;
static bool  water_now_flag    = false;
static bool  light_enabled     = false;
static SemaphoreHandle_t state_mutex;

//---------------------------------------------------------------------------
// GPIO init & helpers
//---------------------------------------------------------------------------
static void gpio_init(void)
{
    gpio_reset_pin(WATERPUMP_PIN);
    gpio_set_direction(WATERPUMP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(WATERPUMP_PIN, 0);

    gpio_reset_pin(LIGHT_PIN);
    gpio_set_direction(LIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LIGHT_PIN, 0);
}

static void pump_set(bool on)
{
    gpio_set_level(WATERPUMP_PIN, on ? 1 : 0);
}

static void light_set(bool on)
{
    gpio_set_level(LIGHT_PIN, on ? 1 : 0);
}

//---------------------------------------------------------------------------
// Watering task
//---------------------------------------------------------------------------
static void watering_task(void *arg)
{
    while (1) {
        // fetch current flags
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool keep_on = watering_enabled;
        bool one_shot = water_now_flag;
        xSemaphoreGive(state_mutex);

        if (one_shot) {
            ESP_LOGI(TAG, "One-shot watering for %d ticks", WATER_NOW_DURATION);
            pump_set(true);
            vTaskDelay(WATER_NOW_DURATION);
            // clear one-shot
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            water_now_flag = false;
            xSemaphoreGive(state_mutex);
            pump_set(false);
        } else if (keep_on) {
            pump_set(true);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            pump_set(false);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

//---------------------------------------------------------------------------
// Lights task
//---------------------------------------------------------------------------
static void lights_task(void *arg)
{
    while (1) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool on = light_enabled;
        xSemaphoreGive(state_mutex);

        light_set(on);
        vTaskDelay(pdMS_TO_TICKS(200)); // small delay to yield
    }
}

//---------------------------------------------------------------------------
// Simple WiFi AP setup (change to STA mode if you prefer)
//---------------------------------------------------------------------------
static void wifi_init_softap(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32_Pot",
            .ssid_len = 0,
            .channel = 1,
            .password = "flowerpot",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "WiFi AP started. SSID=%s", ap_config.ap.ssid);
}

//---------------------------------------------------------------------------
// HTTP Server: handlers & start
//---------------------------------------------------------------------------
static const char *html_page =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>Smart Flower Pot</title></head><body>"
    "<h1>Flower Pot Control</h1>"
    "<form action=\"/toggle_watering\" method=\"GET\">"
      "<button type=\"submit\">Toggle Watering</button>"
    "</form>"
    "<form action=\"/water_now\" method=\"GET\">"
      "<button type=\"submit\">Water Now</button>"
    "</form>"
    "<form action=\"/toggle_light\" method=\"GET\">"
      "<button type=\"submit\">Toggle Lights</button>"
    "</form>"
    "</body></html>";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t toggle_watering_handler(httpd_req_t *req)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    watering_enabled = !watering_enabled;
    // if turning off persistent watering, also clear any one-shot
    if (!watering_enabled) water_now_flag = false;
    ESP_LOGI(TAG, "Watering %s", watering_enabled ? "ENABLED" : "DISABLED");
    xSemaphoreGive(state_mutex);

    // redirect back
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t water_now_handler(httpd_req_t *req)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    water_now_flag = true;
    watering_enabled = false;  // pause persistent
    ESP_LOGI(TAG, "One-shot watering requested");
    xSemaphoreGive(state_mutex);

    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t toggle_light_handler(httpd_req_t *req)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    light_enabled = !light_enabled;
    ESP_LOGI(TAG, "Lights %s", light_enabled ? "ON" : "OFF");
    xSemaphoreGive(state_mutex);

    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        // register URIs
        httpd_uri_t uris[] = {
            { .uri       = "/",                .method = HTTP_GET,  .handler = root_get_handler },
            { .uri       = "/toggle_watering", .method = HTTP_GET,  .handler = toggle_watering_handler },
            { .uri       = "/water_now",       .method = HTTP_GET,  .handler = water_now_handler },
            { .uri       = "/toggle_light",    .method = HTTP_GET,  .handler = toggle_light_handler },
        };
        for (int i = 0; i < sizeof(uris)/sizeof(uris[0]); i++) {
            httpd_register_uri_handler(server, &uris[i]);
        }
    }
    return server;
}

//---------------------------------------------------------------------------
// app_main
//---------------------------------------------------------------------------
void app_main(void)
{
    // init NVS & WiFi
    nvs_flash_init();
    wifi_init_softap();

    // init hardware
    gpio_init();

    // create mutex
    state_mutex = xSemaphoreCreateMutex();

    // start tasks
    xTaskCreate(watering_task, "watering_task", 2048, NULL, 5, NULL);
    xTaskCreate(lights_task,   "lights_task",   2048, NULL, 5, NULL);

    // start web server
    start_webserver();
}
