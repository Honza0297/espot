#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_lcd_io_i2c.h"
#include <stdio.h>
#include "esp_system.h"
#include "i2c_lcd_pcf8574.h"
#include "esp_adc/adc_oneshot.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/event_groups.h"
#include "esp_netif_ip_addr.h"
#include "nvs_flash.h"
// ===== CONFIGURATION =====
#define I2C_MASTER_SCL_IO GPIO_NUM_10        // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO GPIO_NUM_8        // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000   // I2C master clock frequency

#define LCD_ADDR 0x27               // I2C address of the LCD
#define LCD_COLS 16                 // Number of columns in the LCD
#define LCD_ROWS 2                  // Number of rows in the LCD

#define SOIL_ADC_CHANNEL   ADC_CHANNEL_4  // GPIO4 on ESP32-C3
#define LIGHT_ADC_CHANNEL  ADC_CHANNEL_3  // GPIO3 on ESP32-C3

#define WATERING_PIN            GPIO_NUM_5   // GPIO5 for watering control
#define LIGHT_PIN               GPIO_NUM_6   // GPIO6 for light control
#define WATERING_DURATION_MS    5000         // Duration to run pump in milliseconds
#define SENSOR_REFRESH_MS       10000        // milliseconds between updates; change as needed
#define WATERING_CHECK_MS       1000         // milliseconds between watering checks; change as needed
#define WATERING_COOLDOWN_MS    600000       // Cooldown period after watering before restart

#define MQTT_USERNAME "homeassistant" // MQTT username
#define MQTT_PASSWORD "homeassistant" // MQTT password
#define MQTT_SOIL_HUMIDITY_TOPIC "espot/humidity" // MQTT topic for soil humidity
#define MQTT_AMBIENT_LIGHT_TOPIC "espot/ambient" // MQTT topic for ambient light
#define WATER_CMD_TOPIC  "espot/water/set"
#define LIGHT_CMD_TOPIC  "espot/light/set"
#define WATER_STATE_TOPIC "espot/water/state"
#define LIGHT_STATE_TOPIC "espot/light/state"

#define WIFI_SSID "espot" // WiFi SSID
#define WIFI_PASS "espressif" // WiFi password
// Event group and bit
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0



// =========================

static const char *TAG = "sensor_display";
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Global sensor values accessible from any task
volatile int soil_humidity = 0;
volatile int ambient_light = 0;

// Volatile control variables (externally modified)
volatile bool lights = false;
volatile bool water_pump = false;
// Initialize GPIOs
static void gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LIGHT_PIN) | (1ULL << WATERING_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected. Reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = data;
        char ip_str[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "Got IP: %s", ip_str);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    
}

void wifi_init_sta(void)
{
    // 1) Init TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    // 2) Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 3) Create default Wi-Fi sta netif
    esp_netif_create_default_wifi_sta();

    // 4) Register event handlers
    s_wifi_event_group = xEventGroupCreate();
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    // 5) Configure and start Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static void adc_init(adc_oneshot_unit_handle_t *adc1_handle)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc1_handle, SOIL_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc1_handle, LIGHT_ADC_CHANNEL, &chan_cfg));
}

// Helper: publish a boolean as "ON"/"OFF"
static void publish_state(const char *topic, bool state) {
    const char *payload = state ? "ON" : "OFF";
    esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), 1, 1);
    ESP_LOGI(TAG, "Published %s = %s", topic, payload);
}


static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to broker, subscribing...");
        esp_mqtt_client_subscribe(mqtt_client, WATER_CMD_TOPIC, 1);
        esp_mqtt_client_subscribe(mqtt_client, LIGHT_CMD_TOPIC, 1);
        // publish initial states
        publish_state(WATER_STATE_TOPIC, water_pump);
        publish_state(LIGHT_STATE_TOPIC, lights);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_DATA: {
        // Copy topic and payload into C-strings
        char topic[event->topic_len + 1];
        char payload[event->data_len + 1];
        memcpy(topic, event->topic, event->topic_len);
        topic[event->topic_len] = '\0';
        memcpy(payload, event->data, event->data_len);
        payload[event->data_len] = '\0';

        bool new_state = (strcasecmp(payload, "ON") == 0);
        if (strcmp(topic, WATER_CMD_TOPIC) == 0) {
            water_pump = new_state;
            ESP_LOGI(TAG, "Command water_pump = %s", new_state ? "ON" : "OFF");
            publish_state(WATER_STATE_TOPIC, water_pump);
        } else if (strcmp(topic, LIGHT_CMD_TOPIC) == 0) {
            lights = new_state;
            ESP_LOGI(TAG, "Command lights = %s", new_state ? "ON" : "OFF");
            publish_state(LIGHT_STATE_TOPIC, lights);
        }
        break;
    }

    default:
        break;
    }
}


// Initialize and start the MQTT client
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {
                .uri       = NULL,                   // unset so hostname/port are used
                .hostname  = "192.168.0.113",       // your broker’s mDNS name
                .port      = 1883,                   // standard MQTT port
                .transport = MQTT_TRANSPORT_OVER_TCP, 
                .path      = NULL,                   // not used for plain MQTT
            }
        },
        .credentials = {
            .username = "homeassistant",
            .authentication = {
                .password = "homeassistant"
            }
        }
    };

    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// Assuming you have the raw ADC value in adc_raw
int normalize_adc_value(int adc_raw, int adc_min, int adc_max) {
    // Constrain the value to be within the min-max range
    if (adc_raw < adc_min) adc_raw = adc_min;
    if (adc_raw > adc_max) adc_raw = adc_max;
    
    // Map the ADC value from [adc_min, adc_max] to [0, 100]
    int normalized = ((adc_raw - adc_min) * 100) / (adc_max - adc_min);
    
    return normalized;
}
void sensor_task(void *pv)
{
    adc_oneshot_unit_handle_t adc1_handle;

    ESP_LOGI(TAG, "Initializing ADC...");
    adc_init(&adc1_handle);
    ESP_LOGI(TAG, "ADC initialized!");

    // Gets the data from sensors 
    while(42)
    {
        // Read raw ADC values into globals
        // TODO: normalize them (0-100)
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SOIL_ADC_CHANNEL, &raw));
        soil_humidity = normalize_adc_value(raw, 0, 4095); // Assuming 12-bit ADC resolution
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LIGHT_ADC_CHANNEL, &raw));
        ambient_light = normalize_adc_value(raw, 0, 4095); // Assuming 12-bit ADC resolution

        ESP_LOGI(TAG, "Soil Humidity: %d, Ambient Light: %d", soil_humidity, ambient_light);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_REFRESH_MS));
    }
}

void lcd_update_values(void *pv)
{   
    ESP_LOGI(TAG, "Initializing LCD...");
    i2c_lcd_pcf8574_handle_t lcd;
    lcd_init(&lcd, LCD_ADDR, I2C_MASTER_NUM);
    lcd_begin(&lcd, LCD_COLS, LCD_ROWS);
    ESP_LOGI(TAG, "LCD initialized!");
    // Turn on the backlight
    lcd_set_backlight(&lcd, 128);

    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "ESPot is alive!");
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "By Honza0297");
    taskYIELD();
    
    while(42)
    {
        lcd_clear(&lcd);
        char ambient_light_str[16];
        char soil_humidity_str[16];
        snprintf(ambient_light_str, sizeof(ambient_light_str), "Light: %d", ambient_light);
        snprintf(soil_humidity_str, sizeof(soil_humidity_str), "Soil.: %d", soil_humidity);
        lcd_set_cursor(&lcd, 0, 0);
        lcd_print(&lcd, ambient_light_str);
        lcd_set_cursor(&lcd, 0, 1);
        lcd_print(&lcd, soil_humidity_str);
        ESP_LOGI(TAG, "LCD updated with values: %s, %s", ambient_light_str, soil_humidity_str);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_REFRESH_MS));
    
    }
}

// Task to handle lights
static void lights_task(void *pv)
{
    bool prev_lights = false;
    while (1) {
        // Set LIGHT_PIN based on lights variable
        gpio_set_level(LIGHT_PIN, lights ? 1 : 0);
        if (lights != prev_lights)
        {
            ESP_LOGI(TAG, "Switching lights %s", lights ? "ON" : "OFF");
            prev_lights = lights;
        }  
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

// Task to handle watering with early stop and cooldown support
static void watering_task(void *pv)
{
    TickType_t last_stop_tick = xTaskGetTickCount() - pdMS_TO_TICKS(WATERING_COOLDOWN_MS);
    while (1) {
        // Wait for command and cooldown expiration
        if (water_pump) {
            TickType_t now = xTaskGetTickCount();
            if (now - last_stop_tick < pdMS_TO_TICKS(WATERING_COOLDOWN_MS)) {
                ESP_LOGW(TAG, "Cooldown active, cannot start watering");
            } else {
                ESP_LOGI(TAG, "Starting watering");
                gpio_set_level(WATERING_PIN, 1);

                // Water until duration elapses or manual stop
                TickType_t start = xTaskGetTickCount();
                TickType_t duration_ticks = pdMS_TO_TICKS(WATERING_DURATION_MS);
                while ((xTaskGetTickCount() - start) < duration_ticks && water_pump) {
                    vTaskDelay(pdMS_TO_TICKS(WATERING_CHECK_MS));
                }

                // Determine if aborted or completed
                TickType_t elapsed = xTaskGetTickCount() - start;
                bool aborted = (elapsed < duration_ticks) && !water_pump;

                // Stop watering
                gpio_set_level(WATERING_PIN, 0);
                water_pump = false;
                last_stop_tick = xTaskGetTickCount();
                ESP_LOGI(TAG, "Watering %s", aborted ? "aborted" : "complete");
            }
        }
        // Sleep briefly when idle or during cooldown
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to periodically publish sensor values
static void sensor_publish_task(void *pv)
{
    char payload[64];
    while (1) {
        // Publish soil humidity
        int len = snprintf(payload, sizeof(payload), "%d", soil_humidity);
        esp_mqtt_client_publish(mqtt_client, "espot/humidity", payload, len, 1, 1);

        // Publish ambient light
        len = snprintf(payload, sizeof(payload), "%d", ambient_light);
        esp_mqtt_client_publish(mqtt_client, "espot/ambient", payload, len, 1, 1);

        ESP_LOGI(TAG, "Published humidity=%d, ambient=%d", soil_humidity, ambient_light);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_REFRESH_MS));
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESPot...");

    // 0. Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated or version mismatch—erase and re‑init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized!");

    ESP_LOGI(TAG, "Initializing I2C...");
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized!");
    
    ESP_LOGI(TAG, "Initializing GPIO...");
    gpio_init();
    ESP_LOGI(TAG, "GPIO initialized!");    

    ESP_LOGI(TAG, "Initializing Wi-Fi...");
    wifi_init_sta();
    ESP_LOGI(TAG, "Wi-Fi initialized!");

    ESP_LOGI(TAG, "Starting MQTT publisher...");
    mqtt_app_start();
    ESP_LOGI(TAG, "MQTT publisher started!");

    xTaskCreate(
        sensor_publish_task,
        "sensor_publish_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );


    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        lcd_update_values,
        "lcd_update_values",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        0
    );

    xTaskCreate(
        lights_task,
        "lights_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    xTaskCreate(
        watering_task,
        "watering_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

}
