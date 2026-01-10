#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_http_client.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/touch_pad.h"

// New include to satisfy ESP-IDF v5.x hint about esp_mac functions (keeps compatibility)
#include "esp_mac.h"

// cJSON for parsing server JSON response (add dependency in CMakeLists)
#include "cJSON.h"

/* ===== Wi-Fi config ===== */
#define WIFI_SSID "RTBI-GOK"
#define WIFI_PASS "rtbi@gok2019"

/* ===== Server config ===== */
#define CONFIG_SERVER_URI "http://192.168.1.103:8000/upload"

/* ===== Audio / I2S config ===== */
#define AUDIO_SAMPLE_RATE (16000) // Hz
#define AUDIO_BITS (32)           // 32-bit samples (INMP441 -> 24-bit in 32-bit frame)
#define AUDIO_CHANNELS (1)        // mono

#define RECORD_TIME_MS (10000) // 10 seconds

/* ===== Firebase config ===== */
#define FIREBASE_DB_URL "https://esp-mit-default-rtdb.firebaseio.com/toggleswitch"

#define FB_LIGHT_1 "motor"
#define FB_LIGHT_2 "fan"
#define FB_LIGHT_3 "IOT_socket"
#define FB_LIGHT_4 "light"

// I2S pins (ESP32-S3)
#define I2S_WS_GPIO 15  // LRCLK / WS
#define I2S_SCK_GPIO 14 // BCLK
#define I2S_SD_GPIO 16  // DATA IN
#define I2S_PORT I2S_NUM_0

// Button pin
#define BUTTON_GPIO 36 // GPIO36 as input

// Relay pin (copied from main.c)
#define RELAY_1 GPIO_NUM_10

/* ===== Wi-Fi event handling ===== */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define WIFI_MAX_RETRY 5

///////////////////////////////////////user defines
#define I2S_WS_GPIO GPIO_NUM_15  // LRCLK / WS
#define I2S_SCK_GPIO GPIO_NUM_14 // BCLK
#define I2S_SD_GPIO GPIO_NUM_16  // DATA IN
#define I2S_PORT I2S_NUM_0

// Button pin
#define MIC_BUTTON_GPIO GPIO_NUM_36 // GPIO36 as input

// Relay pin definitions
#define RELAY_1 GPIO_NUM_10
#define RELAY_2 GPIO_NUM_11
#define RELAY_3 GPIO_NUM_12
#define RELAY_4 GPIO_NUM_13

#define TOUCH1_gpio1 TOUCH_PAD_NUM1
#define TOUCH2_gpio2 TOUCH_PAD_NUM2
#define TOUCH3_gpio4 TOUCH_PAD_NUM4
#define TOUCH2_gpio5 TOUCH_PAD_NUM5
#define TOUCH_CHANGE_CONFIG 0

#define TOUCH_THRESHOLD 60000

#define fb_water_S1 "water/S1"
#define fb_water_S2 "water/S2"
#define fb_water_S3 "water/S3"
#define fb_water_B1 "water/B1"
#define fb_water_B2 "water/B2"
#define fb_water_B3 "water/B3"

#define fb_motor_status "motor/status"

////////////////////////////////////////////////////////////////////////////////
/* ------------------- GPIO DEFINITIONS ------------------- */
// INPUTS
#define syntax_low_in17 GPIO_NUM_17 // Tank FULL sensor (S1)
#define syntax_mid_in18 GPIO_NUM_18
#define syntax_high_in8 GPIO_NUM_8

#define tank_low_in21 GPIO_NUM_21
#define tank_mid_in46 GPIO_NUM_46
#define tank_high_in9 GPIO_NUM_9 // Sump EMPTY sensor (B3)

// INPUT MASK
#define INPUT_MASK ((1ULL << syntax_low_in17) | (1ULL << syntax_mid_in18) | \
                    (1ULL << syntax_high_in8) | (1ULL << tank_low_in21) |   \
                    (1ULL << tank_mid_in46) | (1ULL << tank_high_in9))

// OUTPUTS
#define syntax_low_led_g35 GPIO_NUM_35
#define syntax_mid_led_g42 GPIO_NUM_42
#define syntax_high_led_37 GPIO_NUM_37

#define tank_low_led_g38 GPIO_NUM_38
#define tank_mid_led_g39 GPIO_NUM_39
#define tank_high_led_g40 GPIO_NUM_40

#define motor_led GPIO_NUM_41

/// OUTPUT MASK
#define WATER_LED_OUTPUT_MASK ((1ULL << syntax_low_led_g35) | (1ULL << syntax_mid_led_g42) | \
                               (1ULL << syntax_high_led_37) | (1ULL << tank_low_led_g38) |   \
                               (1ULL << tank_mid_led_g39) | (1ULL << tank_high_led_g40) |    \
                               (1ULL << motor_led))
////////////////////////////////////////////////////////////////////////////////////////////

/* ===== LDR + PIR ===== */
#define LDR_ADC_CHANNEL ADC_CHANNEL_6 // GPIO7
#define PIR_PIN GPIO_NUM_3            // GPIO3 input
#define LDR_DARK_THRESHOLD 50         // adjust if needed

#define enable_firebase 1
#define enable_pir 1
#define enable_touch 1
#define enable_relay 1
#define enable_microphone 1
#define enable_motor 1

/* ===== GLOBALS ===== */

static const char *TAG = "INMP441_HTTP_RAW";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
i2s_chan_handle_t rx_chan = NULL;

//////////////////////////////bools and ints

static volatile bool s_is_listening = false;

static volatile bool user_toggled_relay_1 = false;
static volatile bool user_toggled_relay_2 = false;
static volatile bool user_toggled_relay_3 = false;
static volatile bool user_toggled_relay_4 = false;

static volatile bool relay_1_state = false;
static volatile bool relay_2_state = false;
static volatile bool relay_3_state = false;
static volatile bool relay_4_state = false;
static volatile bool relay_4_state_motor = false;

static volatile bool prv_relay_1_state = false;
static volatile bool prv_relay_2_state = false;
static volatile bool prv_relay_3_state = false;
static volatile bool prv_relay_4_state = false;

static volatile bool pre_relay_1_state = false;
static volatile bool pre_relay_2_state = false;
static volatile bool pre_relay_3_state = false;
static volatile bool pre_relay_4_state = false;

static int prev_S1 = -1;
static int prev_S2 = -1;
static int prev_S3 = -1;
static int prev_B1 = -1;
static int prev_B2 = -1;
static int prev_B3 = -1;
static bool prev_motor_state = false;

static volatile bool motor_manual_mode_state = false;
static volatile bool fb_motor_manual_mode_state_change = false;

static bool S1_change = false;
static bool S2_change = false;
static bool S3_change = false;
static bool B1_change = false;
static bool B2_change = false;
static bool B3_change = false;
static bool motor_state_change = false;

static volatile bool relay_1_temp = false;
static volatile bool relay_2_temp = false;
static volatile bool relay_3_temp = false;
static volatile bool relay_4_temp = false;

static volatile bool fb_r1_change_signal = false;
static volatile bool fb_r2_change_signal = false;
static volatile bool fb_r3_change_signal = false;
static volatile bool fb_r4_change_signal = false;

static volatile bool is_wifi_connected = false;

static adc_oneshot_unit_handle_t adc1_handle;

volatile int s1_water_syntax_low_in17 = 0;
volatile int s2_water_syntax_mid_in18 = 0;
volatile int s3_water_syntax_high_in8 = 0;
volatile int b1_water_tank_low_in21 = 0;
volatile int b2_water_tank_mid_in46 = 0;
volatile int b3_water_tank_high_in9 = 0;

static volatile bool motor_can_turn_on = false;
static volatile bool motor_must_turn_on = false;

/* -------- PIR TIMER STATE -------- */
static uint64_t pir_relay_expiry_time = 0;
static bool pir_timer_active = false;
static bool pir_owned_relay = false;
static bool pir_check_allowed = true;
static bool pir_extension_checked = false;

bool prev_touch_1 = false;
bool prev_touch_2 = false;
bool prev_touch_3 = false;
bool prev_touch_4 = false;

uint32_t pir_start_time = 0;
uint32_t pir_duration = 10000; // 10 seconds

uint32_t millis()
{
    return (esp_timer_get_time() / 1000);
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Retrying to connect to the AP...");
            is_wifi_connected = false;
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        is_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler,
                                               NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init STA finished. Connecting...");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected to AP: %s", WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Failed to connect to SSID %s", WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        ESP_LOGE(TAG, "Failed to connect to SSID %s", WIFI_SSID);
        vTaskDelay(100);
    }
}

/* ===== I2S (INMP441) init ===== */
static i2s_chan_handle_t init_i2s_rx(void)
{
    ESP_LOGI(TAG, "Initializing I2S RX for INMP441...");

    i2s_chan_handle_t rx_chan;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT, // 32-bit frame
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO, // mono
            .slot_mask = I2S_STD_SLOT_LEFT,  // capture LEFT channel
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
#if !CONFIG_IDF_TARGET_ESP32 && !CONFIG_IDF_TARGET_ESP32S2
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
#endif
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    ESP_LOGI(TAG, "I2S RX channel enabled.");
    return rx_chan;
}

/* ===== Button init (GPIO36 input, active-low) ===== */
static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "Button on GPIO%d initialized (active-low)", BUTTON_GPIO);
}

/* ===== HTTP streaming (chunked) ===== */
static esp_err_t http_stream_audio(i2s_chan_handle_t rx_chan, int duration_ms)
{
    esp_http_client_config_t config = {
        .url = CONFIG_SERVER_URI,
        .timeout_ms = 15000, // increased timeout for more stable uploads
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
    {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_http_client_set_method(client, HTTP_METHOD_POST));
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "Transfer-Encoding", "chunked"));

    char header[32];

    // Audio metadata headers (used by Python server)
    snprintf(header, sizeof(header), "%d", AUDIO_SAMPLE_RATE);
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "x-audio-sample-rates", header));

    snprintf(header, sizeof(header), "%d", AUDIO_BITS);
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "x-audio-bits", header));

    snprintf(header, sizeof(header), "%d", AUDIO_CHANNELS);
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "x-audio-channel", header));

    ESP_LOGI(TAG, "Opening HTTP connection to %s", CONFIG_SERVER_URI);

    // content_length = -1 for chunked
    esp_err_t err = esp_http_client_open(client, -1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    // Small delay to allow TCP + HTTP initialization / ACKs to settle.
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "HTTP connection opened, start streaming audio...");

    const int bytes_per_sample = AUDIO_BITS / 8; // 4 for 32-bit
    const int samples_per_chunk = 1024;
    const int buf_size = samples_per_chunk * bytes_per_sample;
    uint8_t *buf = (uint8_t *)malloc(buf_size);
    if (!buf)
    {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    int64_t start_us = esp_timer_get_time();
    int64_t end_us = start_us + (int64_t)duration_ms * 1000;

    size_t total_bytes_sent = 0;

    while ((esp_timer_get_time() < end_us))
    {
        size_t bytes_read = 0;
        err = i2s_channel_read(rx_chan, buf, buf_size, &bytes_read, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_read error: %s", esp_err_to_name(err));
            break;
        }
        if (bytes_read == 0)
        {
            continue;
        }

        // Write chunk header: "<hex_len>\r\n"
        int wlen = snprintf(header, sizeof(header), "%x\r\n", (unsigned int)bytes_read);
        int tries = 0;
        int written = 0;
        while (tries < 3)
        {
            written = esp_http_client_write(client, header, wlen);
            if (written > 0)
                break;
            ESP_LOGW(TAG, "Header write failed, retrying... try=%d", tries + 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            tries++;
        }
        if (written <= 0)
        {
            ESP_LOGE(TAG, "HTTP write chunk header failed");
            err = ESP_FAIL;
            break;
        }

        // Write chunk data (may need a retry)
        tries = 0;
        written = 0;
        while (tries < 3)
        {
            written = esp_http_client_write(client, (const char *)buf, bytes_read);
            if (written == (int)bytes_read)
                break;
            if (written > 0)
            {
                // partial write - adjust pointer and remaining bytes
                size_t remaining = bytes_read - (size_t)written;
                ESP_LOGW(TAG, "Partial write (%d), remaining %d, retrying...", written, (int)remaining);

                // Try to send remaining part
                int w2 = esp_http_client_write(client, (const char *)(buf + written), remaining);
                if (w2 == (int)remaining)
                {
                    written = bytes_read;
                    break;
                }
            }
            ESP_LOGW(TAG, "Chunk data write failed, retrying... try=%d", tries + 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            tries++;
        }
        if (written <= 0 || written != (int)bytes_read)
        {
            ESP_LOGE(TAG, "HTTP write chunk data failed (written=%d, expected=%d)", written, (int)bytes_read);
            err = ESP_FAIL;
            break;
        }

        // Chunk terminator "\r\n" with retry
        tries = 0;
        written = 0;
        while (tries < 3)
        {
            written = esp_http_client_write(client, "\r\n", 2);
            if (written > 0)
                break;
            ESP_LOGW(TAG, "Write chunk terminator failed, retrying... try=%d", tries + 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            tries++;
        }
        if (written <= 0)
        {
            ESP_LOGE(TAG, "HTTP write chunk terminator failed");
            err = ESP_FAIL;
            break;
        }

        total_bytes_sent += bytes_read;
        ESP_LOGI(TAG, "Total bytes sent: %d", (int)total_bytes_sent);
    }

    // Send zero-length chunk to finish: "0\r\n\r\n"
    if (err == ESP_OK)
    {
        if (esp_http_client_write(client, "0\r\n\r\n", 5) <= 0)
        {
            ESP_LOGE(TAG, "Failed to send final zero chunk");
            err = ESP_FAIL;
        }
    }

    // Read response safely (do not use esp_http_client_flush_response())
    if (err == ESP_OK)
    {
        int hdr_len = esp_http_client_fetch_headers(client);
        ESP_LOGI(TAG, "Header length: %d", hdr_len);

        // Give server a small moment to produce body if it's slightly delayed.
        vTaskDelay(pdMS_TO_TICKS(50));

        char resp[1024];
        int rlen = esp_http_client_read(client, resp, sizeof(resp) - 1);

        if (rlen > 0)
        {
            resp[rlen] = 0;
            ESP_LOGI(TAG, "Server says: %s", resp);

            /* ---------- NEW: parse JSON and handle 'transcript' ---------- */
            cJSON *root = cJSON_Parse(resp);
            if (!root)
            {
                ESP_LOGE(TAG, "Failed to parse server JSON response");
            }
            else
            {
                cJSON *trans = cJSON_GetObjectItem(root, "transcript");
                if (trans && cJSON_IsString(trans))
                {
                    const char *transcript = trans->valuestring;
                    ESP_LOGW(TAG, "Transcript: %s", transcript);

                    if (strstr(transcript, "light 1 on"))
                    {
                        if (user_toggled_relay_4 == false)
                        {
                            relay_4_state = true;
                            fb_r4_change_signal = true;
                            user_toggled_relay_4 = true;
                        }
                    }
                    else if (strstr(transcript, "light 1 off"))
                    {
                        relay_4_state = false;
                        fb_r4_change_signal = true;
                        user_toggled_relay_4 = false;
                    }
                    else if (strstr(transcript, "fan on"))
                    {
                        // else if (strstr(transcript, "light 2 on"))
                        if (relay_2_state == false)
                        {
                            relay_2_state = true;
                            fb_r2_change_signal = true;
                        }
                    }
                    else if (strstr(transcript, "fan off"))
                    {
                        // else if (strstr(transcript, "light 2 off"))
                        if (relay_2_state == true)
                        {
                            relay_2_state = false;
                            fb_r2_change_signal = true;
                        }
                    }
                    else if (strstr(transcript, "socket on"))
                    {
                        // else if (strstr(transcript, "light 3 on"))
                        if (relay_3_state == false)
                        {
                            relay_3_state = true;
                            fb_r3_change_signal = true;
                        }
                    }
                    else if (strstr(transcript, "socket off"))
                    {
                        // else if (strstr(transcript, "light 3 off"))
                        if (relay_3_state == true)
                        {
                            relay_3_state = false;
                            fb_r3_change_signal = true;
                        }
                    }
                    else if (strstr(transcript, "motor on"))
                    {
                        // else if (strstr(transcript, "light 4 on"))
                        if (motor_manual_mode_state == false)
                        {
                            motor_manual_mode_state = true;
                            fb_r1_change_signal = true;
                        }
                    }
                    else if (strstr(transcript, "motor off"))
                    {
                        // else if (strstr(transcript, "light 4 off"))
                        if (motor_manual_mode_state == true)
                        {
                            motor_manual_mode_state = false;
                            fb_r1_change_signal = true;
                        }
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "No 'transcript' field in JSON or not a string");
                }
                cJSON_Delete(root);
            }
            /* ---------- end JSON handling ---------- */
        }
        else
        {
            ESP_LOGW(TAG, "No HTTP response body or read error (rlen=%d)", rlen);
        }
    }

    free(buf);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "HTTP streaming finished, sent %d bytes", (int)total_bytes_sent);
    return err;
}

esp_err_t start_listening(i2s_chan_handle_t rx_chan, int duration_ms)
{
    if (s_is_listening)
    {
        ESP_LOGW(TAG, "Already listening; ignoring start_listening()");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "start_listening(): begin streaming");
    s_is_listening = true;

    esp_err_t err = http_stream_audio(rx_chan, duration_ms);

    s_is_listening = false;
    ESP_LOGI(TAG, "start_listening(): streaming finished");
    return err;
}

////////////////////microphone logic///////////////////
void microphone_logic(void)
{
    // Placeholder for any microphone-specific logic if needed
    // Currently, all logic is handled in start_listening()
    int level = gpio_get_level(BUTTON_GPIO);
    ESP_LOGI(TAG, "Waiting for Button pressed on GPIO%d to starting listening...", BUTTON_GPIO);

    if (level == 0)
    {
        ESP_LOGI(TAG, "Button pressed on GPIO%d, starting listening...", BUTTON_GPIO);
        // Start listening for RECORD_TIME_MS
        start_listening(rx_chan, RECORD_TIME_MS);
        ESP_LOGI(TAG, "Listening session complete");
    }
}

static void build_firebase_url(char *out_url, size_t len, const char *path)
{
    snprintf(out_url, len, "%s/%s.json", FIREBASE_DB_URL, path);
}

static esp_err_t gget_http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP connected");
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP data received (%d bytes)", evt->data_len);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static esp_err_t g1_http_event_handler(esp_http_client_event_t *evt)
{
    return ESP_OK;
}

/* Generic Firebase PUT function for any path (light1, light2, etc.) */
esp_err_t firebase_set_value(const char *thisPath, const char *value)
{
    char url[256];

    if (thisPath == NULL || strlen(thisPath) == 0)
    {
        // full object update
        snprintf(url, sizeof(url), "%s.json", FIREBASE_DB_URL);
        ESP_LOGI(TAG, "the url is, main url: %s", (char *)url);
    }
    else
    {
        build_firebase_url(url, sizeof(url), thisPath);
        ESP_LOGI(TAG, "the url is: %s", (char *)url);
    }

    ESP_LOGI(TAG, "Firebase PUT URL: %s", url);

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = g1_http_event_handler,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .timeout_ms = 20000,
        .keep_alive_enable = false};

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
        return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_method(client, HTTP_METHOD_PUT);

    /* value is already JSON */
    esp_http_client_set_post_field(client, value, strlen(value));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "PUT %s -> %s (HTTP %d)", thisPath, value, status); /// her the thisp ath is the payload
    }
    else
    {
        ESP_LOGE(TAG, "HTTP PUT failed for %s: %s", thisPath, esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

/* ---- NEW robust: Fetch the full toggleswitch JSON in one request ---- */
esp_err_t firebase_get_full_json(char *response, size_t max_len)
{
    const char *full_url = FIREBASE_DB_URL ".json";
    esp_http_client_config_t config = {
        .url = full_url,
        .event_handler = g1_http_event_handler,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .timeout_ms = 20000,
        .keep_alive_enable = false};

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
    {
        ESP_LOGE(TAG, "esp_http_client_init failed (full json)");
        return ESP_FAIL;
    }

    esp_http_client_set_method(client, HTTP_METHOD_GET);

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        esp_http_client_cleanup(client);
        return err;
    }

    int status = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_fetch_headers(client);
    (void)content_length; // maybe -1 for chunked; ignore

    int total_read = 0;
    while (total_read < (int)max_len - 1)
    {
        int r = esp_http_client_read(client, response + total_read, max_len - total_read - 1);
        if (r < 0)
        {
            break;
        }
        else if (r == 0)
        {
            break;
        }
        else
        {
            total_read += r;
        }
    }
    response[total_read] = '\0';

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    return ESP_OK;
}

void fb_motorSensorChangeUpdate()
{
    // SENSOR CHANGE FLAGS
    if (s1_water_syntax_low_in17 != prev_S1)
    {
        prev_S1 = s1_water_syntax_low_in17;
        S1_change = true;
    }
    if (s2_water_syntax_mid_in18 != prev_S2)
    {
        prev_S2 = s2_water_syntax_mid_in18;
        S2_change = true;
    }
    if (s3_water_syntax_high_in8 != prev_S3)
    {
        prev_S3 = s3_water_syntax_high_in8;
        S3_change = true;
    }
    if (b1_water_tank_low_in21 != prev_B1)
    {
        prev_B1 = b1_water_tank_low_in21;
        B1_change = true;
    }
    if (b2_water_tank_mid_in46 != prev_B2)
    {
        prev_B2 = b2_water_tank_mid_in46;
        B2_change = true;
    }
    if (b3_water_tank_high_in9 != prev_B3)
    {
        prev_B3 = b3_water_tank_high_in9;
        B3_change = true;
    }
}

static void ak_motor_logic(void)
{
    // READ INPUTS
    s3_water_syntax_high_in8 = gpio_get_level(syntax_high_in8);
    s2_water_syntax_mid_in18 = gpio_get_level(syntax_mid_in18);
    s1_water_syntax_low_in17 = gpio_get_level(syntax_low_in17);

    b3_water_tank_high_in9 = gpio_get_level(tank_high_in9);
    b2_water_tank_mid_in46 = gpio_get_level(tank_mid_in46);
    b1_water_tank_low_in21 = gpio_get_level(tank_low_in21);

    ESP_LOGI(TAG, "S1 low g17:%d S2 mid g18:%d S3 high g8:%d", s1_water_syntax_low_in17, s2_water_syntax_mid_in18, s3_water_syntax_high_in8);
    ESP_LOGI(TAG, "B1 low g21:%d B2 mid g46:%d B3 high g9:%d", b1_water_tank_low_in21, b2_water_tank_mid_in46, b3_water_tank_high_in9);

    // INIT OUTPUTS
    gpio_set_level(syntax_low_led_g35, s1_water_syntax_low_in17);
    gpio_set_level(syntax_mid_led_g42, s2_water_syntax_mid_in18);
    gpio_set_level(syntax_high_led_37, s3_water_syntax_high_in8);
    gpio_set_level(tank_low_led_g38, b1_water_tank_low_in21);
    gpio_set_level(tank_mid_led_g39, b2_water_tank_mid_in46);
    gpio_set_level(tank_high_led_g40, b3_water_tank_high_in9);

    fb_motorSensorChangeUpdate();

    // MOTOR DECISION
    if (s3_water_syntax_high_in8 == 0 || b1_water_tank_low_in21 == 1)
    {
        motor_must_turn_on = false;
        motor_can_turn_on = false;

        relay_1_state = false;
        fb_r1_change_signal = true;
        gpio_set_level(motor_led, 1); // OFF (inverted)

        if (motor_manual_mode_state == true)
        {
            ESP_LOGW(TAG, "Auto mode override: Sensor indicates motor must be OFF, switching to AUTO mode");
            motor_manual_mode_state = false; // reset to auto mode
            fb_motor_manual_mode_state_change = true;
        }
        ESP_LOGI(TAG, "Motor MUST be OFF due to sensor readings @923");
        return;
    }
    else if (s3_water_syntax_high_in8 == 1 && b1_water_tank_low_in21 == 0)
    {
        motor_can_turn_on = true;
    }

    // MOTOR DECISION
    if (s3_water_syntax_high_in8 == 1 && s2_water_syntax_mid_in18 == 1 && b1_water_tank_low_in21 == 0)
    {
        motor_must_turn_on = true;
    }

    // Motor CONTROL (NO GPIO)
    if (motor_manual_mode_state == true)
    {
        // MANUAL MODE
        if (motor_can_turn_on)
        {
            if (relay_1_state == false)
            {
                relay_1_state = true;
                fb_r1_change_signal = true;
                gpio_set_level(motor_led, 0); // ON (inverted)
            }
            else
            {
                // already ON
            }
        }
        else
        {
            if (relay_1_state == true)
            {
                relay_1_state = false;
                fb_r1_change_signal = true;
                gpio_set_level(motor_led, 1); // OFF (inverted)
            }
            else
            {
                // already OFF
            }
        }
    }
    else
    {
        // AUTO MODE
        if (motor_must_turn_on)
        {
            if (relay_1_state == false)
            {
                relay_1_state = true;
                fb_r1_change_signal = true;
                gpio_set_level(motor_led, 0); // ON (inverted)
            }
            else
            {
                // already ON
            }
        }
        else
        {
            // MOTOR DECISION
            if (s3_water_syntax_high_in8 == 0 || b1_water_tank_low_in21 == 1)
            {
                motor_must_turn_on = false;
                motor_can_turn_on = false;

                relay_1_state = false;
                fb_r1_change_signal = true;
                gpio_set_level(motor_led, 1); // OFF (inverted)

                if (motor_manual_mode_state == true)
                {
                    ESP_LOGW(TAG, "Auto mode override: Sensor indicates motor must be OFF, switching to AUTO mode");
                    motor_manual_mode_state = false; // reset to auto mode
                    fb_motor_manual_mode_state_change = true;
                }
                ESP_LOGI(TAG, "Motor MUST be OFF due to sensor readings @1002");
            }
        }
    }
}

/////////////////////////touch-logic////////////////
static void touch_logic(void)
{
    // static uint32_t last_touch_scan = 0;

    // // Run touch scan every 30 ms (non-blocking)
    // if (millis() - last_touch_scan < 30)
    //     return;

    // last_touch_scan = millis();

    uint32_t t1 = 0;
    uint32_t t2 = 0;
    uint32_t t3 = 0;
    uint32_t t4 = 0;
    touch_pad_read_raw_data(TOUCH1_gpio1, &t1);
    touch_pad_read_raw_data(TOUCH2_gpio2, &t2);
    touch_pad_read_raw_data(TOUCH3_gpio4, &t3);
    touch_pad_read_raw_data(TOUCH2_gpio5, &t4);

    ESP_LOGW(TAG, "Touch readings: T1=%lu, T2=%" PRIu32 ", T3=%" PRIu32 ", T4=%" PRIu32 "", t1, t2, t3, t4);

    // ---- TOUCH 1 ----
    bool curr4 = (t4 > TOUCH_THRESHOLD);
    if (curr4 && !prev_touch_4)
    {
        relay_4_state = !relay_4_state;
        if (relay_4_state == true)
        {
            ESP_LOGW(TAG, "User turned ON relay 4 (bulb) via touch");
            user_toggled_relay_4 = true;
        }
        else
        {
            ESP_LOGW(TAG, "User turned OFF relay 4 (bulb) via touch");
            user_toggled_relay_4 = false;
        }
        fb_r4_change_signal = true;
    }
    prev_touch_4 = curr4;

    // ---- TOUCH 2 ----
    bool curr2 = (t2 > TOUCH_THRESHOLD);
    if (curr2 && !prev_touch_2)
    {
        relay_2_state = !relay_2_state;
        fb_r2_change_signal = true;
    }
    prev_touch_2 = curr2;

    // ---- TOUCH 3 ----
    bool curr3 = (t3 > TOUCH_THRESHOLD);
    if (curr3 && !prev_touch_3)
    {
        relay_3_state = !relay_3_state;
        fb_r3_change_signal = true;
    }
    prev_touch_3 = curr3;

    // ---- TOUCH 4 (Motor) ----
    bool curr1 = (t1 > TOUCH_THRESHOLD);
    if (curr1 && !prev_touch_1)
    {
        motor_manual_mode_state = !motor_manual_mode_state;
        // ESP_LOGW(TAG, "Motor auto mode toggled to: %s via touch", motor_manual_mode_state ? "ON" : "OFF");
        ESP_LOGW(TAG, "User toggled motor mode to: %s via touch", motor_manual_mode_state ? "MANUAL" : "AUTO");
        fb_r1_change_signal = true;
    }
    prev_touch_1 = curr1;
}

///////////////////////adc function///////////////////
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle,
                                               LDR_ADC_CHANNEL,
                                               &config));
}

static void ak_pir_ldr_logic(void)
{
    // If relay is manually controlled, PIR must not interfere
    if (user_toggled_relay_4 == true)
    {
        //////reset timers to deafualts
        ESP_LOGI(TAG, "PIR skipped due to manual override");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "PIR checking...");
        int ldr = 0;
        adc_oneshot_read(adc1_handle, LDR_ADC_CHANNEL, &ldr);
        int pir = gpio_get_level(PIR_PIN);

        ESP_LOGI(TAG, "PIR level: %d, LDR level: %d", pir, ldr);

        /* -------- AUTO ON (PIR + LDR) -------- */
        if (pir == 1 && (ldr < LDR_DARK_THRESHOLD))
        {
            gpio_set_level(RELAY_4, 1);
            pir_start_time = millis();
            ESP_LOGI(TAG, "PIR detected motion, relay 4 ON, startTime: %" PRIu32, pir_start_time);
        }
        else
        {
            ESP_LOGI(TAG, "PIR no motion or LDR too bright");
        }

        if (user_toggled_relay_4 == true)
        {
            ESP_LOGI(TAG, "PIR skipped due to manual override");
            return; // exit if relay is off
        }
        /* -------- 9TH SECOND EXTENSION CHECK -------- */
        if (((millis() - pir_start_time) >= 9000) && ((millis() - pir_start_time) < 10000))
        {
            if (pir == 1)
            {
                pir_start_time = millis();
                ESP_LOGI(TAG, "PIR detected motion at 9th second, relay 4 extended, new startTime: %" PRIu32, pir_start_time);
            }
            else
            {
                ESP_LOGI(TAG, "PIR no motion at 9th second, relay 4 not extended, ending at: %" PRIu32, pir_start_time + 10000);
            }
        }

        /* -------- AUTO OFF -------- */
        if ((millis() - pir_start_time) >= 10000)
        {
            gpio_set_level(RELAY_4, 0);
            ESP_LOGI(TAG, "PIR timer expired, relay 4 OFF, current time: %" PRIu32, millis());
        }
    }
}

//////////////////////realy and led set /////////////////////////
static void apply_relay_and_leds(void)
{

    if (relay_1_state != pre_relay_1_state)
        gpio_set_level(RELAY_1, relay_1_state);
    if (relay_2_state != pre_relay_2_state)
        gpio_set_level(RELAY_2, relay_2_state);
    if (relay_3_state != pre_relay_3_state)
        gpio_set_level(RELAY_3, relay_3_state);
    if (relay_4_state != pre_relay_4_state)
        gpio_set_level(RELAY_4, relay_4_state);

    gpio_set_level(motor_led, relay_1_state ? 0 : 1);

    pre_relay_1_state = relay_1_state;
    pre_relay_2_state = relay_2_state;
    pre_relay_3_state = relay_3_state;
    pre_relay_4_state = relay_4_state;
    ESP_LOGI(TAG, "Applied relay states: R1=%d, R2=%d, R3=%d, R4=%d",
             relay_1_state,
             relay_2_state,
             relay_3_state,
             relay_4_state);
}

bool extract_boolean(const char *json, const char *key)
{
    char on1[64], on2[64];
    char off1[64], off2[64];

    snprintf(on1, sizeof(on1), "\"%s\":\"ON\"", key);
    snprintf(on2, sizeof(on2), "\"%s\":\"\\\"ON\\\"\"", key);

    snprintf(off1, sizeof(off1), "\"%s\":\"OFF\"", key);
    snprintf(off2, sizeof(off2), "\"%s\":\"\\\"OFF\\\"\"", key);

    if (strstr(json, on1) || strstr(json, on2))
        return true;

    if (strstr(json, off1) || strstr(json, off2))
        return false;

    return false;
}

////////////////////////Read firbase//////////////////////
static void read_firebase_and_sync(void)
{
    if (!is_wifi_connected && !enable_firebase)
    {
        ESP_LOGI(TAG, "Skipping Firebase GET (Wi-Fi connected & Firebase enabled)");
        return;
    }

    char full_json[1024] = {0};

    if (firebase_get_full_json(full_json, sizeof(full_json)) != ESP_OK)
    {
        ESP_LOGI(TAG, "No Internet. Please turn on mobile data.");
        return;
    }

    bool fb1 = extract_boolean(full_json, FB_LIGHT_1);
    bool fb2 = extract_boolean(full_json, FB_LIGHT_2);
    bool fb3 = extract_boolean(full_json, FB_LIGHT_3);
    bool fb4 = extract_boolean(full_json, FB_LIGHT_4);

    ESP_LOGI(TAG, "Firebase states: R1=%s, R2=%s, R3=%s, R4=%s",
             fb1 ? "ON" : "OFF",
             fb2 ? "ON" : "OFF",
             fb3 ? "ON" : "OFF",
             fb4 ? "ON" : "OFF");

    if (fb4 != relay_4_state)
    {
        relay_4_state = fb4;
        if (fb4 == true)
        {
            user_toggled_relay_4 = true;
        }
        else
        {
            user_toggled_relay_4 = false;
        }
    }

    if (fb2 != relay_2_state)
    {
        relay_2_state = fb2;
    }

    if (fb3 != relay_3_state)
    {
        relay_3_state = fb3;
    }

    if (fb1 != relay_1_state)
    {
        motor_manual_mode_state = fb1;
    }
}

/////////////////////////SET_FIREBASE_FUNCTION///////////////////////////
static void ak_update_firebase_if_changed(void)
{
    if (!is_wifi_connected && !enable_firebase)
        return;

    bool any_change =
        fb_r1_change_signal || fb_r2_change_signal || fb_r3_change_signal || fb_r4_change_signal ||
        S1_change ||
        S2_change ||
        S3_change ||
        B1_change ||
        B2_change ||
        B3_change ||
        fb_motor_manual_mode_state_change ||
        motor_state_change;

    if (!any_change)
    {
        return;
    }
    else
    {

        char payload[512];
        snprintf(payload, sizeof(payload),
                 "{"
                 "\"%s\":\"%s\","
                 "\"%s\":\"%s\","
                 "\"%s\":\"%s\","
                 "\"%s\":\"%s\","

                 "\"water\":{"
                 "\"S1\":\"%s\","
                 "\"S2\":\"%s\","
                 "\"S3\":\"%s\","
                 "\"B1\":\"%s\","
                 "\"B2\":\"%s\","
                 "\"B3\":\"%s\""
                 "},"

                 "\"motor\":{"
                 "\"status\":\"%s\""
                 "}"
                 "}",
                 FB_LIGHT_1, motor_manual_mode_state ? "ON" : "OFF",
                 FB_LIGHT_2, relay_2_state ? "ON" : "OFF",
                 FB_LIGHT_3, relay_3_state ? "ON" : "OFF",
                 FB_LIGHT_4, relay_4_state ? "ON" : "OFF",

                 s1_water_syntax_low_in17 ? "0" : "1",
                 s2_water_syntax_mid_in18 ? "0" : "1",
                 s3_water_syntax_high_in8 ? "0" : "1",
                 b1_water_tank_low_in21 ? "0" : "1",
                 b2_water_tank_mid_in46 ? "0" : "1",
                 b3_water_tank_high_in9 ? "0" : "1",

                 relay_1_state ? "ON" : "OFF");

        firebase_set_value("", payload);

        if (S1_change)
        {
            S1_change = false;
        }
        if (S2_change)
        {

            S2_change = false;
        }
        if (S3_change)
        {
            S3_change = false;
        }
        if (B1_change)
        {
            B1_change = false;
        }
        if (B2_change)
        {
            B2_change = false;
        }
        if (B3_change)
        {
            B3_change = false;
        }
        if (motor_state_change)
        {
            motor_state_change = false;
        }
        if (fb_r1_change_signal)
        {
            fb_r1_change_signal = false;
        }
        if (fb_r2_change_signal)
        {
            fb_r2_change_signal = false;
        }
        if (fb_r3_change_signal)
        {
            fb_r3_change_signal = false;
        }
        if (fb_r4_change_signal)
        {
            fb_r4_change_signal = false;
        }
        if (fb_motor_manual_mode_state_change)
        {
            fb_motor_manual_mode_state_change = false;
        }
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "==== INMP441 HTTP RAW Stream with Button Start ====");

    // NVS init (for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Init Wi-Fi
    wifi_init_sta();

    // Init I2S
    rx_chan = init_i2s_rx();

    // Init button
    button_init();

    /* Initialize touch pad peripheral. */
    touch_pad_init();
    // for (int i = 0; i < TOUCH_BUTTON_NUM; i++)
    // {
    touch_pad_config(TOUCH1_gpio1);
    touch_pad_config(TOUCH2_gpio2);
    touch_pad_config(TOUCH3_gpio4);
    touch_pad_config(TOUCH2_gpio5);
    // }

    /* Denoise setting at TouchSensor 0. */
    touch_pad_denoise_t denoise = {
        /* The bits to be cancelled are determined according to the noise level. */
        .grade = TOUCH_PAD_DENOISE_BIT4,
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();
    ESP_LOGI(TAG, "Denoise function init");

    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();

    uint32_t touch1_gpio1_value = 0;
    uint32_t touch2_gpio2_value = 0;
    uint32_t touch4_gpio4_value = 0;
    uint32_t touch5_gpio5_value = 0;

    bool touch1_gpio1_state = false;
    bool touch2_gpio2_state = false;
    bool touch4_gpio4_state = false;
    bool touch5_gpio5_state = false;

    /* Wait touch sensor init done */
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("Touch Sensor read, the output format is: \nTouchpad num:[raw data]\n\n");

    ////////////////////////firebase-upload/////////////////////
    char full_json[1024];
    memset(full_json, 0, sizeof(full_json));
    /////////////////////////////////////////////////////////////

    ///////////////// LDR - PIR ////////////////////////////////////////
    /* -------- PIR GPIO SETUP -------- */
    gpio_config_t io_conf_pir = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_PIN),
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // IMPORTANT
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&io_conf_pir);

    /* -------- STATE MEMORY -------- */
    int raw = 0;
    bool ldr_pir_active = false; // REMEMBERS if LDR+PIR turned relay ON

    //////////////////////////////////////////////////////////////////////////

    // INPUT CONFIG
    gpio_config_t input_conf = {
        .pin_bit_mask = INPUT_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&input_conf);

    // OUTPUT CONFIG
    gpio_config_t output_conf = {
        .pin_bit_mask = WATER_LED_OUTPUT_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&output_conf);

    // INIT OUTPUTS
    gpio_set_level(syntax_low_led_g35, 1);
    gpio_set_level(syntax_mid_led_g42, 1);
    gpio_set_level(syntax_high_led_37, 1);
    gpio_set_level(tank_low_led_g38, 1);
    gpio_set_level(tank_mid_led_g39, 1);
    gpio_set_level(tank_high_led_g40, 1);
    gpio_set_level(motor_led, 1); // OFF (inverted)

    gpio_reset_pin(RELAY_1);
    gpio_reset_pin(RELAY_2);
    gpio_reset_pin(RELAY_3);
    gpio_reset_pin(RELAY_4);

    gpio_set_direction(RELAY_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_4, GPIO_MODE_OUTPUT);

    // default off
    gpio_set_level(RELAY_1, 0);
    gpio_set_level(RELAY_2, 0);
    gpio_set_level(RELAY_3, 0);
    gpio_set_level(RELAY_4, 0);

    adc_init();

    while (1)
    {

        ESP_LOGI(TAG, "MIC start ");
        microphone_logic();
        ESP_LOGI(TAG, "MIC end");

        ESP_LOGI(TAG, "motor logic start");
        ak_motor_logic();
        ESP_LOGI(TAG, "motor logic end");

        ESP_LOGI(TAG, "touch detection start ");
        touch_logic();
        ESP_LOGI(TAG, "touch detection end");

        ESP_LOGI(TAG, "PIR_LDR start ");
        ak_pir_ldr_logic();
        ESP_LOGI(TAG, "PIR_LDR end");

        ESP_LOGI(TAG, "Relay start ");
        apply_relay_and_leds();
        ESP_LOGI(TAG, "Relay end");

        ///////////////////////lets create a firebase task instead of doing it here///////////////////////
        ESP_LOGI(TAG, "firebase upload start ");
        ak_update_firebase_if_changed();
        ESP_LOGI(TAG, "firebase upload end");

        ESP_LOGI(TAG, "firebase read start ");
        read_firebase_and_sync();
        ESP_LOGI(TAG, "firebase read end  ");

        ESP_LOGI(TAG, "Relay start ");
        apply_relay_and_leds();
        ESP_LOGI(TAG, "Relay end");

        uint32_t startTimez = millis();
        vTaskDelay(50); // wait for 500ms i.e. 500 milli seconds
        uint32_t endTimez = millis();
        ESP_LOGI(TAG, "Loop Time taken: %lu ms", endTimez - startTimez);
    }

    // (Not reached in this example)
    ESP_LOGI(TAG, "Disabling I2S channel...");
    ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(rx_chan));
}
