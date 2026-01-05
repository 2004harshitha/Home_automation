// record_raw_http.c  (updated to parse server JSON 'transcript' and drive relay)
// Based on your original record_raw_http.c with minimal changes (relay + cJSON)

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <ctype.h>

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
#include "nvs_flash.h"

#include "esp_http_client.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <stdbool.h>
#include "esp_timer.h"

// New include to satisfy ESP-IDF v5.x hint about esp_mac functions (keeps compatibility)
#include "esp_mac.h"

// cJSON for parsing server JSON response (add dependency in CMakeLists)
#include "cJSON.h"

static const char *TAG = "INMP441_HTTP_RAW";

/* ===== Wi-Fi config ===== */
#define WIFI_SSID "RTBI-GoK"
#define WIFI_PASS "rtbi@gok2019"

/* ===== Server config ===== */
#define CONFIG_SERVER_URI "http://192.168.1.186:8000/upload"

/* ===== Audio / I2S config ===== */
#define AUDIO_SAMPLE_RATE (16000) // Hz
#define AUDIO_BITS (32)           // 32-bit samples (INMP441 -> 24-bit in 32-bit frame)
#define AUDIO_CHANNELS (1)        // mono

#define RECORD_TIME_MS (10000) // 10 seconds

/* ===== Firebase config ===== */
#define FIREBASE_DB_URL "https://esp-mit-default-rtdb.firebaseio.com/toggleswitch"

// #define FB_LIGHT_1 "light1"
// #define FB_LIGHT_2 "light2"
// #define FB_LIGHT_3 "light3"
// #define FB_LIGHT_4 "light4"

#define FB_LIGHT_1 "appliance1"
#define FB_LIGHT_2 "appliance2"
#define FB_LIGHT_3 "appliance3"
#define FB_LIGHT_4 "appliance4"

///////////////millis//////////////////////

uint32_t millis()
{
    return (esp_timer_get_time() / 1000);
}

////////////////////////////////////////////////

// I2S pins (ESP32-S3)
#define I2S_WS_GPIO GPIO_NUM_15  // LRCLK / WS
#define I2S_SCK_GPIO GPIO_NUM_14 // BCLK
#define I2S_SD_GPIO GPIO_NUM_16  // DATA IN
#define I2S_PORT I2S_NUM_0

// Button pin
#define MIC_BUTTON_GPIO 36 // GPIO36 as input

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

#define water_S1 "water/S1"
#define water_S2 "water/S2"
#define water_S3 "water/S3"
#define water_B1 "water/B1"
#define water_B2 "water/B2"
#define water_B3 "water/B3"

#define motor_status "motor/status"

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
#define syntax_low_led GPIO_NUM_35
#define syntax_mid_led GPIO_NUM_42
#define syntax_high_led GPIO_NUM_37
#define tank_low_led GPIO_NUM_38
#define tank_mid_led GPIO_NUM_39
#define tank_high_led GPIO_NUM_40
#define motor_led GPIO_NUM_41

#define OUTPUT_MASK ((1ULL << syntax_low_led) | (1ULL << syntax_mid_led) | \
                     (1ULL << syntax_high_led) | (1ULL << tank_low_led) |  \
                     (1ULL << tank_mid_led) | (1ULL << tank_high_led) |    \
                     (1ULL << motor_led))
////////////////////////////////////////////////////////////////////////////////////////////

/* ===== LDR + PIR ===== */
#define LDR_ADC_CHANNEL ADC_CHANNEL_6 // GPIO7
#define PIR_PIN GPIO_NUM_3            // GPIO3 input
#define LDR_DARK_THRESHOLD 400        // adjust if needed

#define enable_firebase 1
#define enable_pir 1
#define enable_touch 1
#define enable_relay 1
#define enable_microphone 1
#define enable_motor 1

// static adc_oneshot_unit_handle_t adc1_handle;
static esp_err_t firebase_set_value_int(const char *thisPath, int value);
esp_err_t firebase_set_value(const char *thisPath, const char *value);

/* ===== Wi-Fi event handling ===== */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static i2s_chan_handle_t rx_chan = NULL;

static int s_retry_num = 0;
#define WIFI_MAX_RETRY 5

// static volatile bool s_firebaseBusy = false;

static volatile bool s_start_listening = false;

static volatile bool relay_1_state = false;
static volatile bool relay_2_state = false;
static volatile bool relay_3_state = false;
static volatile bool relay_4_state = false;
static volatile bool relay_4_state_motor = false;

static volatile bool prv_relay_1_state = false;
static volatile bool prv_relay_2_state = false;
static volatile bool prv_relay_3_state = false;
static volatile bool prv_relay_4_state = false;

static int prev_S1 = -1;
static int prev_S2 = -1;
static int prev_S3 = -1;
static int prev_B1 = -1;
static int prev_B2 = -1;
static int prev_B3 = -1;
static bool prev_motor_state = false;

static volatile bool motor_manual_override = false;

static bool S1_change = false;
static bool S2_change = false;
static bool S3_change = false;
static bool B1_change = false;
static bool B2_change = false;
static bool B3_change = false;
static bool motor_state_change = false;

static volatile bool relay_3_temp = false;
static volatile bool relay_4_temp = false;

static volatile bool r1_change = false;
static volatile bool r2_change = false;
static volatile bool r3_change = false;
static volatile bool r4_change = false;

static volatile bool wifiIsConnected = false;

static adc_oneshot_unit_handle_t adc1_handle;

volatile int s1 = 0;
volatile int s2 = 0;
volatile int s3 = 0;
volatile int b1 = 0;
volatile int b2 = 0;
volatile int b3 = 0;

bool motor_can_turn_on = false;

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

static bool lr1, lr2, lr3, lr4;

////////////////////////////

static inline bool motor_safe_to_turn_on(void)
{
    // SAFETY INTERLOCKS (ALWAYS ACTIVE)
    return (s1 == 1) && (b3 == 0);
}

//////////////////////////////

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
        wifiIsConnected = true;
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
static void mic_button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << MIC_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "Button on GPIO%d initialized (active-low)", MIC_BUTTON_GPIO);
}

/* ===== Listening control ===== */
static volatile bool s_stop_listening_flag = false;
static bool s_is_listening = false;

/* ===== HTTP streaming (chunked) ===== */

static esp_err_t http_stream_audio(i2s_chan_handle_t rx_chan, int duration_ms)
{
    esp_http_client_config_t config = {
        .url = CONFIG_SERVER_URI,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
    {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_http_client_set_method(client, HTTP_METHOD_POST));
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "Transfer-Encoding", "chunked"));

    char header_val[32];
    snprintf(header_val, sizeof(header_val), "%d", AUDIO_SAMPLE_RATE);
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "x-audio-sample-rates", header_val));

    // Sending 16-bit audio
    snprintf(header_val, sizeof(header_val), "%d", 16);
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "x-audio-bits", header_val));

    snprintf(header_val, sizeof(header_val), "%d", AUDIO_CHANNELS);
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "x-audio-channel", header_val));

    ESP_LOGI(TAG, "Opening HTTP connection...");
    esp_err_t err = esp_http_client_open(client, -1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        esp_http_client_cleanup(client);
        s_start_listening = false;
        return err;
    }

    // --- CONFIGURATION ---
    const int samples_per_chunk = 1024;
    const int input_buf_size = samples_per_chunk * 4; // 32-bit input from Mic

    // We allocate two buffers:
    // 1. Raw input from I2S (32-bit)
    uint8_t *i2s_buf = (uint8_t *)malloc(input_buf_size);

    // 2. Network Send Buffer (Header + 16-bit Audio + Footer)
    //    Max Header ~ 10 bytes ("400\r\n")
    //    Audio = 1024 samples * 2 bytes = 2048 bytes
    //    Footer = 2 bytes ("\r\n")
    //    Total ~ 2060 bytes. We alloc 2500 to be safe.
    const int send_buf_size = 2500;
    char *send_buf = (char *)malloc(send_buf_size);

    if (!i2s_buf || !send_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        free(i2s_buf);
        free(send_buf);
        esp_http_client_cleanup(client);
        s_start_listening = false;
        return ESP_ERR_NO_MEM;
    }

    size_t total_bytes_sent = 0;
    size_t target_sent_bytes = (16000 * 2 * (RECORD_TIME_MS / 1000));

    while (!s_stop_listening_flag && (total_bytes_sent < target_sent_bytes))
    {
        size_t bytes_read = 0;
        // Read 32-bit samples
        err = i2s_channel_read(rx_chan, i2s_buf, input_buf_size, &bytes_read, portMAX_DELAY);

        if (err != ESP_OK)
            break;
        if (bytes_read == 0)
            continue;

        // --- STEP 1: Process Audio (32-bit -> 16-bit) ---
        int32_t *samples_32 = (int32_t *)i2s_buf;
        int16_t *samples_16 = (int16_t *)i2s_buf; // Reuse buffer to save RAM
        int sample_count = bytes_read / 4;

        for (int i = 0; i < sample_count; i++)
        {
            samples_16[i] = (int16_t)(samples_32[i] >> 16);
        }
        int audio_data_len = sample_count * 2;

        // --- STEP 2: Combine Everything into ONE Packet ---
        // Format: [HEX_SIZE]\r\n[AUDIO_DATA]\r\n

        // A. Write Header to send_buf
        int header_len = sprintf(send_buf, "%x\r\n", audio_data_len);

        // B. Copy Audio Data after the header
        memcpy(send_buf + header_len, samples_16, audio_data_len);

        // C. Add Footer after the audio data
        memcpy(send_buf + header_len + audio_data_len, "\r\n", 2);

        int total_packet_len = header_len + audio_data_len + 2;

        // --- STEP 3: Send ONCE ---
        int written = esp_http_client_write(client, send_buf, total_packet_len);
        if (written <= 0)
        {
            ESP_LOGE(TAG, "Network Error: Write failed");
            s_start_listening = false;
            break;
        }

        total_bytes_sent += audio_data_len;
    }

    // Finish Stream
    esp_http_client_write(client, "0\r\n\r\n", 5);

    // Read Response
    int hdr_len = esp_http_client_fetch_headers(client);
    if (hdr_len >= 0)
    {
        char resp[1024];
        int rlen = esp_http_client_read(client, resp, sizeof(resp) - 1);
        if (rlen > 0)
        {
            resp[rlen] = 0;
            ESP_LOGI(TAG, "Server says: %s", resp);

            cJSON *root = cJSON_Parse(resp);
            if (root)
            {
                cJSON *trans = cJSON_GetObjectItem(root, "transcript");
                if (trans && cJSON_IsString(trans))
                {
                    const char *transcript = trans->valuestring;
                    ESP_LOGW(TAG, "Transcript: %s", transcript);

                    if (strstr(transcript, "light 1 on"))
                    {
                        relay_1_state = true;
                        r1_change = true;
                    }
                    else if (strstr(transcript, "light 1 off"))
                    {
                        relay_1_state = false;
                        r1_change = true;
                    }
                    else if (strstr(transcript, "light 2 on"))
                    {
                        relay_2_state = true;
                        r2_change = true;
                    }
                    else if (strstr(transcript, "light 2 off"))
                    {
                        relay_2_state = false;
                        r2_change = true;
                    }
                    else if (strstr(transcript, "light 3 on"))
                    {
                        relay_3_state = true;
                        r3_change = true;
                    }
                    else if (strstr(transcript, "light 3 off"))
                    {
                        relay_3_state = false;
                        r3_change = true;
                    }
                    else if (strstr(transcript, "light 4 on"))
                    {
                        relay_4_state = true;
                        r4_change = true;
                    }
                    else if (strstr(transcript, "light 4 off"))
                    {
                        relay_4_state = false;
                        r4_change = true;
                    }
                }
                cJSON_Delete(root);
            }
        }
    }

    free(i2s_buf);
    free(send_buf);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    s_start_listening = false;
    return ESP_OK;
}

void stop_listening(void)
{
    s_stop_listening_flag = true;
    ESP_LOGI(TAG, "stop_listening() called");
}

esp_err_t start_listening(i2s_chan_handle_t rx_chan, int duration_ms)
{
    if (s_is_listening)
    {
        ESP_LOGW(TAG, "Already listening; ignoring start_listening()");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "start_listening(): begin streaming");
    s_stop_listening_flag = false;
    s_is_listening = true;

    esp_err_t err = http_stream_audio(rx_chan, duration_ms);

    s_is_listening = false;
    ESP_LOGI(TAG, "start_listening(): streaming finished");
    return err;
}

// ==========================================

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

static esp_err_t firebase_set_value_int(const char *thisPath, int value)
{

    char url[256];
    build_firebase_url(url, sizeof(url), thisPath);

    esp_http_client_config_t config = {
        .url = url,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .timeout_ms = 8000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
        return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", "application/json");

    char payload[8];
    snprintf(payload, sizeof(payload), "%d", value);

    esp_http_client_set_method(client, HTTP_METHOD_PUT);
    esp_http_client_set_post_field(client, payload, strlen(payload));

    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);

    vTaskDelay(10);
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
    };

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

/////////////////motor logic/////////////////
static void motor_logic(void)
{
    // READ INPUTS
    s3 = gpio_get_level(syntax_high_in8);
    s2 = gpio_get_level(syntax_mid_in18);
    s1 = gpio_get_level(syntax_low_in17);
    b3 = gpio_get_level(tank_high_in9);
    b2 = gpio_get_level(tank_mid_in46);
    b1 = gpio_get_level(tank_low_in21);

    // MOTOR DECISION
    if (s1 == 0 || b3 == 1)
        motor_can_turn_on = false;
    else if (s1 == 1 && s2 == 1 && b3 == 0)
        motor_can_turn_on = true;

    // SENSOR CHANGE FLAGS
    if (s1 != prev_S1)
    {
        prev_S1 = s1;
        S1_change = true;
    }
    if (s2 != prev_S2)
    {
        prev_S2 = s2;
        S2_change = true;
    }
    if (s3 != prev_S3)
    {
        prev_S3 = s3;
        S3_change = true;
    }
    if (b1 != prev_B1)
    {
        prev_B1 = b1;
        B1_change = true;
    }
    if (b2 != prev_B2)
    {
        prev_B2 = b2;
        B2_change = true;
    }
    if (b3 != prev_B3)
    {
        prev_B3 = b3;
        B3_change = true;
    }

    // AUTO MODE RESUME
    if (S1_change || S2_change || S3_change ||
        B1_change || B2_change || B3_change)
    {
        motor_manual_override = false;
    }

    // MOTOR STATE CHANGE
    if (motor_can_turn_on != prev_motor_state)
    {
        prev_motor_state = motor_can_turn_on;
        motor_state_change = true;
    }

    // AUTO CONTROL (NO GPIO)
    if (!motor_manual_override)
    {
        if (motor_can_turn_on && !relay_4_temp)
        {
            relay_4_state = true;
            relay_4_temp = true;
            r4_change = true;
        }
        else if (!motor_can_turn_on && relay_4_temp)
        {
            relay_4_state = false;
            relay_4_temp = false;
            r4_change = true;
        }
    }
}

///////////////////////////////////////////




/////////////////////////touch-logic////////////////

static void touch_logic(void)
{
    static uint32_t last_touch_scan = 0;

    // Run touch scan every 30 ms (non-blocking)
    if (millis() - last_touch_scan < 30)
        return;

    last_touch_scan = millis();

    uint32_t t1, t2, t3, t4;
    touch_pad_read_raw_data(TOUCH1_gpio1, &t1);
    touch_pad_read_raw_data(TOUCH2_gpio2, &t2);
    touch_pad_read_raw_data(TOUCH3_gpio4, &t3);
    touch_pad_read_raw_data(TOUCH2_gpio5, &t4);

    // ---- TOUCH 1 ----
    bool curr1 = (t1 > TOUCH_THRESHOLD);
    if (curr1 && !prev_touch_1)
    {
        relay_1_state = !relay_1_state;
        r1_change = true;
    }
    prev_touch_1 = curr1;

    // ---- TOUCH 2 ----
    bool curr2 = (t2 > TOUCH_THRESHOLD);
    if (curr2 && !prev_touch_2)
    {
        relay_2_state = !relay_2_state;
        r2_change = true;
    }
    prev_touch_2 = curr2;

    // ---- TOUCH 3 ----
    bool curr3 = (t3 > TOUCH_THRESHOLD);
    if (curr3 && !prev_touch_3)
    {
        relay_3_state = !relay_3_state;
        relay_3_temp = relay_3_state;
        r3_change = true;
    }
    prev_touch_3 = curr3;

    // ---- TOUCH 4 (Motor) ----
    bool curr4 = (t4 > TOUCH_THRESHOLD);
    if (curr4 && !prev_touch_4)
    {
        motor_manual_override = true;
        if (motor_safe_to_turn_on())
        {
            relay_4_state = !relay_4_state;
            relay_4_temp = relay_4_state;
            r4_change = true;
        }
    }
    prev_touch_4 = curr4;
}

/////////////////////////////////////////////////////////

///////////////////////adc function///////////////////

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle,
                                               LDR_ADC_CHANNEL,
                                               &config));
}

/////////////////////////////////////////////////////

//////////////pir-ldr-logic////////////////////////

static void pir_ldr_logic(void)
{
    // If relay is manually controlled, PIR must not interfere
    if (relay_3_temp)
        return;

    int ldr = 0;
    adc_oneshot_read(adc1_handle, LDR_ADC_CHANNEL, &ldr);
    int pir = gpio_get_level(PIR_PIN);

    uint64_t now_ms = esp_timer_get_time() / 1000;

    /* -------- AUTO ON (PIR + LDR) -------- */
    if (!pir_timer_active &&
        pir_check_allowed &&
        (pir == 1) &&
        (ldr < LDR_DARK_THRESHOLD))
    {
        relay_3_state = true;
        relay_3_temp = true;
        r3_change = true;

        pir_owned_relay = true;
        pir_timer_active = true;
        pir_check_allowed = false;
        pir_extension_checked = false;
        pir_relay_expiry_time = now_ms + 10000;
    }

    /* -------- 9TH SECOND EXTENSION CHECK -------- */
    if (pir_timer_active)
    {
        uint64_t remaining =
            (pir_relay_expiry_time > now_ms)
                ? (pir_relay_expiry_time - now_ms)
                : 0;

        if (remaining <= 1000 && !pir_extension_checked)
        {
            pir_extension_checked = true;
            pir_check_allowed = true;

            if (pir == 1)
            {
                pir_relay_expiry_time = now_ms + 10000;
                pir_check_allowed = false;
                pir_extension_checked = false;
            }
        }
    }

    /* -------- AUTO OFF -------- */
    if (pir_timer_active &&
        pir_owned_relay &&
        now_ms >= pir_relay_expiry_time)
    {
        relay_3_state = false;
        relay_3_temp = false;
        r3_change = true;

        pir_timer_active = false;
        pir_owned_relay = false;
        pir_check_allowed = true;
        pir_extension_checked = false;
    }
}

//////////////////////////////////////////////////

////////////////////////microphone function //////////////////////////
static void microphone_logic(void)
{
    if (gpio_get_level(MIC_BUTTON_GPIO) == 0)
    {
        s_start_listening = true;
    }

    if (s_start_listening)
    {
        start_listening(rx_chan, RECORD_TIME_MS);
        s_start_listening = false;
    }
}
//////////////////////////////////////////

//////////////////////realy and led set /////////////////////////

static void apply_relay_and_leds(void)
{
  

    if (relay_1_state != lr1)
        gpio_set_level(RELAY_1, relay_1_state);
    if (relay_2_state != lr2)
        gpio_set_level(RELAY_2, relay_2_state);
    if (relay_3_state != lr3)
        gpio_set_level(RELAY_3, relay_3_state);
    if (relay_4_state != lr4)
        gpio_set_level(RELAY_4, relay_4_state);

    gpio_set_level(motor_led, relay_4_state ? 0 : 1);

    lr1 = relay_1_state;
    lr2 = relay_2_state;
    lr3 = relay_3_state;
    lr4 = relay_4_state;
}

/////////////////////////////////////////////////////////////

////////////////////////Read firbase//////////////////////

static void read_firebase_and_sync(void)
{
    if (!wifiIsConnected || !enable_firebase)
        return;

    static uint32_t last_fetch = 0;
    if (millis() - last_fetch < 1000) // rate limit
        return;

    last_fetch = millis();

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

    if (fb1 != relay_1_state)
    {
        relay_1_state = fb1;
        prv_relay_1_state = relay_1_state;
        r1_change = true;
    }

    if (fb2 != relay_2_state)
    {
        relay_2_state = fb2;
        prv_relay_2_state = relay_2_state;
        r2_change = true;
    }

    if (fb3 != relay_3_state)
    {
        relay_3_state = fb3;
        prv_relay_3_state = relay_3_state;

        pir_timer_active = false;
        pir_owned_relay = false;
        pir_check_allowed = true;

        relay_3_temp = relay_3_state;
        r3_change = true;
    }

    if (fb4 != relay_4_state)
    {
        motor_manual_override = true;

        if (fb4 && !motor_safe_to_turn_on())
        {
            ESP_LOGW(TAG, "Firebase ON blocked (SAFETY)");
        }
        else
        {
            relay_4_state = fb4;
            prv_relay_4_state = relay_4_state;
            relay_4_temp = relay_4_state;
            r4_change = true;
        }
    }
}

/////////////////////////////////////////////////////////

/////////////////////////SET_FIREBASE_FUNCTION///////////////////////////

static void update_firebase_if_changed(void)
{
    if (!wifiIsConnected || !enable_firebase)
        return;

    bool any_change =
        r1_change || r2_change || r3_change || r4_change ||
        S1_change ||
        S2_change ||
        S3_change ||
        B1_change ||
        B2_change ||
        B3_change ||
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
                 FB_LIGHT_1, relay_1_state ? "ON" : "OFF",
                 FB_LIGHT_2, relay_2_state ? "ON" : "OFF",
                 FB_LIGHT_3, relay_3_state ? "ON" : "OFF",
                 FB_LIGHT_4, relay_4_state ? "ON" : "OFF",

                 s1 ? "0" : "1",
                 s2 ? "0" : "1",
                 s3 ? "0" : "1",
                 b1 ? "0" : "1",
                 b2 ? "0" : "1",
                 b3 ? "0" : "1",

                 relay_4_state ? "ON" : "OFF");

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

        if (r1_change)
        {
            r1_change = false;
        }
        if (r2_change)
        {
            r2_change = false;
        }
        if (r3_change)
        {
            r3_change = false;
        }
        if (r4_change)
        {
            r4_change = false;
        }
    }
}
/////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

static void touch_read_task(void *params)
{

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

    unsigned long startTimez = 0;

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

    /////////////////////// motor-variables-declaration //////////////////////////
    firebase_set_value(water_S1, "0");
    firebase_set_value(water_S2, "0");
    firebase_set_value(water_S3, "0");
    firebase_set_value(water_B1, "0");
    firebase_set_value(water_B2, "0");
    firebase_set_value(water_B3, "0");

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
        .pin_bit_mask = OUTPUT_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&output_conf);

    // INIT OUTPUTS
    gpio_set_level(syntax_low_led, 1);
    gpio_set_level(syntax_mid_led, 1);
    gpio_set_level(syntax_high_led, 1);
    gpio_set_level(tank_low_led, 1);
    gpio_set_level(tank_mid_led, 1);
    gpio_set_level(tank_high_led, 1);
    gpio_set_level(motor_led, 1); // OFF (inverted)

    // STATE VARIABLES

    ///////////////////////////////////////////////////////////////////

    while (1)
    {
        ESP_LOGI(TAG, "Start of while loop");
        motor_logic();
        ESP_LOGI(TAG, "S1 g17:%d S2 g18:%d S3 g8:%d | B1 g9:%d B2 g46:%d B3 g9:%d", s1, s2, s3, b1, b2, b3);

        ESP_LOGI(TAG, "touch detection start ");
        touch_logic();
        ESP_LOGI(TAG, "touch detection end");

        ESP_LOGI(TAG, "PIR_LDR start ");
        pir_ldr_logic();
        ESP_LOGI(TAG, "PIR_LDR end");

        ESP_LOGI(TAG, "MIC start ");
        microphone_logic();
        ESP_LOGI(TAG, "MIC end");

        ESP_LOGI(TAG, "Relay start ");
        apply_relay_and_leds();
        ESP_LOGI(TAG, "Relay end");

        ESP_LOGI(TAG, "firebase upload start ");
        update_firebase_if_changed();
        ESP_LOGI(TAG, "firebase upload end");

        ESP_LOGI(TAG, "read start ");
        read_firebase_and_sync();
        ESP_LOGI(TAG, "read end  ");

        ESP_LOGI(TAG, "Relay start ");
        apply_relay_and_leds();
        ESP_LOGI(TAG, "Relay end");

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

/* ===== Main entry ===== */

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
    mic_button_init();

    adc_init();

    // Initialize relay GPIO (only added lines — no change to streaming logic)

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

    if (enable_touch)
    {
        xTaskCreatePinnedToCore(touch_read_task, "touch_read_task",64*1024, NULL, 6, NULL, 0);
    }
}