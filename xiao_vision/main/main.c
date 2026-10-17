#include "freertos/FreeRTOS.h"
#include "aruco_detect.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_camera.h"
#include <string.h>
#include <stdio.h>

// ======================================================
// UART (to Pico W)
// ======================================================
#define VISION_UART_NUM      UART_NUM_0
#define VISION_UART_TX_PIN   43   // D6 on XIAO ESP32-S3
#define VISION_UART_RX_PIN   44   // D7 on XIAO ESP32-S3
#define VISION_UART_BAUD     115200
#define BUF_SIZE             256

// ======================================================
// Phase/Mode Management
// ======================================================
typedef enum { MODE_IDLE, MODE_SCAN, MODE_COLOR } vision_mode_t;

// Start in COLOR mode by default
static volatile vision_mode_t g_mode = MODE_SCAN; 

static const char *TAG = "VISION";

// ======================================================
// Camera pins — XIAO ESP32-S3 Sense
// ======================================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ======================================================
// Color detection
// ======================================================
typedef enum { COLOR_NONE = 0, COLOR_RED, COLOR_GREEN, COLOR_BLUE } color_t;

#define STRIDE 4   

static void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b,
                       uint8_t *h, uint8_t *s, uint8_t *v)
{
    uint8_t max = r > g ? (r > b ? r : b) : (g > b ? g : b);
    uint8_t min = r < g ? (r < b ? r : b) : (g < b ? g : b);
    uint8_t delta = max - min;

    *v = max;
    if (max == 0) { *s = 0; *h = 0; return; }
    *s = (uint8_t)(((uint32_t)delta * 255) / max);
    if (delta == 0) { *h = 0; return; }

    int32_t hue;   // use int32_t to avoid any promotion surprises
    if (max == r) {
        hue = 43 * ((int32_t)g - (int32_t)b) / (int32_t)delta;
    } else if (max == g) {
        hue = 85 + 43 * ((int32_t)b - (int32_t)r) / (int32_t)delta;
    } else {
        hue = 171 + 43 * ((int32_t)r - (int32_t)g) / (int32_t)delta;
    }
    if (hue < 0) hue += 255;
    if (hue > 255) hue = 255;
    *h = (uint8_t)hue;
}

static color_t classify_hsv(uint8_t h, uint8_t s, uint8_t v)
{
    if (v < 30 || s < 60) return COLOR_NONE;

    if ((h <= 15 || h >= 240) && s > 70) return COLOR_RED;
    if (h >= 60 && h <= 90 && s > 120)   return COLOR_GREEN;
    if (h >= 120 && h <= 180 && s > 60)  return COLOR_BLUE;

    return COLOR_NONE;
}

typedef struct {
    color_t color;   // R/G/B/N
    int16_t x;       // centroid X in original pixels (0-319)
    int16_t y;       // centroid Y in original pixels (0-239)
    int16_t count;   // number of matching pixels
} detection_t;

static detection_t detect_dominant_color(camera_fb_t *fb)
{
    int counts[4]      = {0};
    int32_t sum_x[4]   = {0};   // use int32_t to avoid overflow
    int32_t sum_y[4]   = {0};
    int total = 0;

    uint8_t *buf = (uint8_t *)fb->buf;
    int width  = fb->width;
    int height = fb->height;
    int bytes_per_pixel = 2;

    for (int y = 0; y < height; y += STRIDE) {
        for (int x = 0; x < width; x += STRIDE) {
            int offset = (y * width + x) * bytes_per_pixel;
            uint16_t pixel = (buf[offset] << 8) | buf[offset + 1];

            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5)  & 0x3F;
            uint8_t b =  pixel        & 0x1F;

            r = (r * 255) / 31;
            g = (g * 255) / 63;
            b = (b * 255) / 31;

            uint8_t h, s, v;
            rgb_to_hsv(r, g, b, &h, &s, &v);

            color_t c = classify_hsv(h, s, v);
            counts[c]++;
            sum_x[c] += x;   // accumulate in stride-space
            sum_y[c] += y;
            total++;
        }
    }

    detection_t result = { .color = COLOR_NONE, .x = -1, .y = -1, .count = 0 };

    if (total == 0) return result;

    // Find the strongest non-N color
    int best = COLOR_NONE;
    int best_count = 0;
    for (int i = COLOR_RED; i <= COLOR_BLUE; i++) {
        if (counts[i] > best_count) {
            best_count = counts[i];
            best = i;
        }
    }

    if (best == COLOR_NONE || (best_count * 100 / total) < 5) {
        return result;   // no color wins
    }

    // Convert centroid from stride-space back to real pixel coords
    result.color = best;
    result.x     = (int16_t)((sum_x[best] / best_count));
    result.y     = (int16_t)((sum_y[best] / best_count));
    result.count = (int16_t)best_count;

    return result;
}

// ======================================================
// UART init
// ======================================================
static void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = VISION_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(VISION_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(VISION_UART_NUM,
                                 VISION_UART_TX_PIN, VISION_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(VISION_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART initialized on TX:%d, RX:%d",
             VISION_UART_TX_PIN, VISION_UART_RX_PIN);
}

// ======================================================
// Camera init
// ======================================================
static esp_err_t init_camera(void)
{
    camera_config_t config = {
        .pin_pwdn     = PWDN_GPIO_NUM,
        .pin_reset    = RESET_GPIO_NUM,
        .pin_xclk     = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_d7       = Y9_GPIO_NUM,
        .pin_d6       = Y8_GPIO_NUM,
        .pin_d5       = Y7_GPIO_NUM,
        .pin_d4       = Y6_GPIO_NUM,
        .pin_d3       = Y5_GPIO_NUM,
        .pin_d2       = Y4_GPIO_NUM,
        .pin_d1       = Y3_GPIO_NUM,
        .pin_d0       = Y2_GPIO_NUM,
        .pin_vsync    = VSYNC_GPIO_NUM,
        .pin_href     = HREF_GPIO_NUM,
        .pin_pclk     = PCLK_GPIO_NUM,
        .xclk_freq_hz = 16000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size   = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count     = 3,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
            // AWB/AEC enabled by default; just let it run for a second
        vTaskDelay(pdMS_TO_TICKS(1500));

        // Then lock the gains at whatever AWB decided
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0);
        s->set_gain_ctrl(s, 0);
        // Optional: set a manual white balance for your lighting
        // s->set_wb_mode(s, 1);    // 0=auto, 1=sunny, 2=cloudy, 3=office, 4=home
    }
    return err;
}

// ======================================================
// app_main
// ======================================================
#define HISTORY_LEN 5
static detection_t history[HISTORY_LEN];
static int history_idx = 0;

// Returns the consensus color and its averaged position
static detection_t history_consensus(void)
{
    int counts[4] = {0};
    for (int i = 0; i < HISTORY_LEN; i++) {
        counts[history[i].color]++;
    }

    // Find the color with the most votes (need at least 3/5)
    color_t winner = COLOR_NONE;
    for (int c = COLOR_RED; c <= COLOR_BLUE; c++) {
        if (counts[c] >= 3) { winner = (color_t)c; break; }
    }

    detection_t result = { .color = winner, .x = -1, .y = -1, .count = 0 };
    if (winner == COLOR_NONE) return result;

    // Average the (x, y) of frames that agreed with the winner
    int32_t sx = 0, sy = 0, n = 0;
    for (int i = 0; i < HISTORY_LEN; i++) {
        if (history[i].color == winner && history[i].x >= 0) {
            sx += history[i].x;
            sy += history[i].y;
            n++;
        }
    }
    if (n > 0) {
        result.x = (int16_t)(sx / n);
        result.y = (int16_t)(sy / n);
    }
    return result;
}


// UART Task to listen for mode commands from the Pico
static void uart_rx_task(void *arg)
{
    uint8_t byte;
    int state = 0;
    char last_cmd = 0;

    while (1) {
        int n = uart_read_bytes(VISION_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (n == 1) {
            if (byte == '\r') continue;
            // Look for the pattern "M,X\n"
            if (state == 0 && byte == 'M') state = 1;
            else if (state == 1 && byte == ',') state = 2;
            else if (state == 2) {
                last_cmd = byte;
                state = 3;
            } 
            else if (state == 3 && byte == '\n') {
                if (last_cmd == 'S') {
                    g_mode = MODE_SCAN;
                    ESP_LOGI(TAG, "Switched to SCAN mode");
                }
                else if (last_cmd == 'C') {
                    g_mode = MODE_COLOR;
                    ESP_LOGI(TAG, "Switched to COLOR mode");
                }
                else if (last_cmd == 'I') g_mode = MODE_IDLE;
                state = 0;
            } else {
                state = 0;  // Reset on unexpected byte
            }
        }
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting color detection...");
    init_uart();

    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL);

    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera failed, halting");
        return;
    }
    ESP_LOGI(TAG, "Camera ready.");

    color_t last_sent_color = COLOR_NONE;
    int16_t last_sent_x = -999;
    int16_t last_sent_y = -999;
    uint32_t last_send_ms = 0;

    while (1) {
            camera_fb_t *fb = esp_camera_fb_get();

            if (!fb) {
                ESP_LOGW(TAG, "Frame capture failed");
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

                // Branch based on mode (MODE_SCAN / MODE_COLOR)
            if (g_mode == MODE_SCAN) {
                // Aruco Scanning
                aruco_result_t res = aruco_detect_frame(fb);

                if (res.detected) {
                    char tx_buffer[48];
                    int len = snprintf(tx_buffer, sizeof(tx_buffer),
                                    "A,%d,%d,%d,%.1f,%.1f\n",
                                    res.marker_id, (int)res.cx, (int)res.cy, res.width_px, res.distance_mm);
                    uart_write_bytes(VISION_UART_NUM, tx_buffer, len);
                }
            } else if (g_mode == MODE_COLOR) {
                // COLOR DETECTION
                detection_t det = detect_dominant_color(fb);
                

                // Push into history
                history[history_idx] = det;
                history_idx = (history_idx + 1) % HISTORY_LEN;

                detection_t consensus = history_consensus();
                uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // Send if the color changed, position moved significantly, or it's been 1s
                bool color_changed = (consensus.color != last_sent_color);
                bool moved_far     = (abs(consensus.x - last_sent_x) > 10) ||
                                    (abs(consensus.y - last_sent_y) > 10);
                bool stale         = (now_ms - last_send_ms) > 1000;

                if (color_changed || moved_far || stale) {
                    const char *name = (consensus.color == COLOR_RED)   ? "R"
                                    : (consensus.color == COLOR_GREEN) ? "G"
                                    : (consensus.color == COLOR_BLUE)  ? "B"
                                    : "N";

                    char tx_buffer[32];
                    int len;
                    if (consensus.color == COLOR_NONE || consensus.x < 0) {
                        len = snprintf(tx_buffer, sizeof(tx_buffer), "C,N\n");
                    } else {
                        len = snprintf(tx_buffer, sizeof(tx_buffer),
                                    "C,%s,%d,%d\n", name, consensus.x, consensus.y);
                    }
                    uart_write_bytes(VISION_UART_NUM, tx_buffer, len);
                    
                    last_sent_color = consensus.color;
                    last_sent_x = consensus.x;
                    last_sent_y = consensus.y;
                    last_send_ms = now_ms;
                }
            }

            
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}