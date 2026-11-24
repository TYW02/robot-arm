/** @file main.c
 *  @brief Capture XIAO camera frames and report detections to the Pico.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "aruco_detect.h"
#include "driver/uart.h"
#include "esp_camera.h"
#include "esp_log.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#define VISION_UART_NUM      UART_NUM_0
#define VISION_UART_TX_PIN   43   // D6 on XIAO ESP32-S3
#define VISION_UART_RX_PIN   44   // D7 on XIAO ESP32-S3
#define VISION_UART_BAUD     115200
#define BUF_SIZE             256

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

#define ARUCO_RESULT_CAPACITY  16
#define UART_FRAME_CAPACITY    64
#define CAMERA_LOOP_MS         50U
#define UART_READ_TIMEOUT_MS   100U
#define REPORT_INTERVAL_MS     1000U

typedef enum
{
    MODE_IDLE,
    MODE_SCAN
} vision_mode_t;

typedef enum
{
    COMMAND_WAIT_PREFIX,
    COMMAND_WAIT_COMMA,
    COMMAND_WAIT_MODE,
    COMMAND_WAIT_NEWLINE
} command_state_t;

static char const * const gp_tag = "VISION";
/* The command task writes the selected mode and physical marker size. */
static vision_mode_t volatile g_mode = MODE_SCAN;
static int volatile g_marker_size_mm = 80;

static void initialise_uart (void);
static esp_err_t initialise_camera (void);
static void camera_uart_rx_task (void * p_argument);
static void report_markers (camera_fb_t * p_frame, uint32_t * p_last_status_ms);

void app_main (void)
{
    ESP_LOGI(gp_tag, "Starting ArUco vision...");
    initialise_uart();

    xTaskCreate(camera_uart_rx_task, "uart_rx", 8192, NULL, 5, NULL);

    if (initialise_camera() != ESP_OK)
    {
        ESP_LOGE(gp_tag, "Camera failed, halting");
        return;
    }

    ESP_LOGI(gp_tag, "Camera ready.");

    uint32_t last_aruco_status_ms = 0U;

    for (;;)
    {
        camera_fb_t * p_frame = esp_camera_fb_get();

        if (p_frame == NULL)
        {
            ESP_LOGW(gp_tag, "Frame capture failed");
        }
        else
        {
            if (g_mode == MODE_SCAN)
            {
                report_markers(p_frame, &last_aruco_status_ms);
            }
            else
            {
                /* Idle mode still returns the acquired frame. */
            }

            esp_camera_fb_return(p_frame);
        }

        vTaskDelay(pdMS_TO_TICKS(CAMERA_LOOP_MS));
    }
}

/** Configure the UART to match the Pico link. */
static void initialise_uart (void)
{
    const uart_config_t uart_config =
    {
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
    ESP_ERROR_CHECK(uart_driver_install(VISION_UART_NUM, BUF_SIZE * 2,
                                        0, 0, NULL, 0));
    ESP_LOGI(gp_tag, "UART initialized on TX:%d, RX:%d",
             VISION_UART_TX_PIN, VISION_UART_RX_PIN);
}

/** Initialise the XIAO camera and enable automatic sensor controls. */
static esp_err_t initialise_camera (void)
{
    camera_config_t const config =
    {
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

    if (err != ESP_OK)
    {
        ESP_LOGE(gp_tag, "Camera init failed: 0x%x", err);
    }

    sensor_t * p_sensor = esp_camera_sensor_get();

    if (p_sensor != NULL)
    {
        /* Keep automatic exposure and white balance across lighting changes. */
        p_sensor->set_whitebal(p_sensor, 1);
        p_sensor->set_awb_gain(p_sensor, 1);
        p_sensor->set_exposure_ctrl(p_sensor, 1);
        p_sensor->set_gain_ctrl(p_sensor, 1);
    }
    return err;
}

/** Apply the existing mode commands sent by the Pico. */
static void apply_mode_command (uint8_t command)
{
    switch (command)
    {
        case '3':
        case '8':
            g_marker_size_mm = (command == '3') ? 30 : 80;
            /* Fall through so the selected size is used in scan mode. */
        case 'S':
            g_mode = MODE_SCAN;
            ESP_LOGI(gp_tag, "Switched to SCAN mode (marker size %d mm)",
                     g_marker_size_mm);
        break;

        case 'I':
            g_mode = MODE_IDLE;
        break;

        default:
            /* Leave the current mode selected for unknown commands. */
        break;
    }
}

/** Read M commands without blocking the camera frame loop. */
static void camera_uart_rx_task (void * p_argument)
{
    command_state_t state = COMMAND_WAIT_PREFIX;
    uint8_t command = 0U;

    (void)p_argument;

    for (;;)
    {
        uint8_t byte = 0U;
        int length = uart_read_bytes(VISION_UART_NUM, &byte, 1,
                                     pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));

        if ((length != 1) || (byte == '\r'))
        {
            continue;
        }

        switch (state)
        {
            case COMMAND_WAIT_PREFIX:
                if (byte == 'M')
                {
                    state = COMMAND_WAIT_COMMA;
                }
            break;

            case COMMAND_WAIT_COMMA:
                state = (byte == ',') ? COMMAND_WAIT_MODE
                                      : COMMAND_WAIT_PREFIX;
            break;

            case COMMAND_WAIT_MODE:
                command = byte;
                state   = COMMAND_WAIT_NEWLINE;
            break;

            case COMMAND_WAIT_NEWLINE:
                if (byte == '\n')
                {
                    apply_mode_command(command);
                }

                state = COMMAND_WAIT_PREFIX;
            break;

            default:
                state = COMMAND_WAIT_PREFIX;
            break;
        }
    }
}

/** Append the UART checksum and terminate the report with a newline. */
static void send_with_checksum (char const * p_format, ...)
{
    static bool b_reported_first_frame = false;
    char frame[UART_FRAME_CAPACITY];
    va_list args;

    va_start(args, p_format);
    int payload_length = vsnprintf(frame, sizeof(frame), p_format, args);
    va_end(args);

    /* Reserve four bytes for the checksum suffix and one for the null. */
    if ((payload_length < 0) ||
        (payload_length > (UART_FRAME_CAPACITY - 5)))
    {
        ESP_LOGW(gp_tag, "UART report exceeds frame capacity");
        return;
    }

    uint8_t checksum = 0U;

    for (int index = 0; index < payload_length; ++index)
    {
        /* XOR the unsigned byte representation of each payload character. */
        checksum ^= (uint8_t)frame[index];
    }

    int suffix_length = snprintf(frame + payload_length,
                                 UART_FRAME_CAPACITY - payload_length,
                                 "*%02X\n", (unsigned int)checksum);

    if (suffix_length != 4)
    {
        return;
    }

    int frame_length = payload_length + suffix_length;
    int written = uart_write_bytes(VISION_UART_NUM, frame, frame_length);

    if (!b_reported_first_frame)
    {
        ESP_LOGI(gp_tag, "UART TX first frame: wrote %d/%d bytes",
                 written, frame_length);
        ESP_LOG_BUFFER_HEXDUMP(gp_tag, frame, frame_length, ESP_LOG_INFO);
        b_reported_first_frame = true;
    }
}

/** Report every visible ArUco marker in the frame. */
static void report_markers (camera_fb_t * p_frame, uint32_t * p_last_status_ms)
{
    aruco_result_t results[ARUCO_RESULT_CAPACITY];
    int marker_size_mm = g_marker_size_mm;

    /* The configured marker sizes are 30 and 80, both exact as floats. */
    int count = aruco_detect_frame(p_frame, (float)marker_size_mm,
                                  results, ARUCO_RESULT_CAPACITY);
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if ((now_ms - *p_last_status_ms) >= REPORT_INTERVAL_MS)
    {
        if (count > 0)
        {
            ESP_LOGI(gp_tag,
                     "ArUco scan: %d marker(s), first ID %d, size %d mm",
                     count, results[0].marker_id, marker_size_mm);
        }
        else
        {
            /* Frame dimensions are QVGA and fit the log argument types. */
            ESP_LOGW(gp_tag,
                     "ArUco scan: no markers; frame %ux%u "
                     "format %d, size %d mm",
                     (unsigned int)p_frame->width,
                     (unsigned int)p_frame->height, (int)p_frame->format,
                     marker_size_mm);
        }

        *p_last_status_ms = now_ms;
    }

    for (int index = 0; index < count; ++index)
    {
        aruco_result_t const * p_marker = &results[index];

        if (p_marker->detected)
        {
            /* Detected corners are within the QVGA frame. */
            send_with_checksum("A,%d,%d,%d,%.1f,%.1f",
                               p_marker->marker_id, (int)p_marker->cx,
                               (int)p_marker->cy, p_marker->width_px,
                               p_marker->distance_mm);
        }
    }
}

/* End of file. */

