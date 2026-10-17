// vision_test.c
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// ======================================================
// UART Configuration
// ======================================================
#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 0
#define UART_RX_PIN 1

void send_mode_command(const char* mode) {
    uart_write_blocking(UART_ID, (const uint8_t*)mode, strlen(mode));
}

int main() {
    stdio_init_all();
    sleep_ms(2000); 

    printf("Pico W Mode Switching Test\n");

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(UART_RX_PIN); 
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    char buffer[128];
    int index = 0;
    
    // Variables to handle automatic mode switching
    bool is_scan_mode = false;
    uint32_t last_switch_time = to_ms_since_boot(get_absolute_time());

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // 1. Switch modes every 10 seconds
        if (now - last_switch_time > 10000) {
            is_scan_mode = !is_scan_mode;
            
            if (is_scan_mode) {
                printf("\n>>> Switched to SCAN (ArUco) Mode <<<\n");
                send_mode_command("M,S\n");
            } else {
                printf("\n>>> Switched to COLOR Mode <<<\n");
                send_mode_command("M,C\n");
            }
            last_switch_time = now;
        }

        // 2. Read data from XIAO
        if (uart_is_readable(UART_ID)) {
            char ch = uart_getc(UART_ID);

            if (ch == '\n') {
                buffer[index] = '\0';

                while (index > 0 && (buffer[index-1] == '\r')) {
                    buffer[--index] = '\0';
                }

                char color;
                int x = -1, y = -1;
                int id;
                float w, dist;
                char *start;

                // Parse ArUco Data
                if ((start = strstr(buffer, "A,")) != NULL) {
                    if (sscanf(start, "A,%d,%d,%d,%f,%f", &id, &x, &y, &w, &dist) == 5) {
                        printf("ARUCO -> ID: %d, X: %d, Y: %d, Dist: %.1f mm\n", id, x, y, dist);
                    }
                } 
                // Parse Color Data
                else if ((start = strstr(buffer, "C,")) != NULL) {
                    if (sscanf(start, "C,%c,%d,%d", &color, &x, &y) == 3) {
                        printf("COLOR -> %c  X=%d  Y=%d\n", color, x, y);
                    } else if (sscanf(start, "C,%c", &color) == 1) {
                        printf("COLOR -> %c\n", color);
                    }
                } 
                index = 0;
            }
            else if (index < sizeof(buffer) - 1) {
                buffer[index++] = ch;
            }
        }
        sleep_ms(1);
    }

    return 0;
}