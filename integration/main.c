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

// Pico W UART0 default pins: TX=GP0, RX=GP1
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// ======================================================
// Main Application
// ======================================================
int main() {
    // Initialize USB serial for printf output
    stdio_init_all();
    sleep_ms(2000); // Wait for USB connection

    printf("Pico W UART Vision Test\n");

    // 1. Initialize UART0 with the specified pins and settings
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(UART_RX_PIN); 
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, true); // Enable FIFO for smoother data handling

    printf("UART0 initialized on GP%d (TX) and GP%d (RX) at %d baud.\n",
           UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
    printf("Waiting for vision data from XIAO...\n");

    char buffer[128];
    int index = 0;

    while (true) {
        // 2. Check if data is available to read
        if (uart_is_readable(UART_ID)) {
            char ch = uart_getc(UART_ID);

            // If we receive a newline, process the full line
            if (ch == '\n') {
                buffer[index] = '\0';

                char color;
                int x = -1, y = -1;

                if (sscanf(buffer, "C,%c,%d,%d", &color, &x, &y) == 3) {
                    printf("COLOR -> %c  X=%d  Y=%d\n", color, x, y);
                } else if (sscanf(buffer, "C,%c", &color) == 1) {
                    printf("COLOR -> %c  (no position)\n", color);
                } else {
                    printf("Unrecognized: '%s'\n", buffer);
                }

                index = 0;
            }
            else if (index < sizeof(buffer) - 1) {
                buffer[index++] = ch;
            }
            // If buffer is full without newline, it will be reset on next newline
        }

        // Small sleep to prevent tight loop, though UART FIFO makes this less critical
        sleep_ms(1);
    }

    return 0;
}