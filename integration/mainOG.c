#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"

// ======================================================
// UART Configuration
// ======================================================
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// ======================================================
// Ring Buffer & IRQ
// ======================================================
#define RX_BUFFER_SIZE 256
static queue_t rx_queue;

// UART Interrupt Service Routine (ISR)
void on_uart_rx() {
    // Read a maximum of 16 bytes per interrupt
    int max_read = 16; 
    while (uart_is_readable(UART_ID) && max_read > 0) {
        uint8_t ch = uart_getc(UART_ID);
        queue_try_add(&rx_queue, &ch);
        max_read--;
    }
    // Clear the RX interrupt flag explicitly
    uart_get_hw(UART_ID)->icr = UART_UARTICR_RXIC_BITS; 
}

void setup_uart_irq() {
    queue_init(&rx_queue, sizeof(uint8_t), RX_BUFFER_SIZE);
    
    // Set interrupt priority lower so it doesn't block USB
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_priority(UART0_IRQ, 0x80); // Lowest priority
    irq_set_enabled(UART0_IRQ, true);
    
    uart_set_irq_enables(UART_ID, true, false); 
}

// ======================================================
// State Machine & Object Storage
// ======================================================
#define MAX_OBJECTS 16

typedef struct {
    int id;
    int x;
    int y;
    float distance;
} aruco_obj_t;

aruco_obj_t shelf_map[MAX_OBJECTS];
int shelf_count = 0;

typedef enum {
    STATE_IDLE,
    STATE_SCAN_SHELF,
    STATE_WAIT_SHELF_RESULTS,
    STATE_MOVE_TO_PAYLOAD,
    STATE_RE_SCAN_PAYLOAD,
    STATE_WAIT_PAYLOAD_RESULTS,
    STATE_EXECUTE_PICK,
    STATE_FAULT
} system_state_t;

void send_mode_command(const char* mode) {
    uart_write_blocking(UART_ID, (const uint8_t*)mode, strlen(mode));
}

int main() {
    stdio_init_all();
    
    // 1. Wait 3 seconds for USB to connect to your PC
    for (int i = 0; i < 6; i++) {
        printf("Booting up... %d\n", i);
        sleep_ms(500);
    }

    printf("Pico W IRQ & State Machine Test\n");

    // Init UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(UART_RX_PIN);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    printf("UART initialized. Starting IRQ in 1 second...\n");
    sleep_ms(1000);

    // 2. Start the IRQ ONLY after USB is fully connected
    setup_uart_irq(); 
    printf("IRQ Started. Entering main loop.\n");

    // State variables
    system_state_t state = STATE_SCAN_SHELF;
    uint32_t state_timer = to_ms_since_boot(get_absolute_time());
    char buffer[128];
    int index = 0;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // ==========================================
        // 1. STATE MACHINE
        // ==========================================
        switch (state) {
            case STATE_SCAN_SHELF:
                printf("\n[State] SCAN_SHELF: Sending M,S...\n");
                shelf_count = 0; // Clear old map
                send_mode_command("M,S\n");
                sleep_ms(50);
                send_mode_command("M,S\n");
                state_timer = now;
                state = STATE_WAIT_SHELF_RESULTS;
                break;

            case STATE_WAIT_SHELF_RESULTS:
                // Wait 5 seconds to gather data
                if (now - state_timer > 5000) {
                    printf("[State] Finished Scanning. Found %d objects.\n", shelf_count);
                    printf("[State] MOVE_TO_PAYLOAD...\n");
                    state_timer = now;
                    state = STATE_MOVE_TO_PAYLOAD;
                }
                break;

            case STATE_MOVE_TO_PAYLOAD:
                // Simulate arm movement time (2 seconds)
                if (now - state_timer > 2000) {
                    printf("[State] RE_SCAN_PAYLOAD: Sending M,C...\n");
                    send_mode_command("M,C\n");
                    sleep_ms(50);
                    send_mode_command("M,C\n");
                    state_timer = now;
                    state = STATE_WAIT_PAYLOAD_RESULTS;
                }
                break;

            case STATE_WAIT_PAYLOAD_RESULTS:
                // Wait 2 seconds for fresh color data
                if (now - state_timer > 2000) {
                    printf("[State] EXECUTE_PICK...\n");
                    state = STATE_EXECUTE_PICK;
                }
                break;

            case STATE_EXECUTE_PICK:
                // Do nothing, wait for user to restart or loop
                if (now - state_timer > 3000) {
                    printf("\n--- Restarting Cycle ---\n");
                    state = STATE_SCAN_SHELF;
                }
                break;

            case STATE_FAULT:
                printf("[FAULT] System halted.\n");
                state = STATE_IDLE; // Or just loop forever
                break;
                
            default:
                break;
        }

        // ==========================================
        // 2. UART RX PROCESSING (From IRQ Queue)
        // ==========================================
                // ==========================================
        // 2. UART RX PROCESSING (From IRQ Queue)
        // ==========================================
        uint8_t ch;
        if (queue_try_remove(&rx_queue, &ch)) {
            if (ch == '\n') {
                buffer[index] = '\0';
                
                // Strip \r
                while (index > 0 && buffer[index-1] == '\r') {
                    buffer[--index] = '\0';
                }

                if (index > 0) {
                    char *start;
                    
                    // 1. FIRST, check if it's a packet we care about
                    if (strstr(buffer, "A,") == NULL && strstr(buffer, "C,") == NULL) {
                        // It's a boot log or noise, ignore it silently.
                        // DO NOT use continue; let it fall through to the bottom.
                    } else {
                        // 2. It IS a vision packet, now verify the checksum
                        char *star = strchr(buffer, '*');
                        bool valid_checksum = false;
                        
                        if (star != NULL) {
                            *star = '\0'; 
                            
                            uint8_t expected = 0;
                            for (char *p = buffer; *p; p++) expected ^= (uint8_t)*p;
                            
                            uint8_t received = 0;
                            sscanf(star + 1, "%hhX", &received);

                            if (expected == received) {
                                valid_checksum = true;
                            } else {
                                printf("[Error] Checksum mismatch!\n");
                            }
                        } else {
                            printf("[Error] Vision packet missing checksum.\n");
                        }

                        // 3. Parse Data if Checksum is valid
                        if (valid_checksum) {
                            char color;
                            int x = -1, y = -1, id;
                            float w, dist;

                            if ((start = strstr(buffer, "A,")) != NULL) {
                                if (sscanf(start, "A,%d,%d,%d,%f,%f", &id, &x, &y, &w, &dist) == 5) {
                                    if (state == STATE_WAIT_SHELF_RESULTS) {
                                        
                                        bool found = false;
                                        // 1. Check if we already have this marker ID
                                        for (int i = 0; i < shelf_count; i++) {
                                            if (shelf_map[i].id == id) {
                                                // Update the position (average it or just overwrite)
                                                shelf_map[i].x = x; 
                                                shelf_map[i].y = y;
                                                shelf_map[i].distance = dist;
                                                found = true;
                                                break;
                                            }
                                        }
                                        
                                        // 2. If it's a brand new marker, add it to the array
                                        if (!found && shelf_count < MAX_OBJECTS) {
                                            shelf_map[shelf_count].id = id;
                                            shelf_map[shelf_count].x = x;
                                            shelf_map[shelf_count].y = y;
                                            shelf_map[shelf_count].distance = dist;
                                            shelf_count++;
                                            printf("  -> NEW ARUCO FOUND! ID:%d (Total: %d)\n", id, shelf_count);
                                        }
                                    }
                                }
                            }
                            else if ((start = strstr(buffer, "C,")) != NULL) {
                                if (state == STATE_WAIT_PAYLOAD_RESULTS) {
                                    if (sscanf(start, "C,%c,%d,%d", &color, &x, &y) == 3) {
                                        printf("  -> Payload found: COLOR %c at X:%d Y:%d\n", color, x, y);
                                    } else if (sscanf(start, "C,%c", &color) == 1) {
                                        printf("  -> Payload: COLOR %c (No position)\n", color);
                                    }
                                }
                            }
                        }
                    }
                }
                index = 0; // Reset for next packet
            } 
            else if (index < sizeof(buffer) - 1) {
                buffer[index++] = ch;
            }
        }

        // Yield to prevent CPU starvation
        tight_loop_contents();
    }

    return 0;
}