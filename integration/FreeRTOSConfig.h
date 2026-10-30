#pragma once
#include <assert.h>

/* Single-core scheduler on core 0 (Core 1 left free; keeps timing analysis simple). */
#define configNUMBER_OF_CORES                   1
#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      125000000
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    6
#define configMINIMAL_STACK_SIZE                256
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            512
#define configUSE_STREAM_BUFFERS                1
#define configUSE_QUEUE_SETS                    0

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   (96 * 1024)

#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configASSERT(x)                         assert(x)

#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define configUSE_EVENT_GROUPS                  1
