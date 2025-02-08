#include "Transmit.h"

#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "error_handler.h"

#include "FreeRTOS.h"

#include <stm32g4xx_hal.h>

#define CAN_ID 3
#define CAN FDCAN1

void hard_error_handler();


// This task runs every 100ms, and adds a CAN message to the queue
void can_add_to_queue_task(void *pvParameters)
{
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();
    while(true)
    {
        // Adds a message on CAN bus, from CAN_ID, with datalength 2, and data 0xfa55
        // If it is not added to the queue correctly, it will call the error_handler
        if (!core_CAN_add_message_to_tx_queue(CAN, CAN_ID, 2, 0xfa55)) hard_error_handler();
        vTaskDelayUntil(&nextWakeTime, 100);
    }
}

// This task runs once, going into an infinite loop inside the "core_CAN_send_from_tx_queue_task" function.
void can_tx_task(void *pvParameters)
{
    (void) pvParameters;
    core_CAN_send_from_tx_queue_task(CAN);
    hard_error_handler();
}

int main(void) {
    HAL_Init();

    // Drivers
    if (!core_clock_init()) hard_error_handler();
    if (!core_CAN_init(CAN)) hard_error_handler();

    int err;
    err = xTaskCreate(can_add_to_queue_task,
    "heartbeat",
    1000,
    NULL,
    4,
    NULL);
    if (err != pdPASS) hard_error_handler();

    err = xTaskCreate(can_tx_task,
    "tx",
    1000,
    NULL,
    2,
    NULL);
    if (err != pdPASS) hard_error_handler();

    // hand control over to FreeRTOS
    vTaskStartScheduler();

    // we should not get here ever
    hard_error_handler();
    return 1;
}

// Called when stack overflows from rtos
// Not needed in header, since included in FreeRTOS-Kernel/include/task.h
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void) xTask;
    (void) pcTaskName;

    hard_error_handler();
}

void hard_error_handler()
{
    // Infinite loop
    error_handler();
}