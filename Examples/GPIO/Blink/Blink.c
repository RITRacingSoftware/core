#include "blink.h"
#include <stdbool.h>
#include <stm32g4xx_hal.h>

#include "gpio.h"
#include "clock.h"

#include "FreeRTOS.h"
#include "task.h"

#define HEARTBEAT_PORT GPIOB
#define HEARTBEAT_PIN GPIO_PIN_9

#define GPIO_PORT GPIOB
#define GPIO_PIN GPIO_PIN_10

void hard_error_handler();

void heartbeat_task(void *pvParameters)
{
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();

    while (true)
    {
        core_GPIO_toggle_heartbeat();
        vTaskDelayUntil(&nextWakeTime, 100);
    }
}

void GPIO_blink_task(void *pvParameters)
{
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();

    while (true)
    {
        core_GPIO_digital_write(GPIO_PORT, GPIO_PIN, true);
        vTaskDelayUntil(&nextWakeTime, 500);
        core_GPIO_digital_write(GPIO_PORT, GPIO_PIN, false);
        vTaskDelayUntil(&nextWakeTime, 500);
    }
}

int main()
{
    HAL_Init();

    // Drivers
    if (!core_clock_init()) hard_error_handler();
    core_GPIO_init(GPIO_PORT, GPIO_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_heartbeat_init(HEARTBEAT_PORT, HEARTBEAT_PIN);

    int err;
    err = xTaskCreate(heartbeat_task,
    "heartbeat",
    1000,
    NULL,
    4,
    NULL);
    if (err != pdPASS) hard_error_handler();

    err = xTaskCreate(GPIO_blink_task,
    "heartbeat",
    1000,
    NULL,
    4,
    NULL);
    if (err != pdPASS) hard_error_handler();

    // hand control over to FreeRTOS
    vTaskStartScheduler();

    // we should not get here ever
    hard_error_handler();
    return 1;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void) xTask;
    (void) pcTaskName;
}

void hard_error_handler()
{
    // Infinite loop
    while(true){};
}