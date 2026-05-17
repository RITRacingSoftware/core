#include "main.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "boot.h"
#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "timeout.h"
#include "rtc.h"
#include "error_handler.h"
#include "watchdog.h"

#include "imu.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_rtc.h>
#include <stm32g4xx_hal_pwr.h>

void watchdog_task(void *pvParameters) {
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();
    while(true) {
        core_watchdog_refresh();
        vTaskDelayUntil(&nextWakeTime, 5000);
    }
}

void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();
    while(true) {
        // core_watchdog_refresh();
        // core_GPIO_toggle_heartbeat();
        vTaskDelayUntil(&nextWakeTime, 100);
    }
}

int main(void) {
    HAL_Init();

    if (!core_clock_init()) error_handler();
    core_heartbeat_init(GPIOB, GPIO_PIN_14);
    core_watchdog_init(false, NULL);

    core_GPIO_set_heartbeat(true);
    HAL_Delay(500);
    core_GPIO_set_heartbeat(false);
    HAL_Delay(500);
    // core_GPIO_set_heartbeat(true);
    // HAL_Delay(500);
    // core_GPIO_set_heartbeat(false);
    HAL_Delay(1000);
    core_watchdog_refresh();

    int err;
    err = xTaskCreate(heartbeat_task, "heartbeat", 1000, NULL, 4, NULL);
    if (err != pdPASS) {
        error_handler();
    }

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // hand control over to FreeRTOS
    vTaskStartScheduler();

    // we should not get here ever
    error_handler();
    return 1;
}

// Called when stack overflows from rtos
// Not needed in header, since included in FreeRTOS-Kernel/include/task.h
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName)
{
    (void) xTask;
    (void) pcTaskName;

    error_handler();
}
