#include "main.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "timeout.h"
#include "rtc.h"
#include "error_handler.h"

#include "imu.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_rtc.h>
#include <stm32g4xx_hal_pwr.h>


void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();
    while(true) {
        core_GPIO_toggle_heartbeat();
        //RTC->ICSR &= ~RTC_ICSR_RSF;
        //while (!(RTC->ICSR & RTC_ICSR_RSF));
        //struct tm time;
        //core_RTC_get_time(&time);
        //sprintf(txbuf, "ssr: %08x, tr: %08x, dr: %08x\r\n", ssr, tr, dr);
        //strftime(txbuf, 128, "%Y/%m/%d %H:%M:%S ", &time);
        //sprintf(txbuf+strlen(txbuf), "%ld\r\n", core_RTC_get_usec());
        //core_USART_transmit(USART1, txbuf, strlen(txbuf));
        //vTaskDelay(100 * portTICK_PERIOD_MS)
        core_CAN_send_message(FDCAN1, 7, 2, 0x55ff);
        vTaskDelayUntil(&nextWakeTime, 100);
    }
}

int main(void) {
    HAL_Init();

    // Drivers
    core_heartbeat_init(GPIOA, GPIO_PIN_8);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();
    if (!core_CAN_init(FDCAN1, 1000000)) error_handler();

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
