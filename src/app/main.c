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

#define TEST_CAN_ID1 3
#define TEST_CAN_ID2 3

#define CAN FDCAN3

#define APPS_A_PORT GPIOA
#define APPS_A_PIN GPIO_PIN_3
#define APPS_B_PORT GPIOA
#define APPS_B_PIN GPIO_PIN_4

CanMessage_s canMessage;

void heartbeat_task(void *pvParameters)
{
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();
    while(true)
    {
        uint16_t appsAVal = 0;
        uint16_t appsBVal = 0;

        core_ADC_read_channel(APPS_A_PORT, APPS_A_PIN, &appsAVal);
        core_ADC_read_channel(APPS_B_PORT, APPS_B_PIN, &appsBVal);
//        uprintf(USART3, "A: %d, B: %d\n", appsAVal, appsBVal);
        vTaskDelayUntil(&nextWakeTime, 100);
    }
}

void can_tx_task(void *pvParameters)
{
    (void) pvParameters;
    error_handler();
}

void can_rx_task(void *pvParameters) {
    (void) pvParameters;
    CanMessage_s msg;
    while (1) {
        if (core_CAN_receive_extended_from_queue(CAN, &msg))
        {
            /*if (msg.data == 0xfa55) */core_GPIO_toggle_heartbeat();
//            sprintf(txbuf, "received %d bytes\r\n", msg.dlc);
//            core_USART_transmit(USART1, txbuf, strlen(txbuf));
        }
        vTaskDelay(100 * portTICK_PERIOD_MS);
    }
}

int main(void) {
    HAL_Init();

    // Drivers
    if (!core_clock_init()) error_handler();
    if (!core_USART_init(USART3, 500000)) error_handler();
    if (!core_ADC_init(ADC1)) error_handler();
    if (!core_ADC_init(ADC2)) error_handler();
    if (!core_ADC_setup_pin(APPS_A_PORT, APPS_A_PIN, 0)) error_handler();
    if (!core_ADC_setup_pin(APPS_B_PORT, APPS_B_PIN, 0)) error_handler();


    int err;
    err = xTaskCreate(heartbeat_task,
        "heartbeat",
        1000,
        NULL,
        4,
        NULL);
    if (err != pdPASS) {
        error_handler();
    }

//    err = xTaskCreate(can_tx_task,
//        "tx",
//        1000,
//        NULL,
//        2,
//        NULL);
//    if (err != pdPASS) {
//        error_handler();
//    }
//    err = xTaskCreate(can_rx_task,
//        "rx",
//        1000,
//        NULL,
//        3,
//        NULL);
//    if (err != pdPASS) {
//        error_handler();
//    }

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