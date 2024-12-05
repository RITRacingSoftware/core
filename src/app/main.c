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

#define CAN FDCAN2

CanMessage_s canMessage;

void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
//    uint32_t tickstart;
//    imu_result_t res;
    TickType_t nextWakeTime = xTaskGetTickCount();
    while(true) {
//        core_GPIO_toggle_heartbeat();
        if (!core_CAN_add_message_to_tx_queue(CAN, 3, 2, 0xfa55)) error_handler();
        vTaskDelayUntil(&nextWakeTime, 100);
    }
}

void can_tx_task(void *pvParameters) {
    (void) pvParameters;
    if (core_CAN_send_from_tx_queue_task(CAN)) core_GPIO_toggle_heartbeat();
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
    //core_heartbeat_init(GPIOB, GPIO_PIN_15);
    core_heartbeat_init(GPIOC, GPIO_PIN_7);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();
    if (!core_CAN_init(CAN)) error_handler();

    int err;
//    err = xTaskCreate(heartbeat_task,
//        "heartbeat",
//        1000,
//        NULL,
//        4,
//        NULL);
//    if (err != pdPASS) {
//        error_handler();
//    }
//
//    err = xTaskCreate(can_tx_task,
//        "tx",
//        1000,
//        NULL,
//        4,
//        NULL);
//    if (err != pdPASS) {
//        error_handler();
//    }
    err = xTaskCreate(can_rx_task,
        "rx",
        1000,
        NULL,
        3,
        NULL);
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
