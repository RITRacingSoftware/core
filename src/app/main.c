#include "main.h"

#include <stdbool.h>
#include <string.h>

#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "timeout.h"
#include "error_handler.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>

#define TEST_CAN_ID1 3
#define TEST_CAN_ID2 3

CanMessage_s canMessage;
core_timeout_t can_timeout;

uint8_t txbuf[128];

void printdec(uint32_t num, uint8_t digits, uint8_t offset) {
    for (uint8_t x=0; x<digits; x++) {
        txbuf[offset+digits-1-x] = '0' + (num % 10);
        num = num / 10;
    }
}

void printhex(uint32_t num, uint8_t digits, uint8_t offset) {
    for (uint8_t x=0; x<digits; x++) {
        txbuf[offset+digits-1-x] = ((num & 0xf) >= 10 ? 'A' + (num&0xf) - 10 : '0' + (num&0xf));
        num = num >> 4;
    }
}

void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
    uint32_t tickstart;
    while(true) {
        uint16_t adc;
        core_timeout_check_all();
        vTaskDelay(50 * portTICK_PERIOD_MS);
    }
}

void can_tx_task(void *pvParameters) {
    (void) pvParameters;
    if (core_CAN_send_from_tx_queue_task(FDCAN3)) error_handler();
}

void receive_CAN_task(void *pvParameters) {
    (void) pvParameters;
    while(true) {
        if (core_CAN_receive_from_queue(FDCAN3, &canMessage)) {
            core_GPIO_toggle_heartbeat();
            strcpy(txbuf, "data: ----\r\n");
            printhex(canMessage.data, 4, 6);
            core_USART_transmit(USART1, txbuf, strlen(txbuf));
            //if (canMessage.data == 0xfa55) core_GPIO_toggle_heartbeat();
        }
        vTaskDelay(5 * portTICK_PERIOD_MS);
    }
}

void can_timeout_callback(core_timeout_t *to) {
    core_GPIO_toggle_heartbeat();
    strcpy(txbuf, "Timed out\r\n");
    core_USART_transmit(USART1, txbuf, strlen(txbuf));
}

int main(void) {
    HAL_Init();

    // Drivers
    core_heartbeat_init(GPIOA, GPIO_PIN_5);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();
    if (!core_USART_init(USART1, 500000)) error_handler();
    if (!core_CAN_init(FDCAN3)) error_handler();
    if (!core_CAN_add_filter(FDCAN3, false, TEST_CAN_ID1, TEST_CAN_ID2)) error_handler();
    
    can_timeout.module = FDCAN3;
    can_timeout.ref = TEST_CAN_ID1;
    can_timeout.timeout = 1000;
    can_timeout.callback = can_timeout_callback;
    core_timeout_insert(&can_timeout);
    core_timeout_start_all();

    strcpy(txbuf, "data: ----\r\n");


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
    err = xTaskCreate(receive_CAN_task,
        "CAN_RX",
        1000,
        NULL,
        4,
        NULL);
    if (err != pdPASS) {
        error_handler();
    }
    /*err = xTaskCreate(can_tx_task,
        "tx_task",
        1000,
        NULL,
        3,
        NULL);
    if (err != pdPASS) {
        error_handler();
    }*/

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
