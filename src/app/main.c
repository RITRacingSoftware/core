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

#include "imu.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_rtc.h>
#include <stm32g4xx_hal_pwr.h>

#define TEST_CAN_ID1 3
#define TEST_CAN_ID2 3

#define CAN FDCAN1

CanMessage_s canMessage;
core_timeout_t can_timeout;

char txbuf[128];
char imubuf[512];
uint32_t imubuflen;
uint8_t counter = 0;

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

void printfloat(float f, uint8_t chars, uint8_t offset) {
    if (f < 0) {
        f = -f;
        txbuf[offset] = '-';
    } else txbuf[offset] = '+';
    // -x.xxxe+yy --> lower 10000
    float lower_limit = 1;
    for (uint8_t i=7; i < chars; i++) lower_limit *= 10;
    int8_t exponent = chars - 7;
    while (f < lower_limit) {
        f *= 10;
        exponent -= 1;
    }
    lower_limit *= 10;
    while (f >= lower_limit) {
        f /= 10;
        exponent += 1;
    }
    printdec(round(f), chars-6, offset+2);
    txbuf[offset+1] = txbuf[offset+2];
    txbuf[offset+2] = '.';
    txbuf[offset+chars-4] = (exponent < 0 ? '-' : '+');
    txbuf[offset+chars-3] = 'e';
    printdec((exponent < 0 ? -exponent : exponent), 2, offset+chars-2);
}

void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
    uint32_t tickstart;
    imu_result_t res;
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
        //vTaskDelay(100 * portTICK_PERIOD_MS);
        vTaskDelayUntil(&nextWakeTime, 500);
    }
}

void can_tx_task(void *pvParameters) {
    (void) pvParameters;
    core_CAN_send_from_tx_queue_task(CAN);
    error_handler();
}

void can_rx_task(void *pvParameters) {
    (void) pvParameters;
    CanExtendedMessage_s msg;
    while (1) {
        if (core_CAN_receive_extended_from_queue(FDCAN2, &msg)) {
            core_GPIO_toggle_heartbeat();
            sprintf(txbuf, "received %d bytes\r\n", msg.dlc);
            core_USART_transmit(USART1, txbuf, strlen(txbuf));
        }
        vTaskDelay(10 * portTICK_PERIOD_MS);
    }
}

int main(void) {
    uint32_t rst = RCC->CSR;
    HAL_Init();

    // Drivers
    //core_heartbeat_init(GPIOB, GPIO_PIN_15);
    core_heartbeat_init(GPIOA, GPIO_PIN_5);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();
    core_boot_init();
    //if (!core_USART_init(USART1, 500000)) error_handler();
    /*if (!core_RTC_init()) error_handler();

    struct tm time;
    memset(&time, 0, sizeof(time));
    time.tm_year = 124;
    time.tm_mon = 10;
    time.tm_mday = 13;
    core_RTC_set_time(&time);

    strcpy(txbuf, "Reset\r\n");
    core_USART_transmit(USART1, txbuf, 7);*/
    

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
    /*err = xTaskCreate(can_tx_task,
        "tx",
        1000,
        NULL,
        3,
        NULL);
    if (err != pdPASS) {
        error_handler();
    }
    err = xTaskCreate(can_rx_task,
        "rx",
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
