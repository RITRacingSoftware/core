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
#include "error_handler.h"

#include "imu.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>

#define TEST_CAN_ID1 3
#define TEST_CAN_ID2 3

CanMessage_s canMessage;
core_timeout_t can_timeout;

char txbuf[128];
char imubuf[512];
uint32_t imubuflen;

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
    while(true) {
        if (imubuflen) {
            tickstart = HAL_GetTick();
            core_USART_update_disable(USART3);
            //sprintf(txbuf, "%08lx: data: %ld bytes, %02x %02x\r\n", tickstart, imubuflen, imubuf[0], imubuf[1]);
            imu_parse(imubuf, &res);
            imubuflen = 0;
            core_USART_update_enable(USART3);
            //sprintf(txbuf, "%08lx: accel %d, %d, %d\r\n", tickstart, (int16_t)(1000*res.AccelX), (int16_t)(1000*res.AccelY), (int16_t)(1000*res.AccelZ));
            strcpy(txbuf, "--------: accel ------------, ------------, ------------\r\n");
            printhex(tickstart, 8, 0);
            printfloat(res.AccelX, 12, 16);
            printfloat(res.AccelY, 12, 30);
            printfloat(res.AccelZ, 12, 44);
            core_USART_transmit(USART1, txbuf, strlen(txbuf));
        }
        vTaskDelay(100 * portTICK_PERIOD_MS);
    }
}

int main(void) {
    HAL_Init();

    // Drivers
    core_heartbeat_init(GPIOA, GPIO_PIN_5);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();
    if (!core_USART_init(USART1, 500000)) error_handler();
    if (!core_USART_init(USART3, 921600)) error_handler();
    if (!core_USART_start_rx(USART3, imubuf, &imubuflen)) error_handler();
    imubuflen = 0;

    strcpy(txbuf, "Reset\r\n");
    core_USART_transmit(USART1, txbuf, 7);
    

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
