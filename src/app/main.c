#include "main.h"

#include <stdbool.h>
#include <string.h>

#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "error_handler.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>

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
    while(true) {
        uint16_t adc;
        core_ADC_read_channel(GPIOA, GPIO_PIN_1, &adc);
        printdec(adc, 4, 7);
        printhex(OPAMP3->CSR, 8, 12);
        core_USART_transmit(USART1, txbuf, strlen(txbuf));
        vTaskDelay(100 * portTICK_PERIOD_MS);
    }
}

int main(void) {
    HAL_Init();

	// Drivers
    core_heartbeat_init(GPIOB, GPIO_PIN_9);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

	if (!core_clock_init()) error_handler();
    if (!core_ADC_init(ADC1)) error_handler();
    if (!core_USART_init(USART1, 500000)) error_handler();
    strcpy(txbuf, "value: ---- --------\r\n");
    core_ADC_setup_pin(GPIOA, GPIO_PIN_1, 1);
    //OPAMP1->CSR = OPAMP_CSR_VMSEL_0 | OPAMP_CSR_VMSEL_1 | 1 | OPAMP_CSR_OPAMPINTEN;


    int err = xTaskCreate(heartbeat_task,
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
