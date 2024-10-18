#include "main.h"

#include <stdbool.h>
#include <string.h>

#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "interrupts.h"
#include "error_handler.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stm32g4xx_hal.h>

uint8_t txbuf[4] = {0xab, 0xbb, 0xcc, 0xdd};
volatile uint8_t rxbuf[4];
volatile uint32_t rxbuflen;

void heartbeat_task(void *pvParameters) {
	(void) pvParameters;
	while(true)
	{
//        if (!fake_CAN_send(3, 2, 0xf5aa)) error_handler();
        //if (!core_CAN_send(CAN2, 3, 2, 0xf5aa)) error_handler();
        //core_SPI_read_write(SPI1, txbuf, 2, rxbuf, 2);
        if (rxbuflen && (memcmp(txbuf, txbuf, 4) == 0)) {
            GPIO_toggle_heartbeat();
            rxbuflen = 0;
        }
        if (!core_USART_transmit(USART2, txbuf, 4)) error_handler();
        //GPIO_set_heartbeat((USART1->ISR & USART_ISR_RXNE) && (USART1->RDR == 0xab));
        //GPIO_set_heartbeat(USART1->CR1 & (1<<2));
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

int main(void)
{
    HAL_Init();
	// Drivers
	if (!core_clock_init(1, 24000, 102400)) error_handler();
    if (!core_CAN_init(CAN2)) error_handler();
    //if (!core_SPI_init(SPI1)) error_handler();
    if (!core_USART_init(USART2, 9600)) error_handler();
    if (!core_USART_init(USART1, 9600)) error_handler();
    if (!core_USART_start_rx(USART1, rxbuf, &rxbuflen)) error_handler();

    heartbeat_init(GPIOA, GPIO_PIN_5);
    GPIO_set_heartbeat(GPIO_PIN_SET);

	int err = xTaskCreate(heartbeat_task, 
        "heartbeat", 
        1000,
        NULL,
        4,
        NULL);
    if (err != pdPASS) {
        error_handler();
    }

    // hand control over to FreeRTOS
    vTaskStartScheduler();

    // we should not get here ever
    error_handler();
}

// Called when stack overflows from rtos
// Not needed in header, since included in FreeRTOS-Kernel/include/task.h
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName)
{
	(void) xTask;
	(void) pcTaskName;

    error_handler();
}
