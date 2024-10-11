#include "main.h"

#include <stdbool.h>
#include <stdio.h>

#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "interrupts.h"
#include "error_handler.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>

CanMessage_s canMessage;
uint8_t txbuf[4];
uint8_t rxbuf[4];

void send_CAN_task (void *pvParameters)
{
    (void) pvParameters;
    core_CAN_send_from_tx_queue_task(FDCAN3);
}

void receive_CAN_task(void *pvParameters)
{
    (void) pvParameters;
    while(true)
    {
        if (core_CAN_receive_from_queue(FDCAN2, &canMessage))
        {
            if (canMessage.data == 0xfa55) GPIO_toggle_heartbeat();
        }
        vTaskDelay(50 * portTICK_PERIOD_MS);
    }
}

void heartbeat_task(void *pvParameters)
{
	(void) pvParameters;
    uint16_t value;
	while(true)
	{
//        CAN_send_message(FDCAN3, 3, 2, 0xfa55);
        GPIO_toggle_heartbeat();
        core_ADC_read_channel(GPIOA, GPIO_PIN_0, &value, NULL);
        txbuf[0] = 0x0f;
        txbuf[1] = value>>8;
        txbuf[2] = value&0xff;
        core_SPI_read_write(SPI1, txbuf, 3, rxbuf, 3);
        vTaskDelay(100 * portTICK_PERIOD_MS);
	}
}

int main(void)
{
    HAL_Init();
	// Drivers

    heartbeat_init(GPIOA, GPIO_PIN_5);
    GPIO_set_heartbeat(GPIO_PIN_RESET);

	if (!core_clock_init()) error_handler();
    if (!core_ADC_init(ADC1)) error_handler();
    if (!core_SPI_init(SPI1)) error_handler();

    core_ADC_setup_pin(GPIOA, GPIO_PIN_0);

	int err = xTaskCreate(heartbeat_task,
    "heartbeat",
    1000,
    NULL,
    4,
    NULL);
    if (err != pdPASS) {
        error_handler();
    }

    /*err = xTaskCreate(send_CAN_task,
    "CAN_TX",
    1000,
    NULL,
    3,
    NULL);
    if (err != pdPASS) {
        error_handler();
    }*/

//    int err = xTaskCreate(receive_CAN_task,
//        "CAN_RX",
//        1000,
//        NULL,
//        4,
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
