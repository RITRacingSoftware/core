#include "ADC_read.h"

#include <stdbool.h>

#include "gpio.h"
#include "clock.h"
#include "rtt.h"
#include "adc.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stm32g4xx_hal.h>

#define HEARTBEAT_PORT GPIOA
#define HEARTBEAT_PIN GPIO_PIN_4

#define ANA_PORT GPIOC
#define ANA_PIN GPIO_PIN_3

void hard_error_handler();

void heartbeat_task(void *pvParameters)
{
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();

    while (true)
    {
        core_GPIO_toggle_heartbeat();
        vTaskDelayUntil(&nextWakeTime, 500);
    }
}

void analog_read_task(void *pvParameters)
{
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();

    uint16_t res;
    while (true)
    {
        core_ADC_read_channel(ANA_PORT, ANA_PIN, &res);
        rprintf("ADC Value: %d. Voltage (mV): %d\n", res, (3300 * res)>>12);
        vTaskDelayUntil(&nextWakeTime, 100);
    }
}

int main()
{
    HAL_Init();
    // Drivers
    if (!core_clock_init()) hard_error_handler();
    core_heartbeat_init(HEARTBEAT_PORT, HEARTBEAT_PIN);
    core_GPIO_digital_write(HEARTBEAT_PORT, HEARTBEAT_PIN, true);
    core_RTT_init();
    rprintf("Init\n");
    if (!core_ADC_init(ADC5)) hard_error_handler();
    if (!core_ADC_setup_pin(ANA_PORT, ANA_PIN, 1)) hard_error_handler();


    int err;

    err = xTaskCreate(heartbeat_task, "heartbeat", 1000, NULL, 4, NULL);
    if (err != pdPASS) hard_error_handler();

    err = xTaskCreate(analog_read_task, "analog_read", 1000, NULL, 4, NULL);
    if (err != pdPASS) hard_error_handler();

    // hand control over to FreeRTOS
    vTaskStartScheduler();

    // we should not get here ever
    hard_error_handler();
    return 1;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void) xTask;
    (void) pcTaskName;
}

void hard_error_handler()
{
    // Infinite loop
    while(true){};
}
