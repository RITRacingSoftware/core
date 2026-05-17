#include "pwm.h"

#include <stdbool.h>

#include "clock.h"

#include "core_config.h"
#include "stm32g4xx_hal.h"

bool core_PWM_input_init(TIM_TypeDef *tim, uint8_t chan, GPIO_TypeDef *port, uint16_t pin, uint8_t af)
{
    /*** Initialize pin ***/
    core_clock_port_init(port);
    GPIO_InitTypeDef pwm_init = {pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, af};
    HAL_GPIO_Init(port, &pwm_init);


    /*** Initialize timer ***/
    core_clock_timer_init(tim);
    // Timers being used by core timestamp
    if (tim == TIM3 || tim == CORE_CAN_TIMER) return false;

    // Only channels 1 and 2 are supported for input PWM
    if (chan == 1) {
        tim->CCMR1 |= 0b01;
        tim->CCER = 0;
        tim->CCMR1 |= (0b10 << 8);
        tim->CCER = 1 << 5;
        tim->SMCR = 0b101 << 4;
        tim->SMCR |= 0b100;
        tim->CCER |= 0b10001;
        // tim->CCMR1 = (0b10 << 8) | 0b01;     // Map input capture 2 and input capture 1 to timer 1
        // tim->CCER = (1 << 5);                // Set input capture 1 active on rising edge, input capture 2 acive on falling edge
        // tim->SMCR = (0b101 << 4) | 0b100;    // Set trigger selection to Filtered Timer Input 1, update register on rising edge
        // tim->CCER |= (1 << 4) | 1;           // Enable Cacpture/Compare 2 and 1
    }
    else if (chan == 2){
        // Timers 16 and 17 only support channel 1 for input PWM
        if (tim == TIM16 || tim == TIM17) return false;
        tim->CCMR1 = (0b01 << 8)| 0b10;     // Map input capture 2 and input capture 1 to timer 2
        tim->CCER = 0;      // Set active on rising edge
    } 

    return true;
}

uint32_t core_PWM_read_pulsewidth(TIM_TypeDef *tim, uint8_t chan) {
    if (chan == 1) return tim->CCR2;
    return 0;
}

uint32_t core_PWM_read_period(TIM_TypeDef *tim, uint8_t chan) {
    if (chan == 1) return tim->CCR1;
    return 0;
}

