#include <stdint.h>
#include <stdbool.h>

#include "stm32g4xx_hal.h"

#include "clock.h"
#include "core_config.h"

#ifdef CORE_CAN_TIMER
#define CORE_TIMESTAMP_MSB CORE_CAN_TIMER
#endif

void core_timestamp_init() {
    core_clock_timer_init(TIM3);
    core_clock_timer_init(CORE_TIMESTAMP_MSB);
    // TIM3 counts at 1MHz and outputs the update (overflow) event
    TIM3->CR1 = 0x0000;
    TIM3->CR2 = TIM_TRGO_UPDATE;
    TIM3->PSC = 159;
    TIM3->ARR = 0xffff;
    TIM3->EGR = TIM_EGR_UG;
    // CORE_TIMESTAMP_MSB is clocked from TIM3's overflow event
    CORE_TIMESTAMP_MSB->CR1 = 0x0000;
    CORE_TIMESTAMP_MSB->CR2 = 0x0000;
    CORE_TIMESTAMP_MSB->SMCR = TIM_TS_ITR2 | TIM_SLAVEMODE_EXTERNAL1;
    CORE_TIMESTAMP_MSB->PSC = 0;
    CORE_TIMESTAMP_MSB->ARR = 0xffffffff;
    // Reset timers
    TIM3->CNT = 0;
    CORE_TIMESTAMP_MSB->CNT = 0;
    // Start counting
    TIM3->CR1 |= TIM_CR1_CEN;
    CORE_TIMESTAMP_MSB->CR1 |= TIM_CR1_CEN;
}

void core_timestamp_read_separate(uint16_t *lsb, uint32_t *msb) {
    uint32_t t1, t2, t3;
    t1 = CORE_TIMESTAMP_MSB->CNT;
    t2 = TIM3->CNT;
    t3 = CORE_TIMESTAMP_MSB->CNT;
    *msb = t1;
    if (t1 != t3) *lsb = 0xffff;
    else *lsb = t2;
}
uint64_t core_timestamp_get_tick() {
    uint32_t t1, t2, t3;
    t1 = CORE_TIMESTAMP_MSB->CNT;
    t2 = TIM3->CNT;
    t3 = CORE_TIMESTAMP_MSB->CNT;
    if (t1 != t3) return (((uint64_t)t1) << 16) | 0xffff;
    else return (((uint64_t)t1) << 16) | t2;
}
