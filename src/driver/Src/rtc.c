#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "rtc.h"
#include "clock.h"
#include "core_config.h"
#include <stm32g4xx_hal.h>

static uint32_t core_RTC_last_usec = 0;

bool core_RTC_init() {
    if (!core_clock_RTC_init()) return false;

    RTC_HandleTypeDef hrtc;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    return (HAL_RTC_Init(&hrtc) == HAL_OK);
}

void core_RTC_get_time(struct tm *tm) {
    uint32_t pre = (RTC->PRER & 0x00007fff);
    uint32_t ssr = RTC->SSR;
    uint32_t tr = RTC->TR;
    uint32_t dr = RTC->DR;
    core_RTC_last_usec = ((pre - ssr) * 1000000ULL) / (pre + 1);
    tm->tm_sec = (tr & 0x0000000f) + ((tr & 0x00000070) >> 4)*10;
    tm->tm_min = ((tr & 0x00000f00) >> 8) + ((tr & 0x00007000) >> 12)*10;
    tm->tm_hour = ((tr & 0x000f0000) >> 16) + ((tr & 0x00300000) >> 20)*10;
    tm->tm_mday = (dr & 0x0000000f) + ((dr & 0x00000030) >> 4)*10;
    tm->tm_mon = ((dr & 0x00000f00) >> 8) + ((dr & 0x00001000) >> 12)*10 - 1;
    tm->tm_year = CORE_RTC_CENTURY + ((dr & 0x000f0000)>>16) + ((dr & 0x00f00000) >> 20)*10 - 1900;
    tm->tm_wday = (dr & 0x0000e000) >> 13;
    if (tm->tm_wday == 7) tm->tm_wday = 0;
    //tm->tm_format = CORE_RTC_FORMAT_BCD;
}

void core_RTC_set_time(struct tm *tm) {
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ICSR |= RTC_ICSR_INIT;
    uint32_t tr = 0, dr = 0;
    tr |= ((tm->tm_sec / 10) << 4) | ((tm->tm_sec % 10));
    tr |= ((tm->tm_min / 10) << 12) | ((tm->tm_min % 10) << 8);
    tr |= ((tm->tm_hour / 10) << 20) | ((tm->tm_hour % 10) << 16);
    dr |= ((tm->tm_mday / 10) << 4) | ((tm->tm_mday % 10));
    uint8_t mon = tm->tm_mon + 1;
    dr |= ((mon / 10) << 12) | ((mon % 10) << 8);
    uint8_t wday = tm->tm_wday;
    if (wday == 0) wday = 7;
    dr |= (wday << 13);
    uint16_t year = tm->tm_year + 1900 - CORE_RTC_CENTURY;
    dr |= (((year / 10) & 0xf) << 20) | (((year % 10) & 0xf) << 16);
    while (!(RTC->ICSR & RTC_ICSR_INITF));
    RTC->TR = tr;
    RTC->DR = dr;
    RTC->ICSR &= ~RTC_ICSR_INIT;
    RTC->WPR = 0xff;
}

uint32_t core_RTC_get_usec() {
    return core_RTC_last_usec;
}

