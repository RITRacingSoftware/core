#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "rtc.h"
#include "clock.h"
#include "core_config.h"
#include "timestamp.h"
#include <stm32g4xx_hal.h>

static uint32_t core_RTC_last_usec = 0;

#ifndef CORE_RTC_ASYNC_PRESCALER
#define CORE_RTC_ASYNC_PRESCALER 128
#endif

#ifndef CORE_RTC_SYNC_PRESCALER
#define CORE_RTC_SYNC_PRESCALER 256
#endif

bool core_RTC_init(bool force_init) {
    if (!core_clock_RTC_init()) return false;

    if (force_init || !(RTC->ICSR & RTC_ICSR_INITS)) {
        RTC->WPR = 0xCA;
        RTC->WPR = 0x53;
        // Enter initialization mode
        RTC->ICSR |= RTC_ICSR_INIT;
        while (!(RTC->ICSR & RTC_ICSR_INITF));
        RTC->CR = 0;
        // Set prescaler
        RTC->PRER = (RTC->PRER & 0xffff0000) | (((CORE_RTC_SYNC_PRESCALER) - 1)&0x7fff);
        RTC->PRER = ((((CORE_RTC_ASYNC_PRESCALER) - 1)&0x7f)<<16) | (((CORE_RTC_SYNC_PRESCALER) - 1)&0x7fff);
        RTC->ICSR &= ~RTC_ICSR_INIT;
        RTC->WPR = 0xff;
    }
    return true;
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

void core_RTC_set_time(struct tm *tm, uint64_t sync) {
    uint32_t tr = 0, dr = 0;
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ICSR |= RTC_ICSR_INIT;
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

