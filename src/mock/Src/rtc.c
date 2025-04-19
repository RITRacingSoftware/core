#include rtc.h

__weak bool core_RTC_init() {}
__weak void core_RTC_get_time(struct tm *tm) {}
__weak void core_RTC_set_time(struct tm *tm) {}
__weak uint32_t core_RTC_get_usec() {}
