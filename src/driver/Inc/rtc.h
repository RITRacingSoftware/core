#include <stdint.h>
#include <time.h>

#define CORE_RTC_FORMAT_BIN 0
#define CORE_RTC_FORMAT_BCD 1

typedef struct core_RTC_tm {
    uint32_t tm_usec;
    uint8_t tm_sec;
    uint8_t tm_min;
    uint8_t tm_hour;
    uint8_t tm_mday;
    uint8_t tm_mon;
    uint16_t tm_year;
    uint8_t tm_yday;
    uint8_t tm_format;
} core_RTC_tm_t;

bool core_RTC_init();
void core_RTC_get_time(struct tm *tm);
void core_RTC_set_time(struct tm *tm);
uint32_t core_RTC_get_usec();

