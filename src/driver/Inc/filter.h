#pragma once
#include <stdint.h>

typedef enum {
    Filter_ROLLING_AVG
} Filter_type_e;

typedef struct core_filter_s {
    Filter_type_e type;
    uint8_t posX;
    uint8_t posY;
    uint8_t orderX;
    uint8_t orderY;
    float *prevX;
    float *prevY;
    float (*func) (float x, struct core_filter_s *filt);
} core_filter_t;

void core_filter_init(core_filter_t *filt);
float core_filter_update(float x, core_filter_t *filt);
