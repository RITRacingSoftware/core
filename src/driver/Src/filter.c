#include "filter.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

static float rolling_avg(float x, core_filter_t *filt);

void core_filter_init(core_filter_t *filt)
{
    if (filt->orderX > 0) filt->prevX = (float *)malloc(sizeof(float) * filt->orderX);
    if (filt->orderY > 0) filt->prevY = (float *)malloc(sizeof(float) * filt->orderY);
    filt->posX = 0;
    filt->posY = 0;

    switch (filt->type)
    {
        case Filter_ROLLING_AVG:
            filt->func = rolling_avg;
            break;
    }
}

float core_filter_update(float x, core_filter_t *filt)
{
    return filt->func(x, filt);
}

static float rolling_avg(float x, struct core_filter_s *filt)
{
    filt->posX = (filt->posX) % filt->orderX;
    filt->prevX[filt->posX] = x;
    filt->posX++;
    int sum = 0;
    for (int i = 0; i < filt->orderX; i++) {
        sum += filt->prevX[i];
    }

    return (sum / ((float)filt->orderX));
}
