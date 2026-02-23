#include "filter.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>


float core_filter_update(float x, core_filter_t *filt)
{
    float y = 0;

    filt->prevX[filt->posX] = x;    // Add in new value

    // Loop through array of X and Y values and multiply by X and Y coefficients
    // Modulus because the prevX and prevY arrays are circular buffers
    for (int i = 0; i < filt->orderX; i++)  {
        y += (filt->prevX[(filt->posX + i) % filt->orderX]) * (filt->coeffX[i]);
    }

    for (int i = 0; i < filt->orderY; i++) { 
        y += (filt->prevY[(filt->posY + i) % filt->orderY]) * (filt->coeffY[i]);
    }

    filt->posX = ++filt->posX % filt->orderX;

    if (filt->orderY > 0) {
        filt->prevY[filt->posY] = y;
        filt->posY = ++filt->posY % filt->orderY;
    }
    return y;
}

bool core_filter_rolling_avg_init(uint8_t order, core_filter_t *filt)
{
    if (order == 0) return false;
    filt->orderX = order;
    filt->prevX = (float *)calloc(order, sizeof(float));
    if (filt->prevX == NULL) return false;
    filt->coeffX = (float *)calloc(order, sizeof(float));
    if (filt->coeffX == NULL) return false;
    filt->orderY = 0;
    filt->posX = 0;
    filt->posY = 0;

    float coeff = 1.0f / (float)order;
    for (int i = 0; i < order; i++) {
        filt->coeffX[i] = coeff;
    }

    return true;
}

bool core_filter_exp_lowpass_init(float a, core_filter_t *filt)
{
    filt->orderX = 1;
    filt->orderY = 1;
    filt->prevX = (float *)calloc(1, sizeof(float));
    if (filt->prevX == NULL) return false;
    filt->prevY = (float *)calloc(1, sizeof(float));
    if (filt->prevY == NULL) return false;
    filt->coeffX = (float *)calloc(1, sizeof(float));
    if (filt->prevX == NULL) return false;
    filt->coeffY = (float *)calloc(1, sizeof(float));
    if (filt->prevY == NULL) return false;
    filt->posX = 0;
    filt->posY = 0;

    filt->coeffX[0] = a;
    filt->coeffY[0] = (1-a);

    return true;
}
