#pragma once
#include "includes.h"

#define M_PI_FLOAT  3.14159265358979323846f
#define REFRESH_RATE  0.00003125f
#define DT  1.0f / REFRESH_RATE

typedef struct ptnFilter_axis_s {
    float state[5];
    float k;
    uint8_t order;
} ptnFilter_axis_t;

typedef struct ptnFilter_s {
    ptnFilter_axis_t x;
    ptnFilter_axis_t y;
    ptnFilter_axis_t z;
} ptnFilter_t;

void ptnFilterInit(ptnFilter_axis_t *filter, uint8_t order, float f_cut);
void ptnFilterUpdate(float f_cut, ptnFilter_axis_t *filter, float ScaleF);
float ptnFilterApply(float input, ptnFilter_axis_t *filter);
