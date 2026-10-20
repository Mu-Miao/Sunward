#include "car_tick.h"

static volatile unsigned kernel_ready;

void CarTick_Reset(void)
{
    kernel_ready = 0U;
}

void CarTick_EnableKernel(void)
{
    kernel_ready = 1U;
}

int CarTick_UseKernel(void)
{
    return kernel_ready != 0U;
}
