#include <assert.h>
#include <stdio.h>
#include "car_tick.h"

int main(void)
{
    CarTick_Reset();
    assert(!CarTick_UseKernel());
    CarTick_EnableKernel();
    assert(CarTick_UseKernel());
    CarTick_Reset();
    assert(!CarTick_UseKernel());
    puts("PASS: pre-scheduler ticks route to HAL; initialized kernel states route to FreeRTOS.");
    return 0;
}
