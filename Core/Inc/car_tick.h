#ifndef CAR_TICK_H
#define CAR_TICK_H

/* 就绪位由首个控制任务置位；置位前 SysTick 不访问任何 RTOS API。 */
void CarTick_Reset(void);
void CarTick_EnableKernel(void);
int CarTick_UseKernel(void);

#endif
