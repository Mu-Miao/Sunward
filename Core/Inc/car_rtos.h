#ifndef CAR_RTOS_H
#define CAR_RTOS_H

/* 创建 10ms 整车控制任务并启动调度器。成功后不返回。 */
int CarRTOS_Start(void);
/* 只写 TIM3 硬件寄存器，不分配内存、不等待、不返回。 */
void CarRTOS_FatalStop(void) __attribute__((noreturn));

#endif
