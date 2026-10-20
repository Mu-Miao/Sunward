#ifndef CAR_RTOS_H
#define CAR_RTOS_H

/* 创建 10ms 整车控制任务并启动调度器。成功后不返回。 */
int CarRTOS_Start(void);
/* 所有致命异常共用的寄存器级停车；不依赖 HAL、堆或调度器。 */
void CarEmergency_Stop(void) __attribute__((noreturn));
/* FreeRTOS 钩子的兼容入口，最终进入同一紧急停车路径。 */
void CarRTOS_FatalStop(void) __attribute__((noreturn));

#endif
