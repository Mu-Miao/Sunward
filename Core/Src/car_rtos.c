#include "car_config.h"
#include "car_rtos.h"
#include "stm32f1xx.h"

void CarRTOS_FatalStop(void)
{
    /* 与 Lab_MotorRawAB(0, 0) 相同的双桥制动值。直接触发更新，
       即使调度器/堆/栈已损坏也不需要 HAL 或 RTOS 服务。 */
    __disable_irq();
    uint32_t brake = TIM3->ARR + 1U;
    TIM3->CCR1 = brake;
    TIM3->CCR2 = brake;
    TIM3->CCR3 = brake;
    TIM3->CCR4 = brake;
    TIM3->EGR = TIM_EGR_UG;
    __DSB();
    __ISB();
    for (;;) {}
}

#if CAR_USE_FREERTOS
#include "car_app.h"
#include "car_tick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

enum {
    CAR_CONTROL_PERIOD_MS = 10U,
    CAR_CONTROL_PRIORITY = 4U,
    CAR_CONTROL_STACK_WORDS = 1024U
};

static void CarControl_Task(void *argument)
{
    (void)argument;
    /* 任务已经真正开始运行，此后 SysTick 才允许进入内核。 */
    CarTick_EnableKernel();
    TickType_t wake = xTaskGetTickCount();
    uint32_t tick10_count = 0U;
    for (;;) {
        CarApp_Poll(++tick10_count);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(CAR_CONTROL_PERIOD_MS));
    }
}

int CarRTOS_Start(void)
{
    CarTick_Reset();
    if (xTaskCreate(CarControl_Task, "CarControl", CAR_CONTROL_STACK_WORDS,
            NULL, CAR_CONTROL_PRIORITY, NULL) != pdPASS) return 0;
    vTaskStartScheduler();
    CarRTOS_FatalStop();
}

void vApplicationTickHook(void)
{
    /* 调度器接管 SysTick 后继续维护 HAL_GetTick/HAL_Delay 的 1ms 时基。 */
    HAL_IncTick();
}

void vApplicationMallocFailedHook(void)
{
    CarRTOS_FatalStop();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    CarRTOS_FatalStop();
}
#else
int CarRTOS_Start(void) { return 0; }
#endif
