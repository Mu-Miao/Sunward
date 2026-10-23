#include "car_config.h"
#include "car_rtos.h"
#include "stm32f1xx.h"
#include <stddef.h>

#if TIM3_BASE != 0x40000400UL
#error "Update CarEmergency_Stop for this MCU's TIM3 base address"
#endif
_Static_assert(offsetof(TIM_TypeDef, EGR) == 20U, "unexpected TIM EGR offset");
_Static_assert(offsetof(TIM_TypeDef, ARR) == 44U, "unexpected TIM ARR offset");
_Static_assert(offsetof(TIM_TypeDef, CCR1) == 52U, "unexpected TIM CCR1 offset");
_Static_assert(offsetof(TIM_TypeDef, CCR4) == 64U, "unexpected TIM CCR4 offset");

/* 裸函数不读写当前栈；即使 HardFault 或栈溢出破坏了 MSP/PSP，
   仍可只用核心寄存器覆盖 TIM3 输出。偏移来自 STM32F1 TIM_TypeDef。 */
void CarEmergency_Stop(void) __attribute__((naked, noreturn));
void CarEmergency_Stop(void)
{
    __asm volatile (
        "cpsid i\n"
        "ldr r0, =0x40000400\n" /* TIM3 */
        "ldr r1, [r0, #44]\n"   /* ARR */
        "adds r1, r1, #1\n"
        "str r1, [r0, #52]\n"   /* CCR1 */
        "str r1, [r0, #56]\n"   /* CCR2 */
        "str r1, [r0, #60]\n"   /* CCR3 */
        "str r1, [r0, #64]\n"   /* CCR4 */
        "movs r1, #1\n"
        "str r1, [r0, #20]\n"   /* EGR.UG */
        "dsb\n"
        "isb\n"
        "b .\n"
    );
}

void CarRTOS_FatalStop(void) __attribute__((alias("CarEmergency_Stop"), noreturn));

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
