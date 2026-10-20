#include "car_config.h"

#if CAR_USE_FREERTOS
#include "car_rtos.h"
#include "car_app.h"
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
    TickType_t wake = xTaskGetTickCount();
    uint32_t tick10_count = 0U;
    for (;;) {
        CarApp_Poll(++tick10_count);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(CAR_CONTROL_PERIOD_MS));
    }
}

int CarRTOS_Start(void)
{
    if (xTaskCreate(CarControl_Task, "CarControl", CAR_CONTROL_STACK_WORDS,
            NULL, CAR_CONTROL_PRIORITY, NULL) != pdPASS) return 0;
    vTaskStartScheduler();
    return 0;
}

void vApplicationTickHook(void)
{
    /* 调度器接管 SysTick 后继续维护 HAL_GetTick/HAL_Delay 的 1ms 时基。 */
    HAL_IncTick();
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}
#else
#include "car_rtos.h"
int CarRTOS_Start(void) { return 0; }
#endif
