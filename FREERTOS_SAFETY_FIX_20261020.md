# FreeRTOS 启动与致命故障修复记录

## SysTick 交接

应用保留唯一的 `SysTick_Handler`，不再把 FreeRTOS port 的 `xPortSysTickHandler` 直接映射到启动向量。

- `HAL_Init()` 后到调度器启动前：就绪位为 0，ISR 只调用 `HAL_IncTick()`，自检、MPU6050、OLED 和 `HAL_Delay()` 不访问 FreeRTOS 列表。
- `vTaskStartScheduler()` 内部关中断完成内核启动，最高优先级 `CarControl` 首次真正执行时才置就绪位。
- 就绪后：ISR 只转发 `xPortSysTickHandler()`；`vApplicationTickHook()` 每个内核 tick 调用一次 `HAL_IncTick()`，不会重复累加 HAL 时基。
- 内核暂停调度时就绪位不清除，SysTick 仍必须由 FreeRTOS 计入挂起 tick。

`test_rtos_boundary.c` 验证就绪位的启动前、启动后和重置语义。交叉构建脚本还会反汇编最终 ELF，确认 RTOS 版 `SysTick_Handler` 同时包含 HAL 路径和受门控的 port 路径，裸机版只包含 HAL 路径。

## 致命故障停车

`CarEmergency_Stop()` 是不使用当前栈的裸汇编入口，不调用调度器、堆分配、HAL 延时或串口：

1. 首先关中断；
2. 直接将 TIM3 CCR1至CCR4 写为 `ARR+1`，与现有 `Lab_MotorRawAB(0, 0)` 的双桥制动状态一致；
3. 直接写 EGR.UG，使预装载比较值立即进入活动寄存器；
4. 执行 DSB/ISB 后死循环。

NMI、HardFault、MemManage、BusFault、UsageFault 使用无栈分支进入该入口；`Error_Handler`、FreeRTOS `configASSERT`、malloc 失败、栈溢出和调度器异常返回也全部进入同一路径。构建脚本会检查最终 ELF 中每个异常入口都引用紧急停车，并确认 CPU fault 包装器不生成栈访问。

## 混合控制器状态边界

统一停车、无效循线样本、任务二直线/半圆交接，以及任务三/四的 HALT/控制器复位都会同步调用 `CarHybrid_Reset()`。重新捕线和进入下一弧段时不再把数百毫秒前的误差按当前 10ms 周期计算微分，也不会沿用上一段的积分。

## RTOS/裸机双构建

`car_config.h` 允许通过编译器覆盖 `CAR_USE_FREERTOS`。`tools/build_firmware.py` 默认分别构建：

- RTOS：编译并链接锁定的 FreeRTOS Kernel；
- baremetal：不编译 FreeRTOS Kernel/port，SVC、PendSV、SysTick 由应用中断文件定义。

STM32CubeIDE Arm GCC 14.3.1 验证结果：

| 配置 | Flash text+data | RAM data+bss | 结果 |
| --- | ---: | ---: | --- |
| RTOS | 80,592 B | 18,400 B | 编译、链接、符号/反汇编检查通过 |
| baremetal | 75,360 B | 5,832 B | 编译、链接、无 FreeRTOS port 符号 |

同次验证中，LQR/PID 单测、SysTick 交接单测和 14 组主机回归均在 ASan/UBSan 下通过。本记录不代表已烧录或实车验收。
