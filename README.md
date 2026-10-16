# Sunward

基于 **STM32F103RCT6 / STM32 HAL** 的双轮小车固件，包含轮速 PI 闭环、编码器里程、MPU6050 航向保持、LF04 巡线，以及任务一至四的路线控制。

## 当前版本

`main` 已集成任务三、四的航向融合及任务四 B 点转向修正：

- 任务三屏幕标题：`TASK 3 FUSE V1 L1`。
- 任务四屏幕标题：`TASK 4 FUSE V3 B`。
- 航向融合使用 **160 mm 轮距**，仅在任务三、四可信的 AC / BD 直线段连续稳定 200 ms 后学习陀螺残余零偏；转向、捕获、巡线和寻线时冻结学习，继续扣除已学习的零偏。
- 任务四在 B 点对正完整 BD 航向，达到 ±1°并停稳后才前进；BD 的 D 点确认采用完整斜线里程门槛。
- 保留 STOP / OFF、串口接收故障、低压、ADC / MPU 异常、节拍、超速、反转、堵转和路线边界等停车保护，并提供停车快照及非阻塞遥测。

主机模拟验证与实际场地验收分别记录。当前 V3 B 的具体修改和验证边界见 [任务四 FUSE V3 B 说明](TASK4_FUSE_V3_B_说明.md)，不能把软件回归通过当作四圈实车验收通过。

## 工程结构

| 路径 | 内容 |
| --- | --- |
| `Core/Inc/car_config.h` | 轮速 PI、编码器、轮距、航向融合和路线参数 |
| `Core/Inc/car_state.h` | 整车模块状态 |
| `Core/Src/car_app.c` | 前台调度、初始化、统一停车与保护 |
| `Core/Src/car_wheel.c` / `car_pi.c` | 编码器反馈和轮速闭环 |
| `Core/Src/car_heading.c` | 静态校准、航向融合与积分 |
| `Core/Src/car_line.c` | LF04 图样判定与巡线 |
| `Core/Src/car_route.c` | 运动模式、距离控制与路线衔接 |
| `Core/Src/car_task3.c` / `car_task3_port.c` / `car_task4.c` | 任务三、四状态机及整车适配 |
| `Core/Src/car_button.c` / `car_command.c` / `car_ui.c` | 按键、调试命令、OLED 和遥测 |
| `Drivers/` | CMSIS 与 STM32 HAL 驱动 |
| `Sunward.ioc` | STM32CubeMX 配置 |
| `.eide/` / `Sunward.code-workspace` | EIDE 构建配置与 VS Code 工作区 |
| `STM32F103RCTX_FLASH.ld` | 链接脚本 |
| `tools/tests/` | 使用模拟 HAL 的主机回归，不访问硬件 |

更多接口与调度说明见 [MODULES.md](MODULES.md)。参数包含当前小车的标定值，换硬件后需重新核对。

## 构建

1. 安装 VS Code、Embedded IDE（`cl.eide`）及 Arm GNU 工具链。
2. 打开 `Sunward.code-workspace`，在 EIDE 中配置工具链路径并选择 **Debug**。
3. 执行 **Rebuild**，输出位于 `build/Debug/`：`Sunward.elf`、`Sunward.hex` 和链接映射文件。
4. 如需下载，在 EIDE 中配置对应调试器；当前 Debug 使用 ST-Link / SWD，下载文件为 `build/Debug/Sunward.hex`。

固件从 `Core/` 和 `Drivers/` 构建；`tools/tests/` 的模拟 HAL 仅用于主机测试，不应加入固件包含路径。`build/` 为本地产物，不提交到 Git。

## 操作入口

实体使能与 PA0（KEY_U）配合使用，每次操作前先松开按键：

- **OFF、空闲时长按 1.5 秒**：循环选择一米直行、半圆巡线、任务二、任务三、任务四。
- **OFF 时短按并松开**：静止校准 MPU6050，等待 `READY`。
- **READY + ON 时按下**：启动所选模式。
- **运行、制动或校准中按下**：停止；实体 OFF 和串口 `STOP!` 也可停止。

USB / 蓝牙调试命令以 `!` 或换行结束：

| 命令 | 用途 |
| --- | --- |
| `CAL!` | OFF 且停车时静态校准 |
| `STOP!` | 停止，优先于运动请求 |
| `GO!` / `GO20!` / `GO100!` | 三秒航向保持 / 20 cm 定距 / 一米定距 |
| `LF20!` | 半圆巡线 |
| `TASK2!` / `TASK3!` / `TASK4!` | 对应路线任务；OFF 下可选模式，ON 下满足条件后可启动 |
| `F!` / `B!` / `L!` / `R!` | 手动方向测试 |
| `V120!` / `TIME3!` | 停车时设置手动速度（mm/s）和时长（秒） |

## 软件回归

需要 Python 3 和主机 C 编译器（Clang / GCC）；脚本仅使用 Python 标准库，无需安装 Python 依赖。

```sh
python3 tools/tests/run_tests.py
python3 tools/tests/run_tests.py --sanitize
```

共 7 类场景，分别经过 `CarApp` 和旧兼容 API 两套入口，共 **14 组回归**，覆盖运动、任务二至四、寻线、转向、航向融合及故障保护。`--sanitize` 需要编译器支持 AddressSanitizer 和 UndefinedBehaviorSanitizer。测试产物放在临时目录，结束后自动清理。

## 调试记录

- [航向融合 V1 机制和诊断字段](HEADING_FUSION_V1_说明.md)
- [任务四 FUSE V2 BD 实测与修改](TASK4_FUSE_V2_BD_实测与修改.md)
- [任务四 FUSE V3 B 当前说明](TASK4_FUSE_V3_B_说明.md)
- [任务四起步与恢复斜坡记录](TASK4_RAMP_V1_调试说明.md)
- [任务四 T29 诊断记录](TASK4_T29_D1_调试说明.md)

旧记录保留各自版本的参数和验证结果；当前行为以源码和 V3 B 说明为准。记录中的本地日志、备份和构建产物路径不属于仓库交付内容。

## 仓库文件

提交源码、驱动、CubeMX / EIDE 工程配置、说明文档和测试。macOS 的 `.DS_Store`、AppleDouble 文件、编辑器临时文件、Python 缓存及构建输出由 `.gitignore` 排除，无需上传。
