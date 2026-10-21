# Sunward

基于 **STM32F103RCT6 / STM32 HAL** 的双轮小车固件，包含轮速 PI 闭环、编码器里程、MPU6050 航向保持、LF04 巡线，以及任务一至四的路线控制。

> `feature/freertos-lqr-pid` 是基于 V11 FAST 的实验分支：已加入 FreeRTOS 10ms 周期调度与 LQR/PID 巡线上层，但尚未完成实车重新验收。架构、公式、构建和上车顺序见 [FreeRTOS + LQR/PID 测试说明](FREERTOS_LQR_PID_TESTING.md)。

## 最新实测（2026-10-18）

**任务四 V11 FAST 已完成一次四圈实测，固件计时 149.606 秒（2 分 29.606 秒）。** 用户现场确认顺利完成，pyOCD 同步记录如下：

| 项目 | 本次结果 |
| --- | --- |
| 固件标题 | `TASK 4 V11 FAST` |
| 完成圈数 / 半圆数 | 4 / 8 |
| 结束状态 / 故障码 | `DONE` / `0` |
| 最终停车 | 两轮目标、输出和实测速度均为 0 |
| 正常直线 / 巡线设定 | 250 / 220 mm/s |
| 航向控制 | 静态 CAL 后的陀螺积分，在线融合关闭 |

详细过程见 [四圈完成记录](TASK4_V11_20261018_实测四圈完成.md)，可追溯数据见 [实测摘要](records/task4/2026-10-18_v11_success.json)。该结果记录一次任务四成功；任务一、二、三在本版上的现场测试结果待补充。

## 当前版本

当前固件使用静态CAL后的陀螺积分、完整角度对正及任务四探头间隙找回；在线航向融合修正暂时关闭：

- 任务三屏幕标题：`TASK 3 V4 SMOOTH`。
- 任务四屏幕标题：`TASK 4 V11 FAST`。
- 仅任务四正常直线240→250mm/s、正常巡线200→220mm/s；任务一至三不提速。转向、捕线、寻线、边缘降速和终点降速规则保留。
- 删除原来固定少转5°偏移，仍以完整几何切线为参考；转向从单一方向首次进入±3°容差窗口即制动，通常形成约3°的可控少转容错，并保留停稳复核。
- `DPT_HEADING_FUSION_ENABLE=0`：任务三、四均不学习或扣除在线零偏，转向与直线使用同一份静态CAL后的陀螺积分。保留160mm轮距和融合内核用于后续分析，FUS诊断输出`EN=0`；正常重新CAL后Y/G接近一致、B=0。
- 任务四在 B 点对正完整 BD 航向、D 点对正完整 DA 切线，均达到 ±3°并停稳后才前进；D 点在原终点窗口内停车并连续确认有效线，不再强制前置探头等到完整车轴几何里程才认线。
- 任务三弧段全白且已有可信巡线历史时，沿冻结的上次方向连续低速搜线；重新见线的30 ms确认期也不刹停，避免弯道跨探头时一顿一顿。搜线仍受500 ms、20/35 mm和12°/18°分级边界约束。
- 按 LF04 的 33/14.5/33 mm 探头间距、约45 mm前伸和19 mm线宽，任务四 CB/DA 的可信内外单探头全白交接使用独立桥接；仍限500 ms，角度/位移边界为18°/35 mm，恢复需连续有效30 ms。其他普通桥接保留原12°/20 mm。
- 四圈实际入弧计数1～8，恢复不重复计数；保存最近两帧可信探头图样，失线时按侧向线索、图样变化和CB/DA方向确定并冻结寻找方向，输出独立HIST诊断帧。
- 保留 STOP / OFF、串口接收故障、低压、ADC / MPU 异常、节拍、超速、反转、堵转和路线边界等停车保护，并提供停车快照及非阻塞遥测。

主机模拟验证与实际场地验收分别记录。当前 V11 FAST 的具体修改和验证边界见 [任务四小幅提速说明](TASK4_V11_FAST_说明.md)，不能把软件回归通过当作四圈实车验收通过。

## 工程结构

| 路径 | 内容 |
| --- | --- |
| `Core/Inc/car_config.h` | 轮速 PI、编码器、轮距、航向融合和路线参数 |
| `Core/Inc/car_state.h` | 整车模块状态 |
| `Core/Src/car_app.c` | 前台调度、初始化、统一停车与保护 |
| `Core/Src/car_wheel.c` / `car_pi.c` | 编码器反馈和轮速闭环 |
| `Core/Src/car_heading.c` | 静态校准、航向融合与积分 |
| `Core/Src/car_line.c` | LF04 图样判定与巡线 |
| `Core/Src/car_hybrid.c` | LQR/PID 混合差速控制 |
| `Core/Src/car_rtos.c` / `FreeRTOSConfig.h` | 10ms 控制任务和 RTOS 配置 |
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

1. 初次克隆执行 `git submodule update --init`，取得仓库锁定的 FreeRTOS Kernel。
2. 安装 VS Code、Embedded IDE（`cl.eide`）及 Arm GNU 工具链。
3. 打开 `Sunward.code-workspace`，在 EIDE 中配置工具链路径并选择 **Debug**。
4. 执行 **Rebuild**，或在终端运行 `python3 tools/build_firmware.py`，同时产生 `Sunward-rtos.elf` 和 `Sunward-baremetal.elf`。
5. 如需下载，在 EIDE 中配置对应调试器；当前 Debug 使用 pyOCD / CMSIS-DAP / SWD，目标为 `stm32f103rc`、速度 4 MHz。

固件从 `Core/` 和 `Drivers/` 构建；`tools/tests/` 的模拟 HAL 仅用于主机测试，不应加入固件包含路径。`build/` 为本地产物，不提交到 Git。

## 操作入口

实体使能与 PA0（KEY_U）配合使用，每次操作前先松开按键：

- **OFF、空闲时长按 1.5 秒**：循环选择一米直行、半圆巡线、任务二、任务三、任务四、`LOG EXPORT`。
- **OFF 时短按并松开**：普通任务模式下静止校准 MPU6050；`LOG EXPORT` 模式下则完整导出一次当前环形日志。
- **READY + ON 时按下**：启动所选模式。
- **运行、制动或校准中按下**：停止；实体 OFF 和串口 `STOP!` 也可停止。
- **故障日志**：必须先长按切到 `LOG EXPORT`，再短按导出；未导出的故障日志不会因切换模式被清除。看到 `RLOG END` 后，长按离开 `LOG EXPORT` 才清除已导出的故障日志。

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
| `LOG!` / `LOGCLR!` | 停车时导出/清除环形运行日志；导出不会阻塞控制周期 |

### 运行日志

`car_run_log` 采用双层环形记录。高频层每50 ms保存任务、阶段、路段、弧段号、四路循线原始位、循线误差、左右目标/实际轮速、航向和故障标志，160条可保留故障前约8秒；全程层每200 ms保存分段进度、航向、实际轮速、线误差、阶段、检查点及线/航向置信度，805条可完整覆盖任务四160秒上限。另有32条事件快照，记录开始、换段、软检查点、丢线、重获和停止。正常运行自动覆盖各层最旧记录，相当于定期清理。

任务未完成时，如果两轮实际速度连续300 ms均接近0，日志立即冻结；即使异常逻辑同时把目标轮速清为0也能保留。任何 STOP/OFF、失线、堵转、反转、超时或传感器故障同样冻结；仅任务顶点制动、转向停稳和换段刹车等明确的计划内停车阶段不会误判。导出依次包含 `JLOG` 全程层、`ELOG` 事件层和 `RLOG` 高频层，直到看到 `RLOG END` 才算完成，可直接保存为CSV文本分析。

## 软件回归

需要 Python 3 和主机 C 编译器（Clang / GCC）；脚本仅使用 Python 标准库，无需安装 Python 依赖。

```sh
python3 tools/tests/run_tests.py
python3 tools/tests/run_tests.py --sanitize
```

首先运行混合控制器、弧线软检查点/置信度观测器、SysTick和双层日志单元测试，再将 7 类场景分别经过 `CarApp` 和旧兼容 API 两套入口，共 **14 组回归**。`--sanitize` 需要编译器支持 AddressSanitizer 和 UndefinedBehaviorSanitizer。

## 调试记录

- [FreeRTOS SysTick 交接、致命制动和双构建验证](FREERTOS_SAFETY_FIX_20261020.md)
- [pyOCD实时只读采集方法](tools/PYOCD_采集说明.md)
- [航向融合 V1 机制和诊断字段](HEADING_FUSION_V1_说明.md)
- [任务四 FUSE V2 BD 实测与修改](TASK4_FUSE_V2_BD_实测与修改.md)
- [任务四 FUSE V3 B 修改记录](TASK4_FUSE_V3_B_说明.md)
- [任务四 FUSE V4 D 修改记录](TASK4_FUSE_V4_D_说明.md)
- [任务四 FUSE V5 D 修改记录](TASK4_FUSE_V5_D_说明.md)
- [任务四 FUSE V6 GAP 修改记录](TASK4_FUSE_V6_GAP_说明.md)
- [任务四 FUSE V7 HIST 修改记录](TASK4_FUSE_V7_HIST_说明.md)
- [V7 第二圈实测与 T28 对照](TASK4_V7_20261018_210218_实测与T28对照.md)
- [任务四 FUSE V8 FULL 修改记录](TASK4_FUSE_V8_FULL_说明.md)
- [任务四 FUSE V9 TOL3 修改记录](TASK4_FUSE_V9_TOL3_说明.md)
- [21:32 第二圈 A 过转分析](TASK4_20261018_213231_A点过转分析.md)
- [任务四 V10 GYRO 修改记录](TASK4_V10_GYRO_说明.md)
- [任务四 V11 FAST 当前说明](TASK4_V11_FAST_说明.md)
- [任务四 V11 实测四圈完成](TASK4_V11_20261018_实测四圈完成.md)
- [任务四起步与恢复斜坡记录](TASK4_RAMP_V1_调试说明.md)
- [任务四 T29 诊断记录](TASK4_T29_D1_调试说明.md)

旧记录保留各自版本的参数和验证结果；当前行为以源码和 V11 FAST 说明为准。记录中的本地日志、备份和构建产物路径不属于仓库交付内容。

## 仓库文件

提交源码、驱动、CubeMX / EIDE 工程配置、说明文档和测试。macOS 的 `.DS_Store`、AppleDouble 文件、编辑器临时文件、Python 缓存及构建输出由 `.gitignore` 排除，无需上传。
