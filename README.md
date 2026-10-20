# Sunward

基于 **STM32F103RCT6 / STM32 HAL** 的双轮小车固件，包含轮速 PI 闭环、编码器里程、MPU6050 航向保持、LF04 巡线，以及任务一至四的路线控制。

## 实测记录（V11 FAST）

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

2026-10-19 同版 V11 再次完成四圈，计时 **152.444 秒**，`DONE / F0 / 4圈 / 8次入弧`，见 [复测摘要](records/task4/2026-10-19_v11_success.json)。同日也出现第一圈 BD 找 D 超时，因此仍需优化重复稳定性；成功记录不代表每次都能完成。

## 当前版本

当前源码为 **V13 ENTRY 入弧与提速试验版**：任务四捕线后增加独立低速入弧阶段，正常直线 **270 mm/s**、稳定巡线 **240 mm/s**；入弧维持 **50 mm/s**，边缘降速与寻线预算保留。本版已完成软件验证、ARM构建及一次实车试跑：**160秒总超时，3圈完成/8次入弧，停于第四圈DA**；尚未完成四圈，V11的四圈成绩不属于V13。详见 [V13 入弧与提速说明](TASK4_V13_入弧与提速_说明.md)和[本次超时分析](TASK4_V13_20261020_实测超时分析.md)。

使用静态CAL后的陀螺积分及任务四探头间隙找回，在线航向融合修正暂时关闭：

- 任务三屏幕标题：`TASK 3 V3 GYRO`。
- 任务四屏幕标题：`TASK 4 V13 ENTRY`。
- 仅任务四正常直线250→270mm/s、正常巡线220→240mm/s；任务一至三不提速。转向、捕线、寻线、边缘降速和终点降速参数保留。
- C/D 连续捕线后进入 `ARC_ENTRY`：50mm/s 低速巡线叠加弧向前馈，至少200ms并连续60ms确认非最外侧有效线才交接；全白冻结最后可信差速，受原桥接预算及总计1500ms/60mm/25°限制。入弧路程计入半圆，交接不重复计数。
- 任务四仅 B→BD 采用独立少转3°参数，四圈均使用固定偏移；A启动对正AC，C/D捕线及顶点搜线仍使用完整切线，任务三偏移为0。各点对正容差统一为±3°，保留停稳复核。
- `DPT_HEADING_FUSION_ENABLE=0`：任务三、四均不学习或扣除在线零偏，转向与直线使用同一份静态CAL后的陀螺积分。保留160mm轮距和融合内核用于后续分析，FUS诊断输出`EN=0`；正常重新CAL后Y/G接近一致、B=0。
- 任务四 B 点对正调整后的 BD 航向，D 点对正完整 DA 切线，均达到 ±3°并停稳后才前进；D 点在原终点窗口内停车并连续确认有效线，不强制前置探头等到完整车轴几何里程才认线。
- 按 LF04 的 33/14.5/33 mm 探头间距、约45 mm前伸和19 mm线宽，任务四 CB/DA 的可信内外单探头全白交接使用独立桥接；仍限500 ms，角度/位移边界为18°/35 mm，恢复需连续有效30 ms。其他普通桥接保留原12°/20 mm。
- 四圈实际入弧计数1～8，恢复不重复计数；保存最近两帧可信探头图样，失线时按侧向线索、图样变化和CB/DA方向确定并冻结寻找方向，输出独立HIST诊断帧。
- 保留 STOP / OFF、串口接收故障、低压、ADC / MPU 异常、节拍、超速、反转、堵转和路线边界等停车保护，并提供停车快照及非阻塞遥测。

主机模拟验证与实际场地验收分别记录。V13 的修改和验证边界见 [入弧与提速说明](TASK4_V13_入弧与提速_说明.md)；V12 的历史修改见 [B点少转说明](TASK4_V12_B点少转3度_说明.md)；V11 的历史参数见 [小幅提速说明](TASK4_V11_FAST_说明.md)。

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
4. 如需下载，在 EIDE 中配置对应调试器；当前 Debug 使用 pyOCD / CMSIS-DAP / SWD，目标为 `stm32f103rc`、速度 4 MHz。下载文件选择 `build/Debug/Sunward.hex`；当前工程的下载文件配置为空，需手动选择。

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

共 8 类场景，分别经过 `CarApp` 和旧兼容 API 两套入口，共 **16 组回归**，覆盖运动、任务二至四、寻线、转向、航向融合及故障保护。`--sanitize` 需要编译器支持 AddressSanitizer 和 UndefinedBehaviorSanitizer。测试产物放在临时目录，结束后自动清理。

## 调试记录

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
- [任务四 V11 FAST 说明](TASK4_V11_FAST_说明.md)
- [任务四 V13 入弧与提速说明](TASK4_V13_入弧与提速_说明.md)
- [任务四 V11 实测四圈完成](TASK4_V11_20261018_实测四圈完成.md)
- [任务四起步与恢复斜坡记录](TASK4_RAMP_V1_调试说明.md)
- [任务四 T29 诊断记录](TASK4_T29_D1_调试说明.md)

旧记录保留各自版本的参数和验证结果；当前行为以源码和 V13 入弧与提速说明为准。记录中的本地日志、备份和构建产物路径不属于仓库交付内容。

## 仓库文件

提交源码、驱动、CubeMX / EIDE 工程配置、说明文档和测试。macOS 的 `.DS_Store`、AppleDouble 文件、编辑器临时文件、Python 缓存及构建输出由 `.gitignore` 排除，无需上传。
