# Sunward：任务二 V22 与任务四 V19 EDGE

基于 STM32F103RCT6 / STM32 HAL 的双轮小车固件，包含轮速 PI、编码器里程、MPU6050 相对航向、LF04 巡线和任务一至四的路线控制。

## main 的合入版本与实测结果

本次合入远端 `codex/task2-speed` 的 V22 提交 `6d12559`，其历史已包含 `codex/task4-v19-edge` 的 `17bb15e`。没有合入原开发工作区中未提交的 V27 改动。

| 任务 | 固件标题 | 用户确认的实车结果 | 对应源码 |
| --- | --- | --- | --- |
| 任务二 | `TASK 2 V22 T4LINE` | 完整路线 22 秒，满足不超过 30 秒要求 | `6d12559` |
| 任务四 | `TASK 4 V19 EDGE` | 四圈约 120 秒 | `17bb15e`，已包含在 V22 基线 |

以上实车成绩由用户在本次合并请求中确认，未补充原始采集文件，不虚构计时精度或重复成功率。V11 FAST 的 149.606 秒保留为历史版本记录，不再作为最新成绩。

任务三、四共用 `car_task3.c` 的单圈八字核心。任务三直接运行核心，任务四增加四圈接续、总超时及独立配置。任务四的连续运动和 EDGE 不会自动在任务三中启用，任务四成绩也不代替任务三复验。

任务二正常直线 270、半圆 240 mm/s，输出上限 2200；弧内采用低速边缘接管和受限失线搜索。任务四正常直线 270、半圆 240 mm/s，EDGE 基速 70、最外侧目标 120/20 mm/s。具体边界见 `Core/Inc/car_config.h`。

## 资料与历史归档

- [完整实验档案](docs/experiments/README.md)：保留 main 原有实验记录与历史固件。
- [合入版本与验证边界](docs/TESTED_MAIN_BASELINE.md)
- [任务四 V19 EDGE 交付包](deliverables/task4-v19-edge/README.md)
- [任务四 V19 EDGE 设计说明](TASK4_V19_EDGE_说明.md)
- [模块接口](MODULES.md)

归档包中的固件和当时的验证状态属于归档时刻。重新构建 main 会同时包含任务二 V22 与任务四 V19 EDGE，不能把旧交付包中的 ELF、HEX 或布局当成新构建产物。

## 构建与测试

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

脚本只使用 Python 标准库。主机回归采用模拟 HAL，不访问硬件。ARM 构建优先使用 STM32CubeIDE GCC，也可传 `--cc /path/to/arm-none-eabi-gcc`，产物在 `build/Debug/`。软件验证与构建不代表已经烧录这次合并后的固件。

## 操作入口

车在 A 点朝 B 摆放。使能 OFF 且空闲时长按 PA0 选择任务，短按进行静态 CAL。READY 后 ON 并按键启动；运行中按键、实体 OFF 或 `STOP!` 停车。保留低压、传感器、节拍、轮速和路线边界保护。
