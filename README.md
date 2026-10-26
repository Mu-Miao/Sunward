# Sunward：四任务提速第一轮

STM32F103RCT6 / STM32 HAL 双轮小车：编码器轮速PI、MPU6050航向、LF04巡线及任务一至四控制。

2026-10-26已将上一版合并并推送到main，提交 `2850bfb`。用户确认任务二V22、任务三V4、任务四V19均能完成，任务三首次测试一把通过；任务一V2当时只完成软件验证。本分支 `codex/all-tasks-speed-v1` 在该版本上提速，尚未烧录或实车验证。

| 任务 | 本分支标题 | 正常直线/斜线 | 正常半圆 | 圈数 / 超时 |
| --- | --- | --- | --- | --- |
| 一 | TASK 1 V3 FAST | 270mm/s | — | 一米 / 13.5秒运行超时 |
| 二 | TASK 2 V23 FAST | 280mm/s | 250mm/s | 一圈 / 45秒开发超时 |
| 三 | TASK 3 V5 EDGE | 280mm/s | 250mm/s | 一圈 / 40秒 |
| 四 | TASK 4 V20 EDGE | 280mm/s | 250mm/s | 四圈 / 160秒 |

任务一提前180mm减速，保留40mm/s末速、200ms停稳确认和500ms完成声光；任务二、三、四直线加速度改为180mm/s²。EDGE接管、转弯、失线搜索速度与原保护边界保持。所有任务输出限幅2200，独立半圆及GO20/手动配置保持。

28组带ASan/UBSan回归与ARM构建通过。相同模拟输入下四任务用时均缩短；这不是实车速度/成功率保证。详见 [本轮说明](ALL_TASKS_SPEED_V1_说明.md) 和 [交付包](deliverables/all-tasks-speed-v1/README.md)。

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

主机测试使用模拟HAL，不访问硬件。ARM构建优先使用STM32CubeIDE GCC，产物在build/Debug。OFF且空闲长按PA0选任务、短按静态CAL，READY后ON并按键启动；运行中按键、OFF、STOP停车，操作方式保持。

历史实测数据和固件仍保留：[合入基线](docs/TESTED_MAIN_BASELINE.md)、[任务三V4说明](TASK3_V4_EDGE_说明.md)、[任务一V2说明](TASK1_V2_FAST_说明.md)、[实验档案](docs/experiments/README.md)。历史包的ELF/HEX及实测结论对应当时版本，不代表本分支。
