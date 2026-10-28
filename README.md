# Sunward：四任务提速V2试验版

基于STM32F103RCT6 / STM32 HAL：编码器轮速PI、MPU6050航向、LF04巡线和任务一至四控制。

当前分支codex/all-tasks-speed-v2，基线V1 26914df；main保留2850bfb。此前用户确认任务二V22、任务三V4、任务四V19能完成，任务三首次测试一把通过；该结果属于旧版本。本轮参数参考2026-10-28悬空PWM测量；V26已烧录，地面动态诊断待采集。

| 任务 | 标题 | 正常直线/斜线 | 正常半圆 | 终止条件 |
| --- | --- | --- | --- | --- |
| 一 | TASK 1 V4 FAST | 400mm/s | — | 一米停稳，13.5秒运行超时 |
| 二 | TASK 2 V26 ARC | 400mm/s | 300mm/s | 一圈停稳，45秒开发超时 |
| 三 | TASK 3 V6 EDGE | 400mm/s | 300mm/s | 一圈停稳，40秒超时 |
| 四 | TASK 4 V21 EDGE | 400mm/s | 300mm/s | 四圈停稳，160秒总超时 |

比赛PI限幅3600（50%），实测轮速FAST500mm/s；其他模式保留旧限幅和350门槛。任务一提前30cm减速；任务二仅CD末端16cm降速捕线，AB保持巡航至捕线确认；任务三/四减速区18cm。EDGE、转向、寻线、声光、连续运动及原失线预算保持。配置变化与依据见[本轮说明](ALL_TASKS_SPEED_V2_说明.md)。

V26任务二已实际共用任务三/四弧内核心，C点锁存出弧参考、CD平滑加速；AB取消预减速与任务一终点减速保持；见[任务二V26说明](TASK2_V26_SHARED_ARC_说明.md)。

30组带ASan/UBSan回归与ARM构建通过。V26经用户授权已烧录并校验，pyOCD正在采集；尚无本次完整动态验收结论。同一主机模拟下任务四四圈122.29→112.85秒；这不是实车成绩。[最新任务二V26交付包](deliverables/task2-v26-shared-arc/README.md)附固件、日志、清单和测量依据。

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

主机测试不访问硬件，构建不自动烧录。OFF且空闲时长按PA0选任务、短按静态CAL，READY后ON并按键启动；运行中按键、OFF、STOP停车。操作方式保持。

[上一轮提速记录](ALL_TASKS_SPEED_V1_说明.md)、[任务三V4](TASK3_V4_EDGE_说明.md)、[任务一V2](TASK1_V2_FAST_说明.md)、[合入基线](docs/TESTED_MAIN_BASELINE.md)、[实验档案](docs/experiments/README.md)均为历史版本记录，不能当作本版验收。
