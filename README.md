# Sunward：V27 巡线平滑试验版

基于STM32F103RCT6 / STM32 HAL：编码器轮速PI、MPU6050航向、LF04巡线和任务一至四控制。

当前分支codex/task234-smooth-v27，基线b5e0a1b（V26）。回退分支codex/v26-before-line-smoothing已固定；codex/all-tasks-speed-v2保持V26，main保留2850bfb。V27未烧录、未完成实车验证。

|任务|标题|正常直线/斜线|正常半圆|终止条件|
|---|---|---|---|---|
|一|TASK 1 V4 FAST|400mm/s|—|一米停稳，13.5秒运行超时|
|二|TASK 2 V27 SMTH|400mm/s|300mm/s|一圈停稳，45秒开发超时|
|三|TASK 3 V27 SMTH|400mm/s|300mm/s|一圈停稳，40秒超时|
|四|TASK 4 V27 SMTH|400mm/s|300mm/s|四圈停稳，160秒总超时|

任务二三四共用正常巡线滤波PD、差速变化率限制与回中加速；轮速PI、任务一、点位几何、EDGE/失线/停车预算保持。任务二AB不预减速，CD末160mm减速；三四末段180mm减速。比赛PWM限幅3600/满量程7200，轮速FAST500mm/s；其他模式350门槛保持。

32组ASan/UBSan回归及ARM构建通过。同一V26探头序列按相同300mm/s基速回放，差速累计变化量降低约15.4%；这不能代替V27实车顺滑度验证。

[新版说明](TASK234_V27_SMOOTH_说明.md) · [V27交付包](deliverables/task234-v27-smooth/README.md) · [V26回退包](deliverables/task2-v26-shared-arc/README.md)

V26实跑记录有任务二正常DONE约20.39秒、任务三DONE25.99秒（过程采样缺失）、任务四四圈DONE100.685秒；也有此前失线/找线失败。上述是回退版本记录，不是V27验收。

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

测试和构建不访问电机或自动烧录。任务操作方式保持：OFF且空闲时长按PA0选任务，短按静态CAL；READY后ON并按键启动。运行中按键、OFF或STOP停车。

[提速V2记录](ALL_TASKS_SPEED_V2_说明.md)、[任务二V26](TASK2_V26_SHARED_ARC_说明.md)、[main历史基线](docs/TESTED_MAIN_BASELINE.md)、[实验档案](docs/experiments/README.md)为历史资料。
