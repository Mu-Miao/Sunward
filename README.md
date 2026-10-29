# Sunward：V28 点位转向提速试验版

STM32F103RCT6 / STM32 HAL：轮速PI、MPU6050、LF04和任务一至四。

新分支codex/task234-corner-v28，基线V27 832fbc8。V27固定在codex/v27-smooth-rollback，原codex/task234-smooth-v27也不动；V26及main基线保留。用户反馈上一轮巡线顺滑，本轮只加快点位转向和确认捕线后的接管。V28尚未烧录、未完成实车验证。

|任务|标题|直线/斜线|普通弧线|结束条件|
|---|---|---|---|---|
|一|TASK 1 V4 FAST|400mm/s|—|一米停稳，13.5秒运行超时|
|二|TASK 2 V28 TURN|400mm/s|300mm/s|一圈停稳，45秒开发超时|
|三|TASK 3 V28 TURN|400mm/s|300mm/s|一圈停稳，40秒超时|
|四|TASK 4 V28 TURN|400mm/s|300mm/s|四圈停稳，160秒总超时|

任务三四连续转向外轮100→120mm/s、内轮20保持；近目标12°开始降速，原3°确认保持。任务二三四有效EDGE接管80mm/s、半差速60，外/内轮140/20；全白仍80/20。正常巡线滤波PD和轮速PI保持V27；失线预算、点位几何、最终停车与任务一保持。

34组ASan/UBSan与ARM构建通过。理想模型任务三28.37→26.83秒，任务四四圈112.85→106.69秒；这不是实车成绩。

[本版说明](TASK234_V28_CORNER_说明.md) · [V28交付](deliverables/task234-v28-corner/README.md) · [V27回退](deliverables/task234-v27-smooth/README.md) · [V26更早回退](deliverables/task2-v26-shared-arc/README.md)

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

测试与构建不会启动电机或自动烧录。OFF空闲时长按PA0选任务、短按静态CAL；READY后ON并按键启动。按键、OFF或STOP停车。PWM满量程7200、比赛限幅3600及轮速FAST500mm/s保持。

[上一轮V27](TASK234_V27_SMOOTH_说明.md)、[V26说明](TASK2_V26_SHARED_ARC_说明.md)、[main历史基线](docs/TESTED_MAIN_BASELINE.md)、[实验档案](docs/experiments/README.md)是历史资料，不等于V28验收。
