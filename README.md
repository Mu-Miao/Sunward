# Sunward：V29 短空白连续运动试验版

轮速PI、MPU6050、LF04和任务一至四。分支codex/task234-gap-v29，基线V28 b4e20a2。用户反馈V28总用时99秒、巡线顺滑；源码固定保留在codex/v28-99s-rollback。V29未烧录、未完成实车验证。

任务二三四稳定正常巡线后的全白前30ms保持上次可信轮速目标；重新见线仍须原30ms连续确认，再从保持速度接回。持续全白、反复闪断、外沿/大偏差和其他无效图样按原处理，原失线几何、时间、停止优先级与终点判定保持。正常PD、轮速PI、V28转向、任务一不改。

|任务|标题|直线/斜线|普通弧线|结束条件|
|---|---|---|---|---|
|一|TASK 1 V4 FAST|400mm/s|—|一米停稳，13.5秒运行超时|
|二|TASK 2 V29 GAP|400mm/s|300mm/s|一圈停稳，45秒开发超时|
|三|TASK 3 V29 GAP|400mm/s|300mm/s|一圈停稳，40秒超时|
|四|TASK 4 V29 GAP|400mm/s|300mm/s|四圈停稳，160秒总超时|

36组ASan/UBSan及ARM构建通过。合成调度测试中，300mm/s时插入1～3帧白区并确认恢复，目标与PI输出能连续保持；这不代表真实场地任意失线都不停车，也不预测实车用时。

[本版说明](TASK234_V29_GAP_说明.md) · [V29固件](deliverables/task234-v29-gap/README.md) · [99秒V28回退](deliverables/task234-v28-corner/README.md) · [V27](deliverables/task234-v27-smooth/README.md)

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

构建和测试不操作电机或自动烧录。操作方式保持：OFF空闲时长按选任务、短按静态CAL；READY后ON并按键启动。按键、OFF或STOP停车。V29需匹配ELF进行pyOCD采集。
