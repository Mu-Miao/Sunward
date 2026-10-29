# Sunward：V31 全白寻线距离5cm试验版

分支codex/task234-white50-v31，基线V30 8172e19。任务二三四的普通全白和可信单探头交接行程上限均为50mm，保持500ms、普通12°/交接18°、恢复30ms等限制。任一先超限就停车，不能保证实际走满5cm。

任务二C点补转仍为3°/30ms、1.5秒/60mm，移动计入CD；正常PD、轮速PI、V29短空白和V28转向方案保持。任务一不改。

|任务|标题|直线/斜线|普通半圆|结束条件|
|---|---|---|---|---|
|一|TASK 1 V4 FAST|400mm/s|—|一米停稳，13.5秒运行超时|
|二|TASK 2 V31 GAP5|400mm/s|300mm/s|一圈停稳，45秒开发超时|
|三|TASK 3 V31 GAP5|400mm/s|300mm/s|一圈停稳，40秒超时|
|四|TASK 4 V31 GAP5|400mm/s|300mm/s|四圈停稳，160秒总超时|

40组ASan/UBSan及ARM构建通过，未烧录、未实车验证。

[本版说明](TASK234_V31_WHITE50_说明.md) · [V31交付](deliverables/task234-v31-white50/README.md) · [V30回退](deliverables/task2-v30-c-align/README.md) · [99秒V28回退](deliverables/task234-v28-corner/README.md)

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

主机测试和构建不连接电机或自动烧录。操作方式保持；采集使用匹配当前Flash的ELF。
