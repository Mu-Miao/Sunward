# Sunward：任务二V30 C点补转试验版

分支codex/task2-c-align-v30，基线V29 c240b1d。任务二C点出弧后保持名义CD方向（初始AB方向+180°），偏差超过3°时低速补转并确认30ms，再正常加速；补转限1.5秒/60mm，移动计入CD一米里程。已经≤3°时直接接续。

|任务|标题|正常直线/斜线|正常半圆|结束条件|
|---|---|---|---|---|
|一|TASK 1 V4 FAST|400mm/s|—|一米停稳，13.5秒运行超时|
|二|TASK 2 V30 ALIGN|400mm/s|300mm/s|一圈停稳，45秒开发超时|
|三|TASK 3 V29 GAP|400mm/s|300mm/s|一圈停稳，40秒超时|
|四|TASK 4 V29 GAP|400mm/s|300mm/s|四圈停稳，160秒总超时|

仅任务二新增本次运动逻辑；任务一、三、四保持原控制。38组ASan/UBSan及ARM构建通过，未烧录、未完成实车验证。用户要求本轮先测试提交推送，不烧录。

[本版说明](TASK2_V30_C_ALIGN_说明.md) · [V30交付](deliverables/task2-v30-c-align/README.md) · [V29回退](deliverables/task234-v29-gap/README.md) · [99秒V28回退](deliverables/task234-v28-corner/README.md)

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

构建与主机测试不访问电机或自动烧录。操作方式保持。V30配套ELF的instance1144字节，采集必须匹配Flash。
