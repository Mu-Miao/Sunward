# TASK 3 V4 EDGE 交付包

基于 main 4c516f0；分支 codex/task3-edge-one-lap。任务三使用任务四 V19 EDGE 与连续运动，A→C→B→D→A 一圈结束停车，40 秒超时。任务二 V22、任务四 V19 参数及运行行为保持不变。

烧录 Sunward.hex；Sunward.elf 与 manifest.json 必须配套保留。OLED 选任务三应显示 TASK 3 V4 EDGE，任务二仍为 TASK 2 V22 T4LINE，任务四仍为 TASK 4 V19 EDGE。

已通过 24 组 ASan/UBSan 主机回归、完整 ARM 构建及 HEX 校验，详见 validation。当前未烧录、未实车验收。操作方式保持原按键与 CAL 流程。
