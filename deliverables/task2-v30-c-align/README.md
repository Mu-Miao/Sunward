# 任务二 V30 C点补转试验版

CD参考固定为AB初始方向+180°。C点偏差超过3°时低速向前补转，稳定30ms后再加速；补转限1.5秒/60mm，移动计入CD原一米里程。任务三四沿用V29控制。

38组ASan/UBSan与ARM构建通过；尚未烧录、未实车验证。

[说明](说明.md) · [新版HEX](Sunward.hex) · [匹配ELF](Sunward.elf) · [原V29](../task234-v29-gap/Sunward.hex) · [99秒V28](../task234-v28-corner/Sunward.hex)

任务二TASK 2 V30 ALIGN，任务三四V29 GAP；instance1144字节。对应ELF不能混用。完整四任务镜像。
