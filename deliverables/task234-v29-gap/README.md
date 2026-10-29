# V29 短空白连续运动试验版

保留V28转向和V27顺滑巡线，仅为刚才稳定正常巡线的短空白增加30ms目标保持，重新见线仍确认30ms；条件满足后从保持速度接回。持续失线、毛刺、大偏差或故障按原逻辑降级/停车，不延长失线预算。

36组回归和ARM构建通过，未烧录、未实车验证。

[说明](说明.md) · [新版HEX](Sunward.hex) · [匹配ELF](Sunward.elf) · [原V28回退HEX](../task234-v28-corner/Sunward.hex)

源码回退分支codex/v28-99s-rollback，固定b4e20a2。V29 instance1112字节，采集必须匹配ELF。
