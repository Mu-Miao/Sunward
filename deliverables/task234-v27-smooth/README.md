# V27 任务二三四巡线平滑试验固件

轮速PI保留；三任务共用正常巡线滤波PD、差速变化率限制和恢复加速。EDGE/失线/停车原边界保留。32组ASan/UBSan及ARM构建通过；未烧录、未完成实车验证。

- [详细说明](说明.md)
- [新版HEX](Sunward.hex) / [匹配ELF](Sunward.elf) / [清单](manifest.json)
- [V26回退HEX](../task2-v26-shared-arc/Sunward.hex)；源码回退分支codex/v26-before-line-smoothing，固定b5e0a1b。

V27 instance=1092字节。采集必须使用匹配ELF；板上仍是原版本时不要直接使用本包ELF采集。
