# V28 点位转向提速试验版

保留V27正常巡线与轮速PI；连续转向100/20→120/20mm/s，已确认捕线EDGE外/内轮120/20→140/20mm/s，全白寻找仍80/20。角度和失线确认、终点停车保持。34组回归及ARM构建通过，未烧录、未实车验证。

[详细说明](说明.md) · [新版HEX](Sunward.hex) · [匹配ELF](Sunward.elf) · [V27回退HEX](../task234-v27-smooth/Sunward.hex)

源码回退分支codex/v27-smooth-rollback，固定832fbc8。不要混用不同版本的ELF采集。
