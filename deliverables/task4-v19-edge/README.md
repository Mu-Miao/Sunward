# TASK 4 V19 EDGE 固件与验证记录

基于V18 `f2802b6`；分支 `codex/task4-v19-edge`。捕线确认后先接管低速边缘巡线，内侧稳定后才提速。

- [HEX固件](firmware/Sunward_V19_EDGE.hex)
- [匹配ELF](firmware/Sunward_V19_EDGE.elf)
- [22组主机回归](validation/host_tests.log)
- [ARM构建记录](validation/arm_build.log)
- [源码、测试与固件SHA256清单](validation/build_manifest.json)
- [ELF离线布局](validation/pyocd_layout.json)
- [完整改动说明](../../TASK4_V19_EDGE_说明.md)

有效外侧单探头时基速70、差速±50，目标20/120mm/s；内侧连续60ms且阶段至少200ms后从70起步渐升。低速阶段限120mm/2.5秒/45°；全白仍限原500ms及位移/角度预算。正常直线270、半圆240mm/s与连续换段保留。

下载后标题应为 `TASK 4 V19 EDGE`，低速接管时显示EDGE。本版未烧录、未实车验证。instance为944字节，下次pyOCD应使用本目录匹配ELF；请勿混用旧V18 ELF。
