# TASK 4 V18 FLOW 固件与验证记录

源码基于 V14 `242b647`，功能提交 `e79193d`；分支 `codex/task4-v14-rolling`。本目录为该提交已构建、验证的原始输出，不重新编译或烧录。

- [HEX 固件](firmware/Sunward_V18_FLOW.hex)
- [匹配 ELF](firmware/Sunward_V18_FLOW.elf)
- [20组ASan/UBSan软件回归](validation/host_tests.log)
- [50个ARM编译单元的构建记录](validation/arm_build.log)
- [源码及固件SHA256清单](validation/build_manifest.json)
- [改动与参数说明](../../TASK4_V18_FLOW_说明.md)

C/D 原窗口见线低速预转，正常换段和前三圈 A 点连续接续，第四圈终点停稳。转弯外轮最高100、内轮20mm/s；任务四位移预算由80改为120mm，保留4秒时限。直线/弧线270/240mm/s、输出上限2200。

本版仅完成软件回归与构建，未烧录、未实车验证。下载后应显示 `TASK 4 V18 FLOW`。历史V14固件在相邻目录，勿混用。

构建清单中workspace为构建时的本机工作树路径；source_commit标明功能源码提交，后续归档提交仅新增交付物与说明。
