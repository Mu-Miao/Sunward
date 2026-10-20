# Sunward / V17：连续换段与转向积分衔接

本分支当前为V17 TURN：正常换段连续运动、转向两轮正向目标，并在低速换段缩减旧PI积分。仍保留120mm/4秒转向及其他保护。

分支：`codex/task4-arc-entry-v13`。18组软件回归及ARM构建通过；V17实车效果尚未验证。保留的V16故障采样仅作为修复依据。

- [当前修复与验证说明](TASK4_V17_C点转向积分衔接_说明.md)
- [模块接口说明](MODULES.md)
- [pyOCD只读采集](tools/PYOCD_采集说明.md)
- [V16 C点故障依据](records/task4/2026-10-20_v16_c_turn_limit.json)

## 构建与使用

使用既有EIDE/CubeIDE工程，选择Debug并执行Rebuild。调参入口为 `Core/Inc/car_config.h`。

主机构建与回归：`python3 tools/tests/run_tests.py --sanitize`。模拟HAL不加入固件包含路径。

固件输出为 `build/Debug/Sunward.hex` 和 `Sunward.elf`；下载时明确选择对应版本，EIDE旧下载路径可能指向其他固件。 本版标题为 `TASK 4 V17 TURN`。

OFF且停车时短按PA0静态CAL；READY后ON并按键启动。运行中按键、实体OFF或 `STOP!` 均可停车；原保护与启动条件保留。

## 历史资料

旧版本说明与完整实验记录统一保留在[main实验档案](https://github.com/Mu-Miao/Sunward/tree/main/docs/experiments)，本分支只保留当前需要的说明。历史文件也可从整理前提交查看；不将其他固件的成功记录计为本版验收。
