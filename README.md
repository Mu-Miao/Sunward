# Sunward / FreeRTOS + LQR/PID 试验分支

以V11为基线的FreeRTOS 10ms调度与LQR/PID巡线上层试验，轮速内环仍为PI。

分支：`codex/freertos-lqr-pid`。验证状态以本分支测试指南为准；main的V11实测成绩不属于本试验版本。

- [分支架构、构建与测试指南](FREERTOS_LQR_PID_TESTING.md)
- [模块接口说明](MODULES.md)
- [pyOCD只读采集](tools/PYOCD_采集说明.md)

## 构建与使用

先执行 `git submodule update --init`，再按本分支指南构建FreeRTOS工程；也可运行 `python3 tools/build_firmware.py`。

主机构建与回归：`python3 tools/tests/run_tests.py --sanitize`。模拟HAL不加入固件包含路径。

固件输出为 `build/Debug/Sunward.hex` 和 `Sunward.elf`；下载时明确选择对应版本，EIDE旧下载路径可能指向其他固件。

OFF且停车时短按PA0静态CAL；READY后ON并按键启动。运行中按键、实体OFF或 `STOP!` 均可停车；原保护与启动条件保留。

## 历史资料

旧版本说明与完整实验记录统一保留在[main实验档案](https://github.com/Mu-Miao/Sunward/tree/main/docs/experiments)，本分支只保留当前需要的说明。历史文件也可从整理前提交查看；不将其他固件的成功记录计为本版验收。
