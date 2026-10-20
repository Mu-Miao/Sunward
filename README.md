# Sunward / V14 PWM22：独立输出上限

固定保留V14控制代码：任务四输出上限2200，其余速度目标与保护沿用该版。包含原停车确认和单轮转向。

分支：`codex/task4-v14-pwm22`。16组软件回归及ARM构建通过；没有匹配V14 ELF校验值的现场采集记录。

- [本版参数与保护说明](TASK4_V14_任务四输出2200_说明.md)
- [模块接口说明](MODULES.md)
- [pyOCD只读采集](tools/PYOCD_采集说明.md)
- [V14原版HEX、ELF及验证数据](deliverables/task4-v14-pwm22/README.md)

## 构建与使用

使用既有EIDE/CubeIDE工程，选择Debug并执行Rebuild。调参入口为 `Core/Inc/car_config.h`。

主机构建与回归：`python3 tools/tests/run_tests.py --sanitize`。模拟HAL不加入固件包含路径。

固件输出为 `build/Debug/Sunward.hex` 和 `Sunward.elf`；下载时明确选择对应版本，EIDE旧下载路径可能指向其他固件。 本版标题为 `TASK 4 V14 PWM22`。

OFF且停车时短按PA0静态CAL；READY后ON并按键启动。运行中按键、实体OFF或 `STOP!` 均可停车；原保护与启动条件保留。

## 历史资料

旧版本说明与完整实验记录统一保留在[main实验档案](https://github.com/Mu-Miao/Sunward/tree/main/docs/experiments)，本分支只保留当前需要的说明。历史文件也可从整理前提交查看；不将其他固件的成功记录计为本版验收。
