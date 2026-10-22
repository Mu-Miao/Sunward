# Sunward / V18 FLOW：V14 基础连续转弯

基于 `codex/task4-v14-pwm22`（`242b647`），本地修改分支为 `codex/task4-v14-rolling`。任务四 C/D 在原点位窗口首次见线时开始低速预转；正常换段与前三圈 A 点接续连续运动，第四圈终点停稳完成。标题为 `TASK 4 V18 FLOW`。

- [改动、参数与验证说明](TASK4_V18_FLOW_说明.md)
- [模块接口](MODULES.md)
- 原 V14 固件继续保留在 `deliverables/task4-v14-pwm22/`，不作为本版下载文件。

## 构建

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

构建脚本优先使用已安装的 STM32CubeIDE GCC，也支持 `--cc /path/to/arm-none-eabi-gcc`。仅使用 Python 标准库。输出为 `build/Debug/Sunward.hex`、`Sunward.elf`，构建日志和校验清单在同目录。也可用现有 EIDE/CubeIDE 工程重新构建。

OFF 且停车时短按 PA0 静态 CAL；READY 后 ON 并按键启动。当前起始摆放方向仍为 A 朝 B。运行中按键、OFF 或 `STOP!` 均可停车。

20 组 ASan/UBSan 软件回归、50 个 ARM 编译单元及 HEX 校验通过；本版未烧录、未实车验证。下载时明确选择 V18 FLOW 文件并核对屏幕标题。
