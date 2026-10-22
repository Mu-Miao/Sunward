# Sunward / V19 EDGE：低速边缘巡线接管

基于V18（`f2802b6`），分支 `codex/task4-v19-edge`。捕线确认后最外侧单探头即可进入受限低速巡线，内侧稳定后才平顺提速。保留连续换段和前三圈A点接续，第四圈终点停稳。屏幕标题 `TASK 4 V19 EDGE`；低速接管期间显示 `EDGE`。

- [V19 HEX/ELF、日志和布局](deliverables/task4-v19-edge/README.md)
- [修改与验证说明](TASK4_V19_EDGE_说明.md)
- [V18现场故障依据](records/task4/2026-10-22_v18_entry_faults.json)
- [模块接口](MODULES.md)

V14、V18原分支及各自归档保留。当前下载应选择V19文件；历史固件在 `deliverables/task4-v14-pwm22/` 和 `deliverables/task4-v18-flow/`。

## 构建与使用

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/build_firmware.py
```

主机回归使用模拟HAL，不访问硬件。构建优先使用STM32CubeIDE GCC，也支持 `--cc /path/to/arm-none-eabi-gcc`。只需Python标准库。输出在 `build/Debug/`。

原启动方式保留：A朝B摆放；OFF且停车时短按PA0静态CAL，READY后ON并按键启动。运行中按键、实体OFF或 `STOP!` 可停车。

22组ASan/UBSan、50个ARM单元和HEX/向量/版本检查通过；V19未烧录、未实车验证。pyOCD当前关闭；下次采集必须用匹配V19的ELF，944字节instance不可与V18的912字节混用。
