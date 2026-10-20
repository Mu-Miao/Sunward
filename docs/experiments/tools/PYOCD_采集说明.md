# pyOCD只读采集

`pyocd_capture.py` 使用现有pyOCD独立环境，通过ATK CMSIS-DAP的SWD读取小车RAM，不需要蓝牙串口。它不烧录、复位、暂停、继续运行或启动任务，不修改应用变量；连接初始化会正常配置调试模块。

先结束VS Code的调试会话以释放探针。采集期间不要再在VS Code按F5连接同一个探针。若CPU已被原调试器暂停，本脚本只报告HALTED，不擅自继续运行。

从项目根目录启动：

```sh
"/Users/mzh/Library/Application Support/pipx/venvs/pyocd/bin/python" tools/pyocd_capture.py
```

默认使用 `ATK-D62E11F3`、`stm32f103rc`、4MHz SWD，每秒20次RAM采样。可用 `--seconds 5` 做短时连接核验，用 `--hz 10` 降低频率，或者用 `--layout-only` 仅离线检查ELF布局而不连接硬件。

脚本直接从当前 `build/Debug/Sunward.elf` 的DWARF解析 `instance` 布局，不把C结构偏移硬编码进采集器。连接后先逐字节核对ELF的所有FLASH加载段与板上内容；不匹配时拒绝解析RAM，防止拿错版本布局读出假数据。不会替用户重新下载固件。

产物在忽略提交的 `build/pyocd_capture/<日期_时间>/`：

- `metadata.json`：连接、ELF指纹、FLASH核验字节数、RAM地址、初始CPU状态。
- `samples.jsonl`：持续追加的主机时间、CPU状态、运行状态、航向、目标角、校准/融合状态、探头、轮速/目标、圈数、阶段、里程及停车快照。
- `latest.json`：最近一次样本，原子替换，方便聊天中随时读取。
- `ended.json`：正常退出时的样本数与结束时间。

`build/pyocd_capture/latest_session.json` 指向最近一次采集目录。退出可使用Ctrl-C，或在该次目录创建名为 `STOP` 的空文件；退出时不会自动继续/暂停CPU。USB断开或SWD读失败会终止并报告错误，不自动复位或重试启动程序。

这是对运行中RAM的异步采样，不是固件内部原子快照。一次860字节读取期间变量可能更新，不能将同一行所有值当成严格同时发生，也不能代替10ms逐周期记录。`read_ms`记录每次读取耗时；本脚本不暂停内核，但SWD总线访问仍有开销。它也不提供车的位置真值或证明实车验收通过。

VS Code手动查看航向时可用 `instance.heading.angle`、`instance.heading.reference`、`instance.heading.error`、`instance.heading.gyro_angle`；静态变量解析有歧义时使用 `'car_app.c'::instance.heading.angle`。

## 2026-10-18首次硬件接通

22:19使用pyOCD 0.45.1及现有ATK探针，以attach模式连接成功。板上53300字节FLASH与当前V10 ELF逐字节相同，RAM的instance位于0x200000a8、长度860字节。20Hz采集实测每次读取约12～15ms；连续100个样本的固件dt_us约9983～10014，CPU保持RUNNING。

日志目录：`build/pyocd_capture/20261018_221932/`。接通时航向READY，在线融合EN=0、B=0，Y/G一致；整车状态已为UART_BAD，保存的已完成圈数为2，属于连接前的任务历史。本次没有恢复、重启或清除此状态。探针USB设备打开需要Codex的系统访问权限；沙箱内可枚举探针但不能打开HID，获授权的系统访问后连接成功。
