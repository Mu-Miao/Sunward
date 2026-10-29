# pyOCD只读采集

`pyocd_capture.py` 使用现有pyOCD独立环境，通过ATK CMSIS-DAP的SWD读取小车RAM，不需要蓝牙串口。它不烧录、复位、暂停、继续运行或启动任务，不修改应用变量；连接初始化会正常配置调试模块。

先结束VS Code的调试会话以释放探针。采集期间不要再在VS Code按F5连接同一个探针。若CPU已被原调试器暂停，本脚本只报告HALTED，不擅自继续运行。

从项目根目录启动：

```sh
"/Users/mzh/Library/Application Support/pipx/venvs/pyocd/bin/python" tools/pyocd_capture.py
```

默认使用 `ATK-D62E11F3`、`stm32f103rc`、1MHz SWD（关闭CMSIS-DAP延迟传输），每秒20次RAM采样。可用 `--seconds 5` 做短时连接核验，用 `--hz 10` 降低频率，或者用 `--layout-only` 仅离线检查ELF布局而不连接硬件。

脚本直接从当前 `build/Debug/Sunward.elf` 的DWARF解析 `instance` 布局，不把C结构偏移硬编码进采集器。连接后先逐字节核对ELF的所有FLASH加载段与板上内容；不匹配时拒绝解析RAM，防止拿错版本布局读出假数据。不会替用户重新下载固件。

产物在忽略提交的 `build/pyocd_capture/<日期_时间>/`：

- `metadata.json`：连接、ELF指纹、FLASH核验字节数、RAM地址、初始CPU状态。
- `samples.jsonl`：持续追加的主机时间、CPU状态、运行状态、航向、目标角、校准/融合状态、探头、轮速/目标、圈数、阶段、里程及停车快照。
- `latest.json`：最近一次样本，原子替换，方便聊天中随时读取。
- `ended.json`：正常退出时的样本数与结束时间。

`build/pyocd_capture/latest_session.json` 指向最近一次采集目录。退出可使用Ctrl-C，或在该次目录创建名为 `STOP` 的空文件；退出时不会自动继续/暂停CPU。USB断开或SWD读失败会终止并报告错误，不自动复位或重试启动程序。

这是对运行中RAM的异步采样，不是固件内部原子快照。一次完整RAM结构读取期间变量可能更新，不能将同一行所有值当成严格同时发生，也不能代替10ms逐周期记录。`read_ms`记录每次读取耗时；本脚本不暂停内核，但SWD总线访问仍有开销。它也不提供车的位置真值或证明实车验收通过。

VS Code手动查看航向时可用 `instance.heading.angle`、`instance.heading.reference`、`instance.heading.error`、`instance.heading.gyro_angle`；静态变量解析有歧义时使用 `'car_app.c'::instance.heading.angle`。


历史接通记录见[main实验档案](https://github.com/Mu-Miao/Sunward/tree/main/docs/experiments)。

任务二V26新增task2_straight及gyro_rate_dps记录；建议`--hz 10 --frequency 1000000`。V26 instance为1060字节，旧版996字节，必须先通过Flash比对才解析。

V27新增line.smooth，包含active、ms、previous_error、derivative、p_term、d_term、base、diff。V27 instance为1092字节；V26为1060。必须使用匹配ELF并通过Flash核验；新固件未烧录时使用原V26 ELF。

V29 instance=1112字节，新增task3.normal_trusted、short_gap_active、short_gap_count及short_gap_left/right_mmps。短窗口最多30ms白区、原30ms恢复确认；计数是尝试次数。V28=1092字节。烧录和采集都应选择同版本HEX/ELF，不要用V29 ELF读取仍运行V28的板子。

V30任务二新增task2_c_align，instance1144字节（V29为1112）。task2_straight.entry_ms是CD正常加速的起点；若执行补转，会晚于C点事件。该左右里程包含补转，分析直线漂移需从task2_c_align.active变0之后开始。
