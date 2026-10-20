# dev：从远程main拓展的五项可调控制

基线为 `origin/main` 的 `b484ed9e07100afb1dd661bd5a6f7f6f8ad37371`（V11 FAST，成功任务四）。按用户确认，五项均实现为可调项，无本版实测依据的参数保留main默认效果。开发分支为 `dev`，不改写main，不合并V13～V17试验路线或FreeRTOS分支。

参数集中在 `Core/Inc/car_config.h` 末尾的 `DPT_DEV_*` 区，修改后重新编译生效；也支持编译器 `-D` 覆盖。默认固件标题为 **TASK4 DEV MAIN**。本版未烧录或实测，main历史149.606秒/152.444秒记录仍只属于原V11。

## 五项实现

| 项目 | 可调参数与默认值 | 行为 |
| --- | --- | --- |
| 轮速跟踪与触顶观察 | `DPT_DEV_TASK4_MAX_CMD=1800`；`DPT_DEV_SATURATION_RATIO=0.98`；`DPT_DEV_TRACK_LOG_ENABLE=0` | 为任务四提供独立输出上限，默认仍1800；支持1800～2400。记录实际PI输出接近限幅的时间及绝对跟踪误差。仅观察，不自动提高速度或上限。其他任务限幅不变。 |
| 巡线PD | `DPT_DEV_LINE_KD=0`；`DPT_DEV_LINE_FILTER_MS=40`；`DPT_DEV_LINE_RATE_LIMIT=60`；`DPT_DEV_LINE_D_MAX_MMPS=20` | 对任务三/四有效巡线误差的变化率限幅、低通滤波，再叠加受限D项。D项随基础速度缩放，总差速仍受原80/200比例限制，两轮巡线目标保持前进。默认KD=0即原P控制。 |
| 转向角速度阻尼 | `DPT_DEV_TURN_KD=0`；`DPT_DEV_TURN_FILTER_MS=30`；`DPT_DEV_TURN_D_MAX_MMPS=30`；`DPT_DEV_TURN_WINDOW_DEG=30` | 滤波静态CAL后、现有航向控制使用的角速度。只在目标角附近且正朝目标转动时减外轮，不增速、不改变转向方向，保留原外轮最低速度与角度确认。默认KD=0即main原角度减速。 |
| LF04权重 | `DPT_DEV_LINE_PHYSICAL_WEIGHTS=0`；四个位置参数默认 `-40.25/-7.25/7.25/40.25mm` | 默认继续使用main的 `-3/-1/1/3`。启用物理模式后，按main已有33/14.5/33mm中心间距换算，并归一到外侧绝对值最大3，内侧约±0.54037。四个位置均可独立调整。未宣称完成新的实物标定。 |
| AC、BD独立调整 | `DPT_DEV_AC_HEADING_DEG`、`DPT_DEV_BD_HEADING_DEG`；`DPT_DEV_AC_APPROACH_DELTA_MM`、`DPT_DEV_BD_APPROACH_DELTA_MM`；`DPT_DEV_AC_ENTRY_DELTA_MM`、`DPT_DEV_BD_ENTRY_DELTA_MM`；全部默认0 | 航向修正分别作用于AC/BD及其对应对正；末段减速距离、候选线窗口开启位置分别可调。几何参考与C/D弧段切线不随之累计偏移，原距离上界与寻线预算保留。 |

LF04四个位置参数全名依次为 `DPT_DEV_LINE_LEFT_OUTER_MM`、`DPT_DEV_LINE_LEFT_INNER_MM`、`DPT_DEV_LINE_RIGHT_INNER_MM`、`DPT_DEV_LINE_RIGHT_OUTER_MM`，按物理左→右填写。必须有限、严格递增、首负末正且绝对值不大于200mm，不能拿无效/不连续黑线图样生成方向。

AC/BD航向修正范围±5°；减速距离增量±50mm，作用后距离须为正且小于斜线长度；候选窗口增量±60mm，作用后窗口须为正且小于半段斜线。正候选增量使窗口更早开启，负值使其更晚；斜线最大允许里程仍为原名义长度加原窗口，不能借此延长故障距离上界。

巡线KD范围0～10，滤波常数5～200ms，变化率限幅为正且不超过600误差单位/秒，D项限幅0～80mm/s（按原200mm/s基准）；转向KD范围0～10、D项限幅0～60mm/s、角窗口3～90°。这些是配置合法性边界，不表示范围内任意组合都适合实车。

## 状态、复位与诊断

- 巡线D项在首次采样、5～20ms范围外的采样间隔、失线、任务/路段/阶段切换后重新建立历史，避免旧偏差导数带入新阶段。时间差使用无符号减法，跨计时回绕也受测试覆盖。任务一/二不应用新增D项。
- 角速度滤波在重新CAL时清零；静态CAL、航向积分、关闭在线融合的main设置保留。朝离目标方向运动时，不额外加阻尼导致纠偏轮提速。
- 轮速统计覆盖本次任务三/四运行中正目标轮的采样，包含各运动阶段，为每趟累计值。`CarWheel_Clear`不抹掉统计证据，停车后仍可查看；新任务启动才清统计。长时间计数会等比例折半，避免溢出。
- 开启跟踪日志后，每秒尝试发送 `SPD TL=... TR=... VL=... VR=... SAT100=... MAE10=... SEG=... P=...`。`SAT100=10000`表示100.00%观察时间接近限幅，`MAE10=125`表示12.5mm/s平均绝对误差。队列满时丢帧，不阻塞或重启任务；单帧不超过现有160字节限制。原DPI/NAV日志继续提供输出命令及路线信息。
- pyOCD采集器新增导出各轮 `observed_us/saturated_us/abs_error_seconds` 和 `turn_rate_dps/line_d/line_error_rate`，兼容旧ELF中缺少这些字段的情况。连接仍先逐字节核对FLASH，不混用参考ELF。

main的STOP/OFF、低压/ADC/IMU/UART、节拍、超速、反向、堵转、失线/寻线/捕线/转向/路线边界、四圈160秒总超时保持。新参数在初始化时校验，非法参数拒绝使能。本分支沿用main原停车确认、单轮前进转向和完整几何目标；未引入V17的连续双轮转向或积分衔接。

## 软件验证与构建

- main原14组ASan/UBSan回归作为基线，dev默认配置全部通过；新增默认功能测试后共16组，两套API均运行。
- 非默认组合（PD KD=0.5、转向KD=0.5、物理权重、任务四上限2200、跟踪日志、AC+2°/BD候选窗口−20mm）两套API测试通过；另有4组非法参数初始化拒绝测试，共 **22组**。
- 相同测试程序分别编译链接main与dev：任务三一圈、任务四四圈的电机命令、目标速度、阶段、点位提示和完成计时一致。14,684次驱动事件校验值均为 `4af157ec22504f00`。这是指定模拟轨迹的默认回归一致性，不代表实车性能完全相同。
- 新增测试覆盖所有16种探头图样、滤波与差速限幅、失线/阶段切换后的D项复位、时钟回绕、双向转向阻尼、角速度滤波/CAL复位、AC/BD独立参数与原距离上界、输出上限隔离、统计计数/溢出/故障保留及发送拥塞。
- 默认配置和上述非零参数配置均完成50个ARM单元编译与链接。默认FLASH **54388B**、RAM **5888B**；非零测试组合FLASH **55472B**、RAM **5888B**。新增代码无编译警告，原main.c未使用诊断函数/变量警告保留。

复验命令：

```sh
python3 tools/tests/run_tests.py --sanitize
python3 tools/tests/compare_main.py --sanitize
```

采用项目既有EIDE/CubeIDE工程重新编译。此次默认固件交付在本地独立构建目录 `build/dev_main_20261021/build/Debug/Sunward.hex`（相对于原Sunward工作区）；不会覆盖当前V17的 `build/Debug/Sunward.hex`。SHA256：

- HEX：`274056f0e22c92e1c0b06e235ea238240b5c989befa309608079e2580b11f3cc`
- ELF：`faa1c8eecc9d1ba14fa959d721a5b9f789e7378c99c927376223f510b9373464`

离线布局instance为936字节。非零配置ELF只用于构建验证，不作为已调好的实车配置；上述交付HEX采用main默认参数。验证摘要见 [records/dev/2026-10-21_main_tuning.json](records/dev/2026-10-21_main_tuning.json)。本轮未连接硬件、烧录、复位、启动电机或进行物理标定。
