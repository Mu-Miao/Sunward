# TASK 4 V14 PWM22 代码、固件与验证数据归档

本分支的工程源码固定在V14提交 `c03fd2e59bb70a90d8dd7dd8315db9a8981d5d30`，新增提交只补充归档文件及入口链接；Core、Drivers、tools等工程代码保持原版。未包含后续V15连续入弧或V16双轮连续换段修改。

任务四输出上限2200，直线270mm/s、正常巡线240mm/s、入弧50mm/s。V14仍保留正常换段停车及单轮转向；详细参数与保护见[原版说明](../../TASK4_V14_任务四输出2200_说明.md)。

| 文件 | 内容 |
| --- | --- |
| [Sunward_V14_PWM22.hex](firmware/Sunward_V14_PWM22.hex) | 当时编译完成的可烧录固件，屏幕标题TASK 4 V14 PWM22 |
| [Sunward_V14_PWM22.elf](firmware/Sunward_V14_PWM22.elf) | 匹配HEX的ELF，也用于pyOCD只读采集布局 |
| [host_tests.log](validation/host_tests.log) | 当时16组ASan/UBSan主机回归日志，模拟HAL，无硬件访问 |
| [arm_build.log](validation/arm_build.log) | 当时50个ARM单元编译及链接日志 |
| [build_manifest.json](validation/build_manifest.json) | 原始构建清单、参数、内存占用及SHA256 |
| [pyocd_layout.json](validation/pyocd_layout.json) | ELF离线结构布局，instance为904字节；zero_decode是零填充结构示例，不是车辆实测 |
| [manifest.json](manifest.json) | 本归档的来源、验证范围及逐文件SHA256 |

## 数据范围

已有验证数据确认16组主机回归和50个ARM单元构建通过，FLASH 55160B、RAM 5856B。这次上传使用原来保存的固件与日志，没有重新编译、烧录或启动电机。备份的80个Core/tools文件已逐一与V14源码提交核对一致，HEX/ELF校验值也与原始清单一致。

本地未找到匹配V14 ELF校验值的pyOCD现场采集记录，因此本目录不提供V14实测用时、成功圈数或稳定性结论。仓库内的V13超时记录、V11完成记录属于各自版本；当前连接采集的V16故障同样不能当作V14数据。

需要烧录时明确选择本目录的HEX，并确认屏幕标题TASK 4 V14 PWM22。pyOCD应使用本目录ELF，并先校验板上FLASH；不要用当前V16的ELF解释V14内存。
