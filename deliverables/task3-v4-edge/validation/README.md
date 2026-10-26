# 验证记录

`host_tests_asan_ubsan.log`：12 套回归 × CarApp / 兼容入口，共 24 组通过。
`preserved_modes_before.log` / `preserved_modes_after.log`：以相同 C 回放程序、相同输入对 main 4c516f0 与本分支运行，文件逐字节一致。包括 Task4 全部配置字段，以及 Task2 / Task4 从启动完成之后到终止的 GPIO、电机、OLED、UART、调度状态轨迹。预启动的任务选择 UI 有 Task3 新标题，因此不纳入比较。

复现：将此处 preserved_modes.c 放到 /tmp/sunward_preserved_modes.c，在待测源码根目录运行 `python3 /path/to/run_preserved_modes.py /tmp/result.log`。基线需单独取出 main 4c516f0。不会连接硬件。

`arm_build.log`：50 个 ARM 编译单元和链接成功，main.c 中保留 10 个未使用实验函数/变量的既有告警。
`elf_layout.json`：仅从新 ELF 解析 DWARF，不连接调试器。instance 地址 0x200000a8，大小 996 字节；实际采集必须使用同包 ELF。

模拟输入与软件验证不等于烧录或实车验收。
