# 软件验证

基线源码：e95c2dcfe01fcf4579e5ff086fed8f0f9342a192（用户已反馈任务三首次实测一把通过、任务二/三/四均能完成）。

- host_tests_asan_ubsan.log：13套×两个入口，共26组回归通过；模拟HAL，无硬件访问。
- preserved_modes_before/after.log：Task4全部配置字段，以及Task2/3/4完整运行轨迹逐字节一致。基线从e95c2dc独立导出并执行同一回放程序，再与本次源码运行结果比较。覆盖电机、GPIO、OLED、UART和调度状态。Task1新标题引起的预启动选择界面变化不纳入运行轨迹比较。
- arm_build.log：50个ARM编译单元、链接成功；main.c保留原10条未使用实验函数/变量告警。
- elf_layout.json：离线DWARF解析，instance=0x200000a8，996字节；不连接调试器。

复现轨迹：把 preserved_modes.c 放到/tmp/sunward_preserved_modes.c，在基线源码或新源码根目录执行 `python3 /path/to/run_preserved_modes.py /tmp/result.log`，比较结果。

原开发目录未提交文件按SHA256核对保持一致。新版本未烧录、未实车验证，模拟6040ms不作为实车成绩。
