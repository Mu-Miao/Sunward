# 验证记录

host_tests_asan_ubsan.log：14套×CarApp/兼容入口，共28组通过。覆盖各任务完整路线/提示/终点、EDGE和连续换段、停止及硬件故障优先级。新增speed测试扫描任务二/三/四全部探头图样，验证有效图样的轮目标峰值330mm/s；原350mm/s实测轮速门槛保留。

comparison.json：与main 2850bfb相同模拟用时的对比、仅8个正常速度/加速度/任务一减速距离宏的差异，以及固件源文件差异审计。baseline_host_tests.log为main版本交付包中完整测试日志。复现比较可在本分支根目录运行compare_speed.py（它读取/tmp/all-speed-tests.log及build/Debug/manifest.json）。所有比较都是主机模拟，不是实测。

arm_build.log：50个ARM编译单元和链接成功。main.c原有10条实验函数/变量未使用告警保持。
elf_layout.json：只从ELF读取DWARF，instance 0x200000a8，996字节；不连接探针。

本版未烧录、未实车验收。配套ELF/HEX及源代码SHA256见manifest.json。
