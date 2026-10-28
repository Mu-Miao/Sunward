# 软件验证与测量依据

host_tests_asan_ubsan.log：14套×两个入口，28组通过，模拟HAL不访问硬件。测试覆盖各任务完成/提示/停车/超时、EDGE与连续运动、500/350mm/s FAST隔离及锁存、3600与旧模式限幅隔离、原故障优先级、任务二提前降速和积分缩减。
arm_build.log：50个编译单元链接成功；main.c原有10条未使用实验函数/变量告警保持。HEX校验和、EOF、四个标题和向量有效。
elf_layout.json：离线DWARF解析，未连接或烧录硬件。
simulation_comparison.json：同一套理想目标跟随输入在V1和V2下的结果，均加速；不是实车成绩。
suspended_wheel_measurement_2026-10-28.json：此前完成的悬空PWM测量，供限幅选择参考，不作为本版地面验收。

未烧录，本版未实测。原始固件源码与HEX/ELF哈希在manifest.json。
