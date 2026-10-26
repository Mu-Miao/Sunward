# 四任务提速第一轮烧录包

基线main 2850bfb；实验分支codex/all-tasks-speed-v1。main保留上版作为回退。

任务一V3：正常270mm/s，提前180mm减速。任务二V23、任务三V5、任务四V20：正常直线280、半圆250mm/s，直线加速度180mm/s²。EDGE、转向、搜索、圈数、超时及硬件保护保持。

使用Sunward.hex烧录，保留匹配的Sunward.elf/manifest.json。28组回归及ARM构建通过；模拟用时改善，不代表实测成绩。本包尚未烧录、未实车验证。

回退固件：main中的deliverables/task1-v2-fast/Sunward.hex（其中包含任务二V22、任务三V4、任务四V19）。
