# 四任务提速V2试验包

正常直线400、半圆300mm/s；比赛PI输出上限3600（50%），比赛FAST门槛500mm/s，其他模式350。提前减速、EDGE与连续换段、圈数/超时和其余保护保留。

标题依次为任务一V4、任务二V24、任务三V6、任务四V21。使用Sunward.hex，保留匹配Sunward.elf/manifest.json。详见本轮说明.md及validation。

本包仅软件验证和ARM构建完成，未烧录、未落地实测。基线codex/all-tasks-speed-v1 26914df；main保持2850bfb。V1回退包位于deliverables/all-tasks-speed-v1。
