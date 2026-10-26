# 任务一 V2 FAST 烧录包

任务一巡航120→240mm/s，加速度100→200mm/s²，PI输出限幅1800→2200，终点前160mm开始减速。保留一米目标、40mm/s末速、200ms停稳确认、500ms完成声光与13.5秒运行超时。OLED显示 TASK 1 V2 FAST。

任务二V22、任务三V4 EDGE、任务四V19 EDGE保持上一实测版本的参数和控制行为。独立半圆、GO20和手动配置保留。26组主机回归与ARM构建通过，验证记录见validation。

使用Sunward.hex烧录，保留配套Sunward.elf、manifest.json。用户先前反馈任务二/三/四能完成，任务三首次测试一把通过；该结果属于上一版。本包还未烧录或实车验证。

分支 codex/task1-speed；基线 e95c2dc。源码工作树 /Users/mzh/.codex/worktrees/task4-v14-rolling/Sunward。按键和CAL流程保持原方式。
