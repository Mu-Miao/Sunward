#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#include "stm32f1xx_hal.h"

/* 集中保存现有标定值和调试参数，DPT_ 命名与原操作记录对应。
 * 参数调整统一在本文件进行，避免各模块分别保存不同数值。 */

/* 初始台架参数，依据此前单轮测试设置。 */
#define DPT_TARGET_MMPS 60.0f
#define DPT_LEFT_KP 32.0f
#define DPT_LEFT_KI 20.0f
#define DPT_RIGHT_KP 32.0f
#define DPT_RIGHT_KI 20.0f
#define DPT_MAX_CMD 1800
#define DPT_DURATION_MS 3000U
#define DPT_MANUAL_MMPS 120U
#define DPT_MANUAL_MAX_MMPS 300U
#define DPT_MANUAL_MAX_CMD 3000
#define DPT_MANUAL_DURATION_MS 10000U
#define DPT_SWITCH_BRAKE_MS 200U
#define DPT_LEFT_CPR 60104.3333f
#define DPT_RIGHT_CPR 59996.3333f
#define DPT_CIRC_M 0.2042035225f
#define DPT_MIN_BAT_RAW 1200U /* 台架测试的启动电压门槛，不作为电池放电截止电压规格。 */

/* 航向纠偏参数：每度角度误差对应的左右轮差速，单位为毫米/秒；不是角速度增益。 */
#define DPT_HEADING_GAIN 1.0f
#define DPT_HEADING_MAX_DIFF 15.0f
#define DPT_HEADING_MAX_ERROR 20.0f
/* V10：暂停在线轮速/陀螺零偏学习与扣除，任务三/四使用静态CAL后的陀螺积分。
 * 21:32实测第二圈A的融合累计修正约8.9度，先隔离该变量；不加A固定补偿。
 * 保留160mm轮距、轮速角速度诊断和融合内核，当前开关0时不参与控制。 */
#define DPT_HEADING_FUSION_ENABLE 0U
#define DPT_TRACK_WIDTH_MM 160.0f
#define DPT_HEADING_FUSION_STRAIGHT_DPS 10.0f
#define DPT_HEADING_FUSION_MIN_MMPS 40.0f
#define DPT_HEADING_FUSION_DISAGREE_DPS 5.0f
#define DPT_HEADING_FUSION_BIAS_GAIN 0.3f
#define DPT_HEADING_FUSION_BIAS_MAX_DPS 1.0f
#define DPT_HEADING_FUSION_STABLE_MS 200U
/* 定距任务：按编码器估算毫米里程，有效滚动周长仍需实测核对。 */
#define DPT_DISTANCE_MM 200.0f
#define DPT_DISTANCE_TIMEOUT_MS 10000U
#define DPT_DISTANCE_SLOW_MM 40.0f
#define DPT_DISTANCE_MIN_MMPS 40.0f
#define DPT_DISTANCE_SETTLE_MS 200U
#define DPT_DISTANCE_BRAKE_TIMEOUT_MS 1500U
/* 一米直行任务参数，与 GO20 的二十厘米测试分别配置。 */
#define DPT_ROUTE_MM 1000.0f
#define DPT_ROUTE_CRUISE_MMPS 120.0f
#define DPT_ROUTE_START_MMPS 40.0f
#define DPT_ROUTE_ACCEL_MMPS2 100.0f
#define DPT_ROUTE_SLOW_MM 100.0f
#define DPT_ROUTE_TIMEOUT_MS 13500U
#define DPT_ROUTE_SIGNAL_MS 500U
/* C10B 的 KEY_U 接 PA0，高电平表示按下；每次操作前必须先稳定松开。 */
#define DPT_KEY_DEBOUNCE_MS 30U
#define DPT_KEY_HOLD_MS 1500U /* 仅在使能 OFF 且空闲时生效；一次长按只切换一次，松手后才能再次切换。 */
/* LF04 接线方向：原始黑线为 0、白底为 1；物理从左到右为 O4/O3/O2/O1。
   增益依据此前短距离方向测试设置，半圆稳定性仍需场地重复验证。 */
#define DPT_LINE_BLACK_LEVEL 0U
#define DPT_LINE_MM (400.0f * 3.1415926f)
#define DPT_LINE_CRUISE_MMPS 200.0f
#define DPT_LINE_START_MMPS 50.0f
#define DPT_LINE_GAIN 64.0f
#define DPT_LINE_MAX_DIFF 80.0f
#define DPT_LINE_TIMEOUT_MS 45000U
#define DPT_LINE_CENTER_MS 100U
#define DPT_LINE_INVALID_MS 100U
/* 任务二路线：AB 直线、BC 半圆、CD 直线、DA 半圆；正常运行无需遥控。 */
#define DPT_TASK2_STRAIGHT_MM 1000.0f
#define DPT_TASK2_ARC_MM 1256.0f /* 实测 BC 半圆全白出口对应平均编码器里程约 1256 毫米，独立半圆测试参数保持原值。 */
#define DPT_TASK2_STRAIGHT_MMPS 200.0f
#define DPT_TASK2_TIMEOUT_MS 45000U /* 开发阶段的故障超时保护；任务二验收用时要求不超过 30 秒。 */
#define DPT_TASK2_ENTRY_WINDOW_MM 200.0f
#define DPT_TASK2_EXIT_WINDOW_MM 200.0f
#define DPT_TASK2_EDGE_MS 30U
#define DPT_TASK2_END_ANGLE_DEG 20.0f
/* 任务三几何与衔接限制：A 点车头朝 B 启动，再由陀螺仪对准 AC；左转角度为正。 */
#define DPT_TASK3_START_FROM_AB 1U /* 约定的初始摆放方向：A 点朝 B。 */
#define DPT_TASK3_SPAN_MM 1000.0f
#define DPT_TASK3_RADIUS_MM 400.0f
#define DPT_TASK3_ARC_MM 1256.0f
#define DPT_TASK3_DIAGONAL_MMPS 240.0f
#define DPT_TASK3_ARC_MMPS 200.0f
#define DPT_TASK3_START_MMPS 50.0f
#define DPT_TASK3_ACCEL_MMPS2 150.0f /* 仅用于 AC、BD 斜线段的加速。 */
#define DPT_TASK3_ARC_ACCEL_MMPS2 100.0f /* 半圆入弧沿用 T17 调速前的加速斜坡。 */
#define DPT_TASK3_APPROACH_MM 100.0f
#define DPT_TASK3_MIN_MMPS 60.0f /* 捕获黑线阶段仍使用独立的 50 毫米/秒速度。 */
#define DPT_TASK3_TURN_MMPS 80.0f
/* 基础任务三使用完整几何目标；任务四 B 点微调见下方独立参数。 */
#define DPT_TASK3_CAPTURE_MMPS 50.0f
#define DPT_TASK3_TIMEOUT_MS 40000U
#define DPT_TASK3_ENTRY_WINDOW_MM 120.0f
#define DPT_TASK3_EXIT_WINDOW_MM 200.0f
#define DPT_TASK3_TURN_LIMIT_MM 80.0f
#define DPT_TASK3_CAPTURE_LIMIT_MM 60.0f
#define DPT_TASK3_TURN_TIMEOUT_MS 4000U
#define DPT_TASK3_CAPTURE_TIMEOUT_MS 2000U
/* 任务三、四受限寻线初值；车身投影是否仍覆盖黑线必须实车确认。 */
#define DPT_TASK3_WHITE_SEEK_MS 300U
#define DPT_TASK3_WHITE_SEEK_MM 20.0f
#define DPT_TASK3_WHITE_SEEK_DEG 12.0f
#define DPT_TASK3_SEEK_MMPS 40.0f
#define DPT_TASK3_VERTEX_SEEK_MS 3000U
#define DPT_TASK3_VERTEX_SEEK_MM 80.0f
#define DPT_TASK3_TURN_SLOW_DEG 15.0f
#define DPT_TASK3_TURN_MIN_MMPS 35.0f
/* 任务四沿用任务三几何；独立配置正常速度，以下时限用于故障保护。 */
#define DPT_TASK4_MAX_CMD 2200 /* V14：仅任务四1800→2200；轮速/堵转/反向等保护阈值保留。 */
#define DPT_TASK4_ROLLING_ENTRY 1U /* V15：C/D边走边确认，切线对正后直接捕线，不等待停车。 */
#define DPT_TASK4_DIAGONAL_MMPS 270.0f /* V13：250→270，仅任务四正常直线；终点降速保留。 */
#define DPT_TASK4_ARC_MMPS 240.0f /* V13：220→240，仅正常巡线；入弧50、边缘120，寻线不提速。 */
#define DPT_TASK4_BD_TURN_RELIEF_DEG 3.0f /* V12：仅 B→BD 少左转3°，BD 直线保持同一目标。 */
/* V13：捕线后先低速入弧；参数是软件验证初值，稳定性需真实场地确认。 */
#define DPT_TASK4_ARC_ENTRY_MS 1500U
#define DPT_TASK4_ARC_ENTRY_MIN_MS 200U
#define DPT_TASK4_ARC_ENTRY_CONFIRM_MS 60U
#define DPT_TASK4_ARC_ENTRY_MM 60.0f
#define DPT_TASK4_ARC_ENTRY_DEG 25.0f
#define DPT_TASK4_ARC_ENTRY_MMPS 50.0f
#define DPT_TASK4_ARC_ENTRY_DIFF_MMPS 10.0f /* 40/60mm/s，对应160mm轮距下约400mm半径。 */
#define DPT_TASK4_LAP_WATCHDOG_MS 80000U
/* 任务四未授权寻线的无效读数保持零输出，最多等待 200 毫秒。
 * 已授权的全白桥接使用下方 500 毫秒预算；恢复需连续有效 30 毫秒。
 * 普通桥接受 20 毫米/12 度约束；DA 外侧交接使用下方独立边界。任务三保持 300 毫秒。
 * 下列数值为待实车验证的容错初值，不改变任务三和全局 PI 标定。 */
#define DPT_TASK4_LOST_MS 200U
#define DPT_TASK4_WHITE_SEEK_MS 500U
/* 外侧探头丢线时两轮正向小半径扫线；给内轮前进目标，避免零驱动下被动回滚。
 * 反转保护与原时间/位移/角度预算均保留，不能保证实物不回滚。 */
#define DPT_TASK4_GAP_SWEEP_MMPS 80.0f
#define DPT_TASK4_GAP_INNER_MMPS 20.0f
/* LF04 中心间距 33/14.5/33mm，线宽19mm，探头前伸约45mm。
 * V6 DA外侧、V7 CB/DA可信内外单探头交接使用更紧的两轮前进扫线；
 * 仍限500ms，独立35mm/18度。保留DA前缀以对应既有参数记录。 */
#define DPT_TASK4_DA_GAP_OUTER_MMPS 140.0f
#define DPT_TASK4_DA_GAP_INNER_MMPS 20.0f
#define DPT_TASK4_DA_GAP_MM 35.0f
#define DPT_TASK4_DA_GAP_DEG 18.0f
#define DPT_TASK4_RECOVERY_MS 30U
#define DPT_TASK4_RESUME_MMPS 50.0f
#define DPT_TASK4_RESUME_ACCEL_MMPS2 100.0f
#define DPT_TASK4_EDGE_MMPS 120.0f
#define DPT_TASK4_TURN_SLOW_DEG 15.0f
#define DPT_TASK4_TURN_MIN_MMPS 35.0f
#define DPT_TASK4_LAPS 4U
#define DPT_TASK4_TIMEOUT_MS 160000U
#endif
