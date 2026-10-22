#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#include "stm32f1xx_hal.h"

/* 1：FreeRTOS 10ms 周期任务；0：保留 TIM2+前台轮询回退路径。 */
#ifndef CAR_USE_FREERTOS
#define CAR_USE_FREERTOS 1U
#endif

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
/* 巡线上层混合控制。LQR 用离线固定增益，PID 补偿 LF04 的离散量化与安装偏差。
 * 这些是保守的上车初值，不是已辨识车模的最优解；调参流程见 FREERTOS_LQR_PID_TESTING.md。 */
#define DPT_HYBRID_ENABLE 1U
#define DPT_HYBRID_LQR_LINE_K 22.0f
#define DPT_HYBRID_LQR_RATE_K 0.050f
#define DPT_HYBRID_LQR_YAW_K 0.80f
#define DPT_HYBRID_PID_KP 20.0f
#define DPT_HYBRID_PID_KI 3.0f
#define DPT_HYBRID_PID_KD 0.020f
#define DPT_HYBRID_LQR_WEIGHT 0.65f
#define DPT_HYBRID_RATE_FILTER 0.25f
#define DPT_HYBRID_INTEGRAL_LIMIT 1.0f
#define DPT_HYBRID_HEADING_K 0.35f
/* 弧线软检查点：每 1/4 弧长只小幅修正后续目标角速度，运行中不停车。 */
#define DPT_ARC_SOFT_CHECKPOINT_ENABLE 1U
#define DPT_ARC_CHECKPOINT_GAIN 0.12f
#define DPT_ARC_CHECKPOINT_STEP_DPS 0.75f
#define DPT_ARC_CHECKPOINT_MAX_BIAS_DPS 2.0f
#define DPT_ARC_CONFIDENCE_FILTER 0.15f
#define DPT_ARC_ENCODER_GOOD_DEG 3.0f
#define DPT_ARC_ENCODER_BAD_DEG 15.0f
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
#define DPT_TASK3_TRACK_WIDTH_MM DPT_TRACK_WIDTH_MM
#define DPT_TASK3_ARC_MM 1256.0f
#define DPT_TASK3_DIAGONAL_MMPS 270.0f
#define DPT_TASK3_ARC_MMPS 200.0f
#define DPT_TASK3_START_MMPS 50.0f
#define DPT_TASK3_ACCEL_MMPS2 150.0f /* 仅用于 AC、BD 斜线段的加速。 */
#define DPT_TASK3_ARC_ACCEL_MMPS2 100.0f /* 半圆入弧沿用 T17 调速前的加速斜坡。 */
#define DPT_TASK3_APPROACH_MM 100.0f
#define DPT_TASK3_MIN_MMPS 60.0f /* 捕获黑线阶段仍使用独立的 50 毫米/秒速度。 */
#define DPT_TASK3_TURN_MMPS 80.0f
/* 车身投影只需覆盖黑线；转向从单一方向逼近时，首次进入 3 度窗口即停，
 * 通常会比完整几何角少转约 3 度，为探头/车身投影留容错。 */
#define DPT_TASK3_ALIGN_TOLERANCE_DEG 3.0f
#define DPT_TASK3_CAPTURE_MMPS 50.0f
#define DPT_TASK3_TIMEOUT_MS 40000U
#define DPT_TASK3_ENTRY_WINDOW_MM 120.0f
#define DPT_TASK3_EXIT_WINDOW_MM 200.0f
#define DPT_TASK3_TURN_LIMIT_MM 80.0f
#define DPT_TASK3_CAPTURE_LIMIT_MM 60.0f
#define DPT_TASK3_TURN_TIMEOUT_MS 4000U
#define DPT_TASK3_CAPTURE_TIMEOUT_MS 2000U
#define DPT_TASK3_SETTLE_MS 30U /* 实测轮速已降至5mm/s后的最小静止确认，不再额外等待200ms。 */
/* 任务三、四受限寻线初值；车身投影是否仍覆盖黑线必须实车确认。 */
#define DPT_TASK3_WHITE_SEEK_MS 500U
#define DPT_TASK3_WHITE_SEEK_MM 20.0f
#define DPT_TASK3_WHITE_SEEK_DEG 12.0f
#define DPT_TASK3_SEEK_MMPS 40.0f
#define DPT_TASK3_GAP_MM 35.0f
#define DPT_TASK3_GAP_DEG 18.0f
#define DPT_TASK3_VERTEX_SEEK_MS 3000U
#define DPT_TASK3_VERTEX_SEEK_MM 80.0f
#define DPT_TASK3_TURN_SLOW_DEG 15.0f
#define DPT_TASK3_TURN_MIN_MMPS 35.0f
/* 任务四沿用任务三几何；独立配置正常速度，以下时限用于故障保护。 */
#define DPT_TASK4_DIAGONAL_MMPS 270.0f /* ARC15：AC/BD 使用见线换段后提高直线速度；弧线保持220。 */
#define DPT_TASK4_ARC_MMPS 220.0f /* V11：正常巡线200→220；边缘仍限120，寻线不提速。 */
#define DPT_TASK4_LAP_WATCHDOG_MS 80000U
/* 任务四未授权寻线的无效读数保持零输出，最多等待 200 毫秒。
 * 已授权的全白桥接使用下方 500 毫秒预算；恢复需连续有效 30 毫秒。
 * 普通桥接受 20 毫米/12 度约束；可信单探头交接使用 35 毫米/18 度边界。
 * 任务三、四的授权全白搜线均最长 500 毫秒。
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
