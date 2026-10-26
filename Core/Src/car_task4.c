/* 任务四独立管理四圈计数、总超时和 A 点接续；单圈运动复用任务三核心。 */
#include "car_task4.h"
#include "car_route.h"
#include "car_config.h"
#include <string.h>

CarTask3Config CarTask4_Config(void)
{
    CarTask3Config config = CarRoute_FigureEightBaseConfig();
    config.diagonal_mmps = DPT_TASK4_DIAGONAL_MMPS;
    config.arc_mmps = DPT_TASK4_ARC_MMPS;
    config.bd_turn_relief_deg = DPT_TASK4_BD_TURN_RELIEF_DEG;
    config.arc_entry_ms = DPT_TASK4_ARC_ENTRY_MS;
    config.arc_entry_min_ms = DPT_TASK4_ARC_ENTRY_MIN_MS;
    config.arc_entry_confirm_ms = DPT_TASK4_ARC_ENTRY_CONFIRM_MS;
    config.arc_entry_mm = DPT_TASK4_ARC_ENTRY_MM;
    config.arc_entry_deg = DPT_TASK4_ARC_ENTRY_DEG;
    config.arc_entry_mmps = DPT_TASK4_ARC_ENTRY_MMPS;
    config.arc_entry_diff_mmps = DPT_TASK4_ARC_ENTRY_DIFF_MMPS;
    config.arc_entry_line_gain = DPT_LINE_GAIN * config.arc_entry_mmps / DPT_LINE_CRUISE_MMPS;
    config.edge_follow = DPT_TASK4_EDGE_FOLLOW;
    config.edge_follow_mmps = DPT_TASK4_EDGE_FOLLOW_MMPS;
    config.edge_follow_diff_mmps = DPT_TASK4_EDGE_FOLLOW_DIFF_MMPS;
    config.edge_follow_mm = DPT_TASK4_EDGE_FOLLOW_MM;
    config.edge_follow_ms = DPT_TASK4_EDGE_FOLLOW_MS;
    config.edge_follow_deg = DPT_TASK4_EDGE_FOLLOW_DEG;
    config.rolling_entry = DPT_TASK4_ROLLING_ENTRY;
    config.continuous_motion = DPT_TASK4_CONTINUOUS;
    config.turn_mmps = DPT_TASK4_TURN_MMPS;
    config.turn_inner_mmps = DPT_TASK4_TURN_INNER_MMPS;
    config.turn_travel_limit_mm = DPT_TASK4_TURN_LIMIT_MM;
    /* 外层统一执行四圈总计 160 秒的超时保护。
       不将任务三的 40 秒比赛时限单独施加到任务四每一圈。 */
    config.deadline_ms = DPT_TASK4_LAP_WATCHDOG_MS;
    config.lost_ms = DPT_TASK4_LOST_MS;
    config.white_seek_ms = DPT_TASK4_WHITE_SEEK_MS;
    config.recovery_ms = DPT_TASK4_RECOVERY_MS;
    /* 2026-10-18：BD 末段已有有效探头观测，满名义距离后却全白搜索超时。
       前置探头的碰线里程不等于车轴到 D 的几何长度；恢复原终点窗口内
       连续确认有效线，不再越过候选线等待满里程。原上界/搜索预算保留。 */
    config.bd_full_distance = 0U;
    /* A/B/C/D均按同一±3度容差移动确认。B与BD共用少转目标。 */
    config.b_full_align = 1U;
    config.b_align_tolerance_deg = config.align_tolerance_deg;
    /* D点使用完整切线与独立容差；边缘探头不能提前约8度结束转向。
       双轮连续转向仍受时间与独立位移预算限制。 */
    config.d_full_align = 1U;
    config.d_align_tolerance_deg = config.align_tolerance_deg;
    config.da_outer_gap = 1U;
    config.history_gap = 1U;
    config.da_gap_mm = DPT_TASK4_DA_GAP_MM;
    config.da_gap_deg = DPT_TASK4_DA_GAP_DEG;
    return config;
}

int CarTask4_Start(CarTask4 *task, CarTask3 *lap,
    const CarTask3Config *config, const CarTask3Input *input)
{
    if (!task || !lap || !config || !input || task->active) return 0;
    if (!CarTask3_Start(lap, config, input)) return 0;
    /* 新任务启动时清圈数；只有成功启动单圈核心后才进入运行状态。 */
    memset(task, 0, sizeof *task);
    task->active = 1U;
    task->start_ms = input->now_ms;
    task->ac_reference_deg = lap->start_heading_deg;
    return 1;
}

void CarTask4_Cancel(CarTask4 *task, CarTask3 *lap)
{
    if (task) task->active = 0U;
    CarTask3_Cancel(lap);
}

void CarTask4_Update(CarTask4 *task, CarTask3 *lap,
    const CarTask3Input *input, CarTask3Output *output)
{
    /* 每次先清空输出，缺少输入或任务已停止时不会保留旧电机目标。 */
    if (!output) return;
    memset(output, 0, sizeof *output);
    if (!task || !lap || !input || !task->active) return;
    task->elapsed_ms = (uint32_t)(input->now_ms - task->start_ms);
    if (task->elapsed_ms >= DPT_TASK4_TIMEOUT_MS) {
        CarTask4_Cancel(task, lap);
        lap->fault = T3_TIMEOUT;
        output->reset_controllers = 1U;
        return;
    }
    CarTask3Phase previous_phase = lap->phase;
    CarTask3Input lap_input = *input;
    lap_input.continue_at_a = lap->config.continuous_motion && task->completed_laps + 1U < DPT_TASK4_LAPS;
    CarTask3_Update(lap, &lap_input, output);
    if ((lap->phase == T3_ARC_ENTRY && previous_phase != T3_ARC_ENTRY) ||
        (lap->phase == T3_ARC && previous_phase != T3_ARC && previous_phase != T3_ARC_ENTRY))
        ++task->arc_number;
    if (lap->phase == T3_FAULT) { task->active = 0U; return; }
    if (lap->phase == T3_ARC) {
        if (output->control == T3_HALT) {
            task->resume_pending = 1U;
            task->resume_active = 0U;
        } else if (output->control == T3_FOLLOW_LINE && task->resume_pending) {
            if (!task->resume_active) { task->resume_active = 1U; task->resume_ms = input->now_ms; }
            float ramp = DPT_TASK4_RESUME_MMPS + DPT_TASK4_RESUME_ACCEL_MMPS2 *
                (float)(uint32_t)(input->now_ms - task->resume_ms) / 1000.0f;
            if (output->base_mmps > ramp) output->base_mmps = ramp;
            if (ramp >= lap->config.arc_mmps) task->resume_pending = task->resume_active = 0U;
        }
    } else task->resume_pending = task->resume_active = 0U;
    if (lap->phase != T3_DONE) return;
    /* A点出口确认后只计圈一次；连续模式前三圈当拍接续，末圈仍停稳完成。 */
    ++task->completed_laps;
    if (task->completed_laps == DPT_TASK4_LAPS) { task->active = 0U; return; }
    if (lap->config.continuous_motion) {
        if (!CarTask3_NextLap(lap, input, output)) task->active = 0U;
        return;
    }
    CarTask3Config config = lap->config;
    config.start_from_ab = 1U; /* 在 A 点复用已有的陀螺仪转向及停稳阶段。 */
    if (!CarTask3_Start(lap, &config, input)) {
        task->active = 0U;
        memset(output, 0, sizeof *output);
        output->reset_controllers = 1U;
        return;
    }
    /* 继续使用首次启动时保存的 AC 参考，不依据本圈终点角度重新定义方向。
       保留本圈结束的 A 点提示及零运动输出，禁止下一圈重复发出启动 A 提示。 */
    lap->start_heading_deg = task->ac_reference_deg;
    lap->pending_start_signal = 0U;
}
