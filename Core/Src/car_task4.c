/* 任务四独立管理四圈计数、总超时和 A 点接续；单圈运动复用任务三核心。 */
#include "car_task4.h"
#include "car_route.h"
#include "car_config.h"
#include <string.h>

CarTask3Config CarTask4_Config(void)
{
    CarTask3Config config = CarRoute_Task3Config();
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
    /* 外层统一执行四圈总计 160 秒的超时保护。
       不将任务三的 40 秒比赛时限单独施加到任务四每一圈。 */
    config.deadline_ms = DPT_TASK4_LAP_WATCHDOG_MS;
    config.lost_ms = DPT_TASK4_LOST_MS;
    config.white_seek_ms = DPT_TASK4_WHITE_SEEK_MS;
    config.recovery_ms = DPT_TASK4_RECOVERY_MS;
    /* 2026-10-18：BD 末段已有有效探头观测，满名义距离后却全白搜索超时。
       前置探头的碰线里程不等于车轴到 D 的几何长度；恢复原终点窗口内
       停车并连续确认有效线，不再越过候选线等待满里程。原上界/搜索预算保留。 */
    config.bd_full_distance = 0U;
    /* 各点统一 ±3 度停稳复核；B 的目标与 BD 行驶共同减去独立少转量。 */
    config.b_full_align = 1U;
    config.b_align_tolerance_deg = config.align_tolerance_deg;
    /* DA 入弧前不能在切线前约 8 度就交给边缘探头反馈；
       仅任务四 D 点完整对正并复核停稳，不改变捕获图样和原转向预算。 */
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
    CarTask3_Update(lap, input, output);
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
    /* T20 单圈核心仅在 A 点两轮停稳后报告完成，此时圈数只增加一次。 */
    ++task->completed_laps;
    if (task->completed_laps == DPT_TASK4_LAPS) { task->active = 0U; return; }
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
