#include "car_task3_port.h"
#include "car_internal.h"
#include <math.h>

CarTask3Input CarTask3Port_Input(const CarAppContext *app, uint32_t now,
    int enabled, int stop_requested, int protection_fault)
{
    CarTask3Input i = {0};
    i.now_ms = now;
    i.sample_age_ms = (uint32_t)(now - app->line.sample_ms);
    i.distance_mm = (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f;
    i.yaw_deg = app->heading.angle;
    i.left_mmps = app->wheel.items[LEFT].velocity;
    i.right_mmps = app->wheel.items[RIGHT].velocity;
    i.heading_ready = app->heading.phase == Y_READY;
    i.line_valid = app->line.valid;
    i.black_count = app->line.count;
    i.centered = app->line.valid && app->line.count == 2U && app->line.black[1] && app->line.black[2];
    i.outermost = app->line.valid && app->line.count == 1U && (app->line.black[0] || app->line.black[3]);
    i.side_probe = app->line.valid && app->line.count == 1U;
    i.enabled = enabled != 0;
    i.stop_requested = stop_requested != 0;
    i.protection_fault = protection_fault != 0;
    return i;
}

void CarTask3Port_Apply(CarAppContext *app, const CarTask3Output *o, uint32_t now)
{
    app->route.targets[LEFT] = app->route.targets[RIGHT] = 0.0f;
    /* 整车已锁存故障时，之前计算的运动输出不得重新使电机启动。 */
    if (app->runtime.state != RUNNING) {
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
        return;
    }
    if (o->reset_controllers || o->control == T3_HALT) {
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
        /* 阶段停车和失线恢复会中断巡线采样，不能让旧微分/积分跨段复用。 */
        CarHybrid_Reset(&app->line.hybrid);
    }
    if (o->control == T3_HOLD_HEADING) {
        app->heading.reference = o->heading_deg;
        app->heading.error = CarMath_WrapDegrees(o->heading_deg - app->heading.angle);
        float limit = fminf(DPT_HEADING_MAX_DIFF, o->base_mmps * 0.5f);
        app->heading.diff = fmaxf(-limit, fminf(limit, DPT_HEADING_GAIN * app->heading.error));
        app->route.targets[LEFT] = o->base_mmps - app->heading.diff;
        app->route.targets[RIGHT] = o->base_mmps + app->heading.diff;
    } else if (o->control == T3_TURN_FORWARD) {
        app->heading.reference = o->heading_deg;
        app->heading.error = CarMath_WrapDegrees(o->heading_deg - app->heading.angle);
        app->route.targets[LEFT] = o->left_mmps;
        app->route.targets[RIGHT] = o->right_mmps;
        /* 任务三、四接近目标角时降低外轮速度，减少不同车架惯性导致的过冲。
           仍按陀螺仪确认角度，并保留原时间、距离及内轮零目标保护。 */
        if (CarMotion_IsFigureEight(app->route.motion)) {
            float slow = app->route.motion == MOVE_TASK4 ? DPT_TASK4_TURN_SLOW_DEG : DPT_TASK3_TURN_SLOW_DEG;
            float floor = app->route.motion == MOVE_TASK4 ? DPT_TASK4_TURN_MIN_MMPS : DPT_TASK3_TURN_MIN_MMPS;
            float scale = fminf(1.0f, fabsf(app->heading.error) / slow);
            for (unsigned w = 0; w < 2U; ++w)
                if (app->route.targets[w] > 0.0f)
                    app->route.targets[w] = fminf(app->route.targets[w],
                        fmaxf(floor, app->route.targets[w] * scale));
            app->route.base = (app->route.targets[LEFT] + app->route.targets[RIGHT]) * 0.5f;
        }
    } else if (o->control == T3_SEEK_LINE) {
        /* 仅核心已授权的弧段全白桥接；不调用正常巡线接口，也不伪造 valid。 */
        if (app->route.task3.phase != T3_ARC || !app->route.task3.loss_seek ||
            !app->route.task3.arc_valid_seen || !app->route.task3_trace.valid_seen ||
            app->line.valid || app->line.count != 0U) {
            app->runtime.drive(0, 0); CarWheel_Clear(app); return;
        }
        float base = fminf(o->base_mmps, 50.0f);
        float heading = CarMath_WrapDegrees(o->heading_deg - app->heading.angle);
        float last_error = app->route.task3_trace.valid_error;
        float direction = last_error > 0.0f ? 1.0f : -1.0f;
        if (app->route.motion == MOVE_TASK4 && app->route.task3.config.history_gap && app->route.task3_trace.lost_direction != 0)
            direction = (float)app->route.task3_trace.lost_direction;
        float diff;
        if (app->route.motion == MOVE_TASK4 && app->route.task3.gap_seek &&
            (app->route.task3.config.history_gap || (app->route.task3.leg == 3U && fabsf(last_error) > 2.5f))) {
            /* 可信 CB/DA 内外单探头历史：覆盖33mm间距与19mm线宽的交接空档。
               内轮保持正目标，方向沿用首次失线快照，不跟随未确认毛刺翻转。 */
            float outer = DPT_TASK4_DA_GAP_OUTER_MMPS, inner = DPT_TASK4_DA_GAP_INNER_MMPS;
            base = (outer + inner) * 0.5f;
            diff = (outer - inner) * 0.5f * direction;
        } else if (app->route.motion == MOVE_TASK4 && fabsf(last_error) > 2.5f) {
            /* 最外侧探头刚离线时，黑线可能落在相邻探头的盲区内。
               两轮正向、外轮快内轮慢，将探头朝相邻通道扫过；不伪造有效线，
               原核心仍先检查 500ms/20mm/12度及更高优先级故障。 */
            float outer = fminf(DPT_TASK4_GAP_SWEEP_MMPS, base * 2.0f);
            float inner = fminf(DPT_TASK4_GAP_INNER_MMPS, outer * 0.5f);
            base = (outer + inner) * 0.5f;
            diff = (outer - inner) * 0.5f * (last_error > 0.0f ? 1.0f : -1.0f);
        } else if (app->route.motion == MOVE_TASK4 && app->route.task3.config.history_gap) {
            /* 中心/多探头后全白仍按普通边界；用冻结方向和当前弧段补足零误差情况。 */
            diff = base * 0.4f * direction;
        } else {
            float limit = base * 0.4f;
            diff = DPT_LINE_GAIN * last_error * base / DPT_LINE_CRUISE_MMPS - DPT_HEADING_GAIN * heading;
            diff = fmaxf(-limit, fminf(limit, diff));
        }
        app->route.targets[LEFT] = base + diff; app->route.targets[RIGHT] = base - diff;
        app->route.base = base; app->line.diff = diff;
        app->heading.reference = o->heading_deg; app->heading.error = heading;
    } else if (o->control == T3_FOLLOW_LINE) {
        CarLine_SetTargets(app, now, o->base_mmps);
    }
    if (app->runtime.state == RUNNING && o->signal_point != T3_NO_POINT) {
        app->signal.ms = now;
        CarSignal_Set(app, 1);
    }
}

TestState CarTask3Port_StopState(CarTask3Fault f)
{
    switch (f) {
    case T3_TIMEOUT: return TIMEOUT;
    case T3_SENSOR: case T3_HEADING: case T3_TURN: return YAW_BAD;
    case T3_LOST: return LINE_LOST;
    case T3_SETTLE: return SETTLE_BAD;
    case T3_EDGE: case T3_CONFIG: case T3_DISTANCE: return LINE_BAD;
    case T3_ABORT: default: return STOPPED;
    }
}
