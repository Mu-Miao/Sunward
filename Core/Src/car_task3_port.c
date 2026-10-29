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
    i.line_error = app->line.error;
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
    int arc_mode = CarMotion_IsFigureEight(app->route.motion) ||
        (app->route.motion == MOVE_TASK2 && CarRoute_IsArc(app));
    float previous_targets[2] = {app->route.targets[LEFT], app->route.targets[RIGHT]};
    if (o->control != T3_FOLLOW_LINE || o->reset_controllers || app->runtime.state != RUNNING)
        CarLine_ResetControl(app);
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
           仍按陀螺仪确认角度；任务三、四连续模式保留正内轮与最小差速。 */
        if (arc_mode) {
            float slow = app->route.task3.config.continuous_motion ? DPT_TASK4_TURN_SLOW_DEG : DPT_TASK3_TURN_SLOW_DEG;
            float floor = app->route.motion == MOVE_TASK4 ? DPT_TASK4_TURN_MIN_MMPS : DPT_TASK3_TURN_MIN_MMPS;
            float scale = fminf(1.0f, fabsf(app->heading.error) / slow);
            if (arc_mode && app->route.task3.config.continuous_motion) {
                unsigned outer = app->route.targets[LEFT] > app->route.targets[RIGHT] ? LEFT : RIGHT;
                unsigned inner = 1U - outer;
                /* 内轮保持20；外轮最低55，保留原35mm/s最小差速。 */
                app->route.targets[outer] = fminf(app->route.targets[outer],
                    fmaxf(floor + app->route.targets[inner], app->route.targets[outer] * scale));
            } else for (unsigned w = 0; w < 2U; ++w)
                if (app->route.targets[w] > 0.0f)
                    app->route.targets[w] = fminf(app->route.targets[w],
                        fmaxf(floor, app->route.targets[w] * scale));
            app->route.base = (app->route.targets[LEFT] + app->route.targets[RIGHT]) * 0.5f;
        }
    } else if (o->control == T3_ENTER_ARC) {
        /* 八字任务捕线后的受限入弧；保留轮速 PI，不使用固定 PWM。 */
        const CarTask3 *t = &app->route.task3;
        if (!arc_mode || t->phase != T3_ARC_ENTRY ||
            !t->config.arc_entry_ms || (!app->line.valid && (app->line.count || !t->loss_seek))) {
            app->runtime.drive(0, 0); CarWheel_Clear(app); return;
        }
        float base = fminf(o->base_mmps, 50.0f);
        float diff = (o->left_mmps - o->right_mmps) * 0.5f;
        float limit = base * 0.4f;
        diff = fmaxf(-limit, fminf(limit, diff));
        app->route.targets[LEFT] = base + diff; app->route.targets[RIGHT] = base - diff;
        app->route.base = base; app->line.diff = diff;
    } else if (o->control == T3_EDGE_FOLLOW) {
        const CarTask3 *t = &app->route.task3;
        if (!arc_mode || t->phase != T3_ARC ||
            !t->config.edge_follow || !t->edge_follow_active || !t->arc_valid_seen ||
            (!app->line.valid && (app->line.count || !t->loss_seek)) ||
            (t->lost_seen && !t->loss_seek)) {
            app->runtime.drive(0, 0); CarWheel_Clear(app); return;
        }
        float base = fminf(o->base_mmps, t->config.edge_follow_mmps);
        if (t->lost_seen) base = fminf(base, t->config.capture_mmps);
        float limit = fminf(t->config.edge_follow_diff_mmps, fmaxf(0.0f, base - t->config.turn_inner_mmps));
        float diff = fmaxf(-limit, fminf(limit, (o->left_mmps - o->right_mmps) * 0.5f));
        app->route.targets[LEFT] = base + diff; app->route.targets[RIGHT] = base - diff;
        app->route.base = base; app->line.diff = diff;
    } else if (o->control == T3_SEEK_LINE) {
        /* 仅核心已授权的弧段桥接及移动恢复确认；不伪造 valid，不续期预算。 */
        if (app->route.task3.phase != T3_ARC || !app->route.task3.loss_seek ||
            !app->route.task3.arc_valid_seen || !app->route.task3_trace.valid_seen ||
            (app->line.valid ? !(app->route.task3.config.continuous_motion && app->route.task3.lost_seen) : app->line.count != 0U)) {
            app->runtime.drive(0, 0); CarWheel_Clear(app); return;
        }
        float base = fminf(o->base_mmps, 50.0f);
        float heading = CarMath_WrapDegrees(o->heading_deg - app->heading.angle);
        float last_error = app->route.task3_trace.valid_error;
        float direction = last_error > 0.0f ? 1.0f : -1.0f;
        if (arc_mode && app->route.task3.config.history_gap && app->route.task3_trace.lost_direction != 0)
            direction = (float)app->route.task3_trace.lost_direction;
        float diff;
        if (arc_mode && app->route.task3.gap_seek &&
            (app->route.task3.config.history_gap || (app->route.task3.leg == 3U && fabsf(last_error) > 2.5f))) {
            /* 可信 CB/DA 内外单探头历史：覆盖33mm间距与19mm线宽的交接空档。
               内轮保持正目标，方向沿用首次失线快照，不跟随未确认毛刺翻转。 */
            float outer = DPT_TASK4_DA_GAP_OUTER_MMPS, inner = DPT_TASK4_DA_GAP_INNER_MMPS;
            base = (outer + inner) * 0.5f;
            diff = (outer - inner) * 0.5f * direction;
        } else if (arc_mode && fabsf(last_error) > 2.5f) {
            /* 最外侧探头刚离线时，黑线可能落在相邻探头的盲区内。
               两轮正向、外轮快内轮慢，将探头朝相邻通道扫过；不伪造有效线，
               原核心仍先检查 500ms/20mm/12度及更高优先级故障。 */
            float outer = fminf(DPT_TASK4_GAP_SWEEP_MMPS, base * 2.0f);
            float inner = fminf(DPT_TASK4_GAP_INNER_MMPS, outer * 0.5f);
            base = (outer + inner) * 0.5f;
            diff = (outer - inner) * 0.5f * (last_error > 0.0f ? 1.0f : -1.0f);
        } else if (arc_mode && app->route.task3.config.history_gap) {
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
        /* Seed the normal controller from the actual prior mode output. */
        app->route.targets[LEFT] = previous_targets[LEFT];
        app->route.targets[RIGHT] = previous_targets[RIGHT];
        CarLine_SetTargets(app, now, o->base_mmps);
    }
    if (app->runtime.state == RUNNING && arc_mode &&
        app->route.task3.config.continuous_motion && o->retarget_integrals &&
        !o->reset_controllers && o->control != T3_HALT) {
        /* 只在已确认的换段/首次低速确认时缩减降速轮积分。
           防止直线大积分带进低速转向；不增加积分、不发零驱动，
           不重置反向/堵转计时。后续每拍仍由原PI和保护处理。 */
        CarWheel_Retarget(app, previous_targets);
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
