/* LF04巡线。电气O1..O4排列为物理左→右O4/O3/O2/O1，黑0白1。 */
#include "car_internal.h"
#include <math.h>
#include <string.h>

void CarLine_ResetControl(CarAppContext *app)
{
    memset(&app->line.smooth, 0, sizeof app->line.smooth);
}

int CarLine_ConfigValid(void)
{
    return isfinite(DPT_LINE_SMOOTH_KP) && DPT_LINE_SMOOTH_KP > 0.0f && DPT_LINE_SMOOTH_KP <= DPT_LINE_GAIN &&
        isfinite(DPT_LINE_SMOOTH_KD) && DPT_LINE_SMOOTH_KD >= 0.0f && DPT_LINE_SMOOTH_KD <= 1.0f &&
        isfinite(DPT_LINE_SMOOTH_D_TAU_S) && DPT_LINE_SMOOTH_D_TAU_S >= 0.02f && DPT_LINE_SMOOTH_D_TAU_S <= 0.15f &&
        isfinite(DPT_LINE_SMOOTH_D_LIMIT) && DPT_LINE_SMOOTH_D_LIMIT >= 0.0f && DPT_LINE_SMOOTH_D_LIMIT <= 20.0f &&
        isfinite(DPT_LINE_SMOOTH_DIFF_RATE) && DPT_LINE_SMOOTH_DIFF_RATE >= 800.0f && DPT_LINE_SMOOTH_DIFF_RATE <= 2400.0f &&
        isfinite(DPT_LINE_SMOOTH_ACCEL) && DPT_LINE_SMOOTH_ACCEL >= 100.0f && DPT_LINE_SMOOTH_ACCEL <= 250.0f;
}

/* Only trusted normal line frames enter this controller. No extra filtering of
 * raw sensors, validity, endpoint confirmation or loss budgets is introduced. */
static void smooth_targets(CarAppContext *app, uint32_t now, float requested)
{
    CarLineState *l = &app->line;
    float dt = (float)(uint32_t)(now - l->smooth.ms) * 0.001f;
    if (l->smooth.active && now == l->smooth.ms) {
        /* Repeated calls do not advance time, but must still obey a lower cap. */
        l->smooth.base = fminf(l->smooth.base, requested);
        float cap = DPT_LINE_MAX_DIFF * l->smooth.base / DPT_LINE_CRUISE_MMPS;
        l->smooth.diff = fmaxf(-cap, fminf(cap, l->smooth.diff));
        goto output;
    }
    if (!l->smooth.active || dt > 0.030f) {
        float seed_base = (app->route.targets[LEFT] + app->route.targets[RIGHT]) * 0.5f;
        l->smooth.base = seed_base > 0.0f ? fminf(requested, seed_base) : requested;
        l->smooth.diff = (app->route.targets[LEFT] - app->route.targets[RIGHT]) * 0.5f;
        l->smooth.derivative = 0.0f;
        dt = app->runtime.dt_us >= 5000U && app->runtime.dt_us <= 30000U ?
            app->runtime.dt_us * 0.000001f : 0.010f;
    } else {
        float raw_derivative = (l->error - l->smooth.previous_error) / dt;
        float alpha = dt / (DPT_LINE_SMOOTH_D_TAU_S + dt);
        l->smooth.derivative += alpha * (raw_derivative - l->smooth.derivative);
    }
    /* Rising base is ramped; any requested slowdown applies immediately. */
    l->smooth.base = fminf(requested, l->smooth.base + DPT_LINE_SMOOTH_ACCEL * dt);
    float scale = l->smooth.base / DPT_LINE_CRUISE_MMPS;
    l->smooth.p_term = DPT_LINE_SMOOTH_KP * l->error * scale;
    l->smooth.d_term = fmaxf(-DPT_LINE_SMOOTH_D_LIMIT, fminf(DPT_LINE_SMOOTH_D_LIMIT,
        DPT_LINE_SMOOTH_KD * l->smooth.derivative)) * scale;
    float limit = DPT_LINE_MAX_DIFF * scale;
    float desired = fmaxf(-limit, fminf(limit, l->smooth.p_term + l->smooth.d_term));
    float step = DPT_LINE_SMOOTH_DIFF_RATE * dt;
    l->smooth.diff += fmaxf(-step, fminf(step, desired - l->smooth.diff));
    /* A lowered base also lowers the differential cap immediately. */
    l->smooth.diff = fmaxf(-limit, fminf(limit, l->smooth.diff));
    l->smooth.previous_error = l->error;
    l->smooth.ms = now;
    l->smooth.active = 1U;
output:
    app->route.base = l->smooth.base;
    l->diff = l->smooth.diff;
    app->route.targets[LEFT] = l->smooth.base + l->diff;
    app->route.targets[RIGHT] = l->smooth.base - l->diff;
}

void CarLine_Read(CarAppContext *app)
{
    app->line.raw[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET;
    app->line.raw[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET;
    app->line.raw[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET;
    app->line.raw[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET;
}

void CarLine_Decode(CarAppContext *app)
{
    static const int weight[4] = {-3, -1, 1, 3};
    CarLine_Read(app);
    int sum = 0, first = -1, last = -1;
    app->line.count = 0U;
    for (unsigned i = 0; i < 4U; ++i) {
        app->line.black[i] = app->line.raw[3U - i] == DPT_LINE_BLACK_LEVEL;
        if (app->line.black[i]) {
            if (first < 0) first = (int)i;
            last = (int)i;
            ++app->line.count;
            sum += weight[i];
        }
    }
    app->line.valid = app->line.count > 0U && app->line.count < 4U && last - first + 1 == (int)app->line.count;
    if (app->line.valid) app->line.error = (float)sum / app->line.count;
}

void CarLine_Sample(CarAppContext *app, uint32_t now, int timing_ok)
{
    if (app->button.selected_motion != MOVE_LINE && app->route.motion != MOVE_LINE && app->button.selected_motion != MOVE_TASK2 && app->route.motion != MOVE_TASK2 && !CarMotion_IsFigureEight(app->button.selected_motion) && !CarMotion_IsFigureEight(app->route.motion)) return;
    if (!timing_ok) { app->line.center_seen = app->line.center_ready = 0U; return; }
    CarLine_Decode(app);
    app->line.sample_ms = now;
    if (app->line.valid && app->line.count == 2U && app->line.black[1] && app->line.black[2]) {
        if (!app->line.center_seen) { app->line.center_seen = 1U; app->line.center_ms = now; }
        app->line.center_ready = (uint32_t)(now - app->line.center_ms) >= DPT_LINE_CENTER_MS;
    } else app->line.center_seen = app->line.center_ready = 0U;
}

void CarLine_SetTargets(CarAppContext *app, uint32_t now, float base)
{
    if (!app->line.valid) {
        CarLine_ResetControl(app);
        /* 无有效线时立即输出零；不足 100 毫秒的短暂失线恢复后可以继续。
           故障锁存后必须重新满足启动条件，不自动搜线。 */
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
        app->route.targets[LEFT] = app->route.targets[RIGHT] = app->line.diff = 0.0f;
        if (!app->line.invalid_seen) { app->line.invalid_seen = 1U; app->line.invalid_ms = now; }
        else if ((uint32_t)(now - app->line.invalid_ms) >= DPT_LINE_INVALID_MS)
            CarApp_Stop(app, app->line.count == 0U ? LINE_LOST : LINE_BAD, '-');
        return;
    }
    app->line.invalid_seen = 0U;
    int competition = app->route.motion == MOVE_TASK2 || CarMotion_IsFigureEight(app->route.motion);
    if (competition && fabsf(app->line.error) < 2.0f) {
        smooth_targets(app, now, base);
        return;
    }
    /* Outermost/large-error correction is immediate; legacy MOVE_LINE unchanged. */
    CarLine_ResetControl(app);
    /* 任务二及八字任务：边缘降速；任务三使用任务四同一巡线配置。 */
    if ((CarMotion_IsFigureEight(app->route.motion) || app->route.motion == MOVE_TASK2) &&
        (app->line.error >= 2.0f || app->line.error <= -2.0f)) {
        float edge = app->route.motion == MOVE_TASK2 ? DPT_TASK2_EDGE_MMPS : DPT_TASK4_EDGE_MMPS;
        if (base > edge) base = edge;
    }
    if (CarMotion_IsFigureEight(app->route.motion) || app->route.motion == MOVE_TASK2) app->route.base = base;
    app->line.diff = DPT_LINE_GAIN * app->line.error * base / DPT_LINE_CRUISE_MMPS;
    float limit = DPT_LINE_MAX_DIFF * base / DPT_LINE_CRUISE_MMPS;
    if (app->line.diff > limit) app->line.diff = limit;
    if (app->line.diff < -limit) app->line.diff = -limit;
    /* 黑线偏右时提高左轮目标速度，使车向右纠偏；两轮目标均保持前进。 */
    app->route.targets[LEFT] = base + app->line.diff;
    app->route.targets[RIGHT] = base - app->line.diff;
}
