/* 运动模式、定距和四段路线状态机。出口由里程/航向/稳定边沿共同判定。 */
#include "car_internal.h"
#include <math.h>
#include <string.h>
#include "car_task4.h"

static void task2_edge_reset(CarRouteState *r)
{
    r->edge_follow = r->edge_follow_pending = r->edge_follow_exit_seen =
        r->edge_follow_resume = r->edge_follow_valid = r->edge_follow_outer =
        r->edge_follow_side = r->edge_follow_lost = r->edge_follow_loss_seek =
        r->edge_follow_gap_seek = r->edge_follow_recovery = 0U;
    r->edge_follow_diff = 0.0f;
}

CarTask3Config CarRoute_FigureEightBaseConfig(void)
{
    CarTask3Config c = CarTask3_DefaultConfig();
    c.start_from_ab = DPT_TASK3_START_FROM_AB;
    c.span_mm = DPT_TASK3_SPAN_MM; c.radius_mm = DPT_TASK3_RADIUS_MM;
    c.arc_mm = DPT_TASK3_ARC_MM;
    c.diagonal_mmps = DPT_TASK3_DIAGONAL_MMPS; c.arc_mmps = DPT_TASK3_ARC_MMPS;
    c.start_mmps = DPT_TASK3_START_MMPS; c.acceleration_mmps2 = DPT_TASK3_ACCEL_MMPS2;
    c.arc_acceleration_mmps2 = DPT_TASK3_ARC_ACCEL_MMPS2;
    c.approach_mm = DPT_TASK3_APPROACH_MM; c.minimum_mmps = DPT_TASK3_MIN_MMPS;
    c.turn_mmps = DPT_TASK3_TURN_MMPS; c.capture_mmps = DPT_TASK3_CAPTURE_MMPS;
    c.deadline_ms = DPT_TASK3_TIMEOUT_MS;
    c.entry_window_mm = DPT_TASK3_ENTRY_WINDOW_MM; c.exit_window_mm = DPT_TASK3_EXIT_WINDOW_MM;
    c.turn_travel_limit_mm = DPT_TASK3_TURN_LIMIT_MM; c.capture_limit_mm = DPT_TASK3_CAPTURE_LIMIT_MM;
    c.align_timeout_ms = DPT_TASK3_TURN_TIMEOUT_MS; c.capture_timeout_ms = DPT_TASK3_CAPTURE_TIMEOUT_MS;
    c.white_seek_ms = DPT_TASK3_WHITE_SEEK_MS;
    c.white_seek_mm = DPT_TASK3_WHITE_SEEK_MM; c.white_seek_deg = DPT_TASK3_WHITE_SEEK_DEG;
    c.seek_mmps = DPT_TASK3_SEEK_MMPS;
    c.vertex_seek_ms = DPT_TASK3_VERTEX_SEEK_MS; c.vertex_seek_mm = DPT_TASK3_VERTEX_SEEK_MM;
    c.recovery_ms = 30U;
    c.settle_ms = DPT_DISTANCE_SETTLE_MS; c.brake_timeout_ms = DPT_DISTANCE_BRAKE_TIMEOUT_MS;
    return c;
}

/* 任务三独立取得任务四已验证配置的副本；不经四圈包装器。
 * 共用基础配置由上方函数提供，避免Task3/Task4配置互相调用递归。 */
CarTask3Config CarRoute_Task3Config(void)
{
    CarTask3Config c = CarTask4_Config();
    c.deadline_ms = DPT_TASK3_TIMEOUT_MS;
    return c;
}

int CarRoute_IsDistance(Motion m)
{
    return m == MOVE_DISTANCE || m == MOVE_ROUTE || m == MOVE_LINE || m == MOVE_TASK2 || CarMotion_IsFigureEight(m);
}

void CarRoute_Select(CarAppContext *app, Motion selected, uint32_t now)
{
    app->runtime.drive(0, 0);
    CarWheel_Clear(app);
    CarSignal_Set(app, 0);
    app->button.selected_motion = selected;
    memset(&app->route.task3, 0, sizeof app->route.task3);
    memset(&app->route.task4, 0, sizeof app->route.task4);
    memset(&app->route.task3_trace, 0, sizeof app->route.task3_trace);
    memset(&app->route.reverse_trace, 0, sizeof app->route.reverse_trace);
    memset(&app->route.vertex_trace, 0, sizeof app->route.vertex_trace);
    memset(&app->route.edge_trace, 0, sizeof app->route.edge_trace);
    app->route.leg = app->route.edge_seen = 0U;
    app->route.leg_start = 0.0f;
    app->route.elapsed_ms = 0U;
    task2_edge_reset(&app->route);
    memset(&app->route.task2_straight, 0, sizeof app->route.task2_straight);
    app->route.motion = MOVE_NONE;    app->route.tracking = 0U;
    app->route.distance_mm[LEFT] = app->route.distance_mm[RIGHT] = 0.0f;
    app->route.goal = app->button.selected_motion == MOVE_LINE ? DPT_LINE_MM :
        (app->button.selected_motion == MOVE_TASK2 ? 2.0f * (DPT_TASK2_STRAIGHT_MM + DPT_TASK2_ARC_MM) : DPT_ROUTE_MM);
    if (CarMotion_IsFigureEight(selected))
        app->route.goal = (selected == MOVE_TASK4 ? DPT_TASK4_LAPS : 1U) *
            2.0f * (hypotf(DPT_TASK3_SPAN_MM, 2.0f * DPT_TASK3_RADIUS_MM) + DPT_TASK3_ARC_MM);
    app->line.center_seen = app->line.center_ready = app->line.invalid_seen = 0U;
    app->line.diff = 0.0f;
    app->ui.button_log_ms = now - 1000U;
    app->ui.render_ms = now - 200U;
}

void CarRoute_Request(CarAppContext *app, Motion requested, uint32_t now)
{
    if (app->runtime.state == BRAKING || app->heading.phase == Y_WARM || app->heading.phase == Y_CAL) return;
    if ((requested == MOVE_HEADING || CarRoute_IsDistance(requested)) && (app->runtime.state == RUNNING || app->heading.phase != Y_READY)) return;
    if (app->runtime.state == RUNNING && (requested == MOVE_TEST || requested == app->route.motion)) return;
    if (requested == MOVE_LINE) {
        CarLine_Decode(app); /* 读取当前探头 GPIO 核对启动条件，不使用屏幕上可能过期的读数。 */
        if (!app->line.center_ready || (uint32_t)(now - app->line.sample_ms) > 15U ||
            !app->line.valid || !app->line.black[1] || !app->line.black[2] || app->line.count != 2U) return;
    }
    if (CarMotion_IsFigureEight(requested)) {
        if ((uint32_t)(now - app->runtime.old_ms) > 15U ||
            fabsf(app->wheel.items[LEFT].velocity) > 5.0f || fabsf(app->wheel.items[RIGHT].velocity) > 5.0f) return;
        CarLine_Decode(app);
        app->line.sample_ms = now;
    }
    if (!app->runtime.on_seen || (uint32_t)(now - app->runtime.on_ms) < 100U) CarApp_Stop(app, OFF, '-');
    else if (!app->power.valid || (uint32_t)(now - app->power.sample_ms) > 150U) CarApp_Stop(app, ADC_BAD, '-');
    else if (app->power.raw < DPT_MIN_BAT_RAW) CarApp_Stop(app, LOW, '-');
    else if (app->runtime.primed) {
        uint8_t was_running = app->runtime.state == RUNNING;
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
        CarSignal_Set(app, 0);
        app->route.motion = requested;
        float speed = (requested == MOVE_TEST || requested == MOVE_HEADING || CarRoute_IsDistance(requested)) ? DPT_TARGET_MMPS : (float)app->route.manual_speed;
        app->route.targets[LEFT] = app->route.targets[RIGHT] = speed;
        if (requested == MOVE_BACK) app->route.targets[LEFT] = app->route.targets[RIGHT] = -speed;
        if (requested == MOVE_LEFT) app->route.targets[LEFT] = speed * 0.5f;
        if (requested == MOVE_RIGHT) app->route.targets[RIGHT] = speed * 0.5f;
        if (requested == MOVE_HEADING || CarRoute_IsDistance(requested)) {
            app->heading.reference = app->heading.angle;
            app->heading.error = app->heading.diff = 0.0f;
        }
        app->route.run_duration = (requested == MOVE_TEST || requested == MOVE_HEADING || CarRoute_IsDistance(requested)) ? DPT_DURATION_MS : app->route.manual_duration;
        app->route.tracking = 0;
        if (CarRoute_IsDistance(requested)) {
            for (unsigned i = 0; i < 2U; ++i) {
                app->route.distance_count[i] = (uint16_t)__HAL_TIM_GET_COUNTER(app->wheel.items[i].encoder);
                app->route.distance_mm[i] = 0.0f;
            }
            if (requested == MOVE_TASK2) {
                memset(&app->route.task2_straight, 0, sizeof app->route.task2_straight);
                memset(&app->route.task3, 0, sizeof app->route.task3);
                app->route.leg = app->route.edge_seen = 0U;
                app->route.leg_start = 0.0f;
                app->route.start_heading = app->heading.reference;
                app->route.elapsed_ms = 0U;
                task2_edge_reset(&app->route);
            }
            app->route.goal = requested == MOVE_TASK2 ? 2.0f * (DPT_TASK2_STRAIGHT_MM + DPT_TASK2_ARC_MM) : requested == MOVE_ROUTE ? DPT_ROUTE_MM : (requested == MOVE_LINE ? DPT_LINE_MM : DPT_DISTANCE_MM);
            app->route.cruise = requested == MOVE_TASK2 ? DPT_TASK2_STRAIGHT_MMPS : requested == MOVE_ROUTE ? DPT_ROUTE_CRUISE_MMPS : (requested == MOVE_LINE ? DPT_LINE_CRUISE_MMPS : DPT_TARGET_MMPS);
            app->route.slow = requested == MOVE_ROUTE ? DPT_ROUTE_SLOW_MM : DPT_DISTANCE_SLOW_MM;
            app->route.base = requested == MOVE_TASK2 ? DPT_TASK2_START_MMPS : requested == MOVE_ROUTE ? DPT_ROUTE_START_MMPS : (requested == MOVE_LINE ? DPT_LINE_START_MMPS : DPT_TARGET_MMPS);
            app->line.invalid_seen = 0U;
            app->line.diff = 0.0f;
            app->route.targets[LEFT] = app->route.targets[RIGHT] = app->route.base;
            app->route.tracking = 1;
            app->route.settle_seen = 0;
            app->route.run_duration = requested == MOVE_TASK2 ? DPT_TASK2_TIMEOUT_MS : requested == MOVE_ROUTE ? DPT_ROUTE_TIMEOUT_MS :
                (requested == MOVE_LINE ? DPT_LINE_TIMEOUT_MS : DPT_DISTANCE_TIMEOUT_MS);
        }
        app->route.start_ms = app->route.switch_ms = now;
        app->route.switching = was_running;
        app->runtime.fault_wheel = '-';
        if (CarMotion_IsFigureEight(requested)) {
            CarTask3Config config = requested == MOVE_TASK4 ? CarTask4_Config() : CarRoute_Task3Config();
            CarTask3Input input = CarTask3Port_Input(app, now, 1, 0, 0);
            int started = requested == MOVE_TASK4 ?
                CarTask4_Start(&app->route.task4, &app->route.task3, &config, &input) :
                CarTask3_Start(&app->route.task3, &config, &input);
            if (!started) {
                CarApp_Stop(app, CarTask3Port_StopState(app->route.task3.fault), '-');
                return;
            }
            memset(&app->route.task3_trace, 0, sizeof app->route.task3_trace);
            memset(&app->route.reverse_trace, 0, sizeof app->route.reverse_trace);
            memset(&app->route.vertex_trace, 0, sizeof app->route.vertex_trace);
            memset(&app->route.edge_trace, 0, sizeof app->route.edge_trace);
            app->route.leg = 0U; app->route.leg_start = 0.0f; app->route.elapsed_ms = 0U;
            app->route.goal = (requested == MOVE_TASK4 ? DPT_TASK4_LAPS : 1U) * 2.0f * (app->route.task3.diagonal_mm + config.arc_mm);
            app->route.cruise = config.diagonal_mmps;
            app->route.base = 0.0f;
            app->route.targets[LEFT] = app->route.targets[RIGHT] = 0.0f;
            app->route.run_duration = requested == MOVE_TASK4 ? DPT_TASK4_TIMEOUT_MS : config.deadline_ms;
        }
        app->runtime.state = RUNNING;
    }
}

static void task2_next_leg(CarAppContext *app, uint32_t now, float travelled)
{
    /* 换段保持连续运动：不停车、不清 PI，下一拍由降速重定向缩减积分。 */
    app->route.edge_seen = app->line.invalid_seen = 0U;
    task2_edge_reset(&app->route);
    if (app->route.leg == 3U) { CarApp_Stop(app, BRAKING, '-'); return; }
    ++app->route.leg;
    app->route.leg_start = travelled;
    if (!CarRoute_IsArc(app)) app->route.task3.phase = T3_IDLE;
    /* 弧入口已捕线：本拍立即按任务四方式低速接管，不再等外沿探头确认。 */
    app->route.edge_follow_pending = CarRoute_IsArc(app) ? 1U : 0U;
    app->signal.ms = now;
    CarSignal_Set(app, 1); /* 经过 B、C、D 时触发声光提示，提示本身不暂停路线执行。 */
    app->route.cruise = CarRoute_IsArc(app) ? DPT_TASK2_ARC_MMPS : DPT_TASK2_STRAIGHT_MMPS;
    app->heading.reference = app->route.leg == 1U || app->route.leg == 2U ?
        CarMath_WrapDegrees(app->route.start_heading + 180.0f) : app->route.start_heading;
    if (CarRoute_IsArc(app)) app->route.task2_arc_exit_heading = app->heading.reference;
    if (app->route.leg == 2U) {
        /* C点已确认出线：保存当前陀螺坐标，不再追回名义180度。 */
        app->route.task2_straight.seen = 1U;
        app->route.task2_straight.entry_ms = now;
        app->route.task2_straight.nominal_heading = app->heading.reference;
        app->route.task2_straight.reference_heading = app->heading.angle;
        app->route.task2_straight.entry_yaw = app->heading.angle;
        app->heading.reference = app->heading.angle;
        for (unsigned w=0;w<2;w++) {
            app->route.task2_straight.entry_targets[w] = app->route.targets[w];
            app->route.task2_straight.entry_velocity[w] = app->wheel.items[w].velocity;
            app->route.task2_straight.entry_integral[w] = app->wheel.items[w].integral;
        }
    }
    app->heading.error = CarMath_WrapDegrees(app->heading.reference - app->heading.angle);
    app->heading.diff = DPT_HEADING_GAIN * app->heading.error;
    if (app->heading.diff > DPT_HEADING_MAX_DIFF) app->heading.diff = DPT_HEADING_MAX_DIFF;
    if (app->heading.diff < -DPT_HEADING_MAX_DIFF) app->heading.diff = -DPT_HEADING_MAX_DIFF;
}

void CarRoute_UpdateTask2(CarAppContext *app, uint32_t now)
{
    if (app->route.motion != MOVE_TASK2 || app->runtime.state != RUNNING) return;
    float travelled = (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f;
    float progress = travelled - app->route.leg_start;
    int arc = CarRoute_IsArc(app);
    float length = arc ? DPT_TASK2_ARC_MM : DPT_TASK2_STRAIGHT_MM;
    float window = arc ? DPT_TASK2_EXIT_WINDOW_MM : DPT_TASK2_ENTRY_WINDOW_MM;
    if (arc) return; /* Arc point detection belongs to the common controller. */
    int expected_edge=app->line.valid;
    if (progress >= length - window && progress <= length + window && expected_edge) {
        if (!app->route.edge_seen) { app->route.edge_seen = 1U; app->route.edge_ms = now; }
        if ((uint32_t)(now - app->route.edge_ms) >= DPT_TASK2_EDGE_MS) {
            task2_next_leg(app,now,travelled);
            return;
        }
    } else app->route.edge_seen = 0U;
    if (progress > length + window)
        CarApp_Stop(app, arc ? LINE_BAD : LINE_LOST, '-');
}

int CarRoute_IsArc(CarAppContext *app)
{
 return app->route.motion == MOVE_TASK2 && (app->route.leg & 1U); 
}

int CarRoute_UsesLine(CarAppContext *app)
{
 return app->route.motion == MOVE_LINE || CarRoute_IsArc(app); 
}

int CarRoute_UsesHeading(CarAppContext *app)
{
 return !CarMotion_IsFigureEight(app->route.motion) && app->route.motion != MOVE_LINE && !CarRoute_IsArc(app);
}

/* Thin adapter: Task2 BC/DA both turn right, using the actual Task3/4 arc core. */
static void task2_line_history(CarAppContext *app, const CarTask3Input *i, int before)
{
    if (before && !i->line_valid && !app->route.task3_trace.lost_seen) {
        app->route.task3_trace.lost_seen=1U; app->route.task3_trace.lost_ms=i->now_ms;
        app->route.task3_trace.lost_arc_number=app->route.leg==1U?1U:2U;
        float e=app->route.task3_trace.valid_error;
        if (fabsf(e)<0.5f && app->route.task3_trace.previous_seen) e-=app->route.task3_trace.previous_error;
        app->route.task3_trace.lost_direction=fabsf(e)>=0.5f?(e>0?1:-1):1;
        app->route.task3_trace.lost_yaw=i->yaw_deg;
        memcpy(app->route.task3_trace.lost_raw,app->line.raw,4U);
        for(unsigned w=0;w<2;w++) {
            app->route.task3_trace.targets[w]=app->route.targets[w];
            app->route.task3_trace.velocity[w]=app->wheel.items[w].velocity;
        }
    }
    if (!before && i->line_valid && !app->route.task3.lost_seen) {
        if (app->route.task3_trace.valid_seen) {
            memcpy(app->route.task3_trace.previous_raw,app->route.task3_trace.valid_raw,4U);
            app->route.task3_trace.previous_error=app->route.task3_trace.valid_error;
            app->route.task3_trace.previous_seen=1U;
        }
        memcpy(app->route.task3_trace.valid_raw,app->line.raw,4U);
        app->route.task3_trace.valid_error=i->line_error;
        app->route.task3_trace.valid_diff=app->line.diff;
        app->route.task3_trace.valid_seen=1U;
    }
}

static int task2_arc_line(CarAppContext *app, uint32_t now, float *base, const float previous[2])
{
    CarRouteState *r=&app->route;
    if (r->motion!=MOVE_TASK2 || !(r->leg&1U)) return 0;
    CarTask3Input input=CarTask3Port_Input(app,now,1,0,0);
    int entered=r->edge_follow_pending;
    if (entered) {
        CarTask3Config config=CarTask4_Config();
        config.arc_mm=DPT_TASK2_ARC_MM; config.arc_mmps=DPT_TASK2_ARC_MMPS;
        config.deadline_ms=DPT_TASK2_TIMEOUT_MS;
        memset(&r->task3_trace,0,sizeof r->task3_trace);
        if (!CarTask3_BeginArc(&r->task3,&config,&input,r->task2_arc_exit_heading,-1)) {
            CarApp_Stop(app,CarTask3Port_StopState(r->task3.fault),'-'); return 1;
        }
        r->edge_follow_pending=0U;
        r->edge_follow_start_mm=input.distance_mm;
    }
    task2_line_history(app,&input,1);
    CarTask3Output output; CarTask3_Update(&r->task3,&input,&output);
    if (entered) output.retarget_integrals=1U;
    if (r->task3.phase==T3_FAULT) {
        TestState why=CarTask3Port_StopState(r->task3.fault);
        if (r->task3.fault==T3_LOST && app->line.count!=0U) why=LINE_BAD;
        task2_edge_reset(r); CarApp_Stop(app,why,'-'); return 1;
    }
    /* Core already confirmed distance + exit heading + stable white for C/A. */
    if (r->task3.phase==T3_FINAL_BRAKE) {
        task2_next_leg(app,now,input.distance_mm);
        if (app->runtime.state==RUNNING) {
            r->base=*base=DPT_TASK2_START_MMPS;
            r->targets[LEFT]=r->targets[RIGHT]=r->base;
        }
        return 1;
    }
    if (r->task3.phase!=T3_ARC) { CarApp_Stop(app,LINE_BAD,'-'); return 1; }
    CarTask3Port_Apply(app,&output,now);
    (void)previous; /* The shared output port performs retargeting exactly once. */
    task2_line_history(app,&input,0);
    r->edge_follow=r->task3.edge_follow_active;
    r->edge_follow_resume=r->task3.resume_active;
    r->edge_follow_valid=r->task3.arc_valid_seen;
    r->edge_follow_outer=r->task3.arc_outer_valid; r->edge_follow_side=r->task3.arc_side_valid;
    r->edge_follow_lost=r->task3.lost_seen; r->edge_follow_loss_seek=r->task3.loss_seek;
    r->edge_follow_gap_seek=r->task3.gap_seek; r->edge_follow_recovery=r->task3.recovery_seen;
    r->edge_follow_ms=r->task3.phase_ms; r->edge_follow_diff=r->task3.entry_diff_mmps;
    r->edge_seen=r->task3.edge_seen; r->edge_ms=r->task3.edge_ms;
    r->edge_follow_exit_seen=r->task3.edge_seen; r->edge_follow_exit_ms=r->task3.edge_ms;
    r->edge_follow_resume_ms=r->task3.resume_ms;
    r->edge_follow_lost_ms=r->task3.lost_start_ms; r->edge_follow_lost_mm=r->task3.lost_start_mm;
    r->edge_follow_lost_yaw=r->task3.lost_yaw_deg; r->edge_follow_recovery_ms=r->task3.recovery_start_ms;
    *base=r->base;
    return 1;
}

void CarRoute_Update(CarAppContext *app, uint32_t now, const uint16_t count[2])
{
    if (CarMotion_IsFigureEight(app->route.motion)) {
        if (app->runtime.state != RUNNING || !app->route.tracking) return;
        for (unsigned i = 0; i < 2U; ++i) {
            if (fabsf(app->wheel.items[i].velocity) > CarWheel_SpeedLimit(app->route.motion)) {
                CarApp_Stop(app, FAST, i == LEFT ? 'L' : 'R');
                return;
            }
        }
        for (unsigned i = 0; i < 2U; ++i) {
            app->route.distance_mm[i] += CarWheel_Millimetres(i, CarMath_Delta16(count[i], app->route.distance_count[i]));
            app->route.distance_count[i] = count[i];
        }
        int enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
        CarTask3Input input = CarTask3Port_Input(app, now, enabled, 0, 0);
        CarTask3Output output;
        if ((app->route.task3.phase == T3_ARC || app->route.task3.phase == T3_ARC_ENTRY) &&
            !input.line_valid && !app->route.task3_trace.lost_seen) {
            /* 在 HALT 清除 PI 和目标速度前锁存失线数据。
               车辆可能已被搬动，之后的 GPIO 读数不能作为当时失线的证据。 */
            app->route.task3_trace.lost_seen = 1U;
            app->route.task3_trace.lost_ms = now;
            app->route.task3_trace.lost_arc_number = app->route.motion == MOVE_TASK4 ? app->route.task4.arc_number : (app->route.task3.leg == 1U ? 1U : 2U);
            float direction_error = app->route.task3_trace.valid_error;
            if (fabsf(direction_error) < 0.5f && app->route.task3_trace.previous_seen)
                direction_error -= app->route.task3_trace.previous_error;
            app->route.task3_trace.lost_direction = fabsf(direction_error) >= 0.5f ? (direction_error > 0.0f ? 1 : -1) : (app->route.task3.leg == 1U ? -1 : 1);
            app->route.task3_trace.lost_yaw = input.yaw_deg;
            memcpy(app->route.task3_trace.lost_raw, app->line.raw, 4U);
            for (unsigned w = 0; w < 2U; ++w) {
                app->route.task3_trace.targets[w] = app->route.targets[w];
                app->route.task3_trace.velocity[w] = app->wheel.items[w].velocity;
            }
        }
        CarTask3Phase prior_phase = app->route.task3.phase;
        uint32_t prior_duration = (uint32_t)(now - app->route.task3.phase_ms);
        float prior_travel = input.distance_mm - app->route.task3.phase_start_mm;
        if (app->route.motion == MOVE_TASK4) CarTask4_Update(&app->route.task4, &app->route.task3, &input, &output);
        else CarTask3_Update(&app->route.task3, &input, &output);
        if (app->route.task3.phase == T3_FAULT && app->route.task3.fault == T3_EDGE && !app->route.edge_trace.seen) {
            const CarTask3 *t = &app->route.task3;
            float progress = input.distance_mm - t->leg_start_mm;
            uint8_t limits = 0U;
            /* 仅解释核心已经锁存的 F6；不改变任何边界或输出。 */
            if (prior_phase == T3_VERTEX_SEEK) {
                if (prior_duration >= t->config.vertex_seek_ms) limits |= T3_LOSS_TIME;
                if (prior_travel >= t->config.vertex_seek_mm || progress > t->diagonal_mm + t->config.entry_window_mm)
                    limits |= T3_LOSS_DISTANCE;
            } else if (prior_phase == T3_CAPTURE) {
                if (prior_duration >= t->config.capture_timeout_ms) limits |= T3_LOSS_TIME;
                if (prior_travel > t->config.capture_limit_mm) limits |= T3_LOSS_DISTANCE;
            } else if (prior_phase == T3_ARC_ENTRY) {
                if (prior_duration >= t->config.arc_entry_ms) limits |= T3_LOSS_TIME;
                if (prior_travel >= t->config.arc_entry_mm) limits |= T3_LOSS_DISTANCE;
            } else if (prior_phase == T3_ARC && t->edge_follow_active) {
                if (prior_duration >= t->config.edge_follow_ms) limits |= T3_LOSS_TIME;
                if (prior_travel >= t->config.edge_follow_mm) limits |= T3_LOSS_DISTANCE;
            } else if (prior_phase == T3_DIAGONAL || prior_phase == T3_ARC) limits |= T3_LOSS_DISTANCE;
            app->route.edge_trace.seen = 1U;
            app->route.edge_trace.phase = (uint8_t)prior_phase;
            app->route.edge_trace.limits = limits;
            memcpy(app->route.edge_trace.raw, app->line.raw, 4U);
            app->route.edge_trace.ms = now;
            app->route.edge_trace.duration_ms = prior_duration;
            app->route.edge_trace.travel_mm = prior_travel;
            app->route.edge_trace.distance_mm = progress;
            app->route.edge_trace.yaw = input.yaw_deg;
        }
        if (output.signal_point == T3_POINT_C || output.signal_point == T3_POINT_D) {
            app->route.vertex_trace.seen = 1U;
            app->route.vertex_trace.point = (uint8_t)output.signal_point;
            app->route.vertex_trace.lap = app->route.motion == MOVE_TASK4 ? app->route.task4.completed_laps + 1U : 1U;
            memcpy(app->route.vertex_trace.raw, app->line.raw, 4U);
            app->route.vertex_trace.distance_mm = input.distance_mm - app->route.task3.leg_start_mm;
            app->route.vertex_trace.yaw = input.yaw_deg;
            app->route.vertex_trace.gyro_yaw = app->heading.gyro_angle;
            app->route.vertex_trace.bias_rate = app->heading.bias_rate;
        }
        app->route.leg = app->route.task3.leg;
        app->route.leg_start = app->route.task3.leg_start_mm;
        app->route.elapsed_ms = app->route.motion == MOVE_TASK4 ? app->route.task4.elapsed_ms : app->route.task3.elapsed_ms;
        app->route.base = output.control == T3_TURN_FORWARD ?
            (output.left_mmps + output.right_mmps) * 0.5f : output.base_mmps;
        if (app->route.task3.phase == T3_FAULT) {
            CarApp_Stop(app, CarTask3Port_StopState(app->route.task3.fault), '-');
        } else if (app->route.task3.phase == T3_DONE) {
            CarApp_Stop(app, DONE, '-');
            app->signal.ms = now;
            CarSignal_Set(app, 1); /* 最终在 A 点停稳后触发一次声光提示。 */
        } else CarTask3Port_Apply(app, &output, now);
        if ((prior_phase != T3_ARC_ENTRY && app->route.task3.phase == T3_ARC_ENTRY) ||
            (prior_phase != T3_ARC && app->route.task3.phase == T3_ARC))
            memset(&app->route.task3_trace, 0, sizeof app->route.task3_trace);
        if (app->runtime.state == RUNNING &&
            (app->route.task3.phase == T3_ARC || app->route.task3.phase == T3_ARC_ENTRY) && input.line_valid &&
            !app->route.task3.lost_seen) {
            if (app->route.task3_trace.valid_seen) {
                memcpy(app->route.task3_trace.previous_raw, app->route.task3_trace.valid_raw, 4U);
                app->route.task3_trace.previous_error = app->route.task3_trace.valid_error;
                app->route.task3_trace.previous_seen = 1U;
            }
            app->route.task3_trace.valid_seen = 1U;
            app->route.task3_trace.lost_seen = 0U; /* 短暂失线恢复后，下一次失线重新建立快照窗口。 */
            memcpy(app->route.task3_trace.valid_raw, app->line.raw, 4U);
            app->route.task3_trace.valid_error = app->line.error;
            app->route.task3_trace.valid_diff = (app->route.targets[LEFT] - app->route.targets[RIGHT]) * 0.5f;
        }
        return; /* 此处结束更新，避免后面的定距和任务二逻辑覆盖任务三、四的输出。 */
    }
                /* 从本次启动时保存的编码器计数开始累加有符号里程。
                   判断定距完成前，先排除采样时间和编码器计数异常。 */
                float previous_base = app->route.base;
                float previous_cruise = app->route.cruise;
                float previous_targets[2] = {app->route.targets[LEFT], app->route.targets[RIGHT]};
                if (app->route.tracking && (app->runtime.state == RUNNING || app->runtime.state == BRAKING)) {
                    if (fabsf(app->wheel.items[LEFT].velocity) > CarWheel_SpeedLimit(app->route.motion))
                        CarApp_Stop(app, FAST, 'L');
                    else if (fabsf(app->wheel.items[RIGHT].velocity) > CarWheel_SpeedLimit(app->route.motion))
                        CarApp_Stop(app, FAST, 'R');
                    else {
                        for (unsigned i = 0; i < 2U; ++i) {
                            float advance=CarWheel_Millimetres(i,CarMath_Delta16(count[i],app->route.distance_count[i]));
                            app->route.distance_mm[i] += advance;
                            if (app->route.motion==MOVE_TASK2 && app->route.leg==2U && app->route.task2_straight.seen) {
                                if (i==LEFT) app->route.task2_straight.left_mm+=advance;
                                else app->route.task2_straight.right_mm+=advance;
                            }
                            app->route.distance_count[i] = count[i];
                        }
                        if (app->route.motion==MOVE_TASK2 && app->route.leg==2U && app->route.task2_straight.seen) {
                            app->route.task2_straight.wheel_yaw_deg=(app->route.task2_straight.right_mm-app->route.task2_straight.left_mm)/DPT_TRACK_WIDTH_MM*57.2957795131f;
                            app->route.task2_straight.gyro_delta_deg=CarMath_WrapDegrees(app->heading.angle-app->route.task2_straight.entry_yaw);
                        }
                        if (app->route.motion == MOVE_TASK2) CarRoute_UpdateTask2(app, now);
                        if (app->route.motion != MOVE_TASK2 && app->runtime.state == RUNNING && (!CarRoute_UsesLine(app) || app->line.valid) && (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f >= app->route.goal)
                            CarApp_Stop(app, BRAKING, '-');
                        if (app->runtime.state == BRAKING) {
                            if (app->wheel.items[LEFT].velocity >= -5.0f && app->wheel.items[LEFT].velocity <= 5.0f &&
                                app->wheel.items[RIGHT].velocity >= -5.0f && app->wheel.items[RIGHT].velocity <= 5.0f) {
                                if (!app->route.settle_seen) { app->route.settle_seen = 1; app->route.settle_ms = now; }
                                else if ((uint32_t)(now - app->route.settle_ms) >= DPT_DISTANCE_SETTLE_MS) CarApp_Stop(app, DONE, '-');
                            } else app->route.settle_seen = 0;
                        }
                    }
                }
                if (app->runtime.state == RUNNING && (app->route.motion == MOVE_HEADING || CarRoute_IsDistance(app->route.motion))) {
                    if (app->heading.phase != Y_READY || (CarRoute_UsesHeading(app) && (app->heading.error > DPT_HEADING_MAX_ERROR || app->heading.error < -DPT_HEADING_MAX_ERROR)))
                        CarApp_Stop(app, YAW_BAD, '-');
                    else {
                        float base = DPT_TARGET_MMPS;
                        if (CarRoute_IsDistance(app->route.motion)) {
                            base = app->route.cruise;
                            float remaining = app->route.motion == MOVE_TASK2 ? DPT_DISTANCE_SLOW_MM : app->route.goal - (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f;
                            if (app->route.motion == MOVE_TASK2 && !CarRoute_IsArc(app)) {
                                float progress = (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f - app->route.leg_start;
                                float end_remaining = DPT_TASK2_STRAIGHT_MM - progress;
                                /* 仅CD预减速；AB保持巡航，确认B捕线后由EDGE接管。 */
                                if (app->route.leg == 2U && end_remaining < DPT_TASK2_APPROACH_MM)
                                    base = fmaxf(DPT_TASK2_APPROACH_MIN_MMPS, app->route.cruise * fmaxf(end_remaining, 0.0f) / DPT_TASK2_APPROACH_MM);
                            } else if (remaining < app->route.slow) {
                                base = app->route.cruise * remaining / app->route.slow;
                                if (base < DPT_DISTANCE_MIN_MMPS) base = DPT_DISTANCE_MIN_MMPS;
                            }
                            if (app->route.motion == MOVE_ROUTE || app->route.motion == MOVE_LINE || app->route.motion == MOVE_TASK2) {
                                float start = app->route.motion == MOVE_TASK2 ? DPT_TASK2_START_MMPS :
                                    (app->route.motion == MOVE_LINE ? DPT_LINE_START_MMPS : DPT_ROUTE_START_MMPS);
                                float accel = app->route.motion == MOVE_TASK2 ? DPT_TASK2_ACCEL_MMPS2 :
                                    (app->route.motion == MOVE_LINE ? DPT_LINE_ACCEL_MMPS2 : DPT_ROUTE_ACCEL_MMPS2);
                                uint32_t ramp_start=app->route.motion==MOVE_TASK2 && app->route.leg==2U && app->route.task2_straight.seen
                                    ? app->route.task2_straight.entry_ms : app->route.start_ms;
                                float ramp = start + accel * ((float)(uint32_t)(now - ramp_start) / 1000.0f);
                                if (base > ramp) base = ramp;
                            }
                            app->route.base = base;
                            float max_diff = DPT_HEADING_MAX_DIFF * base / app->route.cruise;
                            if (app->heading.diff > max_diff) app->heading.diff = max_diff;
                            if (app->heading.diff < -max_diff) app->heading.diff = -max_diff;
                        }
                        if (CarRoute_UsesLine(app)) {
                            /* 弧段由任务四同款弧内逻辑统管：低速接管、桥接、恢复与提速。 */
                            if (!task2_arc_line(app, now, &base, previous_targets)) {
                                if (app->route.motion == MOVE_TASK2 && app->route.edge_seen && !app->line.valid) {
                                    /* 弧末出口全白去抖：沿用最后一次有效差速继续前进，不停车。 */
                                    app->route.targets[LEFT] = base + app->line.diff;
                                    app->route.targets[RIGHT] = base - app->line.diff;
                                } else CarLine_SetTargets(app, now, base);
                            }
                        }
                        else {
                            app->route.targets[LEFT] = base - app->heading.diff;
                            app->route.targets[RIGHT] = base + app->heading.diff;
                        }
                        /* 任务一终点减速时按新目标缩减旧积分，避免高速段积分拖慢降速；
                           不清除反转/堵转计时，不改其他任务的积分处理。 */
                        if (app->route.motion == MOVE_ROUTE && app->route.base < previous_base - 0.001f)
                            CarWheel_Retarget(app, previous_targets);
                        /* 弧入口的降速重定向已由 arc_line 内部完成，避免积分被缩放两次。 */
                        if (app->route.motion == MOVE_TASK2 && !app->route.edge_follow &&
                            (((app->route.cruise != previous_cruise || !CarRoute_IsArc(app)) && app->route.base < previous_base - 0.5f) ||
                             (app->route.leg==2U && app->route.task2_straight.seen && app->route.task2_straight.entry_ms==now)))
                            CarWheel_Retarget(app, previous_targets);
                    }
                }
}
