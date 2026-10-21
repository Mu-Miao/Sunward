/* 双层运行日志：20Hz故障窗口 + 5Hz全程轨迹 + 稀疏关键事件。 */
#include "car_run_log.h"
#include "car_math.h"
#include "lab_uart_async.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

_Static_assert(sizeof(CarRunLogRecord) == 24U, "run log record size changed");
_Static_assert(sizeof(CarRunLogJourneyRecord) == 16U, "journey record size changed");
_Static_assert(sizeof(CarRunLogEventRecord) == 32U, "event record size changed");

CarRunLogStore g_car_run_log;

static int16_t quantize(float value, float scale)
{
    if (!isfinite(value)) return 0;
    value *= scale;
    if (value > 32767.0f) value = 32767.0f;
    if (value < -32768.0f) value = -32768.0f;
    return (int16_t)CarMath_Round(value);
}

static uint8_t raw_bits(const CarAppContext *app)
{
    uint8_t bits = 0U;
    for (unsigned i = 0; i < 4U; ++i)
        if (app->line.raw[i]) bits |= (uint8_t)(1U << i);
    return bits;
}

static uint8_t confidence4(float value)
{
    if (!isfinite(value)) return 0U;
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;
    return (uint8_t)CarMath_Round(value * 15.0f);
}

static uint8_t packed_confidence(const CarAppContext *app)
{
    if (!CarMotion_IsFigureEight(app->route.motion)) return 0xffU;
    return (uint8_t)(confidence4(app->route.task3.line_confidence) |
        (uint8_t)(confidence4(app->route.task3.heading_confidence) << 4));
}

static float segment_progress(const CarAppContext *app)
{
    float average = 0.5f * (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]);
    if (CarMotion_IsFigureEight(app->route.motion))
        return average - app->route.task3.leg_start_mm;
    return average - app->route.leg_start;
}

static void fill_record(CarRunLogRecord *r, const CarAppContext *app, uint32_t now,
                        TestState state, uint8_t extra_flags)
{
    memset(r, 0, sizeof *r);
    r->ms = now;
    for (unsigned i = 0; i < 2U; ++i) {
        r->target[i] = quantize(app->route.targets[i], 1.0f);
        r->speed[i] = quantize(app->wheel.items[i].velocity, 1.0f);
    }
    r->yaw_x10 = quantize(app->heading.angle, 10.0f);
    int line = CarMath_Round(app->line.error * 2.0f);
    if (line > 127) line = 127;
    if (line < -128) line = -128;
    r->line_error_x2 = (int8_t)line;
    r->raw_bits = raw_bits(app);
    r->state = (uint8_t)state;
    r->motion = (uint8_t)app->route.motion;
    r->phase = CarMotion_IsFigureEight(app->route.motion) ? (uint8_t)app->route.task3.phase : 0xffU;
    r->leg = app->route.leg;
    r->arc = app->route.motion == MOVE_TASK4 ? app->route.task4.arc_number :
        (app->route.motion == MOVE_TASK3 && app->route.task3.phase == T3_ARC ?
         (app->route.task3.leg == 1U ? 1U : 2U) : 0U);
    r->flags = extra_flags;
    if (app->line.valid) r->flags |= CAR_RUN_LOG_FLAG_LINE_VALID;
    if (CarMotion_IsFigureEight(app->route.motion) && app->route.task3.lost_seen)
        r->flags |= CAR_RUN_LOG_FLAG_LINE_LOST;
    if (CarMotion_IsFigureEight(app->route.motion) && app->route.task3.loss_seek)
        r->flags |= CAR_RUN_LOG_FLAG_SEEK;
    if (fabsf(app->route.targets[LEFT]) < 1.0f && fabsf(app->route.targets[RIGHT]) < 1.0f)
        r->flags |= CAR_RUN_LOG_FLAG_ZERO_TARGET;
    if (fabsf(app->wheel.items[LEFT].velocity) < 1.0f &&
        fabsf(app->wheel.items[RIGHT].velocity) < 1.0f)
        r->flags |= CAR_RUN_LOG_FLAG_ZERO_SPEED;
}

static void append_high(const CarAppContext *app, uint32_t now, TestState state,
                        uint8_t extra_flags)
{
    CarRunLogRecord *r = &g_car_run_log.records[g_car_run_log.head];
    fill_record(r, app, now, state, extra_flags);
    r->sequence = g_car_run_log.sequence++;
    g_car_run_log.head = (uint16_t)((g_car_run_log.head + 1U) % CAR_RUN_LOG_CAPACITY);
    if (g_car_run_log.count < CAR_RUN_LOG_CAPACITY) ++g_car_run_log.count;
}

static void append_journey(const CarAppContext *app, uint32_t now)
{
    CarRunLogJourneyRecord *r = &g_car_run_log.journey[g_car_run_log.journey_head];
    memset(r, 0, sizeof *r);
    r->ms = now;
    r->progress_mm = quantize(segment_progress(app), 1.0f);
    r->yaw_x10 = quantize(app->heading.angle, 10.0f);
    r->speed_left = quantize(app->wheel.items[LEFT].velocity, 1.0f);
    r->speed_right = quantize(app->wheel.items[RIGHT].velocity, 1.0f);
    int error = CarMath_Round(app->line.error * 2.0f);
    if (error > 127) error = 127;
    if (error < -128) error = -128;
    r->line_error_x2 = (int8_t)error;
    r->phase = CarMotion_IsFigureEight(app->route.motion) ?
        (uint8_t)app->route.task3.phase : 0xffU;
    r->meta = (uint8_t)(app->route.leg & 0x03U);
    if (CarMotion_IsFigureEight(app->route.motion)) {
        r->meta |= (uint8_t)((app->route.task3.checkpoint_index & 0x07U) << 2);
        if (app->route.task3.lost_seen) r->meta |= 0x40U;
        if (app->route.task3.loss_seek) r->meta |= 0x80U;
    }
    if (app->line.valid) r->meta |= 0x20U;
    r->confidence = packed_confidence(app);
    g_car_run_log.journey_head = (uint16_t)((g_car_run_log.journey_head + 1U) %
        CAR_RUN_LOG_JOURNEY_CAPACITY);
    if (g_car_run_log.journey_count < CAR_RUN_LOG_JOURNEY_CAPACITY)
        ++g_car_run_log.journey_count;
}

static void append_event(const CarAppContext *app, uint32_t now, TestState state,
                         CarRunLogEventType event)
{
    CarRunLogEventRecord *r = &g_car_run_log.events[g_car_run_log.event_head];
    memset(r, 0, sizeof *r);
    fill_record(&r->record, app, now, state, 0U);
    r->record.sequence = g_car_run_log.sequence;
    r->event = (uint8_t)event;
    r->confidence = packed_confidence(app);
    if (CarMotion_IsFigureEight(app->route.motion)) {
        r->checkpoint = app->route.task3.checkpoint_index;
        r->expected_yaw_x10 = quantize(app->route.task3.expected_yaw_deg, 10.0f);
        r->encoder_turn_x10 = quantize(app->route.task3.encoder_turn_deg, 10.0f);
        int heading = CarMath_Round(app->route.task3.heading_error_deg * 2.0f);
        if (heading > 127) heading = 127;
        if (heading < -128) heading = -128;
        r->heading_error_x2 = (int8_t)heading;
    }
    g_car_run_log.event_head = (uint8_t)((g_car_run_log.event_head + 1U) %
        CAR_RUN_LOG_EVENT_CAPACITY);
    if (g_car_run_log.event_count < CAR_RUN_LOG_EVENT_CAPACITY) ++g_car_run_log.event_count;
}

static void remember_state(const CarAppContext *app)
{
    if (CarMotion_IsFigureEight(app->route.motion)) {
        g_car_run_log.last_phase = (uint8_t)app->route.task3.phase;
        g_car_run_log.last_checkpoint = app->route.task3.checkpoint_index;
        g_car_run_log.last_lost = app->route.task3.lost_seen;
        g_car_run_log.last_seek = app->route.task3.loss_seek;
    } else {
        g_car_run_log.last_phase = 0xffU;
        g_car_run_log.last_checkpoint = 0U;
        g_car_run_log.last_lost = g_car_run_log.last_seek = 0U;
    }
    g_car_run_log.last_leg = app->route.leg;
}

static void capture_events(const CarAppContext *app, uint32_t now)
{
    if (!CarMotion_IsFigureEight(app->route.motion)) {
        if (app->route.leg != g_car_run_log.last_leg)
            append_event(app, now, app->runtime.state, CAR_RUN_EVENT_PHASE);
        remember_state(app);
        return;
    }
    const CarTask3 *s = &app->route.task3;
    if ((uint8_t)s->phase != g_car_run_log.last_phase || app->route.leg != g_car_run_log.last_leg)
        append_event(app, now, app->runtime.state, CAR_RUN_EVENT_PHASE);
    if (s->checkpoint_event || s->checkpoint_index != g_car_run_log.last_checkpoint)
        append_event(app, now, app->runtime.state, CAR_RUN_EVENT_CHECKPOINT);
    if ((s->lost_seen && !g_car_run_log.last_lost) || (s->loss_seek && !g_car_run_log.last_seek))
        append_event(app, now, app->runtime.state, CAR_RUN_EVENT_LOSS);
    if (!s->lost_seen && g_car_run_log.last_lost && s->phase == T3_ARC)
        append_event(app, now, app->runtime.state, CAR_RUN_EVENT_REACQUIRE);
    remember_state(app);
}

void CarRunLog_Init(uint32_t now)
{
    memset(&g_car_run_log, 0, sizeof g_car_run_log);
    g_car_run_log.sample_ms = g_car_run_log.journey_sample_ms = now;
    g_car_run_log.last_phase = 0xffU;
}

void CarRunLog_Clear(uint32_t now) { CarRunLog_Init(now); }

void CarRunLog_BeginRun(const CarAppContext *app, uint32_t now)
{
    if (g_car_run_log.frozen) return;
    CarRunLog_Clear(now);
    append_high(app, now, RUNNING, 0U);
    append_journey(app, now);
    append_event(app, now, RUNNING, CAR_RUN_EVENT_BEGIN);
    remember_state(app);
}

void CarRunLog_OnStop(const CarAppContext *app, TestState why, uint32_t now)
{
    if (g_car_run_log.frozen || app->route.motion == MOVE_NONE) return;
    append_high(app, now, why, why == DONE || why == BRAKING ? 0U : CAR_RUN_LOG_FLAG_FAULT);
    append_journey(app, now);
    append_event(app, now, why, CAR_RUN_EVENT_STOP);
    if (why != DONE && why != BRAKING) {
        g_car_run_log.frozen = 1U;
        g_car_run_log.reason = (uint8_t)why;
        g_car_run_log.dump_stage = g_car_run_log.exported = 0U;
    }
}

void CarRunLog_Poll(const CarAppContext *app, uint32_t now)
{
    if (g_car_run_log.frozen || app->runtime.state != RUNNING || app->route.motion == MOVE_NONE) return;
    capture_events(app, now);

    int speed_zero = fabsf(app->wheel.items[LEFT].velocity) < 1.0f &&
        fabsf(app->wheel.items[RIGHT].velocity) < 1.0f;
    CarTask3Phase phase = app->route.task3.phase;
    int planned_halt = app->route.switching ||
        (CarMotion_IsFigureEight(app->route.motion) &&
         (phase == T3_VERTEX_BRAKE || phase == T3_FINAL_BRAKE ||
          phase == T3_START_SETTLE || phase == T3_ALIGN_SETTLE));
    if (!planned_halt && speed_zero) {
        if (!g_car_run_log.zero_seen) {
            g_car_run_log.zero_seen = 1U;
            g_car_run_log.zero_ms = now;
        } else if ((uint32_t)(now - g_car_run_log.zero_ms) >= CAR_RUN_LOG_ZERO_FREEZE_MS) {
            append_high(app, now, app->runtime.state, CAR_RUN_LOG_FLAG_FAULT);
            append_journey(app, now);
            append_event(app, now, app->runtime.state, CAR_RUN_EVENT_STOP);
            g_car_run_log.frozen = 1U;
            g_car_run_log.reason = 0x80U;
            g_car_run_log.dump_stage = g_car_run_log.exported = 0U;
            return;
        }
    } else g_car_run_log.zero_seen = 0U;

    if ((uint32_t)(now - g_car_run_log.sample_ms) >= CAR_RUN_LOG_SAMPLE_MS) {
        g_car_run_log.sample_ms = now;
        append_high(app, now, app->runtime.state, 0U);
    }
    if ((uint32_t)(now - g_car_run_log.journey_sample_ms) >= CAR_RUN_LOG_JOURNEY_SAMPLE_MS) {
        g_car_run_log.journey_sample_ms = now;
        append_journey(app, now);
    }
}

int CarRunLog_IsFrozen(void) { return g_car_run_log.frozen != 0U; }
int CarRunLog_IsExported(void) { return g_car_run_log.exported != 0U; }
int CarRunLog_IsDumping(void) { return g_car_run_log.dump_stage != 0U; }

void CarRunLog_RequestDump(void)
{
    if (g_car_run_log.count == 0U && g_car_run_log.journey_count == 0U &&
        g_car_run_log.event_count == 0U) return;
    g_car_run_log.dump_index = (uint16_t)((g_car_run_log.head + CAR_RUN_LOG_CAPACITY -
        g_car_run_log.count) % CAR_RUN_LOG_CAPACITY);
    g_car_run_log.dump_remaining = g_car_run_log.count;
    g_car_run_log.journey_dump_index = (uint16_t)((g_car_run_log.journey_head +
        CAR_RUN_LOG_JOURNEY_CAPACITY - g_car_run_log.journey_count) % CAR_RUN_LOG_JOURNEY_CAPACITY);
    g_car_run_log.journey_dump_remaining = g_car_run_log.journey_count;
    g_car_run_log.event_dump_index = (uint8_t)((g_car_run_log.event_head +
        CAR_RUN_LOG_EVENT_CAPACITY - g_car_run_log.event_count) % CAR_RUN_LOG_EVENT_CAPACITY);
    g_car_run_log.event_dump_remaining = g_car_run_log.event_count;
    g_car_run_log.dump_stage = 1U;
    g_car_run_log.exported = 0U;
}

static int send_text(const char *text, int length)
{
    return length > 0 && length <= (int)LAB_UART_FRAME_MAX &&
        LabUART_TryWrite(text, (uint16_t)length);
}

void CarRunLog_Service(const CarAppContext *app)
{
    if (g_car_run_log.dump_stage == 0U || app->runtime.state == RUNNING ||
        app->runtime.state == BRAKING) return;
    char line[160];
    int n;
    switch (g_car_run_log.dump_stage) {
    case 1U:
        n = snprintf(line, sizeof line, "RLOG BEGIN J=%u E=%u H=%u F=%u WHY=%u RAM=%uB\r\n",
            (unsigned)g_car_run_log.journey_count, (unsigned)g_car_run_log.event_count,
            (unsigned)g_car_run_log.count, (unsigned)g_car_run_log.frozen,
            (unsigned)g_car_run_log.reason, (unsigned)sizeof(CarRunLogStore));
        if (send_text(line, n)) g_car_run_log.dump_stage = 2U;
        break;
    case 2U: {
        static const char header[] = "JLOG,MS,PROG,Y10,VL,VR,E2,PH,META,CONF\r\n";
        if (LabUART_TryWrite(header, sizeof header - 1U)) g_car_run_log.dump_stage = 3U;
        break;
    }
    case 3U:
        if (g_car_run_log.journey_dump_remaining == 0U) { g_car_run_log.dump_stage = 4U; break; }
        {
            const CarRunLogJourneyRecord *r = &g_car_run_log.journey[g_car_run_log.journey_dump_index];
            n = snprintf(line, sizeof line, "JLOG,%lu,%d,%d,%d,%d,%d,%u,%02X,%02X\r\n",
                (unsigned long)r->ms, (int)r->progress_mm, (int)r->yaw_x10,
                (int)r->speed_left, (int)r->speed_right, (int)r->line_error_x2,
                (unsigned)r->phase, (unsigned)r->meta, (unsigned)r->confidence);
            if (send_text(line, n)) {
                g_car_run_log.journey_dump_index = (uint16_t)((g_car_run_log.journey_dump_index + 1U) %
                    CAR_RUN_LOG_JOURNEY_CAPACITY);
                --g_car_run_log.journey_dump_remaining;
            }
        }
        break;
    case 4U: {
        static const char header[] = "ELOG,EV,CP,MS,PH,LEG,RAW,E2,Y10,EY10,ET10,HE2,CONF,TL,TR,VL,VR\r\n";
        if (LabUART_TryWrite(header, sizeof header - 1U)) g_car_run_log.dump_stage = 5U;
        break;
    }
    case 5U:
        if (g_car_run_log.event_dump_remaining == 0U) { g_car_run_log.dump_stage = 6U; break; }
        {
            const CarRunLogEventRecord *e = &g_car_run_log.events[g_car_run_log.event_dump_index];
            const CarRunLogRecord *r = &e->record;
            n = snprintf(line, sizeof line,
                "ELOG,%u,%u,%lu,%u,%u,%02X,%d,%d,%d,%d,%d,%02X,%d,%d,%d,%d\r\n",
                (unsigned)e->event, (unsigned)e->checkpoint, (unsigned long)r->ms,
                (unsigned)r->phase, (unsigned)r->leg, (unsigned)r->raw_bits,
                (int)r->line_error_x2, (int)r->yaw_x10, (int)e->expected_yaw_x10,
                (int)e->encoder_turn_x10, (int)e->heading_error_x2, (unsigned)e->confidence,
                (int)r->target[LEFT], (int)r->target[RIGHT], (int)r->speed[LEFT], (int)r->speed[RIGHT]);
            if (send_text(line, n)) {
                g_car_run_log.event_dump_index = (uint8_t)((g_car_run_log.event_dump_index + 1U) %
                    CAR_RUN_LOG_EVENT_CAPACITY);
                --g_car_run_log.event_dump_remaining;
            }
        }
        break;
    case 6U: {
        static const char header[] = "RLOG,SEQ,MS,ST,M,PH,LEG,ARC,RAW,E2,TL,TR,VL,VR,Y10,FLAGS\r\n";
        if (LabUART_TryWrite(header, sizeof header - 1U)) g_car_run_log.dump_stage = 7U;
        break;
    }
    case 7U:
        if (g_car_run_log.dump_remaining == 0U) { g_car_run_log.dump_stage = 8U; break; }
        {
            const CarRunLogRecord *r = &g_car_run_log.records[g_car_run_log.dump_index];
            n = snprintf(line, sizeof line,
                "RLOG,%u,%lu,%u,%u,%u,%u,%u,%02X,%d,%d,%d,%d,%d,%d,%02X\r\n",
                (unsigned)r->sequence, (unsigned long)r->ms, (unsigned)r->state,
                (unsigned)r->motion, (unsigned)r->phase, (unsigned)r->leg,
                (unsigned)r->arc, (unsigned)r->raw_bits, (int)r->line_error_x2,
                (int)r->target[LEFT], (int)r->target[RIGHT], (int)r->speed[LEFT],
                (int)r->speed[RIGHT], (int)r->yaw_x10, (unsigned)r->flags);
            if (send_text(line, n)) {
                g_car_run_log.dump_index = (uint16_t)((g_car_run_log.dump_index + 1U) %
                    CAR_RUN_LOG_CAPACITY);
                --g_car_run_log.dump_remaining;
            }
        }
        break;
    case 8U: {
        static const char end[] = "RLOG END\r\n";
        if (LabUART_TryWrite(end, sizeof end - 1U)) {
            g_car_run_log.dump_stage = 0U;
            g_car_run_log.exported = 1U;
        }
        break;
    }
    default:
        g_car_run_log.dump_stage = 0U;
        break;
    }
}
