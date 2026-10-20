/* 运行环形日志：控制周期只写定长记录，停车后再分帧导出 CSV。 */
#include "car_run_log.h"
#include "car_math.h"
#include "lab_uart_async.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

_Static_assert(sizeof(CarRunLogRecord) == 24U, "run log record size changed");

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

static void append(const CarAppContext *app, uint32_t now, TestState state, uint8_t extra_flags)
{
    CarRunLogRecord *r = &g_car_run_log.records[g_car_run_log.head];
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
    if (fabsf(app->wheel.items[LEFT].velocity) < 1.0f && fabsf(app->wheel.items[RIGHT].velocity) < 1.0f)
        r->flags |= CAR_RUN_LOG_FLAG_ZERO_SPEED;
    r->sequence = g_car_run_log.sequence++;
    g_car_run_log.head = (uint16_t)((g_car_run_log.head + 1U) % CAR_RUN_LOG_CAPACITY);
    if (g_car_run_log.count < CAR_RUN_LOG_CAPACITY) ++g_car_run_log.count;
}

void CarRunLog_Init(uint32_t now)
{
    memset(&g_car_run_log, 0, sizeof g_car_run_log);
    g_car_run_log.sample_ms = now;
}

void CarRunLog_Clear(uint32_t now)
{
    CarRunLog_Init(now);
}

void CarRunLog_BeginRun(const CarAppContext *app, uint32_t now)
{
    if (g_car_run_log.frozen) return; /* 故障现场在显式导出/清理前不覆盖。 */
    CarRunLog_Clear(now);
    append(app, now, RUNNING, 0U);
}

void CarRunLog_OnStop(const CarAppContext *app, TestState why, uint32_t now)
{
    if (g_car_run_log.frozen || app->route.motion == MOVE_NONE) return;
    append(app, now, why, why == DONE || why == BRAKING ? 0U : CAR_RUN_LOG_FLAG_FAULT);
    if (why != DONE && why != BRAKING) {
        g_car_run_log.frozen = 1U;
        g_car_run_log.reason = (uint8_t)why;
        g_car_run_log.dump_stage = g_car_run_log.exported = 0U;
    }
}

void CarRunLog_Poll(const CarAppContext *app, uint32_t now)
{
    if (g_car_run_log.frozen || app->runtime.state != RUNNING || app->route.motion == MOVE_NONE) return;

    int speed_zero = fabsf(app->wheel.items[LEFT].velocity) < 1.0f &&
        fabsf(app->wheel.items[RIGHT].velocity) < 1.0f;
    CarTask3Phase phase = app->route.task3.phase;
    int planned_halt = app->route.switching ||
        (CarMotion_IsFigureEight(app->route.motion) &&
         (phase == T3_VERTEX_BRAKE || phase == T3_FINAL_BRAKE ||
          phase == T3_START_SETTLE || phase == T3_ALIGN_SETTLE));
    /* 按阶段识别计划内停车，不依赖目标速度。这样既不会把顶点刹停
       误报为卡死，也能捕获异常逻辑同时把两轮目标清零的情况。 */
    if (!planned_halt && speed_zero) {
        if (!g_car_run_log.zero_seen) {
            g_car_run_log.zero_seen = 1U;
            g_car_run_log.zero_ms = now;
        } else if ((uint32_t)(now - g_car_run_log.zero_ms) >= CAR_RUN_LOG_ZERO_FREEZE_MS) {
            append(app, now, app->runtime.state, CAR_RUN_LOG_FLAG_FAULT);
            g_car_run_log.frozen = 1U;
            g_car_run_log.reason = 0x80U; /* 任务未完成且有目标时两轮持续零速。 */
            g_car_run_log.dump_stage = g_car_run_log.exported = 0U;
            return;
        }
    } else g_car_run_log.zero_seen = 0U;

    if ((uint32_t)(now - g_car_run_log.sample_ms) >= CAR_RUN_LOG_SAMPLE_MS) {
        g_car_run_log.sample_ms = now;
        append(app, now, app->runtime.state, 0U);
    }
}

int CarRunLog_IsFrozen(void) { return g_car_run_log.frozen != 0U; }
int CarRunLog_IsExported(void) { return g_car_run_log.exported != 0U; }

void CarRunLog_RequestDump(void)
{
    if (g_car_run_log.count == 0U) return;
    g_car_run_log.dump_index = (uint16_t)((g_car_run_log.head + CAR_RUN_LOG_CAPACITY - g_car_run_log.count) % CAR_RUN_LOG_CAPACITY);
    g_car_run_log.dump_remaining = g_car_run_log.count;
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
    if (g_car_run_log.dump_stage == 0U || app->runtime.state == RUNNING || app->runtime.state == BRAKING) return;
    char line[160];
    int n;
    if (g_car_run_log.dump_stage == 1U) {
        n = snprintf(line, sizeof line, "RLOG BEGIN N=%u F=%u WHY=%u REC=%uB RAM=%uB\r\n",
            (unsigned)g_car_run_log.count, (unsigned)g_car_run_log.frozen,
            (unsigned)g_car_run_log.reason, (unsigned)sizeof(CarRunLogRecord),
            (unsigned)sizeof(CarRunLogStore));
        if (send_text(line, n)) g_car_run_log.dump_stage = 2U;
    } else if (g_car_run_log.dump_stage == 2U) {
        static const char header[] = "RLOG,SEQ,MS,ST,M,PH,LEG,ARC,RAW,E2,TL,TR,VL,VR,Y10,FLAGS\r\n";
        if (LabUART_TryWrite(header, sizeof header - 1U)) g_car_run_log.dump_stage = 3U;
    } else if (g_car_run_log.dump_stage == 3U && g_car_run_log.dump_remaining != 0U) {
        const CarRunLogRecord *r = &g_car_run_log.records[g_car_run_log.dump_index];
        n = snprintf(line, sizeof line,
            "RLOG,%u,%lu,%u,%u,%u,%u,%u,%02X,%d,%d,%d,%d,%d,%d,%02X\r\n",
            (unsigned)r->sequence, (unsigned long)r->ms, (unsigned)r->state,
            (unsigned)r->motion, (unsigned)r->phase, (unsigned)r->leg,
            (unsigned)r->arc, (unsigned)r->raw_bits, (int)r->line_error_x2,
            (int)r->target[LEFT], (int)r->target[RIGHT], (int)r->speed[LEFT],
            (int)r->speed[RIGHT], (int)r->yaw_x10, (unsigned)r->flags);
        if (send_text(line, n)) {
            g_car_run_log.dump_index = (uint16_t)((g_car_run_log.dump_index + 1U) % CAR_RUN_LOG_CAPACITY);
            --g_car_run_log.dump_remaining;
            if (g_car_run_log.dump_remaining == 0U) g_car_run_log.dump_stage = 4U;
        }
    } else if (g_car_run_log.dump_stage == 4U) {
        static const char end[] = "RLOG END\r\n";
        if (LabUART_TryWrite(end, sizeof end - 1U)) {
            g_car_run_log.dump_stage = 0U;
            g_car_run_log.exported = 1U;
        }
    }
}
