#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "car_run_log.h"

static char output[40000];
static size_t output_length;
static int reject_once;

int LabUART_TryWrite(const char *data, uint16_t length)
{
    if (reject_once) { reject_once = 0; return 0; }
    assert(output_length + length < sizeof output);
    memcpy(output + output_length, data, length);
    output_length += length;
    output[output_length] = '\0';
    return 1;
}

static CarAppContext context(void)
{
    CarAppContext app;
    memset(&app, 0, sizeof app);
    app.runtime.state = RUNNING;
    app.route.motion = MOVE_TASK3;
    app.route.tracking = 1U;
    app.route.targets[LEFT] = 120.0f;
    app.route.targets[RIGHT] = 80.0f;
    app.wheel.items[LEFT].velocity = 115.0f;
    app.wheel.items[RIGHT].velocity = 77.0f;
    app.line.valid = 1U;
    app.line.error = 1.0f;
    app.line.raw[0] = app.line.raw[3] = 1U;
    app.heading.angle = -12.3f;
    app.route.task3.phase = T3_ARC;
    app.route.task3.leg = app.route.leg = 1U;
    return app;
}

int main(void)
{
    assert(sizeof(CarRunLogRecord) == 24U);
    assert(sizeof(CarRunLogStore) < 6200U);
    CarAppContext app = context();
    CarRunLog_Init(0U);
    CarRunLog_BeginRun(&app, 0U);
    assert(g_car_run_log.count == 1U && !g_car_run_log.frozen);

    /* 20 Hz logging stays O(1) and periodically overwrites normal old records. */
    for (uint32_t ms = 10U; ms <= 15000U; ms += 10U) {
        app.wheel.items[LEFT].velocity = 110.0f + (float)(ms % 7U);
        CarRunLog_Poll(&app, ms);
    }
    assert(g_car_run_log.count == CAR_RUN_LOG_CAPACITY);
    uint16_t before = g_car_run_log.sequence;
    CarRunLog_OnStop(&app, LINE_LOST, 15010U);
    assert(g_car_run_log.frozen && g_car_run_log.reason == LINE_LOST);
    assert(g_car_run_log.sequence == (uint16_t)(before + 1U));
    uint16_t retained = g_car_run_log.sequence;
    CarRunLog_BeginRun(&app, 16000U);
    assert(g_car_run_log.frozen && g_car_run_log.sequence == retained);

    /* CSV dump retries a full frame when the non-blocking UART queue is full. */
    app.runtime.state = LINE_LOST;
    reject_once = 1;
    CarRunLog_RequestDump();
    CarRunLog_Service(&app);
    assert(output_length == 0U && g_car_run_log.dump_stage == 1U);
    for (unsigned i = 0; i < 300U && !CarRunLog_IsExported(); ++i) CarRunLog_Service(&app);
    assert(CarRunLog_IsExported());
    assert(strstr(output, "RLOG BEGIN N=256 F=1 WHY=16 REC=24B") != NULL);
    assert(strstr(output, "RLOG,SEQ,MS,ST,M,PH,LEG,ARC,RAW,E2,TL,TR,VL,VR,Y10,FLAGS") != NULL);
    assert(strstr(output, "RLOG END") != NULL);

    /* 运行阶段双轮持续零速时自动冻结，即使异常逻辑也清零了目标。 */
    CarRunLog_Clear(20000U);
    app = context();
    app.wheel.items[LEFT].velocity = app.wheel.items[RIGHT].velocity = 0.0f;
    CarRunLog_BeginRun(&app, 20000U);
    for (uint32_t ms = 20010U; ms <= 20310U; ms += 10U) CarRunLog_Poll(&app, ms);
    assert(g_car_run_log.frozen && g_car_run_log.reason == 0x80U);

    CarRunLog_Clear(21000U);
    app.route.targets[LEFT] = app.route.targets[RIGHT] = 0.0f;
    app.route.task3.phase = T3_VERTEX_BRAKE;
    CarRunLog_BeginRun(&app, 21000U);
    for (uint32_t ms = 21010U; ms <= 22000U; ms += 10U) CarRunLog_Poll(&app, ms);
    assert(!g_car_run_log.frozen);

    CarRunLog_Clear(23000U);
    app.route.task3.phase = T3_ARC;
    CarRunLog_BeginRun(&app, 23000U);
    for (uint32_t ms = 23010U; ms <= 23310U; ms += 10U) CarRunLog_Poll(&app, ms);
    assert(g_car_run_log.frozen && g_car_run_log.reason == 0x80U);
    puts("PASS: 24-byte/20Hz circular run log, normal overwrite, fault retention, zero-speed freeze, intentional halt exclusion and retryable CSV dump.");
    return 0;
}
