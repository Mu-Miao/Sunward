#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "car_run_log.h"

static char output[40000];
static size_t output_length;
static int reject_once;
static LabUART_Port output_port;

int LabUART_TryWritePort(LabUART_Port port, const char *data, uint16_t length)
{
    if (reject_once) { reject_once = 0; return 0; }
    output_port = port;
    assert(output_length + length < sizeof output);
    memcpy(output + output_length, data, length);
    output_length += length;
    output[output_length] = '\0';
    return 1;
}

int LabUART_TryWrite(const char *data, uint16_t length)
{
    return LabUART_TryWritePort(LAB_UART_USB, data, length);
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
    assert(sizeof(CarRunLogJourneyRecord) == 16U);
    assert(sizeof(CarRunLogEventRecord) == 32U);
    assert(sizeof(CarRunLogStore) < 22000U);
    CarAppContext app = context();
    /* 任务四160秒上限可完整保存起点、5Hz全程样本和终点，不覆盖开头。 */
    CarRunLog_Init(0U);
    CarRunLog_BeginRun(&app, 0U);
    for (uint32_t ms = 10U; ms <= 160000U; ms += 10U) CarRunLog_Poll(&app, ms);
    CarRunLog_OnStop(&app, DONE, 160010U);
    assert(g_car_run_log.journey_count == 802U);
    assert(g_car_run_log.journey_count < CAR_RUN_LOG_JOURNEY_CAPACITY);
    assert(g_car_run_log.journey[0].ms == 0U);

    CarRunLog_Init(0U);
    CarRunLog_BeginRun(&app, 0U);
    assert(g_car_run_log.count == 1U && !g_car_run_log.frozen);

    /* 20 Hz logging stays O(1) and periodically overwrites normal old records. */
    for (uint32_t ms = 10U; ms <= 15000U; ms += 10U) {
        app.wheel.items[LEFT].velocity = 110.0f + (float)(ms % 7U);
        CarRunLog_Poll(&app, ms);
    }
    assert(g_car_run_log.count == CAR_RUN_LOG_CAPACITY);
    assert(g_car_run_log.journey_count >= 75U);
    assert(g_car_run_log.event_count == 1U);
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
    CarRunLog_RequestDumpPort(LAB_UART_BT);
    assert(CarRunLog_IsDumping());
    CarRunLog_Service(&app);
    assert(output_length == 0U && g_car_run_log.dump_stage == 1U);
    for (unsigned i = 0; i < 500U && !CarRunLog_IsExported(); ++i) CarRunLog_Service(&app);
    assert(CarRunLog_IsExported());
    assert(!CarRunLog_IsDumping());
    assert(output_port == LAB_UART_BT);
    assert(strstr(output, "RLOG BEGIN VER=ARC21 TC=1023-1022 GIT=LOCAL CFG=00000000 MODE=LOCAL J=") != NULL);
    assert(strstr(output, "H=160 F=1 WHY=16") != NULL);
    assert(strstr(output, "JLOG,MS,PROG,Y10,VL,VR,E2,PH,META,CONF") != NULL);
    assert(strstr(output, "ELOG,EV,CP,MS,PH,LEG,RAW,E2,Y10,EY10,ET10,HE2,CONF,TL,TR,VL,VR") != NULL);
    assert(strstr(output, "RLOG,SEQ,MS,ST,M,PH,LEG,ARC,RAW,E2,TL,TR,VL,VR,Y10,FLAGS") != NULL);
    assert(strstr(output, "RLOG END") != NULL);

    output_length = 0U;
    CarRunLog_SendStatusPort(LAB_UART_BT);
    assert(output_port == LAB_UART_BT);
    assert(strstr(output, "LSTAT VER=ARC21 TC=1023-1022") != NULL);
    assert(strstr(output, "D=0 P=1 JR=0 ER=0 HR=0") != NULL);

    /* A normally completed run remains exportable until the next run begins. */
    output_length = 0U;
    CarRunLog_Clear(19000U);
    app = context();
    CarRunLog_BeginRun(&app, 19000U);
    CarRunLog_OnStop(&app, DONE, 19050U);
    assert(!g_car_run_log.frozen && g_car_run_log.count == 2U);
    app.runtime.state = DONE;
    CarRunLog_RequestDump();
    for (unsigned i = 0; i < 20U && !CarRunLog_IsExported(); ++i) CarRunLog_Service(&app);
    assert(CarRunLog_IsExported() && strstr(output, "H=2 F=0 WHY=0") != NULL);

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

    /* 阶段、检查点、丢线和连续重获分别留下稀疏事件。 */
    CarRunLog_Clear(24000U);
    app = context();
    CarRunLog_BeginRun(&app, 24000U);
    app.route.task3.checkpoint_index = app.route.task3.checkpoint_event = 1U;
    CarRunLog_Poll(&app, 24010U);
    app.route.task3.checkpoint_event = 0U;
    app.route.task3.lost_seen = 1U;
    CarRunLog_Poll(&app, 24020U);
    app.route.task3.lost_seen = 0U;
    CarRunLog_Poll(&app, 24030U);
    app.route.task3.phase = T3_VERTEX_BRAKE;
    CarRunLog_Poll(&app, 24040U);
    assert(g_car_run_log.event_count == 5U);
    puts("PASS: 20Hz fault window, 5Hz full-run journey, sparse events, fault retention, zero-speed freeze and retryable CSV dump.");
    return 0;
}
