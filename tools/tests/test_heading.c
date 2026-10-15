/* 航向融合主机测试：纯函数场景 + CarHeading_Sample 集成。不访问硬件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "lab_uart_async.h"
#include "mpu6050.h"

DWT_Type mock_dwt;
CoreDebug_Type mock_core;
static uint32_t ms;
static int16_t gz = -424; /* 静止零偏，与实车静止读数同量级。 */

uint32_t HAL_GetTick(void) { return ms; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return 72000000U; }
void HAL_Delay(uint32_t n) { ms += n; DWT->CYCCNT += n * 72000U; }
int HAL_GPIO_ReadPin(void *p, uint32_t pin) { (void)p; (void)pin; return 0; }
void HAL_GPIO_WritePin(void *p, uint32_t pin, int value) { (void)p; (void)pin; (void)value; }
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *p) { (void)p; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *p, uint32_t n) { (void)p; (void)n; return HAL_OK; }
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *p) { (void)p; return 1320U; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *p) { (void)p; return HAL_OK; }
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw *p)
{
    DWT->CYCCNT += 500U * 72U; /* 模拟 500 微秒软 I2C 读取。 */
    *p = (MPU6050_Raw){.gx = 0, .gy = 0, .gz = gz};
    return HAL_OK;
}
HAL_StatusTypeDef LabUART_Begin(UART_HandleTypeDef *p) { (void)p; return HAL_OK; }
HAL_StatusTypeDef LabUART_BeginPort(LabUART_Port c, UART_HandleTypeDef *p) { (void)c; (void)p; return HAL_OK; }
int LabUART_TryWritePort(LabUART_Port c, const char *s, uint16_t n) { (void)c; (void)s; (void)n; return 1; }
int LabUART_TryWrite(const char *s, uint16_t n) { (void)s; (void)n; return 1; }
int LabUART_TryReadPort(LabUART_Port c, uint8_t *p) { (void)c; (void)p; return 0; }
int LabUART_TakeRxFaultPort(LabUART_Port c) { (void)c; return 0; }
uint32_t LabUART_RxErrors(void) { return 0; }
uint32_t LabUART_TxDropped(void) { return 0; }
uint32_t LabUART_RxErrorsPort(LabUART_Port c) { (void)c; return 0; }
uint32_t LabUART_TxDroppedPort(LabUART_Port c) { (void)c; return 0; }
void TaskOLED_Begin(uint32_t n) { (void)n; }
int TaskOLED_SetLines(const char *const text[4]) { (void)text; return 1; }
void TaskOLED_Poll(uint32_t n) { (void)n; }
uint32_t TaskOLED_LastIOUs(void) { return 100U; }

#include "car_internal.h"

/* 构造 Y_READY 状态并指定轮速与陀螺读数；绕过校准流程直接测试采样融合。 */
static CarAppContext *ready_context(float v_left, float v_right, int16_t gyro_z)
{
    CarAppContext *app = CarApp_TestContext();
    memset(app, 0, sizeof *app);
    app->runtime.hz = 72000000U;
    app->heading.phase = Y_READY;
    app->heading.bias = -424.0f;
    app->heading.cycles = DWT->CYCCNT;
    app->wheel.items[LEFT].velocity = v_left;
    app->wheel.items[RIGHT].velocity = v_right;
    gz = gyro_z;
    return app;
}

static void sample_ticks(CarAppContext *app, unsigned ticks)
{
    for (unsigned i = 0; i < ticks; ++i) {
        DWT->CYCCNT += 720000U; /* 10 毫秒节拍；含 I2C 读取后拍间隔 10.5 毫秒，在合法范围内。 */
        ms += 10U;
        CarHeading_Sample(app, ms, 1, 1);
        assert(app->heading.phase == Y_READY);
    }
}

static void test_fuse_straight_converges(void)
{
    /* 10 秒直线，陀螺含 0.4 度/秒残余漂移：零偏估计收敛，融合角速度回到零。 */
    float bias = 0.0f, fused = 1.0f;
    for (unsigned i = 0; i < 1000U; ++i)
        fused = CarHeading_FuseRate(0.4f, 200.0f, 200.0f, 0.01f, &bias);
    assert(bias > 0.35f && bias < 0.45f);
    assert(fabsf(fused) < 0.05f);
}

static void test_fuse_turn_keeps_gyro(void)
{
    /* 右轮快：轮速差速角速度约 76.4 度/秒，超过直线门限，冻结估计保持纯陀螺。 */
    float bias = 0.0f;
    float fused = CarHeading_FuseRate(76.4f, 100.0f, 300.0f, 0.01f, &bias);
    assert(bias == 0.0f);
    assert(fabsf(fused - 76.4f) < 1e-6f);
    bias = 0.3f; /* 已收敛的估计在转弯段冻结但仍从角速度中扣除。 */
    fused = CarHeading_FuseRate(76.4f, 100.0f, 300.0f, 0.01f, &bias);
    assert(bias == 0.3f);
    assert(fabsf(fused - 76.1f) < 1e-3f);
}

static void test_fuse_low_speed_frozen(void)
{
    /* 平均轮速低于门限：编码器观测不可信，冻结估计。 */
    float bias = 0.0f;
    CarHeading_FuseRate(0.4f, 10.0f, 10.0f, 0.01f, &bias);
    assert(bias == 0.0f);
}

static void test_fuse_disagree_frozen(void)
{
    /* 轮速正常直行但陀螺报告 30 度/秒转动：观测矛盾（打滑/抬轮迹象），冻结估计。 */
    float bias = 0.0f;
    float fused = CarHeading_FuseRate(30.0f, 200.0f, 200.0f, 0.01f, &bias);
    assert(bias == 0.0f);
    assert(fabsf(fused - 30.0f) < 1e-6f);
}

static void test_fuse_bias_clamped(void)
{
    /* 20 度/秒矛盾低于 DISAGREE 门限，估计持续更新直至限幅，防止异常观测带偏。 */
    float bias = 0.0f;
    for (unsigned i = 0; i < 5000U; ++i)
        CarHeading_FuseRate(20.0f, 200.0f, 200.0f, 0.01f, &bias);
    assert(bias == DPT_HEADING_FUSION_BIAS_MAX_DPS);
}

static void test_fuse_wheel_error_learned(void)
{
    /* 固定的轮速差速（轮径/轮距误差）会被直线段学进零偏（本例约 -0.76 度/秒）；
     * 这是融合的设计代价，凸显 DPT_TRACK_WIDTH_MM 必须实测标定。 */
    float bias = 0.0f;
    for (unsigned i = 0; i < 2000U; ++i)
        CarHeading_FuseRate(0.0f, 199.0f, 201.0f, 0.01f, &bias);
    assert(bias < -0.7f && bias > -0.85f);
}

static void test_sample_straight_drift_bounded(void)
{
    /* 直行 800 拍（约 8.4 秒），陀螺含 52/131 ≈ 0.4 度/秒残余漂移。 */
    CarAppContext *app = ready_context(200.0f, 200.0f, -424 + 52);
    sample_ticks(app, 800U);
    assert(app->heading.bias_rate > 0.3f && app->heading.bias_rate < 0.45f);
    /* 角度误差有界（约漂移×收敛时间常数）；无融合时同样时长线性发散约 3.3 度。 */
    assert(app->heading.angle > 0.3f && app->heading.angle < 2.0f);
}

static void test_sample_turn_pure_gyro(void)
{
    /* 转弯 200 拍：轮速差速与陀螺一致（约 76.4 度/秒），估计冻结，角度跟随真实转动。 */
    CarAppContext *app = ready_context(100.0f, 300.0f, -424 + 10008);
    sample_ticks(app, 200U);
    assert(app->heading.bias_rate == 0.0f);
    assert(app->heading.angle > 140.0f && app->heading.angle < 175.0f);
}

int main(void)
{
    test_fuse_straight_converges();
    test_fuse_turn_keeps_gyro();
    test_fuse_low_speed_frozen();
    test_fuse_disagree_frozen();
    test_fuse_bias_clamped();
    test_fuse_wheel_error_learned();
    test_sample_straight_drift_bounded();
    test_sample_turn_pure_gyro();
    printf("heading fusion: straight-bias convergence, turn/low-speed/disagree freeze, clamp, sample integration\n");
    return 0;
}
