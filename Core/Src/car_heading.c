/* MPU 静态校准、受路线门控的轮速/陀螺零偏融合与相对航向积分。 */
#include "car_internal.h"
#include "mpu6050.h"
#include <math.h>

int CarHeading_ConfigValid(void)
{
    return DPT_HEADING_FUSION_ENABLE <= 1U &&
        isfinite(DPT_TRACK_WIDTH_MM) && DPT_TRACK_WIDTH_MM >= 100.0f && DPT_TRACK_WIDTH_MM <= 300.0f &&
        isfinite(DPT_HEADING_FUSION_STRAIGHT_DPS) && DPT_HEADING_FUSION_STRAIGHT_DPS > 0.0f && DPT_HEADING_FUSION_STRAIGHT_DPS <= 30.0f &&
        isfinite(DPT_HEADING_FUSION_MIN_MMPS) && DPT_HEADING_FUSION_MIN_MMPS >= 20.0f && DPT_HEADING_FUSION_MIN_MMPS <= 100.0f &&
        isfinite(DPT_HEADING_FUSION_DISAGREE_DPS) && DPT_HEADING_FUSION_DISAGREE_DPS > 0.0f && DPT_HEADING_FUSION_DISAGREE_DPS <= 90.0f &&
        isfinite(DPT_HEADING_FUSION_BIAS_GAIN) && DPT_HEADING_FUSION_BIAS_GAIN > 0.0f && DPT_HEADING_FUSION_BIAS_GAIN <= 1.0f &&
        isfinite(DPT_HEADING_FUSION_BIAS_MAX_DPS) && DPT_HEADING_FUSION_BIAS_MAX_DPS > 0.0f && DPT_HEADING_FUSION_BIAS_MAX_DPS <= 3.0f &&
        DPT_HEADING_FUSION_STABLE_MS >= 30U && DPT_HEADING_FUSION_STABLE_MS <= 1000U;
}

static float wheel_rate(float left, float right)
{
    return (right - left) * 57.2957795131f / DPT_TRACK_WIDTH_MM;
}

static int trusted_observation(float gyro, float left, float right, float bias)
{
    float wheel = wheel_rate(left, right);
    return left > 0.0f && right > 0.0f && left <= 350.0f && right <= 350.0f &&
        (left + right) * 0.5f >= DPT_HEADING_FUSION_MIN_MMPS &&
        fabsf(wheel) <= DPT_HEADING_FUSION_STRAIGHT_DPS &&
        fabsf(gyro - bias - wheel) <= DPT_HEADING_FUSION_DISAGREE_DPS;
}

float CarHeading_FuseRate(float gyro, float left, float right, float dt, int permit_update, float *bias)
{
    if (bias == NULL || !isfinite(gyro) || !isfinite(left) || !isfinite(right) ||
        !isfinite(dt) || dt < 0.005f || dt > 0.020f || !isfinite(*bias) ||
        fabsf(*bias) > DPT_HEADING_FUSION_BIAS_MAX_DPS) return NAN;
    if (permit_update && trusted_observation(gyro, left, right, *bias)) {
        float residual = gyro - *bias - wheel_rate(left, right);
        *bias += DPT_HEADING_FUSION_BIAS_GAIN * residual * dt;
        *bias = fmaxf(-DPT_HEADING_FUSION_BIAS_MAX_DPS, fminf(DPT_HEADING_FUSION_BIAS_MAX_DPS, *bias));
    }
    return gyro - *bias;
}

static void reset_fusion(CarHeadingState *h)
{
    h->bias_rate = h->gyro_angle = h->previous_gyro_rate = h->wheel_rate = 0.0f;
    h->fusion_active = h->fusion_learning = h->fusion_window_seen = 0U;
    h->fusion_window_ms = 0U;
}

static int straight_learning_allowed(const CarAppContext *app, int enabled)
{
    return enabled && app->runtime.state == RUNNING && !app->route.switching &&
        CarMotion_IsFigureEight(app->route.motion) && app->route.task3.phase == T3_DIAGONAL &&
        !app->route.task3.edge_seen && app->route.targets[LEFT] > 0.0f && app->route.targets[RIGHT] > 0.0f;
}

void CarHeading_Enable(CarAppContext *app, int mpu_initialized)
{
    app->heading.available = mpu_initialized != 0;
    app->heading.phase = app->heading.available ? Y_NONE : Y_FAIL;
    app->heading.samples = app->heading.io_us = 0U;
    reset_fusion(&app->heading);
    app->heading.angle = app->heading.reference = app->heading.error = app->heading.diff = 0.0f;
    app->ui.heading_log_ms = HAL_GetTick();
}

void CarHeading_BeginCalibration(CarAppContext *app, uint32_t now, int enabled)
{
    if (app->runtime.state == RUNNING || app->runtime.state == BRAKING || enabled || !app->heading.available) return;
    app->runtime.drive(0, 0);
    CarWheel_Clear(app);
    CarSignal_Set(app, 0);
    app->heading.phase = Y_WARM;
    app->heading.warm_ms = now;
    app->heading.samples = 0U;
    app->heading.sum = 0;
    reset_fusion(&app->heading);
    app->heading.angle = app->heading.reference = app->heading.error = app->heading.diff = 0.0f;
}

void CarHeading_Sample(CarAppContext *app, uint32_t now, int enabled, int wheels_moved)
{
    app->heading.fusion_learning = 0U;
    if (app->heading.phase == Y_WARM || app->heading.phase == Y_CAL) {
        if (enabled || wheels_moved) { app->heading.phase = Y_FAIL; return; }
        if (app->heading.phase == Y_WARM) {
            if ((uint32_t)(now - app->heading.warm_ms) < 2000U) return;
            app->heading.phase = Y_CAL;
        }
    }
    if (app->heading.phase != Y_CAL && app->heading.phase != Y_READY) return;
    MPU6050_Raw raw;
    uint32_t before = DWT->CYCCNT;
    HAL_StatusTypeDef result = MPU6050_ReadRaw(&raw);
    uint32_t cycles = DWT->CYCCNT;
    app->heading.io_us = (uint32_t)((uint64_t)(cycles - before) * 1000000U / app->runtime.hz);
    if (result != HAL_OK || app->heading.io_us > 5000U ||
        raw.gz >= 32760 || raw.gz <= -32760) { app->heading.phase = Y_FAIL; return; }
    if (app->heading.phase == Y_CAL) {
        const int16_t g[3] = {raw.gx, raw.gy, raw.gz};
        for (unsigned i = 0; i < 3U; ++i) {
            if (app->heading.samples == 0U || g[i] < app->heading.min[i]) app->heading.min[i] = g[i];
            if (app->heading.samples == 0U || g[i] > app->heading.max[i]) app->heading.max[i] = g[i];
        }
        app->heading.sum += raw.gz;
        if (++app->heading.samples < 500U) return;
        if ((int32_t)app->heading.max[0] - app->heading.min[0] > 1000 ||
            (int32_t)app->heading.max[1] - app->heading.min[1] > 1000 ||
            (int32_t)app->heading.max[2] - app->heading.min[2] > 300) { app->heading.phase = Y_FAIL; return; }
        app->heading.bias = (float)app->heading.sum / 500.0f;
        reset_fusion(&app->heading);
        app->heading.angle = app->heading.reference = app->heading.error = app->heading.diff = 0.0f;
        app->heading.previous_rate = ((float)raw.gz - app->heading.bias) / 131.0f;
        app->heading.previous_gyro_rate = app->heading.previous_rate;
        app->heading.cycles = cycles;
        app->heading.phase = Y_READY;
        return;
    }
    uint32_t elapsed = cycles - app->heading.cycles;
    app->heading.cycles = cycles;
    if ((uint32_t)((uint64_t)elapsed * 1000000U / app->runtime.hz) < 5000U ||
        (uint32_t)((uint64_t)elapsed * 1000000U / app->runtime.hz) > 20000U) {
        app->heading.phase = Y_FAIL; return;
    }
    float dt = (float)elapsed / app->runtime.hz;
    float gyro = ((float)raw.gz - app->heading.bias) / 131.0f;
    float left = app->wheel.items[LEFT].velocity, right = app->wheel.items[RIGHT].velocity;
    float rate = gyro;
    app->heading.fusion_active = DPT_HEADING_FUSION_ENABLE && CarMotion_IsFigureEight(app->route.motion);
    app->heading.fusion_learning = 0U;
    app->heading.wheel_rate = wheel_rate(left, right);
    if (app->heading.fusion_active && straight_learning_allowed(app, enabled) &&
        trusted_observation(gyro, left, right, app->heading.bias_rate)) {
        if (!app->heading.fusion_window_seen) {
            app->heading.fusion_window_seen = 1U;
            app->heading.fusion_window_ms = now;
        }
        app->heading.fusion_learning = (uint32_t)(now - app->heading.fusion_window_ms) >= DPT_HEADING_FUSION_STABLE_MS;
    } else app->heading.fusion_window_seen = 0U;
    if (app->heading.fusion_active)
        rate = CarHeading_FuseRate(gyro, left, right, dt, app->heading.fusion_learning, &app->heading.bias_rate);
    if (!isfinite(rate)) { app->heading.phase = Y_FAIL; return; }
    app->heading.gyro_angle = CarMath_WrapDegrees(app->heading.gyro_angle + 0.5f * (gyro + app->heading.previous_gyro_rate) * dt);
    app->heading.previous_gyro_rate = gyro;
    app->heading.angle = CarMath_WrapDegrees(app->heading.angle + 0.5f * (rate + app->heading.previous_rate) * dt);
    app->heading.previous_rate = rate;
    app->heading.error = CarMath_WrapDegrees(app->heading.reference - app->heading.angle);
    app->heading.diff = DPT_HEADING_GAIN * app->heading.error;
    if (app->heading.diff > DPT_HEADING_MAX_DIFF) app->heading.diff = DPT_HEADING_MAX_DIFF;
    if (app->heading.diff < -DPT_HEADING_MAX_DIFF) app->heading.diff = -DPT_HEADING_MAX_DIFF;
}
