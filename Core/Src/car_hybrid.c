#include "car_hybrid.h"
#include "car_config.h"
#include <math.h>
#include <stddef.h>

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void CarHybrid_Reset(CarHybridState *state)
{
    if (state == NULL) return;
    state->previous_error = 0.0f;
    state->filtered_rate = 0.0f;
    state->integral = 0.0f;
    state->initialized = 0U;
}

int CarHybrid_ConfigValid(void)
{
    return DPT_HYBRID_ENABLE <= 1U &&
        isfinite(DPT_HYBRID_LQR_LINE_K) && DPT_HYBRID_LQR_LINE_K >= 0.0f &&
        isfinite(DPT_HYBRID_LQR_RATE_K) && DPT_HYBRID_LQR_RATE_K >= 0.0f &&
        isfinite(DPT_HYBRID_LQR_YAW_K) && DPT_HYBRID_LQR_YAW_K >= 0.0f &&
        isfinite(DPT_HYBRID_PID_KP) && DPT_HYBRID_PID_KP >= 0.0f &&
        isfinite(DPT_HYBRID_PID_KI) && DPT_HYBRID_PID_KI >= 0.0f &&
        isfinite(DPT_HYBRID_PID_KD) && DPT_HYBRID_PID_KD >= 0.0f &&
        isfinite(DPT_HYBRID_LQR_WEIGHT) && DPT_HYBRID_LQR_WEIGHT >= 0.0f &&
        DPT_HYBRID_LQR_WEIGHT <= 1.0f &&
        isfinite(DPT_HYBRID_RATE_FILTER) && DPT_HYBRID_RATE_FILTER > 0.0f &&
        DPT_HYBRID_RATE_FILTER <= 1.0f &&
        isfinite(DPT_HYBRID_INTEGRAL_LIMIT) && DPT_HYBRID_INTEGRAL_LIMIT > 0.0f;
}

float CarHybrid_Update(CarHybridState *state, const CarHybridInput *input)
{
    if (state == NULL || input == NULL || !CarHybrid_ConfigValid() ||
        !isfinite(input->line_error) || !isfinite(input->yaw_rate_dps) ||
        !isfinite(input->desired_yaw_rate_dps) || !isfinite(input->base_mmps) ||
        !isfinite(input->seconds) || !isfinite(input->max_diff_mmps) ||
        input->seconds < 0.005f || input->seconds > 0.020f ||
        input->base_mmps <= 0.0f || input->max_diff_mmps <= 0.0f) return 0.0f;

    float raw_rate = 0.0f;
    if (state->initialized)
        raw_rate = (input->line_error - state->previous_error) / input->seconds;
    else state->initialized = 1U;
    state->previous_error = input->line_error;
    state->filtered_rate += DPT_HYBRID_RATE_FILTER *
        (raw_rate - state->filtered_rate);

    /* 积分只补偿固定偏置，约束在一个探头误差量级，避免丢线后饱和。 */
    state->integral = clampf(state->integral + input->line_error * input->seconds,
        -DPT_HYBRID_INTEGRAL_LIMIT, DPT_HYBRID_INTEGRAL_LIMIT);

    /* diff>0 表示左轮快、车体右转。因此正向期望角速度需要负差速。 */
    float yaw_error = input->yaw_rate_dps - input->desired_yaw_rate_dps;
    float lqr = DPT_HYBRID_LQR_LINE_K * input->line_error +
        DPT_HYBRID_LQR_RATE_K * state->filtered_rate +
        DPT_HYBRID_LQR_YAW_K * yaw_error;
    float pid = DPT_HYBRID_PID_KP * input->line_error +
        DPT_HYBRID_PID_KI * state->integral +
        DPT_HYBRID_PID_KD * state->filtered_rate;
    float output = DPT_HYBRID_LQR_WEIGHT * lqr +
        (1.0f - DPT_HYBRID_LQR_WEIGHT) * pid;
    output *= input->base_mmps / DPT_LINE_CRUISE_MMPS;
    return clampf(output, -input->max_diff_mmps, input->max_diff_mmps);
}
