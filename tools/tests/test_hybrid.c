#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "car_hybrid.h"

static float update(CarHybridState *state, float line, float yaw, float desired,
    float base, float limit)
{
    CarHybridInput input = {
        .line_error = line,
        .yaw_rate_dps = yaw,
        .desired_yaw_rate_dps = desired,
        .line_confidence = 1.0f,
        .heading_confidence = 1.0f,
        .base_mmps = base,
        .seconds = 0.010f,
        .max_diff_mmps = limit
    };
    return CarHybrid_Update(state, &input);
}

int main(void)
{
    assert(CarHybrid_ConfigValid());
    CarHybridState state;
    CarHybrid_Reset(&state);
    float right = update(&state, 1.0f, 0.0f, 0.0f, 200.0f, 80.0f);
    assert(right > 0.0f && right < 80.0f);

    CarHybrid_Reset(&state);
    float left = update(&state, -1.0f, 0.0f, 0.0f, 200.0f, 80.0f);
    assert(left < 0.0f && fabsf(left + right) < 0.01f);

    CarHybrid_Reset(&state);
    float curve_left = update(&state, 0.0f, 0.0f, 28.6479f, 200.0f, 80.0f);
    assert(curve_left < 0.0f); /* 期望左转时右轮目标更快。 */

    CarHybrid_Reset(&state);
    CarHybridInput gated = {.line_error=3.0f, .line_confidence=0.0f,
        .heading_confidence=0.0f, .base_mmps=200.0f, .seconds=0.010f,
        .max_diff_mmps=80.0f};
    assert(fabsf(CarHybrid_Update(&state, &gated)) < 0.001f);
    gated.heading_confidence = 1.0f; gated.heading_error_deg = 5.0f;
    assert(CarHybrid_Update(&state, &gated) < 0.0f);

    CarHybrid_Reset(&state);
    float saturated = update(&state, 3.0f, 200.0f, 0.0f, 200.0f, 40.0f);
    assert(fabsf(saturated - 40.0f) < 0.001f);

    CarHybrid_Reset(&state);
    for (unsigned i = 0; i < 1000U; ++i)
        (void)update(&state, 1.0f, 0.0f, 0.0f, 200.0f, 80.0f);
    assert(state.integral <= 1.0f && state.integral >= -1.0f);
    puts("PASS: hybrid LQR/PID direction, yaw feed-forward, saturation and anti-windup.");
    return 0;
}
