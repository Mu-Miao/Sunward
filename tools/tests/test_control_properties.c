#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "car_hybrid.h"
#include "car_task3.h"

static uint32_t random_state = 0x5a17c9e3U;
static uint32_t random_u32(void)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}
static float random_range(float low, float high)
{
    return low + (high - low) * (float)(random_u32() & 0xffffU) / 65535.0f;
}

static void hybrid_properties(void)
{
    CarHybridState state;
    CarHybrid_Reset(&state);
    for (unsigned n = 0; n < 50000U; ++n) {
        float limit = random_range(5.0f, 120.0f);
        CarHybridInput input = {
            .line_error=random_range(-3.0f, 3.0f),
            .yaw_rate_dps=random_range(-180.0f, 180.0f),
            .desired_yaw_rate_dps=random_range(-90.0f, 90.0f),
            .heading_error_deg=random_range(-180.0f, 180.0f),
            .line_confidence=random_range(0.0f, 1.0f),
            .heading_confidence=random_range(0.0f, 1.0f),
            .base_mmps=random_range(35.0f, 250.0f),
            .seconds=random_range(0.005f, 0.020f),
            .max_diff_mmps=limit
        };
        float output = CarHybrid_Update(&state, &input);
        assert(isfinite(output));
        assert(output <= limit + 0.001f && output >= -limit - 0.001f);
        assert(isfinite(state.integral) && fabsf(state.integral) <= 1.001f);
    }
    CarHybridInput invalid = {.line_error=NAN, .line_confidence=1.0f,
        .heading_confidence=1.0f, .base_mmps=100.0f, .seconds=0.01f,
        .max_diff_mmps=20.0f};
    assert(CarHybrid_Update(&state, &invalid) == 0.0f);
}

static void task3_properties(void)
{
    for (unsigned n = 0; n < 20000U; ++n) {
        CarTask3 task;
        memset(&task, 0, sizeof task);
        CarTask3Config config = CarTask3_DefaultConfig();
        config.start_from_ab = 0U;
        CarTask3Input input = {.now_ms=random_u32(), .heading_ready=1U, .enabled=1U,
            .line_valid=1U, .black_count=2U, .centered=1U};
        CarTask3Output output;
        assert(CarTask3_Start(&task, &config, &input));
        task.phase = T3_ARC;
        task.leg = 1U;
        task.pending_start_signal = 0U;
        input.yaw_deg = task.start_heading_deg + task.diagonal_angle_deg;
        CarTask3_Update(&task, &input, &output);
        assert(task.phase == T3_ARC && output.control == T3_FOLLOW_LINE);

        float progress = random_range(0.0f, config.arc_mm - 10.0f);
        float half_track = config.track_width_mm * 0.5f;
        input.now_ms += 10U;
        input.distance_mm = progress;
        input.left_distance_mm = progress * (config.radius_mm - half_track) / config.radius_mm;
        input.right_distance_mm = progress * (config.radius_mm + half_track) / config.radius_mm;
        input.yaw_deg = task.start_heading_deg + task.diagonal_angle_deg +
            progress / config.radius_mm * 57.2957795131f + random_range(-12.0f, 12.0f);
        input.line_error = random_range(-3.0f, 3.0f);
        input.centered = input.outermost = input.side_probe = 0U;
        unsigned pattern = random_u32() % 6U;
        if (pattern < 3U) {
            input.line_valid = 1U;
            input.black_count = (uint8_t)(pattern + 1U);
            input.centered = pattern == 1U && (random_u32() & 1U);
            input.side_probe = pattern == 0U;
            input.outermost = input.side_probe && (random_u32() & 1U);
        } else {
            input.line_valid = 0U;
            input.black_count = pattern == 3U ? 0U : 4U;
        }
        CarTask3_Update(&task, &input, &output);
        assert(task.phase == T3_ARC || task.phase == T3_FAULT);
        assert(output.control >= T3_HALT && output.control <= T3_SEEK_LINE);
        assert(isfinite(output.base_mmps) && isfinite(output.heading_deg));
        assert(isfinite(output.left_mmps) && isfinite(output.right_mmps));
        assert(output.base_mmps >= 0.0f && output.left_mmps >= 0.0f && output.right_mmps >= 0.0f);
        assert(output.base_mmps <= 250.001f && output.left_mmps <= 100.001f && output.right_mmps <= 100.001f);
        assert(isfinite(task.line_confidence) && task.line_confidence >= 0.0f && task.line_confidence <= 1.0f);
        assert(isfinite(task.heading_confidence) && task.heading_confidence >= 0.0f && task.heading_confidence <= 1.0f);
        assert(isfinite(task.checkpoint_bias_dps));
        assert(fabsf(task.checkpoint_bias_dps) <= config.checkpoint_max_bias_dps + 0.001f);
    }

    CarTask3 task = {0};
    CarTask3Config config = CarTask3_DefaultConfig();
    config.start_from_ab = 0U;
    CarTask3Input input = {.heading_ready=1U, .enabled=1U};
    CarTask3Output output;
    assert(CarTask3_Start(&task, &config, &input));
    input.now_ms = 10U;
    input.yaw_deg = NAN;
    CarTask3_Update(&task, &input, &output);
    assert(task.phase == T3_FAULT && task.fault == T3_SENSOR && output.control == T3_HALT);
}

int main(void)
{
    hybrid_properties();
    task3_properties();
    puts("PASS: 70k deterministic randomized hybrid/task3 cases preserve finite bounded outputs, confidence bounds and sensor-fault halt.");
    return 0;
}
