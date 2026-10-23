#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "car_task3.h"

static float wrap(float value)
{
    value = fmodf(value + 180.0f, 360.0f);
    if (value < 0.0f) value += 360.0f;
    return value - 180.0f;
}

static void begin_arc(CarTask3 *task, CarTask3Input *input, CarTask3Output *output)
{
    memset(task, 0, sizeof *task);
    memset(input, 0, sizeof *input);
    CarTask3Config config = CarTask3_DefaultConfig();
    config.start_from_ab = 0U;
    input->heading_ready = input->enabled = input->line_valid = input->centered = 1U;
    input->black_count = 2U;
    assert(CarTask3_Start(task, &config, input));
    task->phase = T3_ARC;
    task->leg = 1U;
    task->leg_start_mm = task->phase_start_mm = 0.0f;
    task->pending_start_signal = 0U;
    input->yaw_deg = task->start_heading_deg + task->diagonal_angle_deg;
    CarTask3_Update(task, input, output);
    assert(task->observer_ready && task->heading_confidence == 1.0f);
}

static void advance(CarTask3 *task, CarTask3Input *input, CarTask3Output *output,
                    float target_progress, float gyro_scale)
{
    const float start_yaw = task->start_heading_deg + task->diagonal_angle_deg;
    const float half_track = task->config.track_width_mm * 0.5f;
    for (float progress = input->distance_mm + 10.0f;
         progress <= target_progress + 0.01f; progress += 10.0f) {
        float turn = progress / task->config.radius_mm * 57.2957795131f;
        input->now_ms += 10U;
        input->distance_mm = progress;
        input->left_distance_mm = progress *
            (task->config.radius_mm - half_track) / task->config.radius_mm;
        input->right_distance_mm = progress *
            (task->config.radius_mm + half_track) / task->config.radius_mm;
        input->yaw_deg = wrap(start_yaw + gyro_scale * turn);
        CarTask3_Update(task, input, output);
        assert(task->phase == T3_ARC);
        assert(output->control == T3_FOLLOW_LINE);
    }
}

int main(void)
{
    CarTask3 task;
    CarTask3Input input;
    CarTask3Output output;

    /* 理想几何下四个虚拟检查点不会制造额外差速偏置。 */
    begin_arc(&task, &input, &output);
    advance(&task, &input, &output, 1260.0f, 1.0f);
    assert(task.checkpoint_index == 4U);
    assert(fabsf(task.checkpoint_bias_dps) < 0.05f);
    assert(task.heading_confidence > 0.95f && task.line_confidence > 0.95f);
    assert(fabsf(task.gyro_turn_deg - task.encoder_turn_deg) < 0.2f);

    /* 第一四分之一弧欠转约5度，只添加一次小幅偏置且不停车。 */
    begin_arc(&task, &input, &output);
    advance(&task, &input, &output, 320.0f, 40.0f / 45.0f);
    assert(task.checkpoint_index == 1U && task.checkpoint_event == 1U);
    assert(task.heading_error_deg > 4.0f && task.heading_error_deg < 6.5f);
    assert(task.checkpoint_bias_dps > 0.45f && task.checkpoint_bias_dps <= 0.75f);
    assert(output.control == T3_FOLLOW_LINE);

    /* 编码器报告持续转弯而陀螺仪不转时，方向置信度必须降权。 */
    begin_arc(&task, &input, &output);
    advance(&task, &input, &output, 400.0f, 0.0f);
    assert(task.heading_confidence < 0.35f);

    /* 外侧单探头仍保留足够循线权重，避免越偏反而越不修。 */
    input.line_valid = 1U;
    input.centered = 0U;
    input.black_count = 1U;
    input.outermost = 1U;
    input.side_probe = 1U;
    input.line_error = 3.0f;
    for (unsigned i = 0; i < 20U; ++i) {
        input.now_ms += 10U;
        CarTask3_Update(&task, &input, &output);
    }
    assert(task.line_confidence >= 0.45f && task.line_confidence < 0.60f);
    puts("PASS: four soft checkpoints, bounded under-turn correction, gyro/encoder confidence rejection and conservative line-confidence floor.");
    return 0;
}
