#ifndef CAR_HYBRID_H
#define CAR_HYBRID_H

/* 上层轨迹控制器：LQR 状态反馈与小权重 PID 并联，输出左右轮速差。
 * 下层轮速 PI 仍负责把速度目标变成 PWM，本模块不依赖 HAL。 */
typedef struct {
    float previous_error;
    float filtered_rate;
    float integral;
    unsigned initialized;
} CarHybridState;

typedef struct {
    float line_error;
    float yaw_rate_dps;
    float desired_yaw_rate_dps;
    float heading_error_deg;
    float line_confidence;
    float heading_confidence;
    float base_mmps;
    float seconds;
    float max_diff_mmps;
} CarHybridInput;

void CarHybrid_Reset(CarHybridState *state);
int CarHybrid_ConfigValid(void);
float CarHybrid_Update(CarHybridState *state, const CarHybridInput *input);

#endif
