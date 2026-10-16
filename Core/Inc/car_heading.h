#ifndef CAR_HEADING_H
#define CAR_HEADING_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarHeading_Enable(CarAppContext *app, int mpu_initialized);
void CarHeading_BeginCalibration(CarAppContext *app, uint32_t now, int enabled);
void CarHeading_Sample(CarAppContext *app, uint32_t now, int enabled, int wheels_moved);
int CarHeading_ConfigValid(void);
/* 输入均为相同采样拍的度/秒、毫米/秒和秒；permit_update 由路线阶段门控。
 * 无效数值返回 NAN 且不更新状态；暂停更新时仍扣除已有零偏。 */
float CarHeading_FuseRate(float gyro_dps, float left_mmps, float right_mmps,
    float dt_s, int permit_update, float *bias_dps);
#endif
