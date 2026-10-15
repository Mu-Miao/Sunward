#ifndef CAR_HEADING_H
#define CAR_HEADING_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarHeading_Enable(CarAppContext *app, int mpu_initialized);
void CarHeading_BeginCalibration(CarAppContext *app, uint32_t now, int enabled);
void CarHeading_Sample(CarAppContext *app, uint32_t now, int enabled, int wheels_moved);
/* 航向融合纯函数：输入扣除静态零偏后的陀螺角速度（度/秒）、左右轮速（毫米/秒，前进为正）、
 * 拍长（秒）和在线零偏估计状态；输出融合后角速度。
 * 直线段按观测残差收敛零偏估计；转弯、低速、观测矛盾时冻结估计。
 * 不访问硬件与全局状态，主机测试直接调用。 */
float CarHeading_FuseRate(float gyro_dps, float v_left_mmps, float v_right_mmps, float dt_s, float *bias_rate_dps);
#endif
