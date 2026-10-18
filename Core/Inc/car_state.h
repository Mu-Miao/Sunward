#ifndef CAR_STATE_H
#define CAR_STATE_H
#include "stm32f1xx_hal.h"
#include "lab_uart_async.h"
#include "car_task3.h"
#include "car_task4.h"

/* 整车内部状态由 CarApp 持有，只在主循环中访问。
 * 硬件中断通过 lab_uart_async 队列传递数据。 */
typedef enum { IDLE, RUNNING, DONE, OFF, LOW, ADC_BAD, GAP,
    REVERSE, FAST, STALL, UART_BAD, STOPPED, YAW_BAD, BRAKING,
    TIMEOUT, SETTLE_BAD, LINE_LOST, LINE_BAD } TestState;
enum { LEFT = 0, RIGHT = 1 };
typedef enum { MOVE_NONE, MOVE_TEST, MOVE_FORWARD, MOVE_BACK, MOVE_LEFT,
    MOVE_RIGHT, MOVE_HEADING, MOVE_DISTANCE, MOVE_ROUTE, MOVE_LINE, MOVE_TASK2, MOVE_TASK3, MOVE_TASK4 } Motion;
static inline int CarMotion_IsFigureEight(Motion motion)
{
    return motion == MOVE_TASK3 || motion == MOVE_TASK4;
}
typedef enum { Y_NONE, Y_WARM, Y_CAL, Y_READY, Y_FAIL } YawPhase;
typedef enum { KEY_NONE, KEY_CAL, KEY_START, KEY_STOP, KEY_WAIT, KEY_BLOCK, KEY_MODE } KeyAction;
typedef enum { KEY_EVENT_NONE, KEY_EVENT_SHORT, KEY_EVENT_HOLD } KeyEvent;
typedef struct {
    TIM_HandleTypeDef *encoder;
    uint16_t old_count;
    float velocity, integral;
    int command;
    uint32_t bad_sign_us, stall_us;
} Wheel;
typedef struct {
    char text[12];
    unsigned length;
    uint8_t discard;
    uint32_t ms;
} CommandInput;

typedef struct {
    YawPhase phase;
    uint8_t available;
    uint8_t cal_requested;
    uint32_t warm_ms;
    uint32_t cycles;
    uint32_t samples;
    uint32_t io_us;
    int32_t sum;
    int16_t min[3];
    int16_t max[3];
    float bias;
    float bias_rate, gyro_angle, previous_gyro_rate, wheel_rate;
    uint8_t fusion_active, fusion_learning, fusion_window_seen;
    uint32_t fusion_window_ms;
    float angle;
    float previous_rate;
    float reference;
    float error;
    float diff;
} CarHeadingState;

typedef struct {
    KeyAction action;
    uint8_t raw;
    uint8_t stable;
    uint8_t armed;
    uint8_t press_enabled;
    uint8_t off_pending;
    uint8_t press_busy;
    uint32_t change_ms;
    uint32_t events;
    uint32_t hold_ms;
    Motion selected_motion;
} CarButtonState;

typedef struct {
    uint8_t raw[4];
    uint8_t black[4];
    uint8_t valid;
    uint8_t count;
    uint8_t center_seen;
    uint8_t center_ready;
    uint8_t invalid_seen;
    uint32_t center_ms;
    uint32_t sample_ms;
    uint32_t invalid_ms;
    float error;
    float diff;
} CarLineState;

typedef struct {
    Motion motion;
    float distance_mm[2];
    float base;
    float goal;
    float cruise;
    float slow;
    uint16_t distance_count[2];
    uint8_t tracking;
    uint8_t settle_seen;
    uint32_t brake_ms;
    uint32_t settle_ms;
    uint32_t manual_duration;
    uint32_t run_duration;
    uint32_t switch_ms;
    uint32_t start_ms;
    unsigned manual_speed;
    uint8_t switching;
    float targets[2];
    uint8_t leg;
    uint8_t edge_seen;
    uint32_t edge_ms;
    uint32_t elapsed_ms;
    float leg_start;
    float start_heading;
    CarTask3 task3;
    CarTask4 task4;
    struct {
        uint8_t valid_seen, lost_seen, previous_seen;
        uint8_t valid_raw[4], lost_raw[4];
        uint8_t previous_raw[4], lost_arc_number;
        int8_t lost_direction; /* +1向右，-1向左，首次失线时按可信图样/弧段冻结。 */
        uint32_t lost_ms;
        float valid_error, valid_diff, lost_yaw;
        float previous_error;
        float targets[2], velocity[2];
    } task3_trace; /* 保存半圆首次无效采样；F7 停车、关闭使能或搬动车身后仍保留快照。 */
    struct {
        uint8_t seen, phase;
        float yaw, targets[2], velocity[2];
    } reverse_trace; /* 在 REV 清 PI、取消路线前保存，供 OLED 区分真实故障轮与采样。 */
    struct {
        uint8_t seen, point, lap, raw[4];
        float distance_mm, yaw, gyro_yaw, bias_rate;
    } vertex_trace; /* 软件确认 C/D 时的真实输入快照，不代表车身实际已覆盖顶点。 */
    struct {
        uint8_t seen, phase, limits, raw[4];
        uint32_t ms, duration_ms;
        float travel_mm, distance_mm, yaw;
    } edge_trace; /* F6 前阶段与原时间/距离边界，停车后不被实物移动覆盖。 */
} CarRouteState;

typedef struct {
    Wheel items[2];
} CarWheelState;

typedef struct {
    CommandInput inputs[LAB_UART_COUNT];
} CarCommandState;

typedef struct {
    ADC_HandleTypeDef * adc;
    uint8_t valid;
    uint32_t sample_ms;
    uint32_t raw;
} CarPowerState;

typedef struct {
    uint8_t active;
    uint8_t ready_led_on;
    uint32_t ms;
} CarSignalState;

typedef struct {
    uint32_t render_ms;
    uint32_t oled_log_ms;
    uint32_t line_log_ms;
    uint32_t heading_log_ms;
    uint32_t button_log_ms;
    uint32_t motion_log_ms;
} CarUiState;

typedef struct {
    TestState state;
    uint8_t active;
    uint8_t primed;
    uint8_t on_seen;
    uint8_t bluetooth_on;
    uint32_t hz;
    uint32_t old_cycles;
    uint32_t old_tick;
    uint32_t old_ms;
    uint32_t on_ms;
    uint32_t dt_us;
    char fault_wheel;
    void (* drive)(int, int);
} CarRuntimeState;

typedef struct CarAppContext {
    CarHeadingState heading;
    CarButtonState button;
    CarLineState line;
    CarRouteState route;
    CarWheelState wheel;
    CarCommandState command;
    CarPowerState power;
    CarSignalState signal;
    CarUiState ui;
    CarRuntimeState runtime;
} CarAppContext;
#endif
