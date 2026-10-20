#ifndef CAR_RUN_LOG_H
#define CAR_RUN_LOG_H

#include "car_state.h"

#define CAR_RUN_LOG_CAPACITY 256U
#define CAR_RUN_LOG_SAMPLE_MS 50U
#define CAR_RUN_LOG_ZERO_FREEZE_MS 300U

enum {
    CAR_RUN_LOG_FLAG_LINE_VALID = 1U,
    CAR_RUN_LOG_FLAG_LINE_LOST = 2U,
    CAR_RUN_LOG_FLAG_SEEK = 4U,
    CAR_RUN_LOG_FLAG_ZERO_TARGET = 8U,
    CAR_RUN_LOG_FLAG_ZERO_SPEED = 16U,
    CAR_RUN_LOG_FLAG_FAULT = 32U
};

typedef struct {
    uint32_t ms;
    int16_t target[2];
    int16_t speed[2];
    int16_t yaw_x10;
    int8_t line_error_x2;
    uint8_t raw_bits;
    uint8_t state, motion, phase, leg, flags, arc;
    uint16_t sequence;
} CarRunLogRecord;

typedef struct {
    CarRunLogRecord records[CAR_RUN_LOG_CAPACITY];
    uint32_t sample_ms, zero_ms;
    uint16_t head, count, sequence, dump_index, dump_remaining;
    uint8_t frozen, reason, zero_seen, dump_stage, exported;
} CarRunLogStore;

extern CarRunLogStore g_car_run_log;

void CarRunLog_Init(uint32_t now);
void CarRunLog_BeginRun(const CarAppContext *app, uint32_t now);
void CarRunLog_Poll(const CarAppContext *app, uint32_t now);
void CarRunLog_OnStop(const CarAppContext *app, TestState why, uint32_t now);
void CarRunLog_RequestDump(void);
void CarRunLog_Service(const CarAppContext *app);
void CarRunLog_Clear(uint32_t now);
int CarRunLog_IsFrozen(void);
int CarRunLog_IsExported(void);

#endif
