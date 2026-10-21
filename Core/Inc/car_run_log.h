#ifndef CAR_RUN_LOG_H
#define CAR_RUN_LOG_H

#include "car_state.h"

#define CAR_RUN_LOG_CAPACITY 160U
#define CAR_RUN_LOG_SAMPLE_MS 50U
#define CAR_RUN_LOG_JOURNEY_CAPACITY 805U
#define CAR_RUN_LOG_JOURNEY_SAMPLE_MS 200U
#define CAR_RUN_LOG_EVENT_CAPACITY 32U
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

/* 5Hz 全程压缩层：805条覆盖161秒，足以包含任务四总时限及起止记录。 */
typedef struct {
    uint32_t ms;
    int16_t progress_mm, yaw_x10, speed_left, speed_right;
    int8_t line_error_x2;
    uint8_t phase;
    uint8_t meta;       /* bit0..1=leg, bit2..4=checkpoint, bit5=line valid, bit6=lost, bit7=seek */
    uint8_t confidence; /* low nibble=line, high nibble=heading, each 0..15 */
} CarRunLogJourneyRecord;

typedef enum {
    CAR_RUN_EVENT_BEGIN = 1U, CAR_RUN_EVENT_PHASE, CAR_RUN_EVENT_CHECKPOINT,
    CAR_RUN_EVENT_LOSS, CAR_RUN_EVENT_REACQUIRE, CAR_RUN_EVENT_STOP
} CarRunLogEventType;

typedef struct {
    CarRunLogRecord record;
    int16_t expected_yaw_x10, encoder_turn_x10;
    int8_t heading_error_x2;
    uint8_t event, checkpoint, confidence;
} CarRunLogEventRecord;

typedef struct {
    CarRunLogRecord records[CAR_RUN_LOG_CAPACITY];
    CarRunLogJourneyRecord journey[CAR_RUN_LOG_JOURNEY_CAPACITY];
    CarRunLogEventRecord events[CAR_RUN_LOG_EVENT_CAPACITY];
    uint32_t sample_ms, journey_sample_ms, zero_ms;
    uint16_t head, count, sequence, dump_index, dump_remaining;
    uint16_t journey_head, journey_count, journey_dump_index, journey_dump_remaining;
    uint8_t event_head, event_count, event_dump_index, event_dump_remaining;
    uint8_t frozen, reason, zero_seen, dump_stage, exported;
    uint8_t last_phase, last_leg, last_checkpoint, last_lost, last_seek;
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
int CarRunLog_IsDumping(void);

#endif
