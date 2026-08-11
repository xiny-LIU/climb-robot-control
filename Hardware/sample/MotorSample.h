#ifndef MOTOR_SAMPLE_H
#define MOTOR_SAMPLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_SAMPLE_EXPERIMENT_START_LENGTH_MM  0.0f
#define MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM    2000.0f
#define MOTOR_SAMPLE_EXPERIMENT_STEP_MM          100.0f

#define MOTOR_SAMPLE_EXPERIMENT_HOLD_MS          3000U
#define MOTOR_SAMPLE_EXPERIMENT_END_HOLD_MS      10000U
#define MOTOR_SAMPLE_EXPERIMENT_MOVE_TIMEOUT_MS  30000U

/* A1/A2/A3 对应的实验执行对象。数值与串口命令末位保持一致。 */
typedef enum
{
    MOTOR_SAMPLE_EXPERIMENT_LEFT = 1,
    MOTOR_SAMPLE_EXPERIMENT_RIGHT = 2,
    MOTOR_SAMPLE_EXPERIMENT_BOTH = 3
} MotorSampleExperimentMode_t;

typedef enum
{
    MOTOR_SAMPLE_EXPERIMENT_IDLE = 0,
    MOTOR_SAMPLE_EXPERIMENT_EXTENDING,
    MOTOR_SAMPLE_EXPERIMENT_HOLDING_EXTEND,
    MOTOR_SAMPLE_EXPERIMENT_HOLDING_END,
    MOTOR_SAMPLE_EXPERIMENT_RETRACTING,
    MOTOR_SAMPLE_EXPERIMENT_HOLDING_RETRACT,
    MOTOR_SAMPLE_EXPERIMENT_HOLDING_ZERO,
    MOTOR_SAMPLE_EXPERIMENT_COMPLETE,
    MOTOR_SAMPLE_EXPERIMENT_ABORTED,
    MOTOR_SAMPLE_EXPERIMENT_ERROR
} MotorSampleExperimentState_t;

typedef struct
{
    MotorSampleExperimentState_t state;
    float target_length_mm;
    uint32_t target_index;
    uint32_t state_elapsed_ms;
    MotorSampleExperimentMode_t mode;
    uint8_t running;
} MotorSampleExperimentStatus_t;

/* B1/B2/B3 前馈验证状态。 */
typedef enum
{
    MOTOR_SAMPLE_VALIDATION_IDLE = 0,
    MOTOR_SAMPLE_VALIDATION_EXTENDING,
    MOTOR_SAMPLE_VALIDATION_HOLDING_EXTEND,
    MOTOR_SAMPLE_VALIDATION_MOVING_TURNAROUND,
    MOTOR_SAMPLE_VALIDATION_HOLDING_TURNAROUND,
    MOTOR_SAMPLE_VALIDATION_RETRACTING,
    MOTOR_SAMPLE_VALIDATION_HOLDING_RETRACT,
    MOTOR_SAMPLE_VALIDATION_HOLDING_ZERO,
    MOTOR_SAMPLE_VALIDATION_COMPLETE,
    MOTOR_SAMPLE_VALIDATION_ABORTED,
    MOTOR_SAMPLE_VALIDATION_ERROR
} MotorSampleValidationState_t;

typedef struct
{
    MotorSampleValidationState_t state;
    MotorSampleExperimentMode_t mode;
    uint32_t point_index;
    uint32_t state_elapsed_ms;
    float desired_displacement_mm;
    float left_command_mm;
    float right_command_mm;
    uint8_t running;
} MotorSampleValidationStatus_t;

void MotorSample_ExperimentInit(void);

/*
 * 将一条完整串口命令及其长度传入本函数：
 * A1/a1：左臂标定，A2/a2：右臂标定，A3/a3：双臂同步标定。
 * B1/b1：左臂前馈验证，B2/b2：右臂前馈验证，B3/b3：双臂前馈验证。
 * 识别到实验命令时返回1，否则返回0。
 */
uint8_t MotorSample_ExperimentHandleCommand(const uint8_t *command,
                                            uint8_t length);

/* Call every 5 ms from the scheduler or repeatedly from the main loop. */
void MotorSample_ExperimentTask5ms(void);

/* 不经过串口直接启动指定模式；实验已运行或模式无效时返回0。 */
uint8_t MotorSample_ExperimentStart(MotorSampleExperimentMode_t mode);

/* Stop motion safely and retain an ABORTED state for diagnostics. */
void MotorSample_ExperimentAbort(void);

uint8_t MotorSample_ExperimentIsRunning(void);
void MotorSample_ExperimentGetStatus(MotorSampleExperimentStatus_t *status);

uint8_t MotorSample_ValidationStart(MotorSampleExperimentMode_t mode);
void MotorSample_ValidationAbort(void);
uint8_t MotorSample_ValidationIsRunning(void);
void MotorSample_ValidationGetStatus(MotorSampleValidationStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif
