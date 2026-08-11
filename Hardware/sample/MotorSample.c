#include "MotorSample.h"

#include "main.h"
#include "m3508_position.h"

#include <stddef.h>

typedef enum
{
    MOTOR_SAMPLE_DIRECTION_EXTEND = 0,
    MOTOR_SAMPLE_DIRECTION_RETRACT
} MotorSampleDirection_t;

#define MOTOR_SAMPLE_VALIDATION_POINT_COUNT  14U

/* 期望实际伸缩量：0, 100, ..., 1300 mm。 */
static const float g_validation_desired_mm[MOTOR_SAMPLE_VALIDATION_POINT_COUNT] =
{
    0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f,
    700.0f, 800.0f, 900.0f, 1000.0f, 1100.0f, 1200.0f, 1300.0f
};

/* 最终前馈表中的伸出理论指令，左臂和右臂必须分别使用。 */
static const float g_validation_extend_command_mm[2][MOTOR_SAMPLE_VALIDATION_POINT_COUNT] =
{
    {
        0.0f, 165.5f, 299.1f, 434.7f, 570.6f, 706.0f, 840.1f,
        976.2f, 1112.6f, 1248.4f, 1386.5f, 1524.7f, 1658.3f, 1791.6f
    },
    {
        0.0f, 167.2f, 300.2f, 435.2f, 571.5f, 707.2f, 843.2f,
        977.3f, 1112.5f, 1248.8f, 1385.3f, 1523.3f, 1660.3f, 1794.4f
    }
};

/* 最终前馈表中的回缩理论指令。 */
static const float g_validation_retract_command_mm[2][MOTOR_SAMPLE_VALIDATION_POINT_COUNT] =
{
    {
        0.0f, 240.2f, 368.4f, 498.7f, 628.9f, 757.5f, 887.0f,
        1015.7f, 1144.2f, 1273.1f, 1402.9f, 1532.7f, 1662.6f, 1792.9f
    },
    {
        0.0f, 227.0f, 356.6f, 484.7f, 617.1f, 746.8f, 878.9f,
        1009.7f, 1140.6f, 1272.7f, 1405.4f, 1535.6f, 1664.9f, 1795.7f
    }
};

static MotorSampleExperimentStatus_t g_experiment;
static MotorSampleValidationStatus_t g_validation;
static uint32_t g_state_started_ms;
static uint32_t g_move_started_ms;
static uint32_t g_validation_state_started_ms;
static uint32_t g_validation_move_started_ms;

static void MotorSample_ValidationTask5ms(void);

static uint8_t MotorSample_IsModeValid(MotorSampleExperimentMode_t mode)
{
    return ((mode == MOTOR_SAMPLE_EXPERIMENT_LEFT) ||
            (mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ||
            (mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)) ? 1U : 0U;
}

static uint32_t MotorSample_GetElapsedMs(uint32_t started_ms)
{
    return HAL_GetTick() - started_ms;
}

static void MotorSample_EnterState(MotorSampleExperimentState_t state)
{
    g_experiment.state = state;
    g_state_started_ms = HAL_GetTick();
    g_experiment.state_elapsed_ms = 0U;
}

static void MotorSample_SetTarget(float target_length_mm,
                                  MotorSampleDirection_t direction)
{
    g_experiment.target_length_mm = target_length_mm;
    g_experiment.target_index =
        (uint32_t)(target_length_mm / MOTOR_SAMPLE_EXPERIMENT_STEP_MM);

    /* A3模式下必须在同一个状态机节拍中同时更新左右臂目标。 */
    if (g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        M3508_Position_SetTargetLengthBoth(target_length_mm,
                                           target_length_mm);
    }
    else
    {
        M3508_PositionSide_t side =
            (g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
            M3508_POS_RIGHT : M3508_POS_LEFT;
        M3508_Position_SetTargetLength(side, target_length_mm);
    }

    g_move_started_ms = HAL_GetTick();

    MotorSample_EnterState(
        (direction == MOTOR_SAMPLE_DIRECTION_EXTEND) ?
        MOTOR_SAMPLE_EXPERIMENT_EXTENDING :
        MOTOR_SAMPLE_EXPERIMENT_RETRACTING);
}

static uint8_t MotorSample_IsTargetReached(void)
{
    if (g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        return M3508_Position_IsAllTargetReached();
    }

    return M3508_Position_IsTargetReached(
        (g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
        M3508_POS_RIGHT : M3508_POS_LEFT);
}

static uint8_t MotorSample_HasMotorError(void)
{
    M3508_PositionDebug_t debug;

    M3508_Position_GetDebugInfo(&debug);

    /*
     * Current protection remains active in the lower position/PID layer.
     * Transient motor status must not permanently abort the full sequence.
     * Persistent failure to move is handled by the per-step timeout.
     */
    if (g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        return ((debug.left.status == M3508_POS_TARGET_LIMITED) ||
                (debug.right.status == M3508_POS_TARGET_LIMITED)) ? 1U : 0U;
    }

    return (((g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
             debug.right.status : debug.left.status) ==
            M3508_POS_TARGET_LIMITED) ? 1U : 0U;
}

static void MotorSample_StopAtCurrentPosition(void)
{
    if (g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        M3508_Position_SyncTargetToCurrentBoth();
        M3508_Position_StopAll();
    }
    else
    {
        M3508_PositionSide_t side =
            (g_experiment.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
            M3508_POS_RIGHT : M3508_POS_LEFT;
        M3508_Position_SyncTargetToCurrent(side);
        M3508_Position_Stop(side);
    }
}

static void MotorSample_SetError(void)
{
    MotorSample_StopAtCurrentPosition();
    g_experiment.running = 0U;
    MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_ERROR);
}

static uint8_t MotorSample_ValidationGetSideIndex(MotorSampleExperimentMode_t mode)
{
    return (mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ? 1U : 0U;
}

static void MotorSample_ValidationEnterState(MotorSampleValidationState_t state)
{
    g_validation.state = state;
    g_validation_state_started_ms = HAL_GetTick();
    g_validation.state_elapsed_ms = 0U;
}

static void MotorSample_ValidationSetIndexedTarget(
    uint32_t point_index,
    MotorSampleDirection_t direction)
{
    const float (*commands)[MOTOR_SAMPLE_VALIDATION_POINT_COUNT];
    uint8_t side_index;

    if (point_index >= MOTOR_SAMPLE_VALIDATION_POINT_COUNT)
    {
        return;
    }

    commands = (direction == MOTOR_SAMPLE_DIRECTION_EXTEND) ?
        g_validation_extend_command_mm : g_validation_retract_command_mm;

    g_validation.point_index = point_index;
    g_validation.desired_displacement_mm =
        g_validation_desired_mm[point_index];
    g_validation.left_command_mm = commands[0][point_index];
    g_validation.right_command_mm = commands[1][point_index];

    if (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        M3508_Position_SetTargetLengthBoth(
            g_validation.left_command_mm,
            g_validation.right_command_mm);
    }
    else
    {
        side_index = MotorSample_ValidationGetSideIndex(g_validation.mode);
        M3508_Position_SetTargetLength(
            (side_index == 0U) ? M3508_POS_LEFT : M3508_POS_RIGHT,
            commands[side_index][point_index]);
    }

    g_validation_move_started_ms = HAL_GetTick();
    MotorSample_ValidationEnterState(
        (direction == MOTOR_SAMPLE_DIRECTION_EXTEND) ?
        MOTOR_SAMPLE_VALIDATION_EXTENDING :
        MOTOR_SAMPLE_VALIDATION_RETRACTING);
}

static void MotorSample_ValidationSetTurnaroundTarget(void)
{
    g_validation.desired_displacement_mm = -1.0f;
    g_validation.left_command_mm = MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM;
    g_validation.right_command_mm = MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM;

    if (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        M3508_Position_SetTargetLengthBoth(
            MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM,
            MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM);
    }
    else
    {
        M3508_Position_SetTargetLength(
            (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
            M3508_POS_RIGHT : M3508_POS_LEFT,
            MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM);
    }

    g_validation_move_started_ms = HAL_GetTick();
    MotorSample_ValidationEnterState(
        MOTOR_SAMPLE_VALIDATION_MOVING_TURNAROUND);
}

static uint8_t MotorSample_ValidationIsTargetReached(void)
{
    if (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        return M3508_Position_IsAllTargetReached();
    }

    return M3508_Position_IsTargetReached(
        (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
        M3508_POS_RIGHT : M3508_POS_LEFT);
}

static uint8_t MotorSample_ValidationHasMotorError(void)
{
    M3508_PositionDebug_t debug;

    M3508_Position_GetDebugInfo(&debug);
    if (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        return ((debug.left.status == M3508_POS_TARGET_LIMITED) ||
                (debug.right.status == M3508_POS_TARGET_LIMITED)) ? 1U : 0U;
    }

    return (((g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
             debug.right.status : debug.left.status) ==
            M3508_POS_TARGET_LIMITED) ? 1U : 0U;
}

static void MotorSample_ValidationStopAtCurrentPosition(void)
{
    if (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        M3508_Position_SyncTargetToCurrentBoth();
        M3508_Position_StopAll();
    }
    else
    {
        M3508_PositionSide_t side =
            (g_validation.mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
            M3508_POS_RIGHT : M3508_POS_LEFT;
        M3508_Position_SyncTargetToCurrent(side);
        M3508_Position_Stop(side);
    }
}

static void MotorSample_ValidationSetError(void)
{
    MotorSample_ValidationStopAtCurrentPosition();
    g_validation.running = 0U;
    MotorSample_ValidationEnterState(MOTOR_SAMPLE_VALIDATION_ERROR);
}

static void MotorSample_ValidationTask5ms(void)
{
    uint32_t now;

    if (g_validation.running == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    g_validation.state_elapsed_ms =
        now - g_validation_state_started_ms;

    if (MotorSample_ValidationHasMotorError() != 0U)
    {
        MotorSample_ValidationSetError();
        return;
    }

    switch (g_validation.state)
    {
        case MOTOR_SAMPLE_VALIDATION_EXTENDING:
            if (MotorSample_ValidationIsTargetReached() != 0U)
            {
                MotorSample_ValidationEnterState(
                    MOTOR_SAMPLE_VALIDATION_HOLDING_EXTEND);
            }
            else if (MotorSample_GetElapsedMs(
                         g_validation_move_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_MOVE_TIMEOUT_MS)
            {
                MotorSample_ValidationSetError();
            }
            break;

        case MOTOR_SAMPLE_VALIDATION_HOLDING_EXTEND:
            if (MotorSample_ValidationIsTargetReached() == 0U)
            {
                g_validation_move_started_ms = HAL_GetTick();
                MotorSample_ValidationEnterState(
                    MOTOR_SAMPLE_VALIDATION_EXTENDING);
            }
            else if (MotorSample_GetElapsedMs(
                         g_validation_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_HOLD_MS)
            {
                if (g_validation.point_index >=
                    (MOTOR_SAMPLE_VALIDATION_POINT_COUNT - 1U))
                {
                    MotorSample_ValidationSetTurnaroundTarget();
                }
                else
                {
                    MotorSample_ValidationSetIndexedTarget(
                        g_validation.point_index + 1U,
                        MOTOR_SAMPLE_DIRECTION_EXTEND);
                }
            }
            break;

        case MOTOR_SAMPLE_VALIDATION_MOVING_TURNAROUND:
            if (MotorSample_ValidationIsTargetReached() != 0U)
            {
                MotorSample_ValidationEnterState(
                    MOTOR_SAMPLE_VALIDATION_HOLDING_TURNAROUND);
            }
            else if (MotorSample_GetElapsedMs(
                         g_validation_move_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_MOVE_TIMEOUT_MS)
            {
                MotorSample_ValidationSetError();
            }
            break;

        case MOTOR_SAMPLE_VALIDATION_HOLDING_TURNAROUND:
            if (MotorSample_ValidationIsTargetReached() == 0U)
            {
                g_validation_move_started_ms = HAL_GetTick();
                MotorSample_ValidationEnterState(
                    MOTOR_SAMPLE_VALIDATION_MOVING_TURNAROUND);
            }
            else if (MotorSample_GetElapsedMs(
                         g_validation_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_END_HOLD_MS)
            {
                MotorSample_ValidationSetIndexedTarget(
                    MOTOR_SAMPLE_VALIDATION_POINT_COUNT - 1U,
                    MOTOR_SAMPLE_DIRECTION_RETRACT);
            }
            break;

        case MOTOR_SAMPLE_VALIDATION_RETRACTING:
            if (MotorSample_ValidationIsTargetReached() != 0U)
            {
                MotorSample_ValidationEnterState(
                    (g_validation.point_index == 0U) ?
                    MOTOR_SAMPLE_VALIDATION_HOLDING_ZERO :
                    MOTOR_SAMPLE_VALIDATION_HOLDING_RETRACT);
            }
            else if (MotorSample_GetElapsedMs(
                         g_validation_move_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_MOVE_TIMEOUT_MS)
            {
                MotorSample_ValidationSetError();
            }
            break;

        case MOTOR_SAMPLE_VALIDATION_HOLDING_RETRACT:
            if (MotorSample_ValidationIsTargetReached() == 0U)
            {
                g_validation_move_started_ms = HAL_GetTick();
                MotorSample_ValidationEnterState(
                    MOTOR_SAMPLE_VALIDATION_RETRACTING);
            }
            else if (MotorSample_GetElapsedMs(
                         g_validation_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_HOLD_MS)
            {
                MotorSample_ValidationSetIndexedTarget(
                    g_validation.point_index - 1U,
                    MOTOR_SAMPLE_DIRECTION_RETRACT);
            }
            break;

        case MOTOR_SAMPLE_VALIDATION_HOLDING_ZERO:
            if (MotorSample_ValidationIsTargetReached() == 0U)
            {
                g_validation_move_started_ms = HAL_GetTick();
                MotorSample_ValidationEnterState(
                    MOTOR_SAMPLE_VALIDATION_RETRACTING);
            }
            else if (MotorSample_GetElapsedMs(
                         g_validation_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_HOLD_MS)
            {
                MotorSample_ValidationStopAtCurrentPosition();
                g_validation.running = 0U;
                MotorSample_ValidationEnterState(
                    MOTOR_SAMPLE_VALIDATION_COMPLETE);
            }
            break;

        case MOTOR_SAMPLE_VALIDATION_IDLE:
        case MOTOR_SAMPLE_VALIDATION_COMPLETE:
        case MOTOR_SAMPLE_VALIDATION_ABORTED:
        case MOTOR_SAMPLE_VALIDATION_ERROR:
        default:
            g_validation.running = 0U;
            break;
    }
}

void MotorSample_ExperimentInit(void)
{
    g_experiment.state = MOTOR_SAMPLE_EXPERIMENT_IDLE;
    g_experiment.target_length_mm = MOTOR_SAMPLE_EXPERIMENT_START_LENGTH_MM;
    g_experiment.target_index = 0U;
    g_experiment.state_elapsed_ms = 0U;
    g_experiment.mode = MOTOR_SAMPLE_EXPERIMENT_LEFT;
    g_experiment.running = 0U;
    g_state_started_ms = HAL_GetTick();
    g_move_started_ms = HAL_GetTick();

    g_validation.state = MOTOR_SAMPLE_VALIDATION_IDLE;
    g_validation.mode = MOTOR_SAMPLE_EXPERIMENT_LEFT;
    g_validation.point_index = 0U;
    g_validation.state_elapsed_ms = 0U;
    g_validation.desired_displacement_mm = 0.0f;
    g_validation.left_command_mm = 0.0f;
    g_validation.right_command_mm = 0.0f;
    g_validation.running = 0U;
    g_validation_state_started_ms = HAL_GetTick();
    g_validation_move_started_ms = HAL_GetTick();
}

uint8_t MotorSample_ExperimentHandleCommand(const uint8_t *command,
                                            uint8_t length)
{
    MotorSampleExperimentMode_t mode;
    uint8_t validation_command;

    if ((command == NULL) || (length != 2U))
    {
        return 0U;
    }

    if ((command[0] == (uint8_t)'A') ||
        (command[0] == (uint8_t)'a'))
    {
        validation_command = 0U;
    }
    else if ((command[0] == (uint8_t)'B') ||
             (command[0] == (uint8_t)'b'))
    {
        validation_command = 1U;
    }
    else
    {
        return 0U;
    }

    if (command[1] == (uint8_t)'1')
    {
        mode = MOTOR_SAMPLE_EXPERIMENT_LEFT;
    }
    else if (command[1] == (uint8_t)'2')
    {
        mode = MOTOR_SAMPLE_EXPERIMENT_RIGHT;
    }
    else if (command[1] == (uint8_t)'3')
    {
        mode = MOTOR_SAMPLE_EXPERIMENT_BOTH;
    }
    else
    {
        return 0U;
    }

    /* 命令已识别即返回1；若另一状态机正在运行，Start会拒绝启动。 */
    if (validation_command != 0U)
    {
        (void)MotorSample_ValidationStart(mode);
    }
    else
    {
        (void)MotorSample_ExperimentStart(mode);
    }
    return 1U;
}

uint8_t MotorSample_ExperimentStart(MotorSampleExperimentMode_t mode)
{
    if ((g_experiment.running != 0U) ||
        (g_validation.running != 0U) ||
        (MotorSample_IsModeValid(mode) == 0U))
    {
        return 0U;
    }

    /*
     * 启动前提：被选中的机械臂必须处于实验0 mm机械位置，且
     * M3508_Position_Init()已经执行。清零后立即同步目标，防止位置环
     * 追逐上一次实验遗留的目标。
     */
    if (mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        M3508_Position_ResetEncoderAndSyncTargetBoth();
    }
    else
    {
        M3508_Position_ResetEncoderAndSyncTarget(
            (mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
            M3508_POS_RIGHT : M3508_POS_LEFT);
    }

    M3508_Position_Enable(1U);

    g_experiment.mode = mode;
    g_experiment.running = 1U;
    g_experiment.target_index = 0U;

    MotorSample_SetTarget(MOTOR_SAMPLE_EXPERIMENT_STEP_MM,
                          MOTOR_SAMPLE_DIRECTION_EXTEND);
    return 1U;
}

void MotorSample_ExperimentAbort(void)
{
    if (g_validation.running != 0U)
    {
        MotorSample_ValidationAbort();
        return;
    }

    MotorSample_StopAtCurrentPosition();
    g_experiment.running = 0U;
    MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_ABORTED);
}

uint8_t MotorSample_ExperimentIsRunning(void)
{
    return ((g_experiment.running != 0U) ||
            (g_validation.running != 0U)) ? 1U : 0U;
}

void MotorSample_ExperimentGetStatus(MotorSampleExperimentStatus_t *status)
{
    uint32_t primask;

    if (status == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *status = g_experiment;
    __set_PRIMASK(primask);
}

uint8_t MotorSample_ValidationStart(MotorSampleExperimentMode_t mode)
{
    if ((g_validation.running != 0U) ||
        (g_experiment.running != 0U) ||
        (MotorSample_IsModeValid(mode) == 0U))
    {
        return 0U;
    }

    if (mode == MOTOR_SAMPLE_EXPERIMENT_BOTH)
    {
        M3508_Position_ResetEncoderAndSyncTargetBoth();
    }
    else
    {
        M3508_Position_ResetEncoderAndSyncTarget(
            (mode == MOTOR_SAMPLE_EXPERIMENT_RIGHT) ?
            M3508_POS_RIGHT : M3508_POS_LEFT);
    }

    M3508_Position_Enable(1U);
    g_validation.mode = mode;
    g_validation.running = 1U;

    /* 初始0 mm平台由动捕在发送B命令前记录，状态机从100 mm开始。 */
    MotorSample_ValidationSetIndexedTarget(
        1U,
        MOTOR_SAMPLE_DIRECTION_EXTEND);
    return 1U;
}

void MotorSample_ValidationAbort(void)
{
    if (g_validation.running == 0U)
    {
        return;
    }

    MotorSample_ValidationStopAtCurrentPosition();
    g_validation.running = 0U;
    MotorSample_ValidationEnterState(MOTOR_SAMPLE_VALIDATION_ABORTED);
}

uint8_t MotorSample_ValidationIsRunning(void)
{
    return g_validation.running;
}

void MotorSample_ValidationGetStatus(MotorSampleValidationStatus_t *status)
{
    uint32_t primask;

    if (status == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *status = g_validation;
    __set_PRIMASK(primask);
}

void MotorSample_ExperimentTask5ms(void)
{
    uint32_t now;
    float next_target;

    if (g_validation.running != 0U)
    {
        MotorSample_ValidationTask5ms();
        return;
    }

    if (g_experiment.running == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    g_experiment.state_elapsed_ms = now - g_state_started_ms;

    if (MotorSample_HasMotorError() != 0U)
    {
        MotorSample_SetError();
        return;
    }

    switch (g_experiment.state)
    {
        case MOTOR_SAMPLE_EXPERIMENT_EXTENDING:
            if (MotorSample_IsTargetReached() != 0U)
            {
                if (g_experiment.target_length_mm >=
                    MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM)
                {
                    MotorSample_EnterState(
                        MOTOR_SAMPLE_EXPERIMENT_HOLDING_END);
                }
                else
                {
                    MotorSample_EnterState(
                        MOTOR_SAMPLE_EXPERIMENT_HOLDING_EXTEND);
                }
            }
            else if (MotorSample_GetElapsedMs(g_move_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_MOVE_TIMEOUT_MS)
            {
                MotorSample_SetError();
            }
            break;

        case MOTOR_SAMPLE_EXPERIMENT_HOLDING_EXTEND:
            if (MotorSample_IsTargetReached() == 0U)
            {
                /* 停留时若因回弹离开目标，重新进入追踪并重置超时计时。 */
                g_move_started_ms = HAL_GetTick();
                MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_EXTENDING);
            }
            else if (MotorSample_GetElapsedMs(g_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_HOLD_MS)
            {
                next_target = g_experiment.target_length_mm +
                              MOTOR_SAMPLE_EXPERIMENT_STEP_MM;
                if (next_target > MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM)
                {
                    next_target = MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM;
                }
                MotorSample_SetTarget(next_target,
                                      MOTOR_SAMPLE_DIRECTION_EXTEND);
            }
            break;

        case MOTOR_SAMPLE_EXPERIMENT_HOLDING_END:
            if (MotorSample_IsTargetReached() == 0U)
            {
                g_move_started_ms = HAL_GetTick();
                MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_EXTENDING);
            }
            else if (MotorSample_GetElapsedMs(g_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_END_HOLD_MS)
            {
                next_target = MOTOR_SAMPLE_EXPERIMENT_END_LENGTH_MM -
                              MOTOR_SAMPLE_EXPERIMENT_STEP_MM;
                MotorSample_SetTarget(next_target,
                                      MOTOR_SAMPLE_DIRECTION_RETRACT);
            }
            break;

        case MOTOR_SAMPLE_EXPERIMENT_RETRACTING:
            if (MotorSample_IsTargetReached() != 0U)
            {
                if (g_experiment.target_length_mm <=
                    MOTOR_SAMPLE_EXPERIMENT_START_LENGTH_MM)
                {
                    MotorSample_EnterState(
                        MOTOR_SAMPLE_EXPERIMENT_HOLDING_ZERO);
                }
                else
                {
                    MotorSample_EnterState(
                        MOTOR_SAMPLE_EXPERIMENT_HOLDING_RETRACT);
                }
            }
            else if (MotorSample_GetElapsedMs(g_move_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_MOVE_TIMEOUT_MS)
            {
                MotorSample_SetError();
            }
            break;

        case MOTOR_SAMPLE_EXPERIMENT_HOLDING_RETRACT:
            if (MotorSample_IsTargetReached() == 0U)
            {
                g_move_started_ms = HAL_GetTick();
                MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_RETRACTING);
            }
            else if (MotorSample_GetElapsedMs(g_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_HOLD_MS)
            {
                next_target = g_experiment.target_length_mm -
                              MOTOR_SAMPLE_EXPERIMENT_STEP_MM;
                if (next_target < MOTOR_SAMPLE_EXPERIMENT_START_LENGTH_MM)
                {
                    next_target = MOTOR_SAMPLE_EXPERIMENT_START_LENGTH_MM;
                }
                MotorSample_SetTarget(next_target,
                                      MOTOR_SAMPLE_DIRECTION_RETRACT);
            }
            break;

        case MOTOR_SAMPLE_EXPERIMENT_HOLDING_ZERO:
            if (MotorSample_IsTargetReached() == 0U)
            {
                g_move_started_ms = HAL_GetTick();
                MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_RETRACTING);
            }
            else if (MotorSample_GetElapsedMs(g_state_started_ms) >=
                     MOTOR_SAMPLE_EXPERIMENT_HOLD_MS)
            {
                MotorSample_StopAtCurrentPosition();
                g_experiment.running = 0U;
                MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_COMPLETE);
            }
            break;

        case MOTOR_SAMPLE_EXPERIMENT_IDLE:
        case MOTOR_SAMPLE_EXPERIMENT_COMPLETE:
        case MOTOR_SAMPLE_EXPERIMENT_ABORTED:
        case MOTOR_SAMPLE_EXPERIMENT_ERROR:
        default:
            g_experiment.running = 0U;
            break;
    }
}
