#include "MotorSample.h"

#include "main.h"
#include "m3508_position.h"

#include <stddef.h>

typedef enum
{
    MOTOR_SAMPLE_DIRECTION_EXTEND = 0,
    MOTOR_SAMPLE_DIRECTION_RETRACT
} MotorSampleDirection_t;

static MotorSampleExperimentStatus_t g_experiment;
static uint32_t g_state_started_ms;
static uint32_t g_move_started_ms;

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
}

uint8_t MotorSample_ExperimentHandleCommand(const uint8_t *command,
                                            uint8_t length)
{
    MotorSampleExperimentMode_t mode;

    if ((command == NULL) || (length != 2U))
    {
        return 0U;
    }

    if ((command[0] != (uint8_t)'A') &&
        (command[0] != (uint8_t)'a'))
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

    /* 命令已识别即返回1；若实验正在运行，Start会拒绝重复启动。 */
    (void)MotorSample_ExperimentStart(mode);
    return 1U;
}

uint8_t MotorSample_ExperimentStart(MotorSampleExperimentMode_t mode)
{
    if ((g_experiment.running != 0U) ||
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
    MotorSample_StopAtCurrentPosition();
    g_experiment.running = 0U;
    MotorSample_EnterState(MOTOR_SAMPLE_EXPERIMENT_ABORTED);
}

uint8_t MotorSample_ExperimentIsRunning(void)
{
    return g_experiment.running;
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

void MotorSample_ExperimentTask5ms(void)
{
    uint32_t now;
    float next_target;

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
