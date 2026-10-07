#include "omni/applications/chassis_ctrl.h"

#include "omni/config.h"
#include "omni/algorithms/pid.h"

#include <stddef.h>
#include <string.h>

#define MPS_TO_RPM      \
    (GEAR_RATIO * 60.0f / (2.0f * 3.1415926f * WHEEL_R))
#define INV_SQRT2       (0.7071068f)

static M3508_Bus_t *ctrl_bus;
static M3508_t *ctrl_motor[CHASSIS_WHEEL_NUM];
static Pid_t speed_pid[CHASSIS_WHEEL_NUM];
static ChassisCtrlState_t ctrl_state;

static const float motor_dir[CHASSIS_WHEEL_NUM] =
{
    MOTOR_DIR_FL,
    MOTOR_DIR_FR,
    MOTOR_DIR_RL,
    MOTOR_DIR_RR
};

static int16_t ClampCurrent(float current)
{
    if (current > (float)CURRENT_MAX)
    {
        return CURRENT_MAX;
    }
    if (current < -(float)CURRENT_MAX)
    {
        return -CURRENT_MAX;
    }
    return (int16_t)current;
}

static void RecordSend(HAL_StatusTypeDef status)
{
    ctrl_state.last_send_status = status;
    if (status == HAL_OK)
    {
        ctrl_state.send_ok_count++;
    }
    else if (status == HAL_BUSY)
    {
        ctrl_state.send_busy_count++;
    }
    else
    {
        ctrl_state.send_error_count++;
    }
}

static void ResetPid(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        Pid_Reset(&speed_pid[i]);
    }
}

static void StopOutput(void)
{
    HAL_StatusTypeDef status;
    uint8_t i;

    ctrl_state.enabled = 0U;
    ctrl_state.wheel_scale = 1.0f;
    ResetPid();

    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        ctrl_state.target_rpm[i] = 0.0f;
        ctrl_state.current_raw[i] = 0;
    }

    /* 启动后尚未进入过输出状态时不主动发送控制帧。 */
    if (ctrl_state.output_active == 0U)
    {
        return;
    }

    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        M3508_SetCurrent(ctrl_motor[i], 0);
    }

    status = M3508_SendCurrent(ctrl_bus);
    RecordSend(status);
    if (status == HAL_OK)
    {
        ctrl_state.output_active = 0U;
    }
}

/*
 * 旧全向底盘的四轮关系：
 * FL=(+vx+vy)/sqrt(2)+wR，FR=(+vx-vy)/sqrt(2)+wR
 * RL=(-vx+vy)/sqrt(2)+wR，RR=(-vx-vy)/sqrt(2)+wR
 * 四轮旋转项同号，与旧比赛工程的1/2/3/4号轮解算一致。
 */
static void CalcWheelRpm(const ChassisCommand *cmd,
                         float rpm[CHASSIS_WHEEL_NUM])
{
    float vx;
    float vy;
    float wz;
    float max_rpm;
    float value;
    float scale;
    uint8_t i;

    vx = cmd->vx_mps * INV_SQRT2;
    vy = cmd->vy_mps * INV_SQRT2;
    wz = cmd->wz_radps * ROT_R;

    rpm[CHASSIS_WHEEL_FL] = ( vx + vy + wz) * MPS_TO_RPM;
    rpm[CHASSIS_WHEEL_FR] = ( vx - vy + wz) * MPS_TO_RPM;
    rpm[CHASSIS_WHEEL_RL] = (-vx + vy + wz) * MPS_TO_RPM;
    rpm[CHASSIS_WHEEL_RR] = (-vx - vy + wz) * MPS_TO_RPM;

    max_rpm = 0.0f;
    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        rpm[i] *= motor_dir[i];
        value = (rpm[i] >= 0.0f) ? rpm[i] : -rpm[i];
        if (value > max_rpm)
        {
            max_rpm = value;
        }
    }

    scale = 1.0f;
    if (max_rpm > RPM_MAX)
    {
        scale = RPM_MAX / max_rpm;
        for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
        {
            rpm[i] *= scale;
        }
    }
    ctrl_state.wheel_scale = scale;
}

void ChassisCtrl_Init(M3508_Bus_t *bus,
                      M3508_t motor[CHASSIS_WHEEL_NUM])
{
    PidParam_t param;
    uint8_t i;

    ctrl_bus = bus;
    memset(&ctrl_state, 0, sizeof(ctrl_state));

    param.mode = PID_MODE_NORMAL;
    param.kp = PID_KP;
    param.ki = PID_KI;
    param.kd = PID_KD;
    param.alpha = PID_D_ALPHA;
    param.integral_limit = PID_I_MAX;
    param.output_min = -(float)CURRENT_MAX;
    param.output_max = (float)CURRENT_MAX;

    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        ctrl_motor[i] = &motor[i];
        Pid_Init(&speed_pid[i], &param);
    }

    ctrl_state.wheel_scale = 1.0f;
    ctrl_state.last_send_status = HAL_OK;
    ctrl_state.initialized = 1U;
}

void ChassisCtrl_Update(const ChassisCommand *cmd)
{
    HAL_StatusTypeDef status;
    float current;
    uint8_t i;

    ctrl_state.update_count++;
    ctrl_state.all_online = 1U;
    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        ctrl_state.motor_online[i] =
            M3508_CheckOnline(ctrl_motor[i], MOTOR_TIMEOUT_MS);
        ctrl_state.feedback_rpm[i] = ctrl_motor[i]->speed_rpm;
        if (ctrl_state.motor_online[i] == 0U)
        {
            ctrl_state.all_online = 0U;
        }
    }

    if ((!cmd->enable) || (ctrl_state.all_online == 0U))
    {
        StopOutput();
        return;
    }

    ctrl_state.enabled = 1U;
    CalcWheelRpm(cmd, ctrl_state.target_rpm);

    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        current = Pid_Calculate(&speed_pid[i],
                                ctrl_state.target_rpm[i],
                                ctrl_state.feedback_rpm[i],
                                CTRL_DT);
        ctrl_state.current_raw[i] = ClampCurrent(current);
        M3508_SetCurrent(ctrl_motor[i], ctrl_state.current_raw[i]);
    }

    ctrl_state.output_active = 1U;
    status = M3508_SendCurrent(ctrl_bus);
    RecordSend(status);
}

const ChassisCtrlState_t *ChassisCtrl_GetState(void)
{
    return &ctrl_state;
}

const Pid_t *ChassisCtrl_GetPid(void)
{
    return speed_pid;
}
