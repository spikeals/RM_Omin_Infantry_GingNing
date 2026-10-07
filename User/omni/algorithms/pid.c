#include "omni/algorithms/pid.h"

#include <stddef.h>
#include <string.h>

void Pid_Reset(Pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->feedforward = 0.0f;
    pid->sp = 0.0f;
    pid->pv = 0.0f;
    pid->co = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_pv = 0.0f;
    pid->last_diff = 0.0f;
    pid->diff = 0.0f;
    pid->error = 0.0f;
}

void Pid_Init(Pid_t *pid,
              const PidParam_t *param)
{
    if ((pid == NULL) || (param == NULL))
    {
        return;
    }

    memset(pid, 0, sizeof(*pid));
    Pid_SetParam(pid, param);
}

void Pid_SetParam(Pid_t *pid,
                  const PidParam_t *param)
{
    if ((pid == NULL) || (param == NULL))
    {
        return;
    }

    pid->param = *param;
    if ((pid->param.mode != PID_MODE_NORMAL)
        && (pid->param.mode != PID_MODE_DIFFERENTIAL))
    {
        pid->param.mode = PID_MODE_NORMAL;
    }
}

void Pid_SetFeedforward(Pid_t *pid,
                        float feedforward)
{
    if (pid != NULL)
    {
        pid->feedforward = feedforward;
    }
}

float Pid_Calculate(Pid_t *pid,
                    float sp,
                    float pv,
                    float dt_s)
{
    float d_out;

    if (pid == NULL)
    {
        return 0.0f;
    }

    if (dt_s <= 0.0f)
    {
        dt_s = 0.000001f;
    }

    pid->sp = sp;
    pid->pv = pv;
    pid->error = sp - pv;

    if (pid->param.mode == PID_MODE_DIFFERENTIAL)
    {
        /* 微分先行使用反馈变化率，输出时取负号。 */
        pid->diff = (pv - pid->last_pv) / dt_s;
        pid->last_pv = pv;
    }
    else
    {
        /* 普通PID使用误差变化率，输出时取正号。 */
        pid->diff = (pid->error - pid->last_error) / dt_s;
        pid->last_error = pid->error;
    }

    if ((pid->param.alpha >= 0.0f)
        && (pid->param.alpha <= 1.0f))
    {
        pid->diff =
            pid->param.alpha * pid->diff
            + (1.0f - pid->param.alpha) * pid->last_diff;
    }
    pid->last_diff = pid->diff;

    d_out = pid->param.kd * pid->diff;
    if (pid->param.mode == PID_MODE_DIFFERENTIAL)
    {
        d_out = -d_out;
    }

    /* 先用当前积分计算未限幅输出，再判断本拍是否继续积分。 */
    pid->co =
        pid->param.kp * pid->error
        + pid->param.ki * pid->integral
        + d_out
        + pid->feedforward;

    if (!(((pid->co >= pid->param.output_max) && (pid->error > 0.0f))
          || ((pid->co <= pid->param.output_min) && (pid->error < 0.0f))))
    {
        pid->integral += pid->error * dt_s;

        if (pid->param.integral_limit > 0.0f)
        {
            if (pid->integral > pid->param.integral_limit)
            {
                pid->integral = pid->param.integral_limit;
            }
            else if (pid->integral < -pid->param.integral_limit)
            {
                pid->integral = -pid->param.integral_limit;
            }
        }
    }

    /* 积分更新后重新计算最终输出。 */
    pid->co =
        pid->param.kp * pid->error
        + pid->param.ki * pid->integral
        + d_out
        + pid->feedforward;

    if (pid->param.output_max > pid->param.output_min)
    {
        if (pid->co > pid->param.output_max)
        {
            pid->co = pid->param.output_max;
        }
        else if (pid->co < pid->param.output_min)
        {
            pid->co = pid->param.output_min;
        }
    }

    return pid->co;
}
