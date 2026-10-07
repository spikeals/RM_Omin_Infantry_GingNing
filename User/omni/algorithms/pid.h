#ifndef OMNI_ALGORITHMS_PID_H
#define OMNI_ALGORITHMS_PID_H

#include <stdint.h>

/* 普通PID：微分作用于误差。 */
#define PID_MODE_NORMAL        (0U)

/* 微分先行：微分作用于反馈量，避免目标突变造成微分冲击。 */
#define PID_MODE_DIFFERENTIAL  (1U)

/** PID参数，输出使用独立的最小值和最大值限幅。 */
typedef struct
{
    uint8_t mode;
    float kp;
    float ki;
    float kd;
    float alpha;
    float integral_limit;
    float output_min;
    float output_max;
} PidParam_t;

/** PID运行状态，可直接在Keil Debug中观察。 */
typedef struct
{
    PidParam_t param;

    float feedforward;
    float sp;
    float pv;
    float co;

    float integral;
    float last_error;
    float last_pv;
    float last_diff;
    float diff;
    float error;
} Pid_t;

/** 使用调用者提供的静态对象初始化PID，不申请动态内存。 */
void Pid_Init(Pid_t *pid,
              const PidParam_t *param);

/** 更新PID参数，保留当前运行状态。 */
void Pid_SetParam(Pid_t *pid,
                  const PidParam_t *param);

/** 清除积分、微分、误差和输出，保留PID参数。 */
void Pid_Reset(Pid_t *pid);

/** 设置前馈输出。 */
void Pid_SetFeedforward(Pid_t *pid,
                        float feedforward);

/** 按显式dt计算一次PID；输出继续饱和时暂停同方向积分。 */
float Pid_Calculate(Pid_t *pid,
                    float sp,
                    float pv,
                    float dt_s);

#endif
