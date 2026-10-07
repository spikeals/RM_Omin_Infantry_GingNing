#ifndef OMNI_DEBUG_H
#define OMNI_DEBUG_H

#include "omni/algorithms/pid.h"
#include "omni/applications/chassis_ctrl.h"
#include "omni/applications/chassis_safety.h"
#include "omni/applications/chassis_types.h"
#include "omni/drivers/et07.h"
#include "omni/drivers/motor_m3508.h"

#include <stdint.h>

/*
 * Watch调试总览
 *
 * 在Watch1中添加debug即可。带指针的成员直接指向各模块的真实状态，
 * 不额外复制电机、PID和遥控数据。
 * motor、pid和ctrl内的四轮顺序统一为FL、FR、RL、RR。
 */
typedef struct
{
    const ET07_Data_t *rc;                 /* 遥控原始通道与接收状态 */
    const ChassisCommand *input;           /* 遥控转换后的请求命令 */
    const ChassisCommand *cmd;             /* 经过安全门的最终命令 */
    const ChassisCtrlState_t *ctrl;        /* 四轮目标、反馈、电流和发送状态 */
    const Pid_t *pid;                      /* 四个轮速PID */
    const M3508_t *motor;                  /* 四个M3508原始反馈对象 */
    const M3508_Bus_t *bus;                /* CAN电流缓存与发送对象 */

    ChassisSafetyState safety;             /* 0锁定，1解锁确认，2已就绪 */
    uint8_t remote_ok;                     /* 遥控在线且无丢帧/失控标志 */
    uint8_t sticks_center;                 /* CH3和CH4位于解锁中位范围 */

    uint32_t now_ms;                       /* 最近一次调试状态刷新时间 */
    uint32_t loop_count;                   /* 主循环累计次数 */
    uint32_t control_count;                /* 500 Hz控制累计次数 */
} Debug_t;

extern Debug_t debug;

/** 绑定各模块的真实状态对象。 */
void Debug_Init(const M3508_Bus_t *bus,
                const M3508_t motor[CHASSIS_WHEEL_NUM]);

/** 每个控制周期刷新安全状态和调度计数。 */
void Debug_Update(uint32_t now_ms,
                  uint32_t loop_count,
                  uint32_t control_count);

#endif
