#ifndef OMNI_APPLICATIONS_CHASSIS_CTRL_H
#define OMNI_APPLICATIONS_CHASSIS_CTRL_H

#include "omni/algorithms/pid.h"
#include "omni/applications/chassis_types.h"
#include "omni/drivers/motor_m3508.h"

#include <stdint.h>

#define CHASSIS_WHEEL_NUM  (4U)

typedef enum
{
    CHASSIS_WHEEL_FL = 0,
    CHASSIS_WHEEL_FR,
    CHASSIS_WHEEL_RL,
    CHASSIS_WHEEL_RR
} ChassisWheel;

/** 四轮控制状态，可直接加入Keil Watch观察。 */
typedef struct
{
    float target_rpm[CHASSIS_WHEEL_NUM];
    float feedback_rpm[CHASSIS_WHEEL_NUM];
    int16_t current_raw[CHASSIS_WHEEL_NUM];
    uint8_t motor_online[CHASSIS_WHEEL_NUM];

    float wheel_scale;
    uint32_t update_count;
    uint32_t send_ok_count;
    uint32_t send_busy_count;
    uint32_t send_error_count;
    HAL_StatusTypeDef last_send_status;

    uint8_t enabled;
    uint8_t all_online;
    uint8_t output_active;
    uint8_t initialized;
} ChassisCtrlState_t;

/** 绑定CAN发送组和四个物理车轮，初始化四个独立速度PID。 */
void ChassisCtrl_Init(M3508_Bus_t *bus,
                      M3508_t motor[CHASSIS_WHEEL_NUM]);

/** 500 Hz控制入口：运动学解算、四轮速度环和0x200电流帧发送。 */
void ChassisCtrl_Update(const ChassisCommand *cmd);

/** 返回只读调试状态。 */
const ChassisCtrlState_t *ChassisCtrl_GetState(void);

/** 返回四个轮速PID，顺序为FL、FR、RL、RR。 */
const Pid_t *ChassisCtrl_GetPid(void);

#endif
