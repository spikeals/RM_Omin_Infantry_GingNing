#include "omni/applications/chassis_safety.h"

#include "omni/config.h"

#include <stddef.h>

static ChassisSafetyState chassis_safety_state;
static ChassisCommand chassis_safety_command;
static uint32_t chassis_safety_arming_start_ms;

/**
 * @brief 判断当前是否具备进入解锁确认阶段的条件
 *
 * CHASSIS_DISABLED在独立底盘中对应SC上档。结合source_healthy后，
 * 可以把操作者主动选择上档与遥控失联造成的失能区分开。
 */
static bool ChassisSafety_CanArm(const ChassisCommand *requested_command,
                                 bool source_healthy,
                                 bool sticks_centered)
{
    return (requested_command != NULL)
           && source_healthy
           && sticks_centered
           && (requested_command->mode == CHASSIS_DISABLED)
           && (!requested_command->enable);
}

/**
 * @brief 复制上游命令信息，同时强制运动输出归零
 *
 * 保留上游的时间戳和序号，方便后续诊断这条失能结果来自哪一条输入命令。
 */
static void ChassisSafety_ForceDisabled(
    const ChassisCommand *requested_command,
    uint32_t now_ms)
{
    if (requested_command != NULL)
    {
        chassis_safety_command = *requested_command;
    }
    else
    {
        chassis_safety_command.timestamp_ms = now_ms;
        chassis_safety_command.sequence = 0U;
    }

    chassis_safety_command.mode = CHASSIS_DISABLED;
    chassis_safety_command.enable = false;
    chassis_safety_command.vx_mps = 0.0f;
    chassis_safety_command.vy_mps = 0.0f;
    chassis_safety_command.wz_radps = 0.0f;
}

void ChassisSafety_Init(uint32_t now_ms)
{
    chassis_safety_state = CHASSIS_SAFETY_LOCKED;
    chassis_safety_arming_start_ms = now_ms;
    ChassisSafety_ForceDisabled(NULL, now_ms);
}

void ChassisSafety_Update(uint32_t now_ms,
                          const ChassisCommand *requested_command,
                          bool source_healthy,
                          bool sticks_centered)
{
    bool can_arm;

    can_arm = ChassisSafety_CanArm(requested_command,
                                   source_healthy,
                                   sticks_centered);

    switch (chassis_safety_state)
    {
        case CHASSIS_SAFETY_LOCKED:
            ChassisSafety_ForceDisabled(requested_command, now_ms);

            if (can_arm)
            {
                chassis_safety_arming_start_ms = now_ms;
                chassis_safety_state = CHASSIS_SAFETY_ARMING;
            }
            break;

        case CHASSIS_SAFETY_ARMING:
            ChassisSafety_ForceDisabled(requested_command, now_ms);

            if (!can_arm)
            {
                chassis_safety_state = CHASSIS_SAFETY_LOCKED;
            }
            else if ((uint32_t)(now_ms - chassis_safety_arming_start_ms)
                     >= ARM_TIME_MS)
            {
                chassis_safety_state = CHASSIS_SAFETY_READY;
            }
            break;

        case CHASSIS_SAFETY_READY:
            if ((!source_healthy) || (requested_command == NULL))
            {
                ChassisSafety_ForceDisabled(requested_command, now_ms);
                chassis_safety_state = CHASSIS_SAFETY_LOCKED;
            }
            else
            {
                /*
                 * READY只代表安全门已经解锁。若SC回到上档，上游命令本身
                 * 仍是失能命令，因此这里复制后会立即得到零输出。
                 */
                chassis_safety_command = *requested_command;
            }
            break;

        default:
            ChassisSafety_ForceDisabled(requested_command, now_ms);
            chassis_safety_state = CHASSIS_SAFETY_LOCKED;
            break;
    }
}

ChassisSafetyState ChassisSafety_GetState(void)
{
    return chassis_safety_state;
}

const ChassisCommand *ChassisSafety_GetCommand(void)
{
    return &chassis_safety_command;
}
