#include "omni/applications/chassis_input.h"

#include "omni/config.h"
#include "omni/drivers/et07.h"

#include <stdbool.h>
#include <stdint.h>

/* SC三档之间的分界点取相邻两个标准位置的中点。 */
#define SC_UP_MID    ((RC_MIN + RC_CENTER) / 2U)
#define SC_MID_DOWN  ((RC_CENTER + RC_MAX) / 2U)

static ChassisCommand chassis_input_command;
static uint32_t chassis_input_last_frame_count;
static bool chassis_input_remote_healthy;
static bool chassis_input_sticks_centered;

/**
 * @brief 把摇杆原始值转换为-1.0到+1.0
 * @param raw_value ET07解码后的11位通道值
 * @return 去除中位死区后的归一化值
 *
 * 中位两侧的实际行程略有不同，因此正负方向分别使用各自的有效行程。
 * 这样两个端点都能准确到达-1.0或+1.0。超出标定端点的值会被限幅。
 */
static float ChassisInput_NormalizeChannel(uint16_t raw_value)
{
    int32_t offset;
    int32_t deadband;
    int32_t positive_range;
    int32_t negative_range;

    offset = (int32_t)raw_value - (int32_t)RC_CENTER;
    deadband = (int32_t)RC_DEADBAND;

    if ((offset >= -deadband) && (offset <= deadband))
    {
        return 0.0f;
    }

    if (offset > deadband)
    {
        positive_range =
            (int32_t)RC_MAX
            - (int32_t)RC_CENTER
            - deadband;

        offset -= deadband;
        if (offset > positive_range)
        {
            offset = positive_range;
        }

        return (float)offset / (float)positive_range;
    }

    negative_range =
        (int32_t)RC_CENTER
        - (int32_t)RC_MIN
        - deadband;

    offset += deadband;
    if (offset < -negative_range)
    {
        offset = -negative_range;
    }

    return (float)offset / (float)negative_range;
}

/**
 * @brief 将SC原始通道值解释为底盘模式
 * @param sc_value ET07的CH7原始值
 * @return SC上档为失能，中档为手动，下档为小陀螺
 */
static ChassisMode ChassisInput_DecodeMode(uint16_t sc_value)
{
    if (sc_value < SC_UP_MID)
    {
        return CHASSIS_DISABLED;
    }

    if (sc_value < SC_MID_DOWN)
    {
        return CHASSIS_MANUAL;
    }

    return CHASSIS_SPIN;
}

/**
 * @brief 生成一条失能命令
 * @param now_ms 失能状态产生时的系统毫秒时间
 */
static void ChassisInput_Disable(uint32_t now_ms)
{
    chassis_input_command.mode = CHASSIS_DISABLED;
    chassis_input_command.enable = false;
    chassis_input_command.vx_mps = 0.0f;
    chassis_input_command.vy_mps = 0.0f;
    chassis_input_command.wz_radps = 0.0f;
    chassis_input_command.timestamp_ms = now_ms;
}

void ChassisInput_Init(uint32_t now_ms)
{
    ChassisInput_Disable(now_ms);
    chassis_input_command.sequence = 0U;
    chassis_input_last_frame_count = 0U;
    chassis_input_remote_healthy = false;
    chassis_input_sticks_centered = false;
}

void ChassisInput_Update(uint32_t now_ms)
{
    const ET07_Data_t *et07_data;
    ChassisMode requested_mode;
    float vx_input;
    float vy_input;
    bool remote_healthy;

    et07_data = ET07_GetData();

    /*
     * online说明有效帧仍在持续到达；frame_lost和failsafe来自遥控协议状态位。
     * 三项同时健康，当前遥控数据才允许产生运动命令。
     */
    remote_healthy =
        (et07_data->online != 0U)
        && (et07_data->frame_lost == 0U)
        && (et07_data->failsafe == 0U);
    chassis_input_remote_healthy = remote_healthy;

    if (!remote_healthy)
    {
        /* 只在状态真正变化时生成新的失能命令，避免序号在空循环中快速增长。 */
        if ((chassis_input_command.enable)
            || (chassis_input_command.mode != CHASSIS_DISABLED))
        {
            ChassisInput_Disable(now_ms);
            chassis_input_command.sequence++;
        }

        chassis_input_last_frame_count = et07_data->valid_frame_count;
        return;
    }

    /* 没有新遥控帧时保留上一条命令，避免重复处理相同数据。 */
    if (et07_data->valid_frame_count == chassis_input_last_frame_count)
    {
        return;
    }
    chassis_input_last_frame_count = et07_data->valid_frame_count;

    requested_mode =
        ChassisInput_DecodeMode(et07_data->channel[ET07_CHANNEL_7]);

    vx_input =
        ChassisInput_NormalizeChannel(et07_data->channel[ET07_CHANNEL_3]);
    vy_input =
        ChassisInput_NormalizeChannel(et07_data->channel[ET07_CHANNEL_4]);

    /*
     * 这里的5%用于启动解锁确认，范围大于日常控制死区。
     * 因此摇杆存在轻微回中误差时仍能安全完成解锁。
     */
    chassis_input_sticks_centered =
        (vx_input >= -ARM_NEUTRAL)
        && (vx_input <= ARM_NEUTRAL)
        && (vy_input >= -ARM_NEUTRAL)
        && (vy_input <= ARM_NEUTRAL);

    chassis_input_command.mode = requested_mode;
    chassis_input_command.timestamp_ms = et07_data->last_update_ms;
    chassis_input_command.sequence++;

    if (requested_mode == CHASSIS_DISABLED)
    {
        chassis_input_command.enable = false;
        chassis_input_command.vx_mps = 0.0f;
        chassis_input_command.vy_mps = 0.0f;
        chassis_input_command.wz_radps = 0.0f;
        return;
    }

    chassis_input_command.enable = true;
    chassis_input_command.vx_mps =
        vx_input * MAX_VX;
    chassis_input_command.vy_mps =
        vy_input * MAX_VY;

    if (requested_mode == CHASSIS_SPIN)
    {
        chassis_input_command.wz_radps = SPIN_WZ;
    }
    else
    {
        chassis_input_command.wz_radps = 0.0f;
    }
}

const ChassisCommand *ChassisInput_GetCommand(void)
{
    return &chassis_input_command;
}

bool ChassisInput_IsRemoteHealthy(void)
{
    return chassis_input_remote_healthy;
}

bool ChassisInput_AreMovementSticksCentered(void)
{
    return chassis_input_sticks_centered;
}
