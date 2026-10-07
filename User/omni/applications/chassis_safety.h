#ifndef OMNI_APPLICATIONS_CHASSIS_SAFETY_H
#define OMNI_APPLICATIONS_CHASSIS_SAFETY_H

#include "omni/applications/chassis_types.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 底盘安全门状态
 */
typedef enum
{
    /* 启动后或发生通信故障后保持锁定，等待操作者建立安全条件。 */
    CHASSIS_SAFETY_LOCKED = 0,

    /* 安全条件已经满足，正在确认这些条件是否能连续保持200 ms。 */
    CHASSIS_SAFETY_ARMING,

    /* 解锁完成，安全门允许上游命令继续传给后续控制模块。 */
    CHASSIS_SAFETY_READY
} ChassisSafetyState;

/**
 * @brief 初始化底盘安全门
 * @param now_ms 当前系统毫秒时间
 */
void ChassisSafety_Init(uint32_t now_ms);

/**
 * @brief 更新安全状态并生成经过安全门处理的命令
 * @param now_ms 当前系统毫秒时间
 * @param requested_command 输入层请求的底盘命令
 * @param source_healthy 命令源通信和协议状态是否健康
 * @param sticks_centered 与底盘移动有关的摇杆是否位于中位附近
 */
void ChassisSafety_Update(uint32_t now_ms,
                          const ChassisCommand *requested_command,
                          bool source_healthy,
                          bool sticks_centered);

/**
 * @brief 获取当前安全门状态
 */
ChassisSafetyState ChassisSafety_GetState(void);

/**
 * @brief 获取经过安全门处理的最终底盘命令
 * @return 模块内部只读命令指针
 */
const ChassisCommand *ChassisSafety_GetCommand(void);

#endif
