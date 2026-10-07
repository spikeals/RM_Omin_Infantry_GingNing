#ifndef OMNI_APPLICATIONS_CHASSIS_INPUT_H
#define OMNI_APPLICATIONS_CHASSIS_INPUT_H

#include "omni/applications/chassis_types.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化独立底盘的ET07输入转换模块
 * @param now_ms 当前系统毫秒时间
 *
 * 初始化后输出命令处于CHASSIS_DISABLED状态。只有收到健康的ET07数据，
 * 并且SC不在上档时，模块才会生成允许底盘运动的命令。
 */
void ChassisInput_Init(uint32_t now_ms);

/**
 * @brief 根据最新ET07数据更新底盘命令
 * @param now_ms 当前系统毫秒时间
 *
 * 该函数由500 Hz底盘控制节拍调用。它只负责输入解释，不负责启动解锁、
 * 电机故障和急停等最终安全判断；这些条件将在SafetyGate中统一处理。
 */
void ChassisInput_Update(uint32_t now_ms);

/**
 * @brief 获取最近生成的底盘命令
 * @return 模块内部只读命令指针
 */
const ChassisCommand *ChassisInput_GetCommand(void);

/**
 * @brief 查询ET07当前是否可以作为有效命令源
 * @return true表示在线、无丢帧标志且未进入failsafe
 */
bool ChassisInput_IsRemoteHealthy(void);

/**
 * @brief 查询底盘平移摇杆是否位于中位附近
 * @return true表示CH3和CH4都处于各自行程的5%中位范围内
 */
bool ChassisInput_AreMovementSticksCentered(void);

#endif
