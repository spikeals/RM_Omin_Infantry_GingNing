#ifndef DWT_H
#define DWT_H

#include <stdint.h>

/// @brief 初始化 DWT 计数器
/// @param core_clock_hz 系统时钟频率
void Dwt_Init(uint32_t core_clock_hz);

/// @brief 获取当前的循环计数器值
/// @return 当前的循环计数器值，单位为cycles
uint32_t Dwt_GetCycles(void);

/// @brief 获取经过的循环计数器值
/// @param start_cycles 开始时的循环计数器值
/// @return 经过的循环计数器值,单位为cycles
uint32_t Dwt_GetElapsedCycles(uint32_t start_cycles);

/// @brief 将循环计数器值转换为微秒
/// @param cycles 循环计数器值
/// @return 转换后的微秒值
uint32_t Dwt_CyclesToUs(uint32_t cycles);

/// @brief 获取经过的微秒数
/// @param start_cycles 开始时的循环计数器值
/// @return 经过的微秒数
uint32_t Dwt_GetElapsedUs(uint32_t start_cycles);

#endif // DWT_H
