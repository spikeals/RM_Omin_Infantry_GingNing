#include "omni/utils/dwt.h"
#include <stdint.h>
#include "stm32f4xx.h"

static uint32_t dwt_core_clock_hz = 0;

/// @brief 初始化 DWT 计数器
/// @param core_clock_hz 系统时钟频率
/// @note 该函数只能调用一次，重复调用将被忽略
/// @warning 该函数必须在系统初始化时调用，且只能调用一次
/*  @核心逻辑    
        检查参数
        → 打开 Trace
        → 清零 CYCCNT
        → 启动 CYCCNT
        → 保存 CPU 频率*/
void Dwt_Init(uint32_t core_clock_hz)
{   
    if((dwt_core_clock_hz != 0U) || (core_clock_hz == 0U))//防止重复初始化或传入的时钟频率为0
    {
        return; 
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //使能 DWT
    DWT->CYCCNT = 0; // 重置循环计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能循环计数器
    dwt_core_clock_hz = core_clock_hz;// 保存当前时钟频率
}

/// @brief 获取当前的循环计数器值
/// @return 当前的循环计数器值
uint32_t Dwt_GetCycles(void)
{
    return DWT->CYCCNT;
}

/// @brief 获取经过的循环计数器值
/// @param start_cycles 
/// @return 经过的循环计数器值
uint32_t Dwt_GetElapsedCycles(uint32_t start_cycles)
{
    return DWT->CYCCNT - start_cycles;
}//与start=DWT_GetCycles()配合使用

/// @brief 将循环计数器值转换为微秒
/// @param cycles 
/// @return 转换后的微秒值
uint32_t Dwt_CyclesToUs(uint32_t cycles)
{
    if(dwt_core_clock_hz == 0U)
    {
        return 0U; // 如果未初始化，返回0
    }// 防止除以零
    return (uint32_t)(((uint64_t)cycles * 1000000ULL) / dwt_core_clock_hz);// 使用64位整数进行计算，防止溢出
}

/// @brief 获取经过的微秒数
/// @param start_cycles 
/// @return 经过的微秒数
uint32_t Dwt_GetElapsedUs(uint32_t start_cycles)
{
    return Dwt_CyclesToUs(Dwt_GetElapsedCycles(start_cycles));
}

