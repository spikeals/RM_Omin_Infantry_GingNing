#ifndef OMNI_DRIVERS_ET07_H
#define OMNI_DRIVERS_ET07_H

#include "stm32f4xx_hal.h"

/* ET07当前使用前10个11位模拟通道。 */
#define ET07_CHANNEL_COUNT  (10U)

/**
 * @brief ET07通道数组下标
 *
 * 协议中的CH1对应channel[0]，依次排列到CH10对应channel[9]。
 * 使用枚举访问通道可以避免在应用层直接书写含义不明确的数字下标。
 */
typedef enum
{
    ET07_CHANNEL_1 = 0,
    ET07_CHANNEL_2,
    ET07_CHANNEL_3,
    ET07_CHANNEL_4,
    ET07_CHANNEL_5,
    ET07_CHANNEL_6,
    ET07_CHANNEL_7,
    ET07_CHANNEL_8,
    ET07_CHANNEL_9,
    ET07_CHANNEL_10
} ET07_Channel_t;

/**
 * @brief ET07最近一次有效数据和接收诊断状态
 *
 * channel保存协议解码后的原始11位值，范围为0～2047。
 * 中位校准、死区和归一化属于下一层输入处理，不在协议驱动中完成。
 */
typedef struct
{
    uint16_t channel[ET07_CHANNEL_COUNT];

    uint8_t flags;                  /* ET07帧byte[23]的原始状态位 */
    uint8_t frame_lost;             /* 最新有效格式帧是否报告丢帧 */
    uint8_t failsafe;               /* 最新有效格式帧是否进入失控保护 */
    uint8_t online;                 /* 100 ms内是否收到过有效格式帧 */

    uint32_t last_update_ms;        /* 最近有效帧的系统毫秒时间 */
    uint32_t valid_frame_count;     /* 成功通过帧头、长度和帧尾检查的帧数 */
    uint32_t invalid_frame_count;   /* 收满25字节但帧尾错误的帧数 */
} ET07_Data_t;

/**
 * @brief 初始化ET07及其UART接收
 * @param huart ET07连接的HAL UART句柄
 */
HAL_StatusTypeDef ET07_Init(UART_HandleTypeDef *huart);

/**
 * @brief 处理ET07对应UART的ReceiveToIdle接收事件
 * @param size 本次DMA接收到的有效字节数
 */
void ET07_RxEventHandler(uint16_t size);

/**
 * @brief 处理ET07环形缓冲区中的原始字节
 * @param now_ms 当前系统毫秒时间
 */
void ET07_RunOnce(uint32_t now_ms);

/**
 * @brief 获取ET07最近一次有效数据
 * @return 驱动内部只读数据指针
 *
 * 返回对象由ET07驱动持有，调用者只读取，不负责释放，也不能修改。
 */
const ET07_Data_t *ET07_GetData(void);

#endif
