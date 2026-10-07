#include "omni/drivers/et07.h"

#include "omni/config.h"
#include "omni/utils/bsp_uart.h"

#include <string.h>

#define ET07_FRAME_LENGTH  (25U)
#define ET07_FRAME_HEADER  (0x0FU)
#define ET07_FRAME_END     (0x00U)

/* SBUS byte[23]中与链路健康相关的状态位。 */
#define ET07_FLAG_FRAME_LOST_MASK  (1U << 2U)
#define ET07_FLAG_FAILSAFE_MASK    (1U << 3U)

/*
 * ET07专用UART对象。
 */
static BSP_UART_t et07_uart;

/* 当前正在组装的25字节原始帧，以及下一字节的写入位置。 */
static uint8_t et07_frame_buffer[ET07_FRAME_LENGTH];
static uint8_t et07_frame_index;

/* 最近一次有效的解码结果以及接收诊断信息。 */
static ET07_Data_t et07_data;

/**
 * @brief 从ET07帧中提取一个11位通道
 * @param frame 完整的25字节ET07帧
 * @param channel_index 通道下标，0对应CH1，9对应CH10
 * @return 通道原始值，范围0～2047
 *
 * 通道数据从frame[1]开始，以低位优先的方式连续排列，每个通道占11位。
 * 一个11位通道可能跨越两个或三个字节，因此先取包含它的连续3字节，
 * 再根据该通道在第一个字节中的起始位右移，最后保留低11位。
 */
static uint16_t ET07_DecodeChannel(const uint8_t *frame,
                                   uint8_t channel_index)
{
    uint16_t bit_offset;
    uint8_t byte_index;
    uint8_t bit_shift;
    uint32_t packed_data;

    /* 当前通道相对于frame[1]起始位置的位偏移。 */
    bit_offset = (uint16_t)channel_index * 11U;

    /* 找到该通道起始位所在的字节，以及它在字节内部的偏移。 */
    byte_index = (uint8_t)(1U + bit_offset / 8U);
    bit_shift = (uint8_t)(bit_offset % 8U);

    /*
     * 按小端顺序把连续3字节拼成一个32位临时数。
     * 转为uint32_t后再移位，避免8位整数在移位过程中丢失高位。
     */
    packed_data =
        (uint32_t)frame[byte_index]
        | ((uint32_t)frame[byte_index + 1U] << 8U)
        | ((uint32_t)frame[byte_index + 2U] << 16U);

    /* 0x07FF的低11位均为1，只保留当前通道的11位有效数据。 */
    return (uint16_t)((packed_data >> bit_shift) & 0x07FFU);
}

/**
 * @brief 解释SBUS状态字节中的链路健康标志
 * @param flags ET07帧byte[23]的原始值
 *
 * SBUS约定bit2表示本帧之前发生过帧丢失，bit3表示接收机已经进入
 * failsafe。这里保留原始flags，同时生成便于上层安全逻辑读取的0/1值。
 */
static void ET07_DecodeFlags(uint8_t flags)
{
    et07_data.frame_lost =
        ((flags & ET07_FLAG_FRAME_LOST_MASK) != 0U) ? 1U : 0U;

    et07_data.failsafe =
        ((flags & ET07_FLAG_FAILSAFE_MASK) != 0U) ? 1U : 0U;
}

/**
 * @brief 解码一帧已经通过帧头和帧尾检查的ET07数据
 * @param frame 完整的25字节ET07帧
 * @param now_ms 当前系统毫秒时间
 */
static void ET07_DecodeFrame(const uint8_t *frame,
                             uint32_t now_ms)
{
    uint8_t channel_index;

    for (channel_index = 0U;
         channel_index < ET07_CHANNEL_COUNT;
         channel_index++)
    {
        et07_data.channel[channel_index] =
            ET07_DecodeChannel(frame, channel_index);
    }

    /* byte[23]保存协议状态位，同时保留原始值和解释后的健康标志。 */
    et07_data.flags = frame[23];
    ET07_DecodeFlags(et07_data.flags);

    /* 只有完整且帧尾正确的帧才能刷新时间戳和有效帧计数。 */
    et07_data.last_update_ms = now_ms;
    et07_data.valid_frame_count++;
}

/**
 * @brief 根据最后一次有效帧时间更新ET07通信在线状态
 * @param now_ms 当前系统毫秒时间
 *
 * online只描述串口链路是否持续收到格式正确的帧。frame_lost和failsafe
 * 仍然单独保存，后续安全门会同时检查这三个状态。
 */
static void ET07_UpdateOnlineState(uint32_t now_ms)
{
    /* 启动后还没有收到任何有效帧时，设备保持离线。 */
    if (et07_data.valid_frame_count == 0U)
    {
        et07_data.online = 0U;
        return;
    }

    /*
     * uint32_t无符号减法能够自然处理HAL_GetTick()回绕。
     * 时间差不超过100 ms时认为通信仍然在线。
     */
    if ((uint32_t)(now_ms - et07_data.last_update_ms) > RC_TIMEOUT_MS)
    {
        et07_data.online = 0U;
    }
    else
    {
        et07_data.online = 1U;
    }
}

HAL_StatusTypeDef ET07_Init(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;

    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    et07_frame_index = 0U;

    /*
     * et07_data中的每个字段都是对外状态，初始化时统一清零，保证首次
     * 有效帧到来之前通道、标志、时间戳和计数器都具有确定值。
     */
    memset(&et07_data, 0, sizeof(et07_data));

    status = BSP_UART_Init(&et07_uart, huart);
    if (status != HAL_OK)
    {
        return status;
    }

    return BSP_UART_StartReceiveToIdle_DMA(&et07_uart);
}

void ET07_RxEventHandler(uint16_t size)
{
    BSP_UART_RxEventHandler(&et07_uart, size);
}

void ET07_RunOnce(uint32_t now_ms)
{
    uint8_t data;

    while (BSP_UART_ReadByte(&et07_uart, &data) != 0U)
    {
        /* 尚未开始收集时，只接受ET07帧头。 */
        if (et07_frame_index == 0U)
        {
            if (data == ET07_FRAME_HEADER)
            {
                et07_frame_buffer[0] = data;
                et07_frame_index = 1U;
            }

            continue;
        }

        /* 已经找到帧头，继续按顺序收集后续字节。 */
        et07_frame_buffer[et07_frame_index] = data;
        et07_frame_index++;

        if (et07_frame_index < ET07_FRAME_LENGTH)
        {
            continue;
        }

        /* 收满25字节后检查帧尾。 */
        if (et07_frame_buffer[ET07_FRAME_LENGTH - 1U]
            == ET07_FRAME_END)
        {
            ET07_DecodeFrame(et07_frame_buffer, now_ms);
        }
        else
        {
            et07_data.invalid_frame_count++;
        }

        /* 若当前字节正好是下一帧帧头，则直接开始收集下一帧。 */
        if (data == ET07_FRAME_HEADER)
        {
            et07_frame_buffer[0] = data;
            et07_frame_index = 1U;
        }
        else
        {
            et07_frame_index = 0U;
        }
    }

    /* 即使本轮没有新字节，也要持续检查100 ms通信超时。 */
    ET07_UpdateOnlineState(now_ms);
}

const ET07_Data_t *ET07_GetData(void)
{
    return &et07_data;
}
