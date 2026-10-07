#include "omni/utils/bsp_uart.h"

#include <string.h>

/*
 * BSP UART模块负责UART原始字节的非阻塞收发，不负责具体设备协议解析。
 *
 * 接收数据流：
 * UART外设 -> DMA临时缓冲区(doubled buffer) -> 环形缓冲区(ring buffer) -> 主循环中的设备驱动。
 *
 * DMA和UART中断只完成缓冲区切换、重新启动接收和原始字节转存。
 * ET07、裁判系统等协议的组帧与解析应在主循环中完成。
 * 具体UART与设备驱动的对应关系由板级入口使用明确的huart判断完成。
 */

/**
 * @brief 初始化BSP UART对象并绑定HAL UART句柄
 * @param uart BSP UART对象，其存储空间必须在DMA工作期间持续有效
 * @param huart CubeMX生成的HAL UART句柄
 */
HAL_StatusTypeDef BSP_UART_Init(BSP_UART_t *uart,
                                UART_HandleTypeDef *huart)//初始化函数初始化所有“会在赋值前被读取”的变量即可
{
    if ((uart == NULL) || (huart == NULL))
    {
        return HAL_ERROR;
    }
    uart->huart = huart;
    uart->rx_dma_index = 0U;
    uart->rx_overflow_count = 0U;
    uart->rx_restart_error_count = 0U;

    /* 建立接收环形缓冲区。*/
    RingBuffer_Init(&uart->rx_ring,
                    uart->rx_ring_storage,
                    BSP_UART_RX_RING_BUF_SIZE);

    return HAL_OK;
}

/**
 * @brief 使用DMA启动一次非阻塞发送
 * @param uart BSP UART对象
 * @param data 待发送数据
 * @param len 待发送字节数
 * @return HAL_OK已启动，HAL_BUSY上一次发送未结束，HAL_ERROR参数错误
 *
 * 数据会先复制到对象内部的tx_buf。DMA启动后，即使调用者原来的
 * data位于局部数组中，DMA仍然拥有稳定的数据源。
 */
HAL_StatusTypeDef BSP_UART_Transmit_DMA(BSP_UART_t *uart,
                                        const uint8_t *data,
                                        uint16_t len)
{
    if ((uart == NULL)
        || (uart->huart == NULL)
        || (uart->huart->hdmatx == NULL)
        || (data == NULL)
        || (len == 0U)
        || (len > (uint16_t)sizeof(uart->tx_buf)))
    {
        return HAL_ERROR;
    }

    /* HAL使用gState记录发送状态，同一UART当前只允许一包在途数据。 */
    if (uart->huart->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    memcpy(uart->tx_buf, data, len);

    return HAL_UART_Transmit_DMA(uart->huart,
                                 uart->tx_buf,
                                 len);
}

/**
 * @brief 使用当前A/B缓冲区启动ReceiveToIdle DMA接收
 * @param uart BSP UART对象
 * @return HAL UART启动结果
 *
 * rx_dma_index只在初始化和接收事件处理中修改。本函数读取当前索引，
 * 不将索引重新置零，因此能够保持A/B缓冲区交替工作。
 */
HAL_StatusTypeDef BSP_UART_StartReceiveToIdle_DMA(BSP_UART_t *uart)
{
    HAL_StatusTypeDef status;
    
    if ((uart == NULL)
        || (uart->huart == NULL)
        || (uart->huart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        uart->huart,
        uart->rx_dma_buf[uart->rx_dma_index],
        BSP_UART_RX_DMA_BUF_SIZE);

    if (status == HAL_OK)
    {
        /*
         * 关闭DMA半传输中断。
         * 当前设计只保留两种接收事件：
         * 1. UART总线空闲，产生IDLE事件；
         * 2. DMA缓冲区接满，产生TC事件。
         *
         * HAL_UARTEx_ReceiveToIdle_DMA会重新打开DMA中断，因此必须在
         * HAL启动成功之后关闭HT中断。
         */
        __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
    }

    return status;
}

/**
 * @brief 处理一次HAL UART接收事件
 * @param uart 发生接收事件的BSP UART对象
 * @param size 本次DMA缓冲区中的有效字节数
 *
 * 本函数运行在UART或DMA中断上下文中。处理顺序为：
 * 1. 记住已经接收完成的缓冲区；
 * 2. 切换到另一块缓冲区并立即重新启动DMA；
 * 3. 将完成缓冲区中的字节写入环形缓冲区。
 *
 * 重新启动DMA放在数据复制之前，可以缩短串口接收空窗。新的DMA写入
 * 另一块缓冲区，因此不会与CPU当前复制的数据发生冲突。
 */
/*size来源：DMA硬件统计已接收数量
    → HAL计算size
    → HAL_UARTEx_RxEventCallback(huart, size)
    → ET07_RxEventHandler(size)(或者其他设备的RxEventHandler)
    → BSP_UART_RxEventHandler(&et07_uart, size)*/
void BSP_UART_RxEventHandler(BSP_UART_t *uart,
                             uint16_t size)
{
    uint8_t completed_index;
    uint16_t index;

    if ((uart == NULL)
        || (uart->huart == NULL)
        || (size == 0U)
        || (size > BSP_UART_RX_DMA_BUF_SIZE))
    {
        return;
    }

    /* 保存刚接收完成的缓冲区编号，后面从该缓冲区复制数据。 */
    completed_index = uart->rx_dma_index;

    /* 异或1使索引在0和1之间切换：0^1=1，1^1=0。 */
    uart->rx_dma_index ^= 1U;

    /* 先让另一块DMA缓冲区恢复接收，再处理上一块缓冲区。 */
    if (BSP_UART_StartReceiveToIdle_DMA(uart) != HAL_OK)
    {
        uart->rx_restart_error_count++;
    }

    /*
     * 将刚收到的数据转移到环形缓冲区。
     * 后续协议解析由主循环完成。
     */
    for (index = 0U; index < size; index++)
    {
        if (RingBuffer_Write(
                &uart->rx_ring,
                uart->rx_dma_buf[completed_index][index]) == 0U)
        {
            /* 环形缓冲区已满时保留旧数据，并统计本次丢失的字节。 */
            uart->rx_overflow_count++;
        }
    }
}

/**
 * @brief 从接收环形缓冲区读取一个字节
 * @param uart BSP UART对象
 * @param data 用于接收字节的输出地址
 * @return 1读取成功，0表示参数错误或当前没有数据
 *
 * 该接口预期由主循环中的设备驱动调用。
 */
uint8_t BSP_UART_ReadByte(BSP_UART_t *uart,
                          uint8_t *data)
{
    if ((uart == NULL) || (data == NULL))
    {
        return 0U;
    }

    return RingBuffer_Read(&uart->rx_ring, data);
}

/**
 * @brief 获取环形缓冲区当前可读取的字节数
 * @return 可读字节数，参数错误时返回0
 */
uint16_t BSP_UART_GetRxDataLength(BSP_UART_t *uart)
{
    if (uart == NULL)
    {
        return 0U;
    }

    return RingBuffer_Available(&uart->rx_ring);
}
