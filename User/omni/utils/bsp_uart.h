#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32f4xx_hal.h"
#include "omni/utils/ring_buffer.h"

#include <stdint.h>

#define BSP_UART_TX_BUFFER_SIZE  (128U)// DMA发送缓冲区大小
#define BSP_UART_RX_DMA_BUF_SIZE  (64U)// DMA接收缓冲区大小
#define BSP_UART_RX_RING_BUF_SIZE  (256U)// 环形缓冲区大小

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t tx_buf[BSP_UART_TX_BUFFER_SIZE];

    uint8_t rx_dma_buf[2][BSP_UART_RX_DMA_BUF_SIZE];// DMA接收双缓冲区
    uint8_t rx_dma_index;// 当前DMA接收使用的缓冲区索引，0或1

    uint8_t rx_ring_storage[BSP_UART_RX_RING_BUF_SIZE];//主循环读取的数据存储区
    RingBuffer_t rx_ring;//管理head、tail和可用数据量

    /*两个诊断计数*/
    volatile uint32_t rx_overflow_count;//DMA接收时环形缓冲区满了的次数
    volatile uint32_t rx_restart_error_count;//DMA接收重启失败的次数
} BSP_UART_t;

/**
 * @brief 绑定不同UART实例的HAL UART句柄
 * @param uart   BSP_UART实例
 * @param handle HAL UART句柄
 */
HAL_StatusTypeDef BSP_UART_Init(BSP_UART_t *uart,UART_HandleTypeDef *huart);
/**
 * @brief 通过DMA发送数据
 * @param uart BSP_UART实例
 * @param data 待发送数据
 * @param len  待发送数据长度
 */
HAL_StatusTypeDef BSP_UART_Transmit_DMA(BSP_UART_t *uart,const uint8_t *data,uint16_t len);

/**
 * @brief 启动DMA接收，接收完成后会触发空闲中断
 * @param uart BSP_UART实例
 */
HAL_StatusTypeDef BSP_UART_StartReceiveToIdle_DMA(BSP_UART_t *uart);

/**
 * @brief UART接收事件处理函数
 * @param uart BSP_UART实例
 * @param size 接收到的数据长度
 */
void BSP_UART_RxEventHandler(BSP_UART_t *uart,uint16_t size);

/**
 * @brief 从UART接收缓冲区读取一个字节
 * @param uart BSP_UART实例
 * @param data 读取到的数据
 * @return 1表示成功，0表示失败
 */
uint8_t BSP_UART_ReadByte(BSP_UART_t *uart,uint8_t *data);

/**
 * @brief 获取UART接收缓冲区中可读取的数据长度
 * @param uart BSP_UART实例
 * @return 可读取的数据长度
 */
uint16_t BSP_UART_GetRxDataLength(BSP_UART_t *uart);



#endif // BSP_UART_H
