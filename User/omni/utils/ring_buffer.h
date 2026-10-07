#ifndef OMNI_UTILS_RING_BUFFER_H
#define OMNI_UTILS_RING_BUFFER_H

#include <stdint.h>

/**
 * @brief 单进、单出字节环形缓冲区
 *
 * 推荐用法：UART接收中断写入，主循环读取。
 * head指向下一次写入位置，tail指向下一次读取位置。
 * 为了区分空和满，缓冲区会保留一个空位置，实际容量为size - 1。
 */
typedef struct
{
    uint8_t *buffer;          /* 外部提供的字节存储区指针 */
    uint16_t size;            /* 存储区总大小，实际容量为size - 1 *///注意，这里是大小，不是索引值
    volatile uint16_t head;   /* 下一次写入的位置 */
    volatile uint16_t tail;   /* 下一次读取的位置 */
} RingBuffer_t;

/**
 * @brief 初始化环形缓冲区
 * @param rb 环形缓冲区实例
 * @param buffer 外部提供的存储区指针
 * @param size 存储区大小，必须大于1
 */
void RingBuffer_Init(RingBuffer_t *rb,
                     uint8_t *buffer,
                     uint16_t size);

/**
 * @brief 写入一个字节
 * @return 1写入成功，0表示缓冲区已满
 */
uint8_t RingBuffer_Write(RingBuffer_t *rb,
                         uint8_t data);

/**
 * @brief 读取一个字节
 * @return 1读取成功，0表示缓冲区为空
 */
uint8_t RingBuffer_Read(RingBuffer_t *rb,
                        uint8_t *data);

/**
 * @brief 获取当前可读取的字节数
 */
uint16_t RingBuffer_Available(RingBuffer_t *rb);

/**
 * @brief 清空环形缓冲区
 * @note 只重置head和tail，不清零buffer中的原始数据。
 *       应在数据生产停止后调用，例如DMA接收启动前或停止后。
 */
void RingBuffer_Clear(RingBuffer_t *rb);

#endif
