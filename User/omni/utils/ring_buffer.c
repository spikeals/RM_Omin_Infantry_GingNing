#include "omni/utils/ring_buffer.h"

/* 计算下一个读写位置，并在到达存储区末尾时回到0。 */
static uint16_t RingBuffer_NextIndex(RingBuffer_t *rb,
                                     uint16_t index)
{
    index++;
    if (index >= rb->size)
    {
        index = 0U;//到达存储区末尾时回到0
    }
    return index;
}//为了贮存256个字节，下标返回值是uint16_t

static uint8_t RingBuffer_IsEmpty(RingBuffer_t *rb)
{
    /* 读写位置相同时，当前没有可读取的数据。 */
    return rb->head == rb->tail;
}

static uint8_t RingBuffer_IsFull(RingBuffer_t *rb)
{
    /* head的下一个位置追上tail时，缓冲区已满。 */
    return RingBuffer_NextIndex(rb, rb->head) == rb->tail;
}

void RingBuffer_Init(RingBuffer_t *rb,
                     uint8_t *buffer,
                     uint16_t size)
{
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0U;
    rb->tail = 0U;
}

uint8_t RingBuffer_Write(RingBuffer_t *rb,
                         uint8_t data)
{
    if (RingBuffer_IsFull(rb))
    {
        /* 不覆盖尚未读取的数据，由调用者决定如何记录或处理丢包。 */
        return 0U;
    }

    /* 先保存数据，再发布新的head位置。 */
    rb->buffer[rb->head] = data;
    rb->head = RingBuffer_NextIndex(rb, rb->head);

    return 1U;
}

uint8_t RingBuffer_Read(RingBuffer_t *rb,
                        uint8_t *data)//这里data是指针，是为了直接改变实参，write里的data是值传递，改变的是ring buffer中的数据
{
    if (RingBuffer_IsEmpty(rb))
    {
        return 0U;
    }

    /* 先取出数据，再推进tail，释放当前位置。 */
    *data = rb->buffer[rb->tail];
    rb->tail = RingBuffer_NextIndex(rb, rb->tail);

    return 1U;
}

//RingBuffer_Write：data是值传递，rb的*buffer = data;
//RingBuffer_Read：*data是指针，  *data = rb的*buffer;

uint16_t RingBuffer_Available(RingBuffer_t *rb)
{
    if (rb->head >= rb->tail)
    {
        /* 数据区域没有跨越数组末尾。 */
        return rb->head - rb->tail;
    }

    /* 数据区域跨越数组末尾，由尾段和头段共同组成。 */
    return rb->size - rb->tail + rb->head;
}

void RingBuffer_Clear(RingBuffer_t *rb)
{
    /* 原始字节无需清零；head与tail相等已经表示逻辑为空。 */
    rb->head = 0U;
    rb->tail = 0U;
}
