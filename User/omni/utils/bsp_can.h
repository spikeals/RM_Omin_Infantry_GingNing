#ifndef OMNI_UTILS_BSP_CAN_H
#define OMNI_UTILS_BSP_CAN_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

/* STM32F407的CAN1和CAN2共享28个过滤器组，当前固定在Bank 14处分区。 */
#define BSP_CAN_MAX_RX_INSTANCES   (28U)
#define BSP_CAN_FILTER_SPLIT_BANK  (14U)
#define BSP_CAN_FILTER_BANK_COUNT  (28U)

typedef struct BSP_CAN_RxInstance BSP_CAN_RxInstance_t;

/**
 * @brief 某个标准ID收到新帧后执行的设备回调
 * @note 回调运行在CAN接收中断上下文，必须保持短小、确定且不阻塞。
 */
typedef void (*BSP_CAN_RxCallback_t)(BSP_CAN_RxInstance_t *instance);

/**
 * @brief 一种固定标准ID和DLC的CAN发送实例
 *
 * 设备驱动先在data中组织完整报文，再调用BSP_CAN_Transmit。HAL会在
 * 函数返回前把数据复制到bxCAN硬件邮箱，因此data可以长期复用。
 */
typedef struct
{
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t mailbox;

    /* 发送诊断。HAL_OK表示成功放入硬件邮箱，不表示总线已完成发送。 */
    uint32_t transmit_count;
    uint32_t transmit_ok_count;
    uint32_t transmit_busy_count;
    uint32_t transmit_error_count;
    uint32_t last_error_code;
    HAL_StatusTypeDef last_status;
    uint8_t initialized;
} BSP_CAN_TxInstance_t;

/**
 * @brief 一个标准ID的接收注册项
 *
 * 每个注册项拥有一个精确硬件过滤器。收到匹配报文后，BSP先更新header、
 * data和时间诊断，再调用callback。owner用于把通用CAN实例关联回具体设备对象。
 */
struct BSP_CAN_RxInstance
{
    CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint32_t id;
    uint32_t fifo;
    uint8_t data[8];
    uint8_t length;
    void *owner;
    BSP_CAN_RxCallback_t callback;

    /* 以下字段由CAN中断更新，主循环或调试器可以只读观察。 */
    volatile uint32_t receive_count;
    volatile uint32_t last_receive_ms;
    volatile uint32_t receive_interval_ms;
    volatile uint8_t registered;
};

/**
 * @brief 初始化一个固定ID、固定长度的发送实例
 */
HAL_StatusTypeDef BSP_CAN_TxInit(BSP_CAN_TxInstance_t *instance,
                                 CAN_HandleTypeDef *hcan,
                                 uint32_t standard_id,
                                 uint8_t length);

/**
 * @brief 注册一个标准数据帧接收实例并配置精确硬件过滤器
 * @param fifo CAN_RX_FIFO0或CAN_RX_FIFO1
 * @param owner 设备对象地址，由回调解释；BSP不访问其内部内容
 */
HAL_StatusTypeDef BSP_CAN_RxRegister(BSP_CAN_RxInstance_t *instance,
                                     CAN_HandleTypeDef *hcan,
                                     uint32_t standard_id,
                                     uint32_t fifo,
                                     void *owner,
                                     BSP_CAN_RxCallback_t callback);

/**
 * @brief 启动CAN并打开已经注册使用的FIFO消息挂起通知
 * @note 同一路CAN的所有接收实例应在调用本函数前完成注册。
 */
HAL_StatusTypeDef BSP_CAN_Start(CAN_HandleTypeDef *hcan);

/**
 * @brief 尝试把发送实例中的报文放入一个空闲硬件邮箱
 * @return HAL_BUSY表示三个邮箱均被占用；函数不会等待邮箱释放。
 */
HAL_StatusTypeDef BSP_CAN_Transmit(BSP_CAN_TxInstance_t *instance);

#endif
