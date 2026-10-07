#include "omni/utils/bsp_can.h"

#include <stddef.h>
#include <string.h>

#define BSP_CAN_FIFO0_USED  (1U << 0U)
#define BSP_CAN_FIFO1_USED  (1U << 1U)

/*
 * 注册表只在系统初始化阶段写入，CAN启动后保持不变。
 * 接收中断通过CAN实例、FIFO和标准ID查找对应的设备回调。
 */
static BSP_CAN_RxInstance_t *bsp_can_rx_instances[BSP_CAN_MAX_RX_INSTANCES];
static uint8_t bsp_can_rx_count;

/* CAN1使用Bank 0～13，CAN2使用Bank 14～27。 */
static uint8_t bsp_can1_filter_bank;
static uint8_t bsp_can2_filter_bank = BSP_CAN_FILTER_SPLIT_BANK;

static uint8_t bsp_can1_fifo_usage;
static uint8_t bsp_can2_fifo_usage;
static uint8_t bsp_can1_started;
static uint8_t bsp_can2_started;

/**
 * @brief 判断句柄是否对应本模块支持的bxCAN外设
 */
static uint8_t BSP_CAN_IsSupported(const CAN_HandleTypeDef *hcan)
{
    return (uint8_t)((hcan != NULL)
        && ((hcan->Instance == CAN1) || (hcan->Instance == CAN2)));
}

/**
 * @brief 比较两个HAL句柄是否指向同一个CAN硬件实例
 */
static uint8_t BSP_CAN_IsSameInstance(const CAN_HandleTypeDef *first,
                                      const CAN_HandleTypeDef *second)
{
    return (uint8_t)((first != NULL)
        && (second != NULL)
        && (first->Instance == second->Instance));
}

/**
 * @brief 为一个标准ID配置16位列表模式精确过滤器
 *
 * 标准ID在16位过滤器半字中左移5位。一个Bank具有四个16位列表槽位，
 * 当前把四个槽位都填成同一个ID。该做法占用资源较多，但注册关系直观，
 * 每路CAN最多14个ID，足以覆盖当前四个底盘电机。
 */
static HAL_StatusTypeDef BSP_CAN_AddStandardFilter(
    BSP_CAN_RxInstance_t *instance)
{
    CAN_FilterTypeDef filter;
    uint16_t shifted_id;
    uint8_t filter_bank;
    HAL_StatusTypeDef status;

    if ((instance == NULL) || (BSP_CAN_IsSupported(instance->hcan) == 0U))
    {
        return HAL_ERROR;
    }

    if (instance->hcan->Instance == CAN1)
    {
        if (bsp_can1_filter_bank >= BSP_CAN_FILTER_SPLIT_BANK)
        {
            return HAL_ERROR;
        }
        filter_bank = bsp_can1_filter_bank;
    }
    else
    {
        if (bsp_can2_filter_bank >= BSP_CAN_FILTER_BANK_COUNT)
        {
            return HAL_ERROR;
        }
        filter_bank = bsp_can2_filter_bank;
    }

    memset(&filter, 0, sizeof(filter));
    shifted_id = (uint16_t)(instance->id << 5U);

    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = shifted_id;
    filter.FilterIdLow = shifted_id;
    filter.FilterMaskIdHigh = shifted_id;
    filter.FilterMaskIdLow = shifted_id;
    filter.FilterFIFOAssignment =
        (instance->fifo == CAN_RX_FIFO1)
        ? CAN_FILTER_FIFO1
        : CAN_FILTER_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = BSP_CAN_FILTER_SPLIT_BANK;

    status = HAL_CAN_ConfigFilter(instance->hcan, &filter);
    if (status == HAL_OK)
    {
        /* 只在HAL配置成功后消耗Bank，失败后仍允许调用者重试。 */
        if (instance->hcan->Instance == CAN1)
        {
            bsp_can1_filter_bank++;
        }
        else
        {
            bsp_can2_filter_bank++;
        }
    }

    return status;
}

HAL_StatusTypeDef BSP_CAN_TxInit(BSP_CAN_TxInstance_t *instance,
                                 CAN_HandleTypeDef *hcan,
                                 uint32_t standard_id,
                                 uint8_t length)
{
    if ((instance == NULL)
        || (BSP_CAN_IsSupported(hcan) == 0U)
        || (standard_id > 0x07FFU)
        || (length > 8U))
    {
        return HAL_ERROR;
    }

    memset(instance, 0, sizeof(*instance));
    instance->hcan = hcan;
    instance->header.StdId = standard_id;
    instance->header.ExtId = 0U;
    instance->header.IDE = CAN_ID_STD;
    instance->header.RTR = CAN_RTR_DATA;
    instance->header.DLC = length;
    instance->header.TransmitGlobalTime = DISABLE;
    instance->last_status = HAL_OK;
    instance->initialized = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef BSP_CAN_RxRegister(BSP_CAN_RxInstance_t *instance,
                                     CAN_HandleTypeDef *hcan,
                                     uint32_t standard_id,
                                     uint32_t fifo,
                                     void *owner,
                                     BSP_CAN_RxCallback_t callback)
{
    uint8_t index;

    if ((instance == NULL)
        || (BSP_CAN_IsSupported(hcan) == 0U)
        || (standard_id > 0x07FFU)
        || ((fifo != CAN_RX_FIFO0) && (fifo != CAN_RX_FIFO1))
        || (callback == NULL)
        || (bsp_can_rx_count >= BSP_CAN_MAX_RX_INSTANCES))
    {
        return HAL_ERROR;
    }

    /* 同一路CAN上的同一个标准ID只能有一个所有者。 */
    for (index = 0U; index < bsp_can_rx_count; index++)
    {
        if ((BSP_CAN_IsSameInstance(
                bsp_can_rx_instances[index]->hcan,
                hcan) != 0U)
            && (bsp_can_rx_instances[index]->id == standard_id))
        {
            return HAL_ERROR;
        }
    }

    memset(instance, 0, sizeof(*instance));
    instance->hcan = hcan;
    instance->id = standard_id;
    instance->fifo = fifo;
    instance->owner = owner;
    instance->callback = callback;

    if (BSP_CAN_AddStandardFilter(instance) != HAL_OK)
    {
        return HAL_ERROR;
    }

    instance->registered = 1U;
    bsp_can_rx_instances[bsp_can_rx_count] = instance;
    bsp_can_rx_count++;

    /* 记录实际使用的FIFO，启动时只开启需要的通知。 */
    if (hcan->Instance == CAN1)
    {
        bsp_can1_fifo_usage |=
            (fifo == CAN_RX_FIFO1) ? BSP_CAN_FIFO1_USED : BSP_CAN_FIFO0_USED;
    }
    else
    {
        bsp_can2_fifo_usage |=
            (fifo == CAN_RX_FIFO1) ? BSP_CAN_FIFO1_USED : BSP_CAN_FIFO0_USED;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BSP_CAN_Start(CAN_HandleTypeDef *hcan)
{
    uint8_t *started;
    uint8_t fifo_usage;
    uint32_t notifications;
    HAL_StatusTypeDef status;

    if (BSP_CAN_IsSupported(hcan) == 0U)
    {
        return HAL_ERROR;
    }

    if (hcan->Instance == CAN1)
    {
        started = &bsp_can1_started;
        fifo_usage = bsp_can1_fifo_usage;
    }
    else
    {
        started = &bsp_can2_started;
        fifo_usage = bsp_can2_fifo_usage;
    }

    if (*started != 0U)
    {
        return HAL_OK;
    }

    /* 至少注册一个接收ID后再启动，保证第一帧到来时过滤器已经生效。 */
    if (fifo_usage == 0U)
    {
        return HAL_ERROR;
    }

    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK)
    {
        return status;
    }

    notifications = 0U;
    if ((fifo_usage & BSP_CAN_FIFO0_USED) != 0U)
    {
        notifications |= CAN_IT_RX_FIFO0_MSG_PENDING;
    }
    if ((fifo_usage & BSP_CAN_FIFO1_USED) != 0U)
    {
        notifications |= CAN_IT_RX_FIFO1_MSG_PENDING;
    }

    status = HAL_CAN_ActivateNotification(hcan, notifications);
    if (status != HAL_OK)
    {
        /* 保持启动操作的原子性，通知失败时回到停止状态。 */
        (void)HAL_CAN_Stop(hcan);
        return status;
    }

    *started = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef BSP_CAN_Transmit(BSP_CAN_TxInstance_t *instance)
{
    HAL_StatusTypeDef status;

    if ((instance == NULL)
        || (instance->initialized == 0U)
        || (BSP_CAN_IsSupported(instance->hcan) == 0U))
    {
        return HAL_ERROR;
    }

    instance->transmit_count++;

    /* 发送路径保持非阻塞，邮箱全满时立即交由上层决定何时重试。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(instance->hcan) == 0U)
    {
        instance->transmit_busy_count++;
        instance->last_status = HAL_BUSY;
        instance->last_error_code = HAL_CAN_GetError(instance->hcan);
        return HAL_BUSY;
    }

    status = HAL_CAN_AddTxMessage(instance->hcan,
                                  &instance->header,
                                  instance->data,
                                  &instance->mailbox);
    instance->last_status = status;
    instance->last_error_code = HAL_CAN_GetError(instance->hcan);

    if (status == HAL_OK)
    {
        instance->transmit_ok_count++;
    }
    else if (status == HAL_BUSY)
    {
        instance->transmit_busy_count++;
    }
    else
    {
        instance->transmit_error_count++;
    }

    return status;
}

/**
 * @brief 排空指定硬件FIFO并分发给已注册设备
 *
 * 本函数由HAL接收回调调用，处于中断上下文。设备回调可以做固定长度协议
 * 解码和状态快照更新，不应等待、打印日志或执行控制环。
 */
static void BSP_CAN_DispatchRx(CAN_HandleTypeDef *hcan,
                               uint32_t fifo)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t index;
    uint32_t now_ms;

    if ((BSP_CAN_IsSupported(hcan) == 0U)
        || ((fifo != CAN_RX_FIFO0) && (fifo != CAN_RX_FIFO1)))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK)
        {
            return;
        }

        if ((header.IDE != CAN_ID_STD)
            || (header.RTR != CAN_RTR_DATA)
            || (header.DLC > 8U))
        {
            continue;
        }

        for (index = 0U; index < bsp_can_rx_count; index++)
        {
            BSP_CAN_RxInstance_t *instance;

            instance = bsp_can_rx_instances[index];
            if ((instance->registered != 0U)
                && (BSP_CAN_IsSameInstance(instance->hcan, hcan) != 0U)
                && (instance->fifo == fifo)
                && (instance->id == header.StdId))
            {
                now_ms = HAL_GetTick();
                instance->header = header;
                instance->length = (uint8_t)header.DLC;
                memset(instance->data, 0, sizeof(instance->data));
                memcpy(instance->data, data, header.DLC);
                instance->receive_interval_ms =
                    now_ms - instance->last_receive_ms;
                instance->last_receive_ms = now_ms;
                instance->receive_count++;

                instance->callback(instance);
                break;
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_CAN_DispatchRx(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_CAN_DispatchRx(hcan, CAN_RX_FIFO1);
}
