#include "omni/drivers/motor_m3508.h"

#include <string.h>

#define M3508_ECD_RANGE  (8192.0f)
#define MOTOR_TWO_PI     (6.28318530718f)

/** 从CAN数据中读取一个大端有符号16位数。 */
static int16_t GetI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

/** 把一个有符号16位数按大端顺序写入CAN数据。 */
static void PutI16Be(uint8_t *data,
                     int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value >> 8U);
    data[1] = (uint8_t)value;
}

/**
 * @brief 解析M3508反馈帧
 *
 * 数据布局：编码器[0..1]、转速[2..3]、电流[4..5]、温度[6]。
 * 该回调运行在CAN接收中断中，只做定长解析和简单状态更新。
 */
static void FeedbackCallback(BSP_CAN_RxInstance_t *rx)
{
    M3508_t *motor;
    int16_t delta;

    if ((rx == NULL) || (rx->owner == NULL) || (rx->length < 7U))
    {
        return;
    }

    motor = (M3508_t *)rx->owner;
    motor->last_ecd = motor->ecd;
    motor->ecd =
        (uint16_t)(((uint16_t)rx->data[0] << 8U) | rx->data[1]);
    motor->speed_raw = GetI16Be(&rx->data[2]);
    motor->current_raw = GetI16Be(&rx->data[4]);
    motor->temperature_c = rx->data[6];

    if (motor->angle_initialized != 0U)
    {
        delta = (int16_t)(motor->ecd - motor->last_ecd);
        if (delta > 4096)
        {
            motor->round_count--;
        }
        else if (delta < -4096)
        {
            motor->round_count++;
        }
    }
    else
    {
        motor->angle_initialized = 1U;
    }

    motor->angle_rad =
        (float)motor->ecd * MOTOR_TWO_PI / M3508_ECD_RANGE;
    motor->total_angle_rad =
        ((float)motor->round_count * M3508_ECD_RANGE +
         (float)motor->ecd) * MOTOR_TWO_PI / M3508_ECD_RANGE;
    motor->speed_rpm = (float)motor->speed_raw;
    motor->current_a =
        (float)motor->current_raw * M3508_CURRENT_MAX_A /
        (float)M3508_CURRENT_RAW_MAX;
    motor->last_update_ms = HAL_GetTick();
    motor->online = 1U;
}

HAL_StatusTypeDef M3508_BusInit(M3508_Bus_t *bus,
                                CAN_HandleTypeDef *hcan)
{
    if ((bus == NULL) || (hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(bus, 0, sizeof(*bus));
    bus->hcan = hcan;

    if ((BSP_CAN_TxInit(&bus->current_tx_200,
                        hcan,
                        0x200U,
                        8U) != HAL_OK)
        || (BSP_CAN_TxInit(&bus->current_tx_1ff,
                           hcan,
                           0x1FFU,
                           8U) != HAL_OK))
    {
        return HAL_ERROR;
    }

    bus->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef M3508_Init(M3508_t *motor,
                             M3508_Bus_t *bus,
                             uint8_t id,
                             uint32_t rx_fifo)
{
    if ((motor == NULL)
        || (bus == NULL)
        || (bus->initialized == 0U)
        || (id < M3508_ID_MIN)
        || (id > M3508_ID_MAX))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->id = id;

    if (BSP_CAN_RxRegister(&motor->rx,
                           bus->hcan,
                           0x200U + id,
                           rx_fifo,
                           motor,
                           FeedbackCallback) != HAL_OK)
    {
        return HAL_ERROR;
    }

    bus->active_mask |= (uint8_t)(1U << (id - 1U));
    motor->initialized = 1U;
    return HAL_OK;
}

void M3508_SetCurrent(M3508_t *motor,
                      int16_t current_raw)
{
    if ((motor == NULL)
        || (motor->initialized == 0U)
        || (motor->bus == NULL))
    {
        return;
    }

    if (current_raw > M3508_CURRENT_RAW_MAX)
    {
        current_raw = M3508_CURRENT_RAW_MAX;
    }
    else if (current_raw < -M3508_CURRENT_RAW_MAX)
    {
        current_raw = -M3508_CURRENT_RAW_MAX;
    }

    motor->bus->current_command[motor->id - 1U] = current_raw;
    motor->current_command_a =
        (float)current_raw * M3508_CURRENT_MAX_A /
        (float)M3508_CURRENT_RAW_MAX;
}

void M3508_SetCurrentA(M3508_t *motor,
                       float current_a)
{
    if ((motor == NULL) || (motor->initialized == 0U))
    {
        return;
    }

    if (current_a > M3508_CURRENT_MAX_A)
    {
        current_a = M3508_CURRENT_MAX_A;
    }
    else if (current_a < -M3508_CURRENT_MAX_A)
    {
        current_a = -M3508_CURRENT_MAX_A;
    }

    M3508_SetCurrent(
        motor,
        (int16_t)(current_a * (float)M3508_CURRENT_RAW_MAX /
                  M3508_CURRENT_MAX_A));
}

HAL_StatusTypeDef M3508_SendCurrent(M3508_Bus_t *bus)
{
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef group_status;
    uint8_t index;

    if ((bus == NULL) || (bus->initialized == 0U))
    {
        return HAL_ERROR;
    }

    status = HAL_OK;

    if ((bus->active_mask & 0x0FU) != 0U)
    {
        for (index = 0U; index < 4U; index++)
        {
            PutI16Be(&bus->current_tx_200.data[index * 2U],
                     bus->current_command[index]);
        }
        status = BSP_CAN_Transmit(&bus->current_tx_200);
    }

    if ((bus->active_mask & 0xF0U) != 0U)
    {
        for (index = 0U; index < 4U; index++)
        {
            PutI16Be(&bus->current_tx_1ff.data[index * 2U],
                     bus->current_command[index + 4U]);
        }
        group_status = BSP_CAN_Transmit(&bus->current_tx_1ff);
        if (status == HAL_OK)
        {
            status = group_status;
        }
    }

    return status;
}

HAL_StatusTypeDef M3508_OutputCurrent(M3508_t *motor,
                                      int16_t current_raw)
{
    if ((motor == NULL)
        || (motor->initialized == 0U)
        || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }

    M3508_SetCurrent(motor, current_raw);
    return M3508_SendCurrent(motor->bus);
}

HAL_StatusTypeDef M3508_OutputCurrentA(M3508_t *motor,
                                       float current_a)
{
    if ((motor == NULL)
        || (motor->initialized == 0U)
        || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }

    M3508_SetCurrentA(motor, current_a);
    return M3508_SendCurrent(motor->bus);
}

void M3508_ClearCurrent(M3508_Bus_t *bus)
{
    if (bus != NULL)
    {
        memset(bus->current_command, 0, sizeof(bus->current_command));
    }
}

HAL_StatusTypeDef M3508_Stop(M3508_t *motor)
{
    return M3508_OutputCurrentA(motor, 0.0f);
}

uint8_t M3508_CheckOnline(M3508_t *motor,
                          uint32_t timeout_ms)
{
    if ((motor == NULL)
        || (motor->initialized == 0U)
        || (motor->online == 0U))
    {
        return 0U;
    }

    if ((uint32_t)(HAL_GetTick() - motor->last_update_ms) > timeout_ms)
    {
        motor->online = 0U;
    }

    return motor->online;
}
