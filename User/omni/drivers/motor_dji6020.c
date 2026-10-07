#include "omni/drivers/motor_dji6020.h"

#include <string.h>

#define DJI6020_ID_MIN       (1U)
#define DJI6020_ID_MAX       (7U)
#define DJI6020_ECD_RANGE    (8192.0f)
#define DJI6020_CURRENT_RAW  (16384.0f)
#define MOTOR_TWO_PI         (6.28318530718f)

static int16_t GetI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void PutI16Be(uint8_t *data,
                     int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value >> 8U);
    data[1] = (uint8_t)value;
}

/** 解析编码器、转速、电流和温度，并更新累计圈数。 */
static void FeedbackCallback(BSP_CAN_RxInstance_t *rx)
{
    DJI6020_t *motor;
    int16_t delta;

    if ((rx == NULL) || (rx->owner == NULL) || (rx->length < 7U))
    {
        return;
    }

    motor = (DJI6020_t *)rx->owner;
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
        (float)motor->ecd * MOTOR_TWO_PI / DJI6020_ECD_RANGE;
    motor->total_angle_rad =
        ((float)motor->round_count * DJI6020_ECD_RANGE +
         (float)motor->ecd) * MOTOR_TWO_PI / DJI6020_ECD_RANGE;
    motor->speed_rpm = (float)motor->speed_raw;
    motor->current_a =
        (float)motor->current_raw * DJI6020_CURRENT_MAX_A /
        DJI6020_CURRENT_RAW;
    motor->last_update_ms = HAL_GetTick();
    motor->online = 1U;
}

HAL_StatusTypeDef DJI6020_BusInit(DJI6020_Bus_t *bus,
                                  CAN_HandleTypeDef *hcan)
{
    if ((bus == NULL) || (hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(bus, 0, sizeof(*bus));
    bus->hcan = hcan;

    if ((BSP_CAN_TxInit(&bus->current_tx_1fe,
                        hcan,
                        0x1FEU,
                        8U) != HAL_OK)
        || (BSP_CAN_TxInit(&bus->current_tx_2fe,
                           hcan,
                           0x2FEU,
                           8U) != HAL_OK))
    {
        return HAL_ERROR;
    }

    bus->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef DJI6020_Init(DJI6020_t *motor,
                               DJI6020_Bus_t *bus,
                               uint8_t id,
                               uint32_t rx_fifo)
{
    if ((motor == NULL)
        || (bus == NULL)
        || (bus->initialized == 0U)
        || (id < DJI6020_ID_MIN)
        || (id > DJI6020_ID_MAX))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->id = id;

    if (BSP_CAN_RxRegister(&motor->rx,
                           bus->hcan,
                           0x204U + id,
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

void DJI6020_SetCurrentA(DJI6020_t *motor,
                         float current_a)
{
    if ((motor == NULL) || (motor->initialized == 0U))
    {
        return;
    }

    if (current_a > DJI6020_CURRENT_MAX_A)
    {
        current_a = DJI6020_CURRENT_MAX_A;
    }
    else if (current_a < -DJI6020_CURRENT_MAX_A)
    {
        current_a = -DJI6020_CURRENT_MAX_A;
    }

    motor->bus->current_command[motor->id - 1U] =
        (int16_t)(current_a * DJI6020_CURRENT_RAW /
                  DJI6020_CURRENT_MAX_A);
    motor->current_command_a = current_a;
}

HAL_StatusTypeDef DJI6020_SendCurrent(DJI6020_Bus_t *bus)
{
    HAL_StatusTypeDef status;
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
            PutI16Be(&bus->current_tx_1fe.data[index * 2U],
                     bus->current_command[index]);
        }
        status = BSP_CAN_Transmit(&bus->current_tx_1fe);
    }

    if ((bus->active_mask & 0x70U) != 0U)
    {
        for (index = 0U; index < 3U; index++)
        {
            PutI16Be(&bus->current_tx_2fe.data[index * 2U],
                     bus->current_command[index + 4U]);
        }

        /* 0x2FE最后一个槽位没有对应的6020，固定填0。 */
        PutI16Be(&bus->current_tx_2fe.data[6], 0);
        if (BSP_CAN_Transmit(&bus->current_tx_2fe) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }

    return status;
}

HAL_StatusTypeDef DJI6020_Stop(DJI6020_t *motor)
{
    if ((motor == NULL) || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }

    DJI6020_SetCurrentA(motor, 0.0f);
    return DJI6020_SendCurrent(motor->bus);
}

uint8_t DJI6020_CheckOnline(DJI6020_t *motor,
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
