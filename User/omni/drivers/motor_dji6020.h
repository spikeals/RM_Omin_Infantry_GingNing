#ifndef OMNI_DRIVERS_MOTOR_DJI6020_H
#define OMNI_DRIVERS_MOTOR_DJI6020_H

#include "omni/utils/bsp_can.h"

#include <stdint.h>

#define DJI6020_CURRENT_MAX_A  (3.0f)

/** DJI6020电流模式发送组：ID 1～4使用0x1FE，ID 5～7使用0x2FE。 */
typedef struct
{
    CAN_HandleTypeDef *hcan;
    BSP_CAN_TxInstance_t current_tx_1fe;
    BSP_CAN_TxInstance_t current_tx_2fe;
    int16_t current_command[7];
    uint8_t active_mask;
    uint8_t initialized;
} DJI6020_Bus_t;

/** 单台DJI6020对象，反馈字段由CAN接收中断直接更新。 */
typedef struct
{
    BSP_CAN_RxInstance_t rx;
    DJI6020_Bus_t *bus;
    uint8_t id;

    uint16_t ecd;
    uint16_t last_ecd;
    int32_t round_count;
    int16_t speed_raw;
    int16_t current_raw;
    uint8_t temperature_c;

    float angle_rad;
    float total_angle_rad;
    float speed_rpm;
    float current_a;
    float current_command_a;

    uint32_t last_update_ms;
    uint8_t angle_initialized;
    uint8_t online;
    uint8_t initialized;
} DJI6020_t;

HAL_StatusTypeDef DJI6020_BusInit(DJI6020_Bus_t *bus,
                                  CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef DJI6020_Init(DJI6020_t *motor,
                               DJI6020_Bus_t *bus,
                               uint8_t id,
                               uint32_t rx_fifo);
void DJI6020_SetCurrentA(DJI6020_t *motor,
                         float current_a);
HAL_StatusTypeDef DJI6020_SendCurrent(DJI6020_Bus_t *bus);
HAL_StatusTypeDef DJI6020_Stop(DJI6020_t *motor);
uint8_t DJI6020_CheckOnline(DJI6020_t *motor,
                            uint32_t timeout_ms);

#endif
