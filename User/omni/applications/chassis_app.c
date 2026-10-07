#include "omni/applications/chassis_app.h"

#include "omni/config.h"
#include "omni/debug.h"
#include "omni/applications/chassis_ctrl.h"
#include "omni/applications/chassis_input.h"
#include "omni/applications/chassis_safety.h"
#include "omni/drivers/et07.h"
#include "omni/drivers/motor_m3508.h"
#include "omni/utils/bsp_can.h"

#include <stdint.h>

static const uint8_t motor_id[CHASSIS_WHEEL_NUM] =
{
    MOTOR_ID_FL,
    MOTOR_ID_FR,
    MOTOR_ID_RL,
    MOTOR_ID_RR
};

static Chassis_AppState_t app_state;
static uint32_t control_last_ms;
static M3508_Bus_t motor_bus;
static M3508_t motor[CHASSIS_WHEEL_NUM];
static UART_HandleTypeDef *remote_uart;

static HAL_StatusTypeDef Motor_Init(CAN_HandleTypeDef *hcan)
{
    uint8_t i;

    if (M3508_BusInit(&motor_bus, hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++)
    {
        if (M3508_Init(&motor[i],
                       &motor_bus,
                       motor_id[i],
                       CAN_RX_FIFO0) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    if (BSP_CAN_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    ChassisCtrl_Init(&motor_bus, motor);
    return HAL_OK;
}

HAL_StatusTypeDef ChassisApp_Init(CAN_HandleTypeDef *hcan,
                                  UART_HandleTypeDef *huart,
                                  uint32_t now_ms)
{
    remote_uart = huart;
    if ((Motor_Init(hcan) != HAL_OK)
        || (ET07_Init(huart) != HAL_OK))
    {
        return HAL_ERROR;
    }

    app_state.loop_counter = 0U;
    app_state.control_counter = 0U;
    control_last_ms = now_ms;
    ChassisInput_Init(now_ms);
    ChassisSafety_Init(now_ms);
    Debug_Init(&motor_bus, motor);
    Debug_Update(now_ms, 0U, 0U);
    return HAL_OK;
}

void ChassisApp_RunOnce(uint32_t now_ms)
{
    ET07_RunOnce(now_ms);
    app_state.loop_counter++;
    if (now_ms - control_last_ms >= CTRL_MS)
    {
        app_state.control_counter++;
        control_last_ms = now_ms;

        ChassisInput_Update(now_ms);
        ChassisSafety_Update(
            now_ms,
            ChassisInput_GetCommand(),
            ChassisInput_IsRemoteHealthy(),
            ChassisInput_AreMovementSticksCentered());

        ChassisCtrl_Update(ChassisSafety_GetCommand());
        Debug_Update(now_ms,
                     app_state.loop_counter,
                     app_state.control_counter);
    }
}

void ChassisApp_RxEvent(UART_HandleTypeDef *huart,
                        uint16_t size)
{
    if (huart == remote_uart)
    {
        ET07_RxEventHandler(size);
    }
}
