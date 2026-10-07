#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

typedef struct
{
    volatile uint32_t loop_counter;
    volatile uint32_t control_counter;
} Chassis_AppState_t;

HAL_StatusTypeDef ChassisApp_Init(CAN_HandleTypeDef *hcan,
                                  UART_HandleTypeDef *huart,
                                  uint32_t now_ms);
void ChassisApp_RunOnce(uint32_t now_ms);
void ChassisApp_RxEvent(UART_HandleTypeDef *huart,
                        uint16_t size);

#endif
