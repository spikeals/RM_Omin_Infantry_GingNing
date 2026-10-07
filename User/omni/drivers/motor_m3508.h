#ifndef OMNI_DRIVERS_MOTOR_M3508_H
#define OMNI_DRIVERS_MOTOR_M3508_H

#include "omni/utils/bsp_can.h"

#include <stdint.h>

/*
 * DJI M3508 + C620 CAN驱动。
 * 电机ID 1～4共享0x200控制帧，ID 5～8共享0x1FF控制帧。
 * 反馈帧标准ID为0x200 + 电机ID，每路电流命令占两个大端字节。
 */
#define M3508_ID_MIN           (1U)
#define M3508_ID_MAX           (8U)
#define M3508_CURRENT_RAW_MAX  (16384)
#define M3508_CURRENT_MAX_A    (20.0f)

/**
 * @brief 同一路CAN上的M3508发送组
 *
 * current_command按电机ID保存八个电流槽位。设置接口只更新缓存，
 * 调用M3508_SendCurrent后才会把对应组帧送入CAN邮箱。
 */
typedef struct
{
    CAN_HandleTypeDef *hcan;
    BSP_CAN_TxInstance_t current_tx_200;
    BSP_CAN_TxInstance_t current_tx_1ff;
    int16_t current_command[M3508_ID_MAX];
    uint8_t active_mask;
    uint8_t initialized;
} M3508_Bus_t;

/**
 * @brief 单台M3508对象
 *
 * 反馈字段由CAN接收中断直接更新，方便初版在Debug中直接观察。
 * angle_rad、total_angle_rad和speed_rpm均为电机转子侧量。
 */
typedef struct
{
    BSP_CAN_RxInstance_t rx;
    M3508_Bus_t *bus;
    uint8_t id;

    uint16_t ecd;              /* 当前单圈编码器值，范围0～8191 */
    uint16_t last_ecd;         /* 上一帧编码器值 */
    int32_t round_count;       /* 根据编码器跨零累计的整圈数 */
    int16_t speed_raw;         /* 反馈转速，单位rpm */
    int16_t current_raw;       /* 反馈转矩电流原始值 */
    uint8_t temperature_c;     /* 电机温度，单位摄氏度 */

    float angle_rad;           /* 单圈机械角度，范围[0, 2*pi) */
    float total_angle_rad;     /* 累计机械角度，单位rad */
    float speed_rpm;           /* 浮点形式的反馈转速 */
    float current_a;           /* 反馈转矩电流，单位A */
    float current_command_a;   /* 最近设置的目标电流，单位A */

    uint32_t last_update_ms;   /* 最近一次有效反馈时间 */
    uint8_t angle_initialized; /* 收到首帧后置1 */
    uint8_t online;            /* 最近一次在线检查结果 */
    uint8_t initialized;
} M3508_t;

/** 初始化一组M3508发送缓存，不启动CAN，也不发送报文。 */
HAL_StatusTypeDef M3508_BusInit(M3508_Bus_t *bus,
                                CAN_HandleTypeDef *hcan);

/** 注册一台ID为1～8的M3508及其反馈ID。 */
HAL_StatusTypeDef M3508_Init(M3508_t *motor,
                             M3508_Bus_t *bus,
                             uint8_t id,
                             uint32_t rx_fifo);

/** 按协议原始值更新目标电流缓存。 */
void M3508_SetCurrent(M3508_t *motor,
                      int16_t current_raw);

/** 按安培更新目标电流缓存，输入限制在-20 A～+20 A。 */
void M3508_SetCurrentA(M3508_t *motor,
                       float current_a);

/** 提交该总线上已经激活的0x200/0x1FF成组电流帧。 */
HAL_StatusTypeDef M3508_SendCurrent(M3508_Bus_t *bus);

/** 设置单台电机原始电流并立即提交其所在总线的成组帧。 */
HAL_StatusTypeDef M3508_OutputCurrent(M3508_t *motor,
                                      int16_t current_raw);

/** 设置单台电机安培电流并立即提交其所在总线的成组帧。 */
HAL_StatusTypeDef M3508_OutputCurrentA(M3508_t *motor,
                                       float current_a);

/** 清零总线的八个电流缓存，本函数本身不发送报文。 */
void M3508_ClearCurrent(M3508_Bus_t *bus);

/** 把该电机目标电流设为0，并立即提交成组帧。 */
HAL_StatusTypeDef M3508_Stop(M3508_t *motor);

/** 根据最近反馈时间更新并返回在线状态。 */
uint8_t M3508_CheckOnline(M3508_t *motor,
                          uint32_t timeout_ms);

#endif
