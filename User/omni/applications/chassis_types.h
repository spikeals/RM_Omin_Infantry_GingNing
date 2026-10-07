#ifndef OMNI_APPLICATIONS_CHASSIS_TYPES_H
#define OMNI_APPLICATIONS_CHASSIS_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 底盘工作模式
 *
 * 模式用于选择“底盘应该采用哪一种运动控制方法”。
 * 当前独立底盘首先使用DISABLED、MANUAL和SPIN，其余模式为后续IMU、
 * 云台和整车控制保留。所有输入源使用同一组模式，避免重复定义。
 */
typedef enum
{
    /* 底盘停止工作，所有轮电机目标和输出都应归零。 */
    CHASSIS_DISABLED = 0,

    /* 遥控器直接给出车体坐标系下的前后、左右和旋转速度。 */
    CHASSIS_MANUAL,

    /* 小陀螺模式：底盘持续自转，同时允许叠加平移命令。 */
    CHASSIS_SPIN,

    /* 底盘根据云台相对Yaw角主动旋转，使车头跟随云台方向。 */
    CHASSIS_FOLLOW_GIMBAL,

    /* 使用IMU闭环保持指定航向，抵抗外界扰动和行驶偏航。 */
    CHASSIS_HEADING_HOLD,

    /* 将地面坐标系速度转换到车体坐标系，实现方向不随车头旋转。 */
    CHASSIS_FIELD_ORIENTED
} ChassisMode;

/**
 * @brief 一次完整的底盘运动命令
 *
 * 该结构是输入层、安全门和底盘控制器之间的统一接口。
 * ET07、整车CAN等输入源先生成该命令，安全门检查后再交给控制器。
 *
 * 坐标约定：
 * +x：车体前方；
 * +y：车体左方；
 * +z：从车体上方观察时逆时针方向。
 */
typedef struct
{
    /* 选择底盘运动控制方法，例如手动、小陀螺或云台跟随。 */
    ChassisMode mode;

    /*
     * 执行器输出许可。
     * false表示无论mode和速度值是什么，最终输出都必须归零；
     * true表示输入源提出使能请求，仍需经过SafetyGate检查。
     */
    bool enable;

    /* 车体前后方向目标速度，单位m/s，正值表示向前。 */
    float vx_mps;

    /* 车体左右方向目标速度，单位m/s，正值表示向左。 */
    float vy_mps;

    /* 车体绕z轴目标角速度，单位rad/s，正值表示俯视逆时针。 */
    float wz_radps;

    /*
     * 输入源最后一次生成或刷新该命令的系统毫秒时间。
     * SafetyGate通过now_ms - timestamp_ms判断命令是否超时。
     */
    uint32_t timestamp_ms;

    /*
     * 每生成一条新命令时递增的序列号。
     * 可用于发现重复命令、漏更新或通信跳变；到达65535后自然回绕。
     */
    uint16_t sequence;
} ChassisCommand;

#endif
