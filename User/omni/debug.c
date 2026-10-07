#include "omni/debug.h"

#include "omni/applications/chassis_input.h"

Debug_t debug;

void Debug_Init(const M3508_Bus_t *bus,
                const M3508_t motor[CHASSIS_WHEEL_NUM])
{
    debug.rc = ET07_GetData();
    debug.input = ChassisInput_GetCommand();
    debug.cmd = ChassisSafety_GetCommand();
    debug.ctrl = ChassisCtrl_GetState();
    debug.pid = ChassisCtrl_GetPid();
    debug.motor = motor;
    debug.bus = bus;
}

void Debug_Update(uint32_t now_ms,
                  uint32_t loop_count,
                  uint32_t control_count)
{
    debug.safety = ChassisSafety_GetState();
    debug.remote_ok = ChassisInput_IsRemoteHealthy() ? 1U : 0U;
    debug.sticks_center =
        ChassisInput_AreMovementSticksCentered() ? 1U : 0U;
    debug.now_ms = now_ms;
    debug.loop_count = loop_count;
    debug.control_count = control_count;
}
