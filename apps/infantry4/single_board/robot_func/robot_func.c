#include "module_remote.h"
#include <stdint.h>
#include <string.h>
#include "infantry_def.h"
#include "user_lib.h"
#include "robot_func.h"
void RemoteControlSet( Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl) {
if (!Gimbal_Ctrl) return;
uint8_t state = Module_Remote_get_offline_status();
if (state & 0x01)
{
    Gimbal_Ctrl->yaw -= 0.001f * (float)(Module_Remote_get_channel(3));
}
}