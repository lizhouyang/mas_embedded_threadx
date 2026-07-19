/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 20:21:52
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-19 09:26:42
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_func\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "module_remote.h"
#include <stdint.h>
#include <string.h>
#include "infantry_def.h"
#include "user_lib.h"
#include "robot_func.h"
int ch3=0;
int ch4=0;
int ch1=0;
int ch2=0;
void RemoteControlSet( Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl) 
{
if (!Gimbal_Ctrl || !Chassis_Ctrl) return;
uint8_t state = Module_Remote_get_offline_status();

if (state & 0x01)
{   ch1=Module_Remote_get_channel(1);
    ch2=Module_Remote_get_channel(2);
    ch3=Module_Remote_get_channel(3);
    ch4=Module_Remote_get_channel(4);
    Chassis_Ctrl->vx = (float)(ch3);
    Chassis_Ctrl->vy = (float)(ch4);
    Chassis_Ctrl->wz = 0.0f;
 /* 摇杆 → 速度比例 (-1.0 ~ +1.0) */
    Chassis_Ctrl->vx = (float)ch1 / (float)(DT7_CH_VALUE_MAX - DT7_CH_VALUE_MIN);
    Chassis_Ctrl->vy = -(float)ch2 / (float)(DT7_CH_VALUE_MAX - DT7_CH_VALUE_MIN);
    Gimbal_Ctrl->yaw += 0.001f * (float)(ch4);
    Gimbal_Ctrl->pitch += 0.001f * (float)(ch3);
    VAL_LIMIT(Gimbal_Ctrl->pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
}
else
{
    Gimbal_Ctrl->yaw = 0.0f;
    Gimbal_Ctrl->pitch = 0.0f;
    Chassis_Ctrl->vx = 0.0f;
    Chassis_Ctrl->vy = 0.0f;
}
}