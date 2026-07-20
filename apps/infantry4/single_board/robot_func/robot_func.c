/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 20:21:52
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-19 20:37:17
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_func\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 20:21:52
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-19 16:12:40
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
int ch5=0;
int ch6=0;
int ch8=0;
void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl)
{
    if (!Gimbal_Ctrl || !Chassis_Ctrl || !Shoot_Ctrl) return;
    
    uint8_t state = Module_Remote_get_offline_status();

    if (state & 0x01)  // 遥控器在线
    {   
        ch1 = Module_Remote_get_channel(1);
        ch2 = Module_Remote_get_channel(2);
        ch3 = Module_Remote_get_channel(3);
        ch4 = Module_Remote_get_channel(4);
        ch5 = Module_Remote_get_channel(5);  
        ch6 = Module_Remote_get_channel(6);
        ch8 = Module_Remote_get_channel(8);
        if (ch6 < 500)  //左二上拨,云台底盘发射机构都不动
        {
            Chassis_Ctrl->chassis_mode = chassis_zero_force;
            Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
            Shoot_Ctrl->shoot_mode = shoot_off;
        }
        else if (ch6 > 500 && ch6 < 1500)  // 中间，云台动，底盘不动
        {
            Chassis_Ctrl->chassis_mode = chassis_zero_force;
            Gimbal_Ctrl->gimbal_mode   = gimbal_gyro_mode;
        }
        else if (ch6 > 1500)  // 云台动，底盘动，底盘选模式
        {
            Gimbal_Ctrl->gimbal_mode   = gimbal_gyro_mode;
            if (ch5 > 1500)  
            {
                Chassis_Ctrl->chassis_mode = chassis_rotate;
            }
            else if (ch5 < 500)  
            {
                Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;
            }
            else  
            {
                Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
            }
        }
        
        if (ch8 < 500)  // 右二上拨射击模式关闭
        {
            Shoot_Ctrl->shoot_mode = shoot_off;
            Shoot_Ctrl->friction_mode = friction_off;
            Shoot_Ctrl->load_mode = load_stop;
        }
        else if (ch8 < 1500)  // 中间位置射击模式开启(单发)
        {
            Shoot_Ctrl->shoot_mode = shoot_on;
            Shoot_Ctrl->friction_mode = friction_on;
            Shoot_Ctrl->load_mode = load_1_bullet;
        }
        else  // 右二下拨射击模式开启(连发)
        {
            Shoot_Ctrl->shoot_mode = shoot_on;
            Shoot_Ctrl->friction_mode = friction_on;
            Shoot_Ctrl->load_mode = load_1_bullet;
            
        }

        /* 摇杆 → 速度比例 (-1.0 ~ +1.0) */
        Chassis_Ctrl->vx = -(float)ch1 / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
        Chassis_Ctrl->vy = -(float)ch2 / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
        Gimbal_Ctrl->yaw -= 0.0003f * (float)(ch4);
        Gimbal_Ctrl->pitch += 0.001f * (float)(ch3);
        VAL_LIMIT(Gimbal_Ctrl->pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
    }
    else  // 遥控器离线
    {
        Gimbal_Ctrl->yaw = 0.0f;
        Gimbal_Ctrl->pitch = 0.0f;
        Chassis_Ctrl->vx = 0.0f;
        Chassis_Ctrl->vy = 0.0f;
        Shoot_Ctrl->shoot_mode = shoot_off;
        Shoot_Ctrl->friction_mode = friction_off;
        Shoot_Ctrl->load_mode = load_stop;
    }
}
int16_t CalcOffsetAngle(float getyawangle)
{
    float offset_ecd;

    const float ECD_MAX  = 8191.0f;
    const float ECD_HALF = 4095.5f;

    offset_ecd = getyawangle - YAW_CHASSIS_ALIGN_ECD;
    // 归一化到最小旋转角度对应的编码器差值 ([-ECD_HALF, ECD_HALF])
    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return (int16_t)offset_ecd;
}