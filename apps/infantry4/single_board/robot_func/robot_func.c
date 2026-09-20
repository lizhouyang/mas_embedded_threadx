/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 20:21:52
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-09-11 20:50:46
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_func\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 20:21:52
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-19 16:12:40
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_func\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "module_remote.h"
#include <stdint.h>
#include <string.h>
#include "infantry_def.h"
#include "user_lib.h"
#include "robot_func.h"
/**
 * @brief 遥控器通道数据数组
 * 索引 1-8 对应遥控器通道 1-8
 */
static int16_t ch[9] = {0}; // 通道数组，索引直接对应通道号

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl)
{
    if (!Gimbal_Ctrl || !Chassis_Ctrl || !Shoot_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    if (state & 0x01) // 遥控器在线
    {
        // 批量读取通道数据
        for (int i = 1; i <= 6; i++)
        {
            ch[i] = Module_Remote_get_channel(i);
        }
        ch[7] = Module_Remote_get_channel(7);
        ch[8] = Module_Remote_get_channel(8);

        /* ── ch8 底盘模式: 向上=关闭(无力), 中间=底盘跟随云台, 向下=自转 ── */
        if (ch[8] < 500) // 向上
        {
            Chassis_Ctrl->chassis_mode = chassis_zero_force;
        }
        else if (ch[8] <= 1500) // 中间
        {
            Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
        }
        else // 向下
        {
            Chassis_Ctrl->chassis_mode = chassis_rotate;
        }

        /* ── ch6 云台模式(兼总断电): 向上=云台底盘都无力, 中间=云台由遥控器控制, 向下=视觉控制 ── */
        if (ch[6] < 500) // 向上, 云台与底盘都无力
        {
            Gimbal_Ctrl->gimbal_mode = gimbal_zero_force;
        }
        else if (ch[6] <= 1500) // 中间, 云台由遥控器摇杆控制
        {
            Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
            /* 左摇杆 → yaw/pitch 微调 (仅手动档生效, 视觉档云台由视觉接管) */
            Gimbal_Ctrl->yaw -= 0.00015f * (float)(ch[4]);
            Gimbal_Ctrl->pitch -= 0.001f * (float)(ch[3]);
            VAL_LIMIT(Gimbal_Ctrl->pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        }
        else // 向下, 视觉控制: 切到 auto 模式, 云台目标由控制任务内的视觉数据更新
        {
            Gimbal_Ctrl->gimbal_mode = gimbal_auto_mode;
        }

        /* ── ch5/ch7 发射机构: 摩擦轮为主开关, 拨弹盘必须在摩擦轮开启后才允许启动 ──
         * ch5 向下=摩擦轮开(发射使能); 向上/中间=摩擦轮关, 发射机构(含拨弹盘)不启动
         * ch7 向下=拨弹盘开(单发供弹); 其余位置=拨盘关 */
        if (ch[5] > 1500) // ch5 向下: 摩擦轮开启
        {
            Shoot_Ctrl->shoot_mode    = shoot_on;
            Shoot_Ctrl->friction_mode = friction_on;
            Shoot_Ctrl->load_mode     = (ch[7] > 1500) ? load_1_bullet : load_stop;
        }
        else // ch5 向上/中间: 摩擦轮关闭, 拨弹盘不会启动
        {
            Shoot_Ctrl->shoot_mode    = shoot_off;
            Shoot_Ctrl->friction_mode = friction_off;
            Shoot_Ctrl->load_mode     = load_stop;
        }

        /* 摇杆 → 速度比例 (-1.0 ~ +1.0) */
        Chassis_Ctrl->vx = (float)ch[2] / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
        Chassis_Ctrl->vy = -(float)ch[1] / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
    }
    else // 遥控器离线
    {
        /*Gimbal_Ctrl->yaw = 0.0f;
        Gimbal_Ctrl->pitch = 0.0f;
        Chassis_Ctrl->vx = 0.0f;
        Chassis_Ctrl->vy = 0.0f;
        Shoot_Ctrl->shoot_mode = shoot_off;
        Shoot_Ctrl->friction_mode = friction_off;
        Shoot_Ctrl->load_mode = load_stop;*/
        Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
        Chassis_Ctrl->chassis_mode = chassis_zero_force;
        Shoot_Ctrl->shoot_mode     = shoot_off;
        Shoot_Ctrl->friction_mode  = friction_off;
        Shoot_Ctrl->load_mode      = load_stop;
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