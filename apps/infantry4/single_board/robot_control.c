/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 18:55:11
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-09-11 20:08:14
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_control.h"
#include "module_ins.h"
#include "module_vision.h"
#include "module_offline.h"
#include "module_remote.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "infantry_def.h"
#include "gimbal_func.h"
#include "shoot_func.h"
#include "chassis_func.h"
#include "robot_func.h"
#include "user_lib.h"
#include"motor_damiao.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static Gimbal_Ctrl_Cmd_t  gimbal_cmd;
static Chassis_Ctrl_Cmd_t chassis_cmd;
static Shoot_Ctrl_Cmd_t   shoot_cmd;
static uint16_t           yaw_ecd;
// 视觉(上位机)数据
static ReceivePacket *receive_packet = NULL;
static SendPacket     send_packet;
// 姿态角数据
const static Ins_t *ins = NULL;

/**
 * @brief 视觉瞄准: 仅在 gimbal_auto_mode 下生效, 用上位机目标角度更新云台 yaw/pitch
 * - yaw:   上位机给 -180°~180°(rad), 对齐到 INS 连续累计角(可多圈)的最近等效值, 避免跳变
 * - pitch: 上位机给目标俯仰角(rad, 低头为正), 与本车 pitch 指令同号, 直接转度后按机械限位限幅
 * - 视觉离线或未识别到目标: 保持当前指向, 等待视觉重新捕获
 */
static void GimbalVisionAuto(Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (Gimbal_Ctrl == NULL || Gimbal_Ctrl->gimbal_mode != gimbal_auto_mode)
    {
        return;
    }

    if (Module_Vision_Get_offline_state() == STATE_OFFLINE)
    {
        return;
    }

    if (receive_packet != NULL && receive_packet->found != 0)
    {
        float pitch_deg = receive_packet->target_pitch * RAD_2_DEGREE;
        VAL_LIMIT(pitch_deg, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal_Ctrl->pitch = pitch_deg;

        float yaw_now_deg = ins->YawTotalAngle_rad * RAD_2_DEGREE;      // INS 连续累计角(可多圈)
        float yaw_tgt_deg = receive_packet->target_yaw * RAD_2_DEGREE;  // 上位机目标 (-180°~180°)
        float yaw_delta   = yaw_tgt_deg - yaw_now_deg;
        while (yaw_delta > 180.0f) yaw_delta -= 360.0f;
        while (yaw_delta < -180.0f) yaw_delta += 360.0f;
        Gimbal_Ctrl->yaw = yaw_now_deg + yaw_delta;
    }
}

static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        /* 遥控器控制输入 */
        RemoteControlSet(&chassis_cmd, &gimbal_cmd, &shoot_cmd);

        /* 视觉数据交换: 上发云台姿态四元数, 接收目标角度 */
        send_packet.mode = 0; //蓝1红0
        send_packet.q[0] = ins->q[0];
        send_packet.q[1] = ins->q[1];
        send_packet.q[2] = ins->q[2];
        send_packet.q[3] = ins->q[3];
        Module_Vision_Send(&send_packet, TX_NO_WAIT);
        receive_packet = Module_Vision_Receive();
        /* 视觉模式: 云台由视觉控制 */
        GimbalVisionAuto(&gimbal_cmd);

        /* 视觉模式: 视觉建议开火且 ch5(摩擦轮)在下档时供弹, 其余情况一律关闭发射机构 */
        if (gimbal_cmd.gimbal_mode == gimbal_auto_mode)
        {
            int16_t ch5 = Module_Remote_get_channel(5);
            if (receive_packet != NULL && receive_packet->fire_advice && ch5 > 1500)
            {
                shoot_cmd.shoot_mode    = shoot_on;
                shoot_cmd.friction_mode = friction_on;
                shoot_cmd.load_mode     = load_1_bullet;
            }
            else
            {
                shoot_cmd.shoot_mode    = shoot_off;
                shoot_cmd.friction_mode = friction_off;
                shoot_cmd.load_mode     = load_stop;
            }
        }

        /* 云台控制 */
        gimbal_func(&gimbal_cmd, &yaw_ecd);
        /* 发射机构控制 */
        shoot_func(&shoot_cmd);
        /* 底盘控制 */
        //chassis_cmd.offset_angle = CalcOffsetAngle(yaw_ecd) * 360.0f / 8191.0f;
       // chassis_cmd.vx           = chassis_cmd.vx * CHASSIS_MAX_SPEED_MPS;
        //chassis_cmd.vy           = chassis_cmd.vy * CHASSIS_MAX_SPEED_MPS;
        //chassis_cmd.wz           = chassis_cmd.wz * CHASSIS_MAX_SPEED_MPS;
        //chassis_func(&chassis_cmd);

        tx_thread_sleep(2);

    }
}

void robot_control_init(void)
{
    UINT status;

    /* 获取 INS 指针(云台姿态, 用于视觉上发) */
    ins = Module_INS_get();
    if (ins == NULL)
    {
        return;
    }

    gimbal_init();
    chassis_init();
    shoot_init();
    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 10, 10,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        return;
    }
}
