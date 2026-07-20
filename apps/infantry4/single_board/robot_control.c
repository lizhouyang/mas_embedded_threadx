/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 18:55:11
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-19 19:53:14
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_control.h"
//#include "referee_protocol.h"
//#include "module_referee.h"
//#include "module_ins.h"
//#include "module_vision.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "infantry_def.h"
#include "gimbal_func.h"
#include "chassis_func.h"
#include "robot_func.h"
#include "shoot_func.h"



static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];


static Gimbal_Ctrl_Cmd_t  gimbal_cmd;
static Chassis_Ctrl_Cmd_t chassis_cmd;
static Shoot_Ctrl_Cmd_t shoot_cmd;
static uint16_t yaw_ecd;


static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        /* 遥控器控制输入 */
        RemoteControlSet(&chassis_cmd,&gimbal_cmd,&shoot_cmd);
        /* 云台控制 */
        gimbal_func(&gimbal_cmd, &yaw_ecd);
        /* 底盘控制 */
        shoot_func(&shoot_cmd);
        chassis_cmd.offset_angle = CalcOffsetAngle(yaw_ecd) * 360.0f / 8191.0f;
        chassis_cmd.vx           = chassis_cmd.vx * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.vy           = chassis_cmd.vy * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.wz           = chassis_cmd.wz * CHASSIS_MAX_SPEED_MPS;
        chassis_func(&chassis_cmd);
      
        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;

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