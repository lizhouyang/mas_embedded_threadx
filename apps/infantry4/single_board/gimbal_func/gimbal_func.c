/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 19:15:45
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-18 16:47:34
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\gimbal_func\gimbal_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 19:15:45
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-17 20:25:30
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\gimbal_func\gimbal_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 19:15:45
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-17 20:13:48S
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\gimbal_func\gimbal_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "gimbal_func.h"
//#include "module_bmi088.h"
//#include "module_ins.h"
#include "motor_dji.h"
#include "motor_damiao.h"
#include "user_lib.h"
#include "bsp_can.h" 

static DJI_Motor_t *yaw_motor   = NULL; 
void gimbal_init(void)
{
   
    Motor_Init_Config_s yaw_config = {
        .offline_init_config =
            {
                .name       = "6020",
                .timeout_ms = 100,
                .beep_times = 3,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 1,
            },
        .controller_init_config = {
                                   .lqr_init =
                                       {
                                           .K         = {0.2f},
                                           .state_dim = 1,
                                       }},
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_LQR,
                .feedback_reverse_flag = 0,
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = ANGLE_LOOP,
            },
        .motor_init_info = {.motor_type = GM6020_CURRENT, .gear_ratio = 1, .max_torque = 2.223f, .torque_constant = 0.741f}};
    yaw_motor = Motor_DJI_Init(&yaw_config);
    if (yaw_motor == NULL)
    {
        
        return;
    }


}

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    if (gimbal_cmd != NULL)
    {
        if (!Module_Offline_get_device_status(yaw_motor->base.offline_dev) )
        {
            Motor_DJI_Start(yaw_motor);
            Motor_DJI_SetRef(yaw_motor, (gimbal_cmd->yaw * DEGREE_2_RAD));
           
        }
        else
        {
            Motor_DJI_Stop(yaw_motor);
        }
    }
    // 数据反馈
    if ( yaw_ecd != NULL)
    {
        *yaw_ecd = yaw_motor->measure.ecd;
    }
}
