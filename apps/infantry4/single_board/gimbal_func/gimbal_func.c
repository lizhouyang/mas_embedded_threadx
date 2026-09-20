/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 19:15:45
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-09-18 20:30:12
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
#include "module_bmi088.h"
#include "module_ins.h"
#include "motor_dji.h"
#include "motor_damiao.h"
#include "vofa.h"
#include "user_lib.h"
#include "bsp_can.h" 
static DM_Motor_t  *pitch_motor = NULL; // pitch电机指针
// 姿态角数据
const static Ins_t           *ins        = NULL;
const static Bmi088_device_t *bmi088_dev = NULL;

static DJI_Motor_t *yaw_motor   = NULL;

/* VOFA 上行镜像: p轴扭矩(Nm) / p轴欧拉角(rad), 由 gimbal_func 每周期刷新 */
static float g_pitch_torque = 0.0f;
static float g_pitch_euler  = 0.0f;

/* ── pitch 重力前馈 (实测拟合) ───────────────────────────────────────
 * 数据来源: 电机使能并稳住时读 measure.torque, 最小二乘拟合 T_hold(θ) = K·θ + C
 *           K = -0.7146 Nm/rad, C = 0.5956 Nm, R² = 0.968 (残差 ≤ 0.021 Nm)
 * 物理量:   最大重力矩 m·g·d = √(K²+C²) = 0.930 Nm, 力臂偏角 φ = 0.876 rad
 * 为什么需要: 纯 PD(LQR) 无积分项, 重力只能靠 K0×角度误差去扛 → 存在稳态误差;
 *             误差小到力矩不足以克服静摩擦时轴干脆不动 → "小角度不跟随"(死区)。
 * 注意: 实测窗口仅 θ ∈ [-0.10, +0.19] rad, 线性式在该范围内最准; 若要覆盖
 *       -15°~+35° 全行程, 建议改为 cos 形式 0.9303·cos(θ+0.876) 并补测两侧端点。 */
#define PITCH_FF_GRAV_K (-0.7146f) /* Nm/rad */
#define PITCH_FF_GRAV_C (0.5956f)  /* Nm     */

static void pitch_gravity_feedforward(void)
{
    if (pitch_motor == NULL || ins == NULL) return;
    Motor_SetForwardTorque(&pitch_motor->base, PITCH_FF_GRAV_K * ins->euler_rad[1] + PITCH_FF_GRAV_C);
}

void gimbal_init(void)
{
    ins = Module_INS_get();
    if (ins == NULL)
    {
        return;
    }
    bmi088_dev = Module_BMI088_get_device();
    if (bmi088_dev == NULL)
    {

        return;
    }
   
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
                                .other_angle_feedback_ptr = &ins->YawTotalAngle_rad,
                                .other_speed_feedback_ptr = &bmi088_dev->gyro[2],
                                   .lqr_init =
                                       {
                                           .K         = {10.0f, 0.86f},//5.8 0.6
                                           .state_dim = 2,
                                       }},
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_LQR,
                .feedback_reverse_flag = 0,
                .angle_feedback_source = 1,
                .speed_feedback_source = 1,
                .loop_type             = ANGLE_LOOP,
            },
        .motor_init_info = {.motor_type = GM6020_CURRENT, .gear_ratio = 1, .max_torque = 2.223f, .torque_constant = 0.741f}};
    yaw_motor = Motor_DJI_Init(&yaw_config);
    if (yaw_motor == NULL)
    {
        
        return;
    }
Motor_Init_Config_s pitch_config = {.offline_init_config =
                                            {
                                                .name       = "dm4310",
                                                .timeout_ms = 100,
                                                .beep_times = 4,
                                                .enable     = 1,
                                            },
                                        .transport = MOTOR_TRANSPORT_CAN,
                                        .transport_config.can =
                                            {
                                                .hcan  = BSP_CAN_HANDLE2,
                                                .tx_id = 0X01,
                                                .rx_id = 0Xf1,
                                            },
                                        .controller_init_config =
                                            {
                                                .other_angle_feedback_ptr = &ins->euler_rad[1],
                                                .other_speed_feedback_ptr = &bmi088_dev->gyro[0], 
                                                .lqr_init =
                                                    {
                                                        .K         = {12.0f, 0.92f}, // 28.7312f,2.5974f
                                                        .state_dim = 2,
                                                    },
                                                    
                                            },
                                        .setting_init_config =
                                            {
                                                .algorithm_type        = CONTROL_LQR,
                                                .feedback_reverse_flag = 1,
                                                .angle_feedback_source = 1,
                                                .speed_feedback_source = 1,
                                                .loop_type             = ANGLE_LOOP,
                                                .motor_reverse_flag    = 1,
                                            },
                                        .motor_init_info = {.motor_type = DM4310, .gear_ratio = 10, .max_torque = 10, .torque_constant = 0.093f}};
    pitch_motor = Motor_DM_Init(&pitch_config, DM_MIT_MODE);
    if (pitch_motor == NULL)
    {
        return;
    }
    /* 初值, 之后由 pitch_gravity_feedforward() 每周期更新 */
    pitch_motor->base.controller.feedforward_torque = PITCH_FF_GRAV_C;

    /* VOFA 上行注册: p轴扭矩(Nm) / p轴欧拉角(rad) */
    VOFA_RegisterTx("p_torque", &g_pitch_torque);
    VOFA_RegisterTx("p_euler",  &g_pitch_euler);
}
int count=0;
void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    count++;
    /* VOFA 采样: 每周期刷新 p轴扭矩/欧拉角 (TX 线程每 10ms 自动上报) */
    if (pitch_motor != NULL) g_pitch_torque = pitch_motor->measure.torque;
    if (ins != NULL)         g_pitch_euler  = ins->euler_rad[1];

    if (gimbal_cmd != NULL)
    {
        if (!Module_Offline_get_device_status(yaw_motor->base.offline_dev) 
    && !Module_Offline_get_device_status(pitch_motor->base.offline_dev))
{
    if (gimbal_cmd->gimbal_mode == gimbal_zero_force)
        {
            Motor_Stop(&yaw_motor->base);
            Motor_Stop(&pitch_motor->base);
            
        }
    else
        {
            Motor_Start(&yaw_motor->base);
            Motor_SetRef(&yaw_motor->base, (gimbal_cmd->yaw * DEGREE_2_RAD));
           Motor_Start(&pitch_motor->base);
           Motor_SetRef(&pitch_motor->base, gimbal_cmd->pitch * DEGREE_2_RAD);
           pitch_gravity_feedforward();   /* 重力前馈: 消掉稳态误差与死区 */
        }
}
    // 数据反馈
    if (!Module_Offline_get_device_status(yaw_motor->base.offline_dev) && yaw_ecd != NULL)
    {
        *yaw_ecd = yaw_motor->measure.ecd;
    }
    }
            if(count==250){
            Motor_DM_Cmd(pitch_motor, DM_CMD_MOTOR_START); 
            count=0;
        }
}