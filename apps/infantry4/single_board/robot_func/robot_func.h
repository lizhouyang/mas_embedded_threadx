/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-17 20:22:14
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-19 09:23:10
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_func\robot_func.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_
#include "infantry_def.h" 
void RemoteControlSet( Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);
int16_t CalcOffsetAngle(float getyawangle);
#endif // _ROBOT_FUNC_H_

