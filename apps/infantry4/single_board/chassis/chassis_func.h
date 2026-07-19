/*
 * @Author: 陈冠宁 635936689@qq.com
 * @Date: 2026-07-18 18:57:44
 * @LastEditors: 陈冠宁 635936689@qq.com
 * @LastEditTime: 2026-07-19 09:13:04
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\chassis\chassis_func.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CHASSIS_FUNC_H_
#define _CHASSIS_FUNC_H_

#include "infantry_def.h"


void chassis_init(void);

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd);

#endif // _CHASSIS_FUNC_H_