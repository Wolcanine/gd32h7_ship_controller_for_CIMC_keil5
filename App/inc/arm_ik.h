// arm_ik.h — 4-DOF 机械臂逆运动学（地面目标 → 关节角度）
// 输入：地面目标点 (x, y, z=0)，输出：关节 0~3 偏离默认值的角度（度）
// 本模块不做限幅，限幅由 servo_arm 层负责
// 返回 0=成功, -1=几何不可达(太近/超出臂展)
// 关节 4(腕旋转) 和 5(夹爪) 不受 IK 影响
//
// 坐标系：基座为原点，X 轴正前方，Y 轴正左方
#ifndef ARM_IK_H
#define ARM_IK_H

#include "gd32h7xx.h"

/*!
    \brief   将地面目标点 (x, y, z=0) 转换为机械臂关节角度
    \param[in]  x  目标 X 坐标 (mm，正前方)
    \param[in]  y  目标 Y 坐标 (mm，正左方)
    \param[out] theta1  关节0(底座)角度（度，-90~90）
    \param[out] theta2  关节1(大臂)角度（度，-90~90）
    \param[out] theta3  关节2(小臂)角度（度，-90~90）
    \param[out] theta4  关节3(腕俯仰)角度（度，恒为 0 = 水平）
    \retval 0  成功
    \retval -1 目标不可达（超出运动范围）
*/
int ArmIK_Solve(double x, double y,
                double *theta1, double *theta2,
                double *theta3, double *theta4);

#endif
