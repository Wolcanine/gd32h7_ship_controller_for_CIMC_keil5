/*******************************************************************************
 * 文件名          arm_ik.c
 * 描述            4-DOF 机械臂逆运动学解算 (XY → 关节角度)
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-19      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

// arm_ik.c — 4-DOF 机械臂逆运动学解算
// 给定地面目标 (x, y, z=0)，计算关节 0(底座)~3(腕俯仰) 角度
//
// 机械臂几何模型（单位：mm）：
//   基座 → 关节1(大臂) → 关节2(小臂) → 关节3(腕俯仰) → 夹爪中心
//
// 常数由实物测量标定：
//   E_x   关节3→夹爪中心的 X 偏移（负值=向基座侧偏）
//   E_y   夹爪中心的 Y 偏移（关节5偏置）
//   E_z   关节3→夹爪中心的 Z 向总长度
//   v12_z 关节1→关节2 的 Z 向长度
#include "arm_ik.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

#define DEG2RAD  (M_PI / 180.0)
#define RAD2DEG  (180.0 / M_PI)

#define JOINT_LIMIT_DEG  90.0   // 各关节 ±90° 限位

int ArmIK_Solve(double x, double y,
                double *theta1, double *theta2,
                double *theta3, double *theta4)
{
    // ==================== 机械臂几何常数（实物测量标定）====================
    const double E_x   = -26.14;   // 关节3→夹爪 X 偏移
    const double E_y   = -11.5;    // 夹爪 Y 偏移
    const double E_z   = 148.00;   // 关节3→夹爪 Z 向总长
    const double v12_z = 20.87;    // 关节1→关节2 Z 向长度

    const double joint_limit = JOINT_LIMIT_DEG * DEG2RAD;

    // 1. 柱坐标：水平距离 r 和方位角 phi
    double r = sqrt(x * x + y * y);
    if (r < fabs(E_y) + 1e-6) {
        return -1;  // 目标太近，无法补偿 Y 偏移
    }
    double phi = atan2(y, x);

    // 2. 关节0(底座)角度 θ1
    double sin_psi = fabs(E_y) / r;
    if (sin_psi > 1.0 + 1e-9) return -1;
    double psi = asin(sin_psi);

    double t1_cand[2];
    t1_cand[0] = phi - psi;
    t1_cand[1] = phi - M_PI + psi;

    int t1_ok = 0;
    *theta1 = 0.0;
    for (int i = 0; i < 2; i++) {
        // 规范化到 [-π, π]
        double a = fmod(t1_cand[i], 2.0 * M_PI);
        if (a > M_PI)  a -= 2.0 * M_PI;
        if (a < -M_PI) a += 2.0 * M_PI;
        if (fabs(a) <= joint_limit + 1e-6) {
            *theta1 = a;
            t1_ok = 1;
            break;
        }
    }
    if (!t1_ok) return -1;

    // 3. 臂平面内的水平到达距离 Hx（正值 = 向前伸展）
    double Hx = sqrt(r * r - E_y * E_y);

    // 4. 关节2(小臂)角度 θ3 — 使用余弦定理+双解
    double Ex2_Ez2 = E_x * E_x + E_z * E_z;
    double M = Ex2_Ez2 + v12_z * v12_z;
    double N = 2.0 * v12_z * sqrt(Ex2_Ez2);
    double delta = atan2(-E_x, E_z);  // 注意 atan2(y, x) 参数顺序

    double val = (Hx * Hx - M) / N;
    if (fabs(val) > 1.0 + 1e-6) return -1;
    if (val > 1.0)  val = 1.0;
    if (val < -1.0) val = -1.0;

    double alpha = asin(val);
    double t3_cand[2];
    t3_cand[0] = alpha - delta;
    t3_cand[1] = M_PI - alpha - delta;

    int t3_ok = 0;
    *theta3 = 0.0;
    for (int i = 0; i < 2; i++) {
        double a = fmod(t3_cand[i], 2.0 * M_PI);
        if (a > M_PI)  a -= 2.0 * M_PI;
        if (a < -M_PI) a += 2.0 * M_PI;
        if (fabs(a) <= joint_limit + 1e-6) {
            *theta3 = a;
            t3_ok = 1;
            break;
        }
    }
    if (!t3_ok) return -1;

    // 5. 从 θ3 计算中间量 Gx, Gz
    double Gx = E_x * cos(*theta3) + E_z * sin(*theta3);
    double Gz = -E_x * sin(*theta3) + E_z * cos(*theta3) + v12_z;

    // 6. 关节1(大臂)角度 θ2（z=0 自动满足）
    *theta2 = atan2(Gz, Gx);
    if (fabs(*theta2) > joint_limit + 1e-6) return -1;

    // 7. 关节3(腕俯仰)始终水平（如需接地可调整）
    *theta4 = 0.0;

    // 转角度制
    *theta1 *= RAD2DEG;
    *theta2 *= RAD2DEG;
    *theta3 *= RAD2DEG;

    return 0;
}
