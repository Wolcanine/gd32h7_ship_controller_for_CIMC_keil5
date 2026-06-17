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
 * 2026-06-17      CIMC            移除关节限幅，限幅由 servo_arm 层负责
 ******************************************************************************/

// arm_ik.c — 4-DOF 机械臂逆运动学解算
// 给定地面目标 (x, y, z=0)，计算关节 0(底座)~3(腕俯仰) 偏离默认值的角度
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

int ArmIK_Solve(double x, double y,
                double *theta1, double *theta2,
                double *theta3, double *theta4)
{
    const double E_x   = -26.14;
    const double E_y   = -11.5;
    const double E_z   = 148.00;
    const double v12_z = 20.87;

    // ---- 1. 柱坐标 ----
    double r = sqrt(x * x + y * y);
    if (r < fabs(E_y) + 1e-6) return -1;

    double phi = atan2(y, x);

    // ---- 2. 底座角度 θ1 ----
    double psi = asin(fabs(E_y) / r);

    double t1_cand[2] = { phi - psi, phi - M_PI + psi };

    // ---- 3. 臂平面内水平到达距离 ----
    double Hx = sqrt(r * r - E_y * E_y);

    // ---- 4. 小臂角度 θ3 (余弦定理+双解) ----
    double Ex2_Ez2 = E_x * E_x + E_z * E_z;
    double M_val   = Ex2_Ez2 + v12_z * v12_z;
    double N_val   = 2.0 * v12_z * sqrt(Ex2_Ez2);
    double delta   = atan2(-E_x, E_z);

    double val = (Hx * Hx - M_val) / N_val;
    if (fabs(val) > 1.0 + 1e-6) return -1;
    if (val > 1.0)  val = 1.0;
    if (val < -1.0) val = -1.0;

    double alpha = asin(val);
    double t3_cand[2] = { alpha - delta, M_PI - alpha - delta };

    // ---- 遍历 θ1 和 θ3 的候选组合，选第一组满足约束的解 ----
    for (int i = 0; i < 2; i++) {
        double t1 = fmod(t1_cand[i], 2.0 * M_PI);
        if (t1 > M_PI)  t1 -= 2.0 * M_PI;
        if (t1 < -M_PI) t1 += 2.0 * M_PI;

        for (int j = 0; j < 2; j++) {
            double t3 = fmod(t3_cand[j], 2.0 * M_PI);
            if (t3 > M_PI)  t3 -= 2.0 * M_PI;
            if (t3 < -M_PI) t3 += 2.0 * M_PI;

            // ---- 5. 大臂角度 θ2 ----
            double Gx = E_x * cos(t3) + E_z * sin(t3);
            double Gz = -E_x * sin(t3) + E_z * cos(t3) + v12_z;
            double t2 = atan2(Gz, Gx);

            // ---- 6. 腕俯仰 ----
            double t4 = 0.0;

            // ---- 约束: 大臂/小臂/腕俯仰偏离 >= 0 ----
            if (t2 < 0.0 || t3 < 0.0 || t4 < 0.0) continue;

            // ---- 转度数 ----
            *theta1 = t1 * RAD2DEG;
            *theta2 = t2 * RAD2DEG;
            *theta3 = t3 * RAD2DEG;
            *theta4 = t4 * RAD2DEG;
            return 0;
        }
    }

    return -1;
}
