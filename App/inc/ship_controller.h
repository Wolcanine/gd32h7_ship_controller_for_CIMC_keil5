// ship_controller.h
#ifndef SHIP_CONTROLLER_H
#define SHIP_CONTROLLER_H

#include "gd32h7xx.h"

typedef struct {
    // ---------- 可调参数 ----------
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float yaw_integral_limit;
    float max_yaw_rate_cmd;     // 遥控打满时期望的偏航角速度 (deg/s)
    float max_diff_duty;        // 转向产生的最大差速占空比 (0~1)

    // ---------- 死区参数 ----------
    float throttle_deadzone;
    float steering_deadzone;

    // ---------- 内部状态（禁止外部直接修改） ----------
    float yaw_integral;
    float prev_yaw_error;
} ShipController;

void ShipController_Init(ShipController *sc, float kp, float ki, float kd,
                         float int_limit, float max_yaw, float max_diff);

/**
 * @brief 核心更新函数
 * @param throttle  油门设定值 (-1.0 ~ 1.0，由 AutoNav 或 PS2 提供)
 * @param steering  转向设定值 (-1.0 ~ 1.0，由 AutoNav 或 PS2 提供)
 *        内部读取陀螺仪完成 PID 闭环，直接输出到电机 PWM。
 *        应在固定周期（50Hz）中调用。
 */
void ShipController_Update(ShipController *sc, float throttle, float steering);

#endif
