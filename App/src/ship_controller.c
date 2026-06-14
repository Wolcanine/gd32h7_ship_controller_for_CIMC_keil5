/*******************************************************************************
 * 文件名          ship_controller.c
 * 描述            船舶偏航 PID 闭环控制器
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-07      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

// ship_controller.c
#include "ship_controller.h"
#include "MPU6050.h"
#include "pwm_output.h"
#include <math.h>

// 从 MPU6050 Z 轴角速度缓存（0.01°/s）转换为 deg/s 浮点数
static inline float mpu6050_get_yaw_rate(void)
{
    return (float)MPU6050_GetGyroZ_dps100() / 100.0f;
}

void ShipController_Init(ShipController *sc, float kp, float ki, float kd,
                         float int_limit, float max_yaw, float max_diff)
{
    sc->yaw_kp = kp;
    sc->yaw_ki = ki;
    sc->yaw_kd = kd;
    sc->yaw_integral_limit = int_limit;
    sc->max_yaw_rate_cmd = max_yaw;
    sc->max_diff_duty = max_diff;

    sc->throttle_deadzone = 0.05f;
    sc->steering_deadzone = 0.05f;

    sc->yaw_integral = 0.0f;
    sc->prev_yaw_error = 0.0f;
}

static inline float apply_deadzone(float value, float threshold)
{
    if (value > threshold)
        return (value - threshold) / (1.0f - threshold);
    else if (value < -threshold)
        return (value + threshold) / (1.0f - threshold);
    else
        return 0.0f;
}

void ShipController_Update(ShipController *sc, float throttle, float steering)
{
    // 1. 获取偏航角速度（MPU6050 失效时跳过 PID，直接透传油门）
    float measured_yaw;
    if (MPU6050_IsHealthy()) {
        measured_yaw = mpu6050_get_yaw_rate();
    } else {
        sc->yaw_integral = 0.0f;       // 防止积分饱和残留
        sc->prev_yaw_error = 0.0f;
        pwm_set_left_duty(throttle);
        pwm_set_right_duty(throttle);
        return;
    }

    // 2. 死区处理
    float thr = apply_deadzone(throttle, sc->throttle_deadzone);
    float str = apply_deadzone(steering, sc->steering_deadzone);

    // 3. 基础推力
    float base_duty = thr;

    // 4. 期望偏航角速度
    float desired_yaw_rate = str * sc->max_yaw_rate_cmd;

    // 5. PID 计算差速
    float yaw_error = desired_yaw_rate - measured_yaw;

    float p_out = sc->yaw_kp * yaw_error;

    // 积分（带限幅）
    sc->yaw_integral += yaw_error;
    if (sc->yaw_integral > sc->yaw_integral_limit)
        sc->yaw_integral = sc->yaw_integral_limit;
    else if (sc->yaw_integral < -sc->yaw_integral_limit)
        sc->yaw_integral = -sc->yaw_integral_limit;
    float i_out = sc->yaw_ki * sc->yaw_integral;

    // 微分
    float d_out = sc->yaw_kd * (yaw_error - sc->prev_yaw_error);
    sc->prev_yaw_error = yaw_error;

    float diff = p_out + i_out + d_out;

    // 差速限幅
    if (diff > sc->max_diff_duty)
        diff = sc->max_diff_duty;
    if (diff < -sc->max_diff_duty)
        diff = -sc->max_diff_duty;

    // 抗积分饱和
    if ((diff >= sc->max_diff_duty && yaw_error > 0) ||
        (diff <= -sc->max_diff_duty && yaw_error < 0)) {
        sc->yaw_integral -= yaw_error;
        if (sc->yaw_integral > sc->yaw_integral_limit)
            sc->yaw_integral = sc->yaw_integral_limit;
        else if (sc->yaw_integral < -sc->yaw_integral_limit)
            sc->yaw_integral = -sc->yaw_integral_limit;
    }

    // 6. 左右双电机组分配
    float left  = base_duty - diff;
    float right = base_duty + diff;

    // 总占空比限幅
    float max_abs = fabsf(left);
    if (fabsf(right) > max_abs) max_abs = fabsf(right);
    if (max_abs > 1.0f) {
        float scale = 1.0f / max_abs;
        left  *= scale;
        right *= scale;
    }

    // 7. 直接通过 pwm_output 模块输出
    pwm_set_left_duty(left);
    pwm_set_right_duty(right);
}
