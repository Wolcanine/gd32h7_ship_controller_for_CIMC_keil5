/*******************************************************************************
 * 文件名          motor_driver.c
 * 描述            电机驱动抽象层实现 — GPIOC 软件 PWM / PCA9685 I2C PWM
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-07-01      CIMC           初始版本
 ******************************************************************************/

#include "motor_driver.h"

/* =================================================================================
 *  GPIOC 软件 PWM 路径 (默认)
 * ================================================================================= */
#ifndef MOTOR_USE_PCA9685

#include "pwm_output.h"

void motor_init(void)
{
    pwm_output_init();
}

void motor_set_left_duty(float duty)
{
    pwm_set_left_duty(duty);
}

void motor_set_right_duty(float duty)
{
    pwm_set_right_duty(duty);
}

void motor_throttle_tick(void)
{
    pwm_throttle_tick();
}

void motor_stop(void)
{
    /* 两侧立即归零 + 提交到硬件 */
    pwm_set_left_duty(0.0f);
    pwm_set_right_duty(0.0f);
    pwm_throttle_tick();
}

/* =================================================================================
 *  PCA9685 I2C PWM 路径 (MOTOR_USE_PCA9685 定义时)
 * ================================================================================= */
#else

#include "MyI2C.h"
#include "pca9685.h"

/* ---- PCA9685 电机通道分配 (与舵机 CH0~CH5 共享芯片) ---- */
#define PCA_MOTOR_CH_LF   6    /* 左电机正转 */
#define PCA_MOTOR_CH_LR   7    /* 左电机反转 */
#define PCA_MOTOR_CH_RF   8    /* 右电机正转 */
#define PCA_MOTOR_CH_RR   9    /* 右电机反转 */

#define PCA_MOTOR_FREQ    244.0f
#define PCA_MOTOR_MAX     1024    /* 25% 占空比上限 @ 4096 step */

#define PCA9685_CHIP_ADDR  PCA9685_I2C_ADDR

/* ---- 油门状态 (简化版, 无缓启动 — 可后续添加) ---- */
static float left_target  = 0.0f;
static float right_target = 0.0f;

/* ---- 单路 H 桥输出 ---- */
static void motor_output_one(uint8_t ch_fwd, uint8_t ch_rev, float duty)
{
    if (duty == 0.0f) {
        /* 滑行 (0, 0) */
        pca9685_set_pwm(PCA9685_CHIP_ADDR, ch_fwd, 0, 0);
        pca9685_set_pwm(PCA9685_CHIP_ADDR, ch_rev, 0, 0);
    } else if (duty > 0.0f) {
        /* 正转 (PWM, 0) */
        uint16_t p = (uint16_t)(duty * (float)PCA_MOTOR_MAX);
        pca9685_set_pwm(PCA9685_CHIP_ADDR, ch_fwd, 0, p);
        pca9685_set_pwm(PCA9685_CHIP_ADDR, ch_rev, 0, 0);
    } else {
        /* 反转 (0, PWM) */
        uint16_t p = (uint16_t)(-duty * (float)PCA_MOTOR_MAX);
        pca9685_set_pwm(PCA9685_CHIP_ADDR, ch_fwd, 0, 0);
        pca9685_set_pwm(PCA9685_CHIP_ADDR, ch_rev, 0, p);
    }
}

/* ---- API ---- */
void motor_init(void)
{
    MyI2C_Init();
    pca9685_init(PCA9685_CHIP_ADDR);
    pca9685_set_pwm_freq(PCA9685_CHIP_ADDR, PCA_MOTOR_FREQ);

    /* 初始滑行 */
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_LF, 0, 0);
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_LR, 0, 0);
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_RF, 0, 0);
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_RR, 0, 0);
}

void motor_set_left_duty(float duty)
{
    left_target = duty;
}

void motor_set_right_duty(float duty)
{
    right_target = duty;
}

void motor_throttle_tick(void)
{
    motor_output_one(PCA_MOTOR_CH_LF, PCA_MOTOR_CH_LR, left_target);
    motor_output_one(PCA_MOTOR_CH_RF, PCA_MOTOR_CH_RR, right_target);
}

void motor_stop(void)
{
    /* 刹车: 四个通道全部拉到 ON=4096 (短路制动) */
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_LF, 4096, 0);
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_LR, 4096, 0);
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_RF, 4096, 0);
    pca9685_set_pwm(PCA9685_CHIP_ADDR, PCA_MOTOR_CH_RR, 4096, 0);
    left_target  = 0.0f;
    right_target = 0.0f;
}

#endif /* MOTOR_USE_PCA9685 */
