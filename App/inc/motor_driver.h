/*******************************************************************************
 * 文件名          motor_driver.h
 * 描述            电机驱动抽象接口 — 统一 GPIOC 软件 PWM 和 PCA9685 I2C PWM
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 编译开关
 *   MOTOR_USE_PCA9685  未定义 = GPIOC 软件 PWM (pwm_output.c)
 *                      定义   = PCA9685 I2C PWM @ 244Hz (CH6~CH9)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-07-01      CIMC           初始版本：抽象 GPIOC / PCA9685 双后端
 ******************************************************************************/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "gd32h7xx.h"

/* ==================== 电机驱动后端选择 ==================== */
/* 注释 = GPIOC 软件 PWM，取消注释 = PCA9685 I2C PWM */
/* #define MOTOR_USE_PCA9685 */

/* ==================== API ==================== */

/**
 * @brief 初始化电机驱动硬件
 *        GPIOC 模式: 初始化软件 PWM 四路引脚 (PC2/PC3/PC5/PC10)
 *        PCA9685 模式: 初始化 I2C + PCA9685 @ 244Hz, 通道 CH6~CH9
 */
void motor_init(void);

/**
 * @brief 设置左侧双电机目标占空比
 * @param duty  -1.0 ~ +1.0 (正=前进, 负=倒车, 0=滑行)
 *              内部映射: [THR_MIN_RUN, 1.0]，带启动突加和方向切换保护
 */
void motor_set_left_duty(float duty);

/**
 * @brief 设置右侧双电机目标占空比
 * @param duty  -1.0 ~ +1.0
 */
void motor_set_right_duty(float duty);

/**
 * @brief 每帧油门状态机更新 (50Hz 调用)
 *        处理缓启动、启动突加、方向切换刹车，提交实际占空比到硬件
 */
void motor_throttle_tick(void);

/**
 * @brief 紧急停车 — 所有电机立即制动（短路刹车）
 */
void motor_stop(void);

#endif /* MOTOR_DRIVER_H */
