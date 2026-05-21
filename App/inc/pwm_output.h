/*******************************************************************************
 * 文件名          pwm_output.h
 * 描述            双路电机驱动接口 (IN1/IN2/ENA 三线驱动板)
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-07      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include "gd32h7xx.h"

/* ==================== 引脚配置 ==================== */
#define MOTOR_LEFT_ENA_PORT     GPIOA
#define MOTOR_LEFT_ENA_PIN      GPIO_PIN_6
#define MOTOR_LEFT_IN1_PORT     GPIOC
#define MOTOR_LEFT_IN1_PIN      GPIO_PIN_6
#define MOTOR_LEFT_IN2_PORT     GPIOC
#define MOTOR_LEFT_IN2_PIN      GPIO_PIN_0

#define MOTOR_RIGHT_ENA_PORT    GPIOA
#define MOTOR_RIGHT_ENA_PIN     GPIO_PIN_7
#define MOTOR_RIGHT_IN1_PORT    GPIOC
#define MOTOR_RIGHT_IN1_PIN     GPIO_PIN_7
#define MOTOR_RIGHT_IN2_PORT    GPIOC
#define MOTOR_RIGHT_IN2_PIN     GPIO_PIN_1

#define PWM_MAX_DUTY        0.25f   /* 低速航行限制 */

void pwm_output_init(void);
void pwm_set_left_duty(float duty);     /* -1.0 ~ 1.0 */
void pwm_set_right_duty(float duty);    /* -1.0 ~ 1.0 */

#endif
