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
 * 2026-06-03      CIMC            引脚重分配：避开摄像头/LCD/SDRAM 冲突
 * 2026-06-11      CIMC            电机引脚就近归并：PC5+PF9 替代 PD4+PG3
 ******************************************************************************/

#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include "gd32h7xx.h"

/* ==================== 引脚配置 ==================== */
/* 2026-06-11  就近归并：左电机三连排(J4-32/33/34)，右电机集中(J4-41/47/48) */
/*             注意: ENA1/ENA2 必须在同一 GPIO 端口 (gd32_driver_pwm.c 单端口限制) */
#define MOTOR_LEFT_ENA_PORT     GPIOC
#define MOTOR_LEFT_ENA_PIN      GPIO_PIN_2       /* J4-32, PC2 */
#define MOTOR_LEFT_IN1_PORT     GPIOC
#define MOTOR_LEFT_IN1_PIN      GPIO_PIN_3       /* J4-33, PC3 */
#define MOTOR_LEFT_IN2_PORT     GPIOC
#define MOTOR_LEFT_IN2_PIN      GPIO_PIN_5       /* J4-34, PC5 (原 PD4 J4-22, 迁至三连排) */

#define MOTOR_RIGHT_ENA_PORT    GPIOC
#define MOTOR_RIGHT_ENA_PIN     GPIO_PIN_12      /* J4-47, PC12 (与 ENA1 同端口 GPIOC) */
#define MOTOR_RIGHT_IN1_PORT    GPIOF
#define MOTOR_RIGHT_IN1_PIN     GPIO_PIN_9       /* J4-41, PF9 (原 PG3 J4-57, 迁近 PC12/PD2) */
#define MOTOR_RIGHT_IN2_PORT    GPIOD
#define MOTOR_RIGHT_IN2_PIN     GPIO_PIN_2       /* J4-48, PD2 */

#define PWM_MAX_DUTY        0.25f   /* 低速航行限制 */

void pwm_output_init(void);
void pwm_set_left_duty(float duty);     /* -1.0 ~ 1.0 */
void pwm_set_right_duty(float duty);    /* -1.0 ~ 1.0 */

#endif
