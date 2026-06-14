/*******************************************************************************
 * 文件名          pwm_output.h
 * 描述            四电机直流驱动接口 — 双引脚 H 桥控制，两板并联
 *                 左侧双电机并联: D0/D1  |  右侧双电机并联: D2/D3
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 驱动板控制逻辑
 *                 正转调速:  (Dx=1/PWM,  Dy=0)
 *                 反转调速:  (Dx=0,      Dy=1/PWM)
 *                 自由滑行:  (Dx=0,      Dy=0)
 *                 快速刹车:  (Dx=1,      Dy=1)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-07      AI助手          初始版本 (IN1/IN2/ENA 三线制)
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 * 2026-06-03      CIMC            引脚重分配：避开摄像头/LCD/SDRAM 冲突
 * 2026-06-11      CIMC            电机引脚就近归并：PC5+PF9 替代 PD4+PG3
 * 2026-06-12      CIMC            更换驱动板：三线→双线H桥，4引脚全部迁至GPIOC
 ******************************************************************************/

#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include "gd32h7xx.h"

/* ==================== 引脚配置 ==================== */
/* 四电机驱动 — 两块双路 H 桥驱动板，同侧双电机共用控制信号（并联）
 *   左侧双电机 MOTOR1: D0 (正转PWM) / D1 (反转PWM)    → 驱动板A CH1+CH2 并联
 *   右侧双电机 MOTOR2: D2 (正转PWM) / D3 (反转PWM)    → 驱动板B CH1+CH2 并联
 *
 *   约束: gd32_driver_pwm 软件 PWM 要求所有引脚在同一 GPIO 端口
 *         → 4 个引脚全部选在 GPIOC
 *
 *   PC10 原为 UART_CAM TX (J4-51)，双MCU方案已废弃，复用为 D3
 */
#define MOTOR_LEFT_D0_PORT     GPIOC
#define MOTOR_LEFT_D0_PIN      GPIO_PIN_2       /* J4-32, D0 — 左侧双电机 正转 PWM (驱动板A CH1+CH2 并联) */
#define MOTOR_LEFT_D1_PORT     GPIOC
#define MOTOR_LEFT_D1_PIN      GPIO_PIN_3       /* J4-33, D1 — 左侧双电机 反转 PWM (驱动板A CH1+CH2 并联) */

#define MOTOR_RIGHT_D2_PORT    GPIOC
#define MOTOR_RIGHT_D2_PIN     GPIO_PIN_5       /* J4-34, D2 — 右侧双电机 正转 PWM (驱动板B CH1+CH2 并联) */
#define MOTOR_RIGHT_D3_PORT    GPIOC
#define MOTOR_RIGHT_D3_PIN     GPIO_PIN_10      /* J4-51, D3 — 右侧双电机 反转 PWM (驱动板B CH1+CH2 并联, 原 UART_CAM TX) */

#define PWM_MAX_DUTY        0.25f   /* 低速航行限制 */

void pwm_output_init(void);
void pwm_set_left_duty(float duty);     /* -1.0~1.0, 左侧双电机组 */
void pwm_set_right_duty(float duty);    /* -1.0~1.0, 右侧双电机组 */

#endif
