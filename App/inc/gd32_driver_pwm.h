/*!
 *  \file    gd32_driver_pwm.h
 *  \brief   软件 PWM 模块
 *
 *  \version 2026-05-05, V1.0.0
 *
 *  基于定时器更新中断的软件 PWM，可驱动任意 GPIO 引脚。
 *  适用于引脚无硬件 PWM 通道的场景 (如 PB12/PB13)。
 *
 *  定时器资源: 默认使用 TIMER4
 *  ISR 频率: PWM_RESOLUTION × PWM_FREQ (默认 200kHz)
 *  PWM 频率: 默认 200Hz (周期 5ms, 1000 级占空比)
 *
 *  使用示例:
 *    pwm_init(GPIOB, GPIO_PIN_12);              // PB12 初始化为 PWM 输出
 *    pwm_init(GPIOB, GPIO_PIN_13);              // PB13 初始化为 PWM 输出
 *    pwm_set_duty(GPIOB, GPIO_PIN_12, 500);     // PB12 占空比 50%
 *
 *    // 在 gd32h7xx_it.c 中添加:
 *    void TIMER4_IRQHandler(void) { pwm_isr_handler(); }
 */

#ifndef _GD32_DRIVER_PWM_H_
#define _GD32_DRIVER_PWM_H_

#include "gd32h7xx.h"

/* ---------- PWM 参数宏 ---------- */
#define PWM_TIMER           TIMER4              /* 使用的定时器                 */
#define PWM_RESOLUTION      1000                /* 占空比分辨率 (0-999)         */
#define PWM_HW_PERIOD       4                   /* 硬件定时器周期 (200kHz ISR)  */
                                                /* 200kHz / 1000 = 200Hz PWM    */
#define PWM_DUTY_MAX        (PWM_RESOLUTION - 1)/* 最大占空比值: 999            */

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_init
// 描述:     初始化一个引脚为软件 PWM 输出
// 参数:     gpio_port   GPIO 端口 (GPIOA / GPIOB / ...)
//           gpio_pin    GPIO 引脚 (GPIO_PIN_0 ~ GPIO_PIN_15)
// 返回值:   void
// 示例:     pwm_init(GPIOB, GPIO_PIN_12);
// 备注:     引脚配置为推挽输出，初始高电平（共阳 LED 熄灭）。
//           首次调用时自动配置 TIMER4 作为 PWM 时基。
//           最多支持 16 个引脚（同一端口）。
//---------------------------------------------------------------------------------------------------------------------
void pwm_init(uint32_t gpio_port, uint16_t gpio_pin);

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_set_duty
// 描述:     设置指定引脚的 PWM 占空比
// 参数:     gpio_port   GPIO 端口
//           gpio_pin    GPIO 引脚
//           duty        占空比 (0 ~ PWM_DUTY_MAX，0=全灭, 999=全亮)
// 返回值:   void
// 示例:     pwm_set_duty(GPIOB, GPIO_PIN_12, 500);   // 50% 占空比
//---------------------------------------------------------------------------------------------------------------------
void pwm_set_duty(uint32_t gpio_port, uint16_t gpio_pin, uint16_t duty);

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_gamma_correct
// 描述:     Gamma 校正：将线性亮度映射为人眼感知均匀的亮度
// 参数:     linear      线性占空比 (0 ~ max)
//           max         最大值
// 返回值:   uint16_t    校正后的值
// 示例:     duty = pwm_gamma_correct(500, PWM_DUTY_MAX);
// 备注:     使用平方 gamma (γ≈2.0)，使人眼对呼吸亮度的感知更加平滑
//---------------------------------------------------------------------------------------------------------------------
uint16_t pwm_gamma_correct(uint16_t linear, uint16_t max);

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_isr_handler
// 描述:     PWM 中断服务函数（在 TIMER4_IRQHandler 中调用）
// 参数:     void
// 返回值:   void
// 示例:     void TIMER4_IRQHandler(void) { pwm_isr_handler(); }
// 备注:     共阳 LED: 低电平点亮，高电平熄灭
//---------------------------------------------------------------------------------------------------------------------
void pwm_isr_handler(void);

#endif /* _GD32_DRIVER_PWM_H_ */
