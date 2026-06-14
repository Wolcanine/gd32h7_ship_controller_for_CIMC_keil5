// pca9685.h — I2C PWM 发生器驱动 (PCA9685)
// 基于 Adafruit PCA9685 库移植，使用现有 MyI2C 软件 I2C 驱动
// 与 MPU6050 共享 I2C 总线（PE13/PE15）
#ifndef PCA9685_H
#define PCA9685_H

#include "gd32h7xx.h"

#define PCA9685_I2C_ADDR        0x40        // 默认 I2C 地址（ADDR 悬空/拉低）
#define PCA9685_SERVO_FREQ      50          // 舵机标准频率 50Hz
#define PCA9685_SERVO_ANGLE_MAX 180.0f      // 舵机最大角度范围

/* ==================== OE 使能引脚 ==================== */
/* PCA9685 OE 引脚 — 低电平有效: L=输出使能, H=输出关闭(高阻)
 * PC12 (J4-47) 原为旧电机驱动板 ENA，已释放，复用为舵机使能 */
#define PCA9685_OE_PORT         GPIOC
#define PCA9685_OE_PIN          GPIO_PIN_12
#define PCA9685_OE_RCU          RCU_GPIOC

// ==================== 初始化与配置 ====================
void pca9685_init(uint8_t addr);
void pca9685_set_pwm_freq(uint8_t addr, float freq);

// ==================== 输出使能控制 ====================
void pca9685_output_enable(void);           /* OE=L, 使能舵机输出      */
void pca9685_output_disable(void);          /* OE=H, 关闭舵机输出(省电) */

// ==================== PWM 输出 ====================
void pca9685_set_pwm(uint8_t addr, uint8_t ch, uint16_t on, uint16_t off);
void pca9685_set_pin(uint8_t addr, uint8_t ch, uint16_t val, uint8_t invert);

// ==================== 舵机角度换算工具 ====================
// 将角度 (0~180°) 转换为对应的脉宽计数值
// servo_min/max 对应 0°/180° 的脉宽计数值，默认 150/600
uint16_t pca9685_angle_to_pulse(float angle, uint16_t servo_min, uint16_t servo_max);

#endif
