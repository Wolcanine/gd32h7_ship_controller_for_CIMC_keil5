/*******************************************************************************
 * 文件名          pwm_output.c
 * 描述            双路电机 PWM 输出 — IN1/IN2/ENA 三线驱动板
 *                 正反转切换自动插入 100ms 刹车保护
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-07      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

// pwm_output.c
// 电机 PWM 输出实现：适配 IN1+IN2+ENA 三线驱动板
// 正反转切换自动插入 100ms 刹车保护
#include "pwm_output.h"
#include "gd32_driver_pwm.h"

// 映射系数：1.0 → PWM_DUTY_MAX
#define DUTY_SCALE  ((float)PWM_DUTY_MAX)

// 方向切换刹车保持周期数（50Hz × 5 = 100ms）
#define BRAKE_CYCLES 5

// ---------- 单个电机的方向切换状态 ----------
typedef struct {
    int8_t  brake_cnt;      // >0: 正在刹车倒计时
    float   last_duty;      // 上次正常输出时的 duty（用于方向突变检测）
} MotorSwitchState;

static MotorSwitchState left_sw  = {0, 0.0f};
static MotorSwitchState right_sw = {0, 0.0f};

// ---------- 内部：设置引脚电平 ----------
static inline void pin_write(uint32_t port, uint16_t pin, uint8_t val)
{
    if (val) gpio_bit_set(port, pin);
    else     gpio_bit_reset(port, pin);
}

// ---------- 内部：使能 GPIO 时钟 ----------
static void gpio_clk_enable(uint32_t port)
{
    if      (port == GPIOA) rcu_periph_clock_enable(RCU_GPIOA);
    else if (port == GPIOB) rcu_periph_clock_enable(RCU_GPIOB);
    else if (port == GPIOC) rcu_periph_clock_enable(RCU_GPIOC);
    else if (port == GPIOD) rcu_periph_clock_enable(RCU_GPIOD);
    else if (port == GPIOE) rcu_periph_clock_enable(RCU_GPIOE);
}

// ---------- 内部：初始化一个输出引脚 ----------
static void out_pin_init(uint32_t port, uint16_t pin)
{
    if (pin == 0) return;
    gpio_clk_enable(port);
    gpio_mode_set(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
    gpio_output_options_set(port, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, pin);
    gpio_bit_reset(port, pin);  // 默认低电平
}

// ---------- 内部：设置单个电机的 IN1/IN2/ENA ----------
static void pwm_set_motor_duty(
    uint32_t ena_port, uint16_t ena_pin,
    uint32_t in1_port, uint16_t in1_pin,
    uint32_t in2_port, uint16_t in2_pin,
    float duty, MotorSwitchState *sw)
{
    uint8_t  in1, in2;
    uint16_t pwm_val;

    // --- 钳位（低速航行限制 PWM_MAX_DUTY） ---
    if (duty > PWM_MAX_DUTY)   duty = PWM_MAX_DUTY;
    if (duty < -PWM_MAX_DUTY)  duty = -PWM_MAX_DUTY;

    if (sw->brake_cnt > 0) {
        // 仍在刹车保持阶段
        sw->brake_cnt--;
        in1 = 0; in2 = 0;
        pwm_val = 0;
    }
    else if (duty == 0.0f) {
        // 停止 → 刹车
        in1 = 0; in2 = 0;
        pwm_val = 0;
    }
    else {
        // 检测方向突变（正↔反转切换）
        if ((sw->last_duty > 0 && duty < 0) || (sw->last_duty < 0 && duty > 0)) {
            sw->brake_cnt = BRAKE_CYCLES;
            in1 = 0; in2 = 0;
            pwm_val = 0;
        }
        else {
            // 正常方向输出
            if (duty > 0) { in1 = 1; in2 = 0; }  // 正转
            else          { in1 = 0; in2 = 1; }  // 反转

            pwm_val = (uint16_t)((duty >= 0.0f ? duty : -duty) * DUTY_SCALE);
            if (pwm_val > PWM_DUTY_MAX) pwm_val = PWM_DUTY_MAX;
        }
    }

    // --- 写 GPIO ---
    pin_write(in1_port, in1_pin, in1);
    pin_write(in2_port, in2_pin, in2);
    pwm_set_duty(ena_port, ena_pin, pwm_val);

    // --- 更新 last_duty（仅正常输出状态时记录，刹车时不更新） ---
    if (sw->brake_cnt == 0) {
        sw->last_duty = duty;
    }
}

// ==================== 公共 API ====================

void pwm_output_init(void)
{
    // ENA PWM 引脚
    pwm_init(MOTOR_LEFT_ENA_PORT,  MOTOR_LEFT_ENA_PIN);
    pwm_init(MOTOR_RIGHT_ENA_PORT, MOTOR_RIGHT_ENA_PIN);

    // IN1、IN2 方向引脚
    out_pin_init(MOTOR_LEFT_IN1_PORT,  MOTOR_LEFT_IN1_PIN);
    out_pin_init(MOTOR_LEFT_IN2_PORT,  MOTOR_LEFT_IN2_PIN);
    out_pin_init(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN);
    out_pin_init(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN);

    // 初始状态：刹车（IN1=0, IN2=0）
    pin_write(MOTOR_LEFT_IN1_PORT,  MOTOR_LEFT_IN1_PIN,  0);
    pin_write(MOTOR_LEFT_IN2_PORT,  MOTOR_LEFT_IN2_PIN,  0);
    pin_write(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, 0);
    pin_write(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, 0);

    left_sw.brake_cnt  = 0;
    left_sw.last_duty  = 0.0f;
    right_sw.brake_cnt = 0;
    right_sw.last_duty = 0.0f;
}

void pwm_set_left_duty(float duty)
{
    pwm_set_motor_duty(
        MOTOR_LEFT_ENA_PORT,  MOTOR_LEFT_ENA_PIN,
        MOTOR_LEFT_IN1_PORT,  MOTOR_LEFT_IN1_PIN,
        MOTOR_LEFT_IN2_PORT,  MOTOR_LEFT_IN2_PIN,
        duty, &left_sw);
}

void pwm_set_right_duty(float duty)
{
    pwm_set_motor_duty(
        MOTOR_RIGHT_ENA_PORT,  MOTOR_RIGHT_ENA_PIN,
        MOTOR_RIGHT_IN1_PORT,  MOTOR_RIGHT_IN1_PIN,
        MOTOR_RIGHT_IN2_PORT,  MOTOR_RIGHT_IN2_PIN,
        duty, &right_sw);
}
