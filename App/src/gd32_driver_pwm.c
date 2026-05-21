/*!
 *  \file    gd32_driver_pwm.c
 *  \brief   软件 PWM 模块实现
 *
 *  \version 2026-05-05, V1.0.0
 *
 *  通过 TIMER4 更新中断在 GPIO 引脚上产生 PWM 波形。
 *  适用于无硬件 PWM 通道的引脚 (PB12/PB13 等)。
 */

#include "gd32_driver_pwm.h"

/* ---------- 获取 TIMER4 实际时钟 (APB1×2, H7@300MHz) ---------- */
static uint32_t pwm_timer_clock(void)
{
    return rcu_clock_freq_get(CK_APB1) * 2U;
}

/* ---------- 软件 PWM 内部状态 ---------- */
#define PWM_MAX_PINS        16                          /* 最多支持的引脚数         */

static uint32_t pwm_port = 0;                           /* 当前使用的 GPIO 端口    */
static uint16_t pwm_active_mask = 0;                    /* 已激活的引脚位掩码      */
static uint16_t pwm_duty_map[PWM_MAX_PINS];             /* 各引脚占空比 (0-999)    */
static uint16_t pwm_counter = 0;                        /* 软件 PWM 计数器         */
static uint8_t  pwm_timer_inited = 0;                   /* 定时器是否已初始化      */

/* ---------- 内部：初始化 TIMER4 时基 ---------- */
static void pwm_timer_init(void)
{
    if (pwm_timer_inited) return;
    pwm_timer_inited = 1;

    rcu_periph_clock_enable(RCU_TIMER4);

    timer_parameter_struct tim_init = {0};
    timer_deinit(PWM_TIMER);
    tim_init.prescaler         = (uint16_t)(pwm_timer_clock() / 1000000U - 1U);  /* ~1MHz timer clk */
    tim_init.alignedmode       = TIMER_COUNTER_EDGE;
    tim_init.counterdirection  = TIMER_COUNTER_UP;
    tim_init.period            = PWM_HW_PERIOD;         /* 1MHz / 5 = 200kHz ISR  */
    tim_init.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(PWM_TIMER, &tim_init);

    /* 使能更新中断 */
    timer_interrupt_flag_clear(PWM_TIMER, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(PWM_TIMER, TIMER_INT_UP);
    nvic_irq_enable(TIMER4_IRQn, 0, 0);

    timer_auto_reload_shadow_enable(PWM_TIMER);
    timer_enable(PWM_TIMER);

    /* 初始化占空比表 */
    for (uint8_t i = 0; i < PWM_MAX_PINS; i++) {
        pwm_duty_map[i] = 0;
    }
}

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_init
// 描述:     初始化一个引脚为软件 PWM 输出
//---------------------------------------------------------------------------------------------------------------------
void pwm_init(uint32_t gpio_port, uint16_t gpio_pin)
{
    uint8_t pin_idx = 0;

    /* 计算引脚在端口内的索引 (0-15) */
    uint16_t pin_mask = gpio_pin;
    while (pin_mask > 1) { pin_mask >>= 1; pin_idx++; }

    /* 首次调用时初始化定时器 */
    pwm_timer_init();

    /* 记录端口（所有引脚必须位于同一端口） */
    pwm_port = gpio_port;

    /* 使能 GPIO 时钟 */
    if (gpio_port == GPIOA)      rcu_periph_clock_enable(RCU_GPIOA);
    else if (gpio_port == GPIOB) rcu_periph_clock_enable(RCU_GPIOB);
    else if (gpio_port == GPIOC) rcu_periph_clock_enable(RCU_GPIOC);
    else if (gpio_port == GPIOD) rcu_periph_clock_enable(RCU_GPIOD);
    else if (gpio_port == GPIOE) rcu_periph_clock_enable(RCU_GPIOE);

    /* 配置为推挽输出，初始高电平（共阳 LED 熄灭） */
    gpio_mode_set(gpio_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gpio_pin);
    gpio_output_options_set(gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, gpio_pin);
    GPIO_BOP(gpio_port) = gpio_pin;

    /* 标记此引脚已激活 */
    pwm_active_mask |= (1 << pin_idx);
    pwm_duty_map[pin_idx] = 0;
}

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_set_duty
// 描述:     设置指定引脚的 PWM 占空比
//---------------------------------------------------------------------------------------------------------------------
void pwm_set_duty(uint32_t gpio_port, uint16_t gpio_pin, uint16_t duty)
{
    uint8_t pin_idx = 0;

    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    uint16_t pin_mask = gpio_pin;
    while (pin_mask > 1) { pin_mask >>= 1; pin_idx++; }

    if (gpio_port == pwm_port && (pwm_active_mask & (1 << pin_idx))) {
        pwm_duty_map[pin_idx] = duty;
    }
}

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_gamma_correct
// 描述:     Gamma 校正: 平方映射使亮度感知均匀
//---------------------------------------------------------------------------------------------------------------------
uint16_t pwm_gamma_correct(uint16_t linear, uint16_t max)
{
    return (uint16_t)(((uint32_t)linear * linear) / max);
}

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pwm_isr_handler
// 描述:     PWM 中断服务（在 TIMER4_IRQHandler 中调用）
// 备注:     共阳 LED: 低电平 → 点亮, 高电平 → 熄灭
//---------------------------------------------------------------------------------------------------------------------
void pwm_isr_handler(void)
{
    if (timer_interrupt_flag_get(PWM_TIMER, TIMER_INT_FLAG_UP) == SET) {
        timer_interrupt_flag_clear(PWM_TIMER, TIMER_INT_FLAG_UP);

        pwm_counter++;
        if (pwm_counter >= PWM_RESOLUTION)
            pwm_counter = 0;

        if (pwm_counter == 0) {
            /* PWM 周期开始: 所有激活的非零占空比引脚输出低电平 (LED 亮) */
            uint16_t on_mask = 0;
            for (uint8_t i = 0; i < PWM_MAX_PINS; i++) {
                if ((pwm_active_mask & (1 << i)) && (pwm_duty_map[i] > 0)) {
                    on_mask |= (1 << i);
                }
            }
            if (on_mask) GPIO_BC(pwm_port) = on_mask;
        } else {
            /* 检查每个激活的引脚是否到达占空比时刻，到达则输出高电平 (LED 灭) */
            for (uint8_t i = 0; i < PWM_MAX_PINS; i++) {
                if ((pwm_active_mask & (1 << i))
                    && (pwm_duty_map[i] > 0)
                    && (pwm_duty_map[i] < PWM_RESOLUTION)
                    && (pwm_counter == pwm_duty_map[i])) {
                    GPIO_BOP(pwm_port) = (uint16_t)(1 << i);
                }
            }
        }
    }
}
