/*!
 *  \file    gd32_driver_pit.c
 *  \brief   PIT (Periodic Interrupt Timer) 模块实现
 *
 *  \version 2026-05-06, V1.0.0
 */

#include "gd32_driver_pit.h"

/* ---------- 根据定时器获取实际时钟频率 ----------
 * H7 时钟树: SYSCLK=600MHz → AHB=300MHz
 * TIMER1 在 APB2 (div=1): CK_TIMER = 300MHz
 * TIMER2~6 在 APB1 (div=2): CK_TIMER = CK_APB1 × 2 = 300MHz
 */
static uint32_t pit_get_timer_clock(pit_index_enum pit_n)
{
    if (pit_n == PIT_TIMER1) {
        return rcu_clock_freq_get(CK_APB2);       /* APB2 div=1, timer=APB2 */
    }
    return rcu_clock_freq_get(CK_APB1) * 2U;       /* APB1 div=2, timer=APB1×2 */
}

/* ---------- 定时器基地址查找表 (TIMER1 ~ TIMER6) ---------- */
static const uint32_t pit_timer_base[] =
{
    TIMER1, TIMER2, TIMER3,
    TIMER4, TIMER5, TIMER6,
};

/* ---------- 定时器时钟使能查找表 ---------- */
static const rcu_periph_enum pit_clock[] =
{
    RCU_TIMER1, RCU_TIMER2, RCU_TIMER3,
    RCU_TIMER4, RCU_TIMER5, RCU_TIMER6,
};

/* ---------- 定时器更新中断号查找表 (H7: TIMER5→TIMER5_DAC_UDR_IRQn) ---------- */
static const IRQn_Type pit_irq[] =
{
    TIMER1_IRQn,
    TIMER2_IRQn,
    TIMER3_IRQn,
    TIMER4_IRQn,
    TIMER5_DAC_UDR_IRQn,        /* TIMER5 with DAC underrun (H7) */
    TIMER6_IRQn,
};

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pit_init
// 描述:     PIT 初始化，自动计算预分频与重装载值
// 参数:     pit_n       PIT 定时器编号
//           period      中断间隔（SystemCoreClock 周期数）
// 返回值:   void
//---------------------------------------------------------------------------------------------------------------------
void pit_init(pit_index_enum pit_n, uint32_t period)
{
    uint16_t freq_div;
    uint16_t period_temp;
    uint32_t timer_clk;

    /* H7 定时器时钟可能与 SystemCoreClock 不同 (H7: 300MHz vs 600MHz),
     * 将 period 从 SystemCoreClock 周期数换算为实际定时器时钟周期数 */
    timer_clk = pit_get_timer_clock(pit_n);
    if (timer_clk != SystemCoreClock) {
        period = (uint32_t)((uint64_t)period * timer_clk / SystemCoreClock);
    }

    /* 自动计算预分频与重装载值 */
    freq_div    = (uint16_t)(period >> 15);
    period_temp = (uint16_t)(period / (freq_div + 1));

    rcu_periph_clock_enable(pit_clock[pit_n]);

    timer_parameter_struct tim_init = {0};
    timer_deinit(pit_timer_base[pit_n]);
    tim_init.prescaler         = freq_div;
    tim_init.alignedmode       = TIMER_COUNTER_EDGE;
    tim_init.counterdirection  = TIMER_COUNTER_UP;
    tim_init.period            = period_temp - 1;
    tim_init.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(pit_timer_base[pit_n], &tim_init);

    /* 使能更新中断并启动 */
    timer_interrupt_flag_clear(pit_timer_base[pit_n], TIMER_INT_FLAG_UP);
    timer_interrupt_enable(pit_timer_base[pit_n], TIMER_INT_UP);
    nvic_irq_enable(pit_irq[pit_n], 0, 0);

    timer_enable(pit_timer_base[pit_n]);
}

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pit_enable
// 描述:     使能 PIT 更新中断
//---------------------------------------------------------------------------------------------------------------------
void pit_enable(pit_index_enum pit_n)
{
    timer_interrupt_flag_clear(pit_timer_base[pit_n], TIMER_INT_FLAG_UP);
    timer_interrupt_enable(pit_timer_base[pit_n], TIMER_INT_UP);
    nvic_irq_enable(pit_irq[pit_n], 0, 0);
}

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   pit_disable
// 描述:     禁止 PIT 更新中断（定时器继续计数）
//---------------------------------------------------------------------------------------------------------------------
void pit_disable(pit_index_enum pit_n)
{
    timer_interrupt_disable(pit_timer_base[pit_n], TIMER_INT_UP);
}
