/*******************************************************************************
 * 文件名          gd32h7xx_it.c
 * 描述            中断服务函数
 *                 TIMER3: 50Hz 控制周期
 *                 TIMER4: 软件 PWM 时基
 *                 SysTick: 毫秒延时 + g_sys_ms 递增
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

#include "gd32h7xx_it.h"
#include "main.h"
#include "systick.h"
#include "gd32_driver_pwm.h"

extern volatile uint32_t g_sys_ms;

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void)
{
    while (1) {}
}

void PendSV_Handler(void)
{
    while (1) {}
}

/*******************************************************************************
 * 函数名    SysTick_Handler
 * 描述      SysTick 1ms 中断：递减 delay 计数器 + 递增全局毫秒计数器
 ******************************************************************************/
void SysTick_Handler(void)
{
    delay_decrement();
    g_sys_ms++;
}

/*******************************************************************************
 * 函数名    TIMER3_IRQHandler
 * 描述      50Hz 控制周期触发（20ms），设置 timer20ms_flag 唤醒主循环
 ******************************************************************************/
void TIMER3_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET) {
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
        timer20ms_flag = 1;
    }
}

/*******************************************************************************
 * 函数名    TIMER4_IRQHandler
 * 描述      软件 PWM 时基中断，驱动 GPIO 引脚产生 PWM 波形
 ******************************************************************************/
void TIMER4_IRQHandler(void)
{
    pwm_isr_handler();
}
