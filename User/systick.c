/*******************************************************************************
 * 文件名          systick.c
 * 描述            SysTick 时基配置 — 提供 1ms 滴答延时
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植，修复 H7 时钟比例
 ******************************************************************************/

#include "gd32h7xx.h"
#include "systick.h"

volatile static uint32_t delay;

/*******************************************************************************
 * 函数名    systick_config
 * 描述      配置 SysTick 定时器为 1000Hz 中断
 *           注意：H7 的 SysTick 使用 CK_AHB（300MHz）而非 SystemCoreClock（600MHz）
 * 参数      none
 * 返回值    none
 ******************************************************************************/
void systick_config(void)
{
    /* SysTick 使用处理器时钟 (SYSCLK=600MHz)，不是 AHB(300MHz)
     * 因此用 SystemCoreClock(600M) 而非 CK_AHB(300M) 计算 reload */
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        while (1) {}
    }
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*******************************************************************************
 * 函数名    delay_1ms
 * 描述      阻塞式毫秒延时
 * 参数      count    延时毫秒数
 * 返回值    none
 ******************************************************************************/
void delay_1ms(uint32_t count)
{
    delay = count;
    while (0U != delay) {}
}

/*******************************************************************************
 * 函数名    delay_decrement
 * 描述      SysTick 中断中调用，递减 delay 计数器
 ******************************************************************************/
void delay_decrement(void)
{
    if (0U != delay) {
        delay--;
    }
}
