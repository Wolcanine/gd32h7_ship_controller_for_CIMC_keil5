/************************************************************
 * 版权：2025CIMC Copyright。
 * 文件：led.c
 * 作者: Jialei Zhao
 * 平台: 2025CIMC IHD-V04
 * 版本: Jialei Zhao     2026/2/5     V0.01    original
************************************************************/

/************************* 头文件 *************************/

#include "LED.h"

/************************ 全局变量定义 ************************/


/************************************************************
 * Function :       LED_Init
 * Comment  :       用于初始化LED端口
 * Parameter:       null
 * Return   :       null
 * Author   :       Jialei Zhao
 * Date     :       2026-02-05 V0.1 original
************************************************************/

void LED_Init(void)
{

	rcu_periph_clock_enable(RCU_GPIOE);    // 初始化GPIO_E总线时钟
	rcu_periph_clock_enable(RCU_GPIOH);    // 初始化GPIO_H总线时钟

	//初始化LED1端口
	gpio_mode_set(GPIOE , GPIO_MODE_OUTPUT , GPIO_PUPD_NONE , GPIO_PIN_2| GPIO_PIN_5);   			// GPIO模式设置为输出
	gpio_output_options_set(GPIOE , GPIO_OTYPE_PP , GPIO_OSPEED_60MHZ , GPIO_PIN_2| GPIO_PIN_5);   // 输出参数设置
	gpio_bit_reset(GPIOE , GPIO_PIN_2| GPIO_PIN_5);  											// 引脚初始电平为低电平

	//初始化LED2
	gpio_mode_set(GPIOH , GPIO_MODE_OUTPUT , GPIO_PUPD_NONE , GPIO_PIN_7);   			// GPIO模式设置为输出
	gpio_output_options_set(GPIOH , GPIO_OTYPE_PP , GPIO_OSPEED_60MHZ , GPIO_PIN_7);     // 输出参数设置
	gpio_bit_reset(GPIOH , GPIO_PIN_7);  											// 引脚初始电平为低电平

}

/****************************End*****************************/

