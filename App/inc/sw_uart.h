/*******************************************************************************
 * 文件名          sw_uart.h
 * 描述            软件 UART (bit-bang) — 两路全双工 @ 115200bps
 *                 使用 TIMER1 @ 3x 波特率 (345.6kHz) 驱动收发状态机
 *                 RX 下降沿检测 + 3倍过采样，TX 精确位定时
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 硬件连接:
 *   SW_UART1: PE2(J5-56)=TX / PE5(J5-58)=RX (GPIOE, 同端口相邻)
 *   SW_UART2: PF7(J4-17)=TX / PF6(J4-19)=RX
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-06-18      CIMC           初始版本，替代故障硬件串口
 ******************************************************************************/

#ifndef __SW_UART_H
#define __SW_UART_H

#include "gd32h7xx.h"

/* ==================== 波特率 ==================== */
#define SW_UART_BAUD        115200

/* ==================== 通道 1 引脚 ==================== */
#define SW1_TX_PORT         GPIOE
#define SW1_TX_PIN          GPIO_PIN_2      /* PE2  J5-56 (GPIOE, 空闲) */
#define SW1_TX_RCU          RCU_GPIOE
#define SW1_RX_PORT         GPIOE
#define SW1_RX_PIN          GPIO_PIN_5      /* PE5  J5-58 (GPIOE, 空闲) */
#define SW1_RX_RCU          RCU_GPIOE

/* ==================== 通道 2 引脚 ==================== */
#define SW2_TX_PORT         GPIOF
#define SW2_TX_PIN          GPIO_PIN_7      /* PF7  J4-17 */
#define SW2_TX_RCU          RCU_GPIOF
#define SW2_RX_PORT         GPIOF
#define SW2_RX_PIN          GPIO_PIN_6      /* PF6  J4-19 */
#define SW2_RX_RCU          RCU_GPIOF

/* ==================== API ==================== */

void SwUart_Init(void);
void SwUart1_SendByte(uint8_t byte);
void SwUart2_SendByte(uint8_t byte);
void SwUart1_SendString(const char *str);
void SwUart2_SendString(const char *str);
uint8_t SwUart1_QueryByte(uint8_t *byte);  /* 返回 1=有数据 */
uint8_t SwUart2_QueryByte(uint8_t *byte);
void SwUart_ISR(void);                     /* 在 TIMER1_IRQHandler 中调用 */
extern volatile uint32_t sw_isr_count;     /* ISR 触发次数, 调试用 */
extern volatile uint32_t sw1_edge_cnt;     /* CH1 下降沿检测次数 */
extern volatile uint32_t sw2_edge_cnt;     /* CH2 下降沿检测次数 */
extern volatile uint32_t sw1_tx_timeout;   /* CH1 TX 超时丢字节次数 */
extern volatile uint32_t sw2_tx_timeout;   /* CH2 TX 超时丢字节次数 */

#endif
