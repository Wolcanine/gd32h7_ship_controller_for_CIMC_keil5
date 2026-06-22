/*******************************************************************************
 * 文件名          sw_uart.c
 * 描述            软件 UART 实现 — 两路全双工
 *                 TIMER1 @ 3x 波特率 = 28.8kHz, 每tick处理两路TX+RX状态机
 *                 CH1: GPS模块, CH2: 预留
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-06-18      CIMC           初始版本
 * 2026-06-18      CIMC           修复: RX时序+上拉+超时诊断; CH1 RX PD6→PA3
 * 2026-06-18      CIMC           波特率降至9600 (适配GPS模块):
 *                                  TIMER1@28.8kHz, period=10417, pit_init=20834
 ******************************************************************************/

#include "sw_uart.h"
#include "gd32_driver_pit.h"

/* ==================== 诊断计数器 ==================== */
volatile uint32_t sw_isr_count     = 0;  /* ISR 触发次数 */
volatile uint32_t sw1_edge_cnt     = 0;  /* CH1 下降沿检测次数 */
volatile uint32_t sw2_edge_cnt     = 0;  /* CH2 下降沿检测次数 */
volatile uint32_t sw1_tx_timeout   = 0;  /* CH1 TX 超时丢字节次数 */
volatile uint32_t sw2_tx_timeout   = 0;  /* CH2 TX 超时丢字节次数 */

/* ==================== 内部常量 ==================== */
/* TIMER1 在 APB2 (300MHz), pit_init 按 SystemCoreClock(600MHz) 折算,
 * 所以 period 参数需 ×2 补偿.
 * 9600×3=28800Hz, 300M/28800=10417, → pit_init传20834 */
#define SW_TIMER_TICKS_PER_BIT  3
#define SW_TIMER_PERIOD         20834   /* → 300MHz/10417 ≈ 28.8kHz */

/* ==================== 通道1 数据结构 ==================== */
static struct {
    /* TX */
    volatile uint8_t  tx_byte;
    volatile int8_t   tx_phase;     /* -1=idle, 0=start, 1-8=data, 9=stop */
    volatile uint8_t  tx_tick;      /* 0..2 within each bit */

    /* RX */
    volatile uint8_t  rx_state;     /* 0=idle, 1=start_detect, 2=receiving */
    volatile uint8_t  rx_byte;
    volatile int8_t   rx_bit;       /* -1=idle, 0=start, 0-7=data, 8=stop */
    volatile uint8_t  rx_tick;
    volatile uint8_t  rx_prev;      /* previous pin state, for edge detect */
    volatile uint8_t  rx_ready;
    volatile uint8_t  rx_byte_rd;
} ch1;

/* ==================== 通道2 数据结构 ==================== */
static struct {
    volatile uint8_t  tx_byte;
    volatile int8_t   tx_phase;
    volatile uint8_t  tx_tick;

    volatile uint8_t  rx_state;
    volatile uint8_t  rx_byte;
    volatile int8_t   rx_bit;
    volatile uint8_t  rx_tick;
    volatile uint8_t  rx_prev;
    volatile uint8_t  rx_ready;
    volatile uint8_t  rx_byte_rd;
} ch2;

/* ==================== 初始化 ==================== */
void SwUart_Init(void)
{
    /* ---- 使能 GPIO 时钟 ---- */
    rcu_periph_clock_enable(SW1_TX_RCU);   /* RCU_GPIOE for PE2/PE5 */
    /* SW1_RX_RCU = RCU_GPIOE (same as TX, already enabled above) */
    rcu_periph_clock_enable(SW2_TX_RCU);   /* RCU_GPIOF for PF7 */
    /* SW2_RX_RCU = RCU_GPIOF (same as TX, already enabled above) */

    /* ---- CH1 TX: PE2 推挽输出, 初始 HIGH (J5-56, GPIOE) ---- */
    gpio_mode_set(SW1_TX_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW1_TX_PIN);
    gpio_output_options_set(SW1_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, SW1_TX_PIN);
    gpio_bit_set(SW1_TX_PORT, SW1_TX_PIN);

    /* ---- CH1 RX: PE5 上拉输入 (J5-58, GPIOE) ---- */
    gpio_mode_set(SW1_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, SW1_RX_PIN);

    /* ---- CH2 TX: PF7 推挽输出, 初始 HIGH ---- */
    gpio_mode_set(SW2_TX_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW2_TX_PIN);
    gpio_output_options_set(SW2_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, SW2_TX_PIN);
    gpio_bit_set(SW2_TX_PORT, SW2_TX_PIN);

    /* ---- CH2 RX: PF6 上拉输入 (UART 空闲=高电平) ---- */
    gpio_mode_set(SW2_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, SW2_RX_PIN);

    /* ---- 状态复位 ---- */
    ch1.tx_phase = -1;  ch1.rx_state = 0;  ch1.rx_bit = -1;
    ch2.tx_phase = -1;  ch2.rx_state = 0;  ch2.rx_bit = -1;

    /* ---- 启动 TIMER1 @ 345.6kHz ---- */
    pit_init(PIT_TIMER1, SW_TIMER_PERIOD);
}

/* ==================== TX API ==================== */
void SwUart1_SendByte(uint8_t byte)
{
    uint32_t timeout = 100000;
    while (ch1.tx_phase >= 0 && --timeout);  /* 超时保护 */
    if (timeout == 0) { sw1_tx_timeout++; return; }  /* ISR 未触发, 放弃 */
    ch1.tx_byte  = byte;
    ch1.tx_phase = 0;
    ch1.tx_tick  = 0;
}

void SwUart2_SendByte(uint8_t byte)
{
    uint32_t timeout = 100000;
    while (ch2.tx_phase >= 0 && --timeout);
    if (timeout == 0) { sw2_tx_timeout++; return; }
    ch2.tx_byte  = byte;
    ch2.tx_phase = 0;
    ch2.tx_tick  = 0;
}

void SwUart1_SendString(const char *str)
{
    while (*str) SwUart1_SendByte((uint8_t)*str++);
}

void SwUart2_SendString(const char *str)
{
    while (*str) SwUart2_SendByte((uint8_t)*str++);
}

/* ==================== RX API ==================== */
uint8_t SwUart1_QueryByte(uint8_t *byte)
{
    if (ch1.rx_ready) {
        *byte = ch1.rx_byte_rd;
        ch1.rx_ready = 0;
        return 1;
    }
    return 0;
}

uint8_t SwUart2_QueryByte(uint8_t *byte)
{
    if (ch2.rx_ready) {
        *byte = ch2.rx_byte_rd;
        ch2.rx_ready = 0;
        return 1;
    }
    return 0;
}

/* ==================== 中断服务 (在 TIMER1_IRQHandler 中调用) ==================== */

/* 处理单个通道的 RX
 * 3x 过采样时序: bit_time=3ticks, 下降沿后等2tick到start bit中心验证, 之后每3tick采样 */
static void sw_rx_process(volatile uint8_t *rx_state, volatile uint8_t *rx_byte,
                          volatile int8_t *rx_bit, volatile uint8_t *rx_tick,
                          volatile uint8_t *rx_prev, volatile uint8_t *rx_ready,
                          volatile uint8_t *rx_byte_rd,
                          volatile uint32_t *edge_cnt,
                          uint32_t rx_port, uint32_t rx_pin)
{
    uint8_t cur = (uint8_t)((GPIO_ISTAT(rx_port) & rx_pin) ? 1 : 0);

    switch (*rx_state) {
    case 0: /* 空闲, 检测下降沿 */
        if (*rx_prev == 1 && cur == 0) {
            (*edge_cnt)++;
            *rx_state = 1;
            *rx_tick  = 0;
        }
        break;

    case 1: /* 等2tick到start bit中心验证, 然后收数据 */
        (*rx_tick)++;
        if (*rx_tick == 2) {
            if (cur != 0) { *rx_state = 0; break; }  /* 假start */
            *rx_state = 2;
            *rx_byte  = 0;
            *rx_bit   = 0;   /* D0 */
            *rx_tick  = 0;
        }
        break;

    case 2: /* 收数据位: 每3tick采样一次 */
        (*rx_tick)++;
        if (*rx_tick == 2) {   /* 采样时刻 (bit中心) */
            if (*rx_bit < 8) {
                if (cur) *rx_byte |= (1 << (*rx_bit));
                (*rx_bit)++;
            } else {
                /* stop bit: 可选验证 */
                *rx_byte_rd = *rx_byte;
                *rx_ready   = 1;
                *rx_state   = 0;
            }
        } else if (*rx_tick >= 3) {
            *rx_tick = 0;
            if (*rx_bit >= 9) {   /* 收完8bit+stop */
                *rx_state = 0;
            }
        }
        break;
    }

    *rx_prev = cur;
}

/* 处理单个通道的 TX */
static void sw_tx_process(volatile uint8_t *tx_byte, volatile int8_t *tx_phase,
                          volatile uint8_t *tx_tick,
                          uint32_t tx_port, uint32_t tx_pin)
{
    if (*tx_phase < 0) return;  /* 空闲 */

    (*tx_tick)++;
    if (*tx_tick >= SW_TIMER_TICKS_PER_BIT) {
        *tx_tick = 0;

        if (*tx_phase == 0) {
            /* start bit: 拉低 */
            gpio_bit_reset(tx_port, tx_pin);
        } else if (*tx_phase <= 8) {
            /* 数据位 D0-D7 (LSB first) */
            if (*tx_byte & (1 << ((*tx_phase) - 1)))
                gpio_bit_set(tx_port, tx_pin);
            else
                gpio_bit_reset(tx_port, tx_pin);
        } else {
            /* stop bit: 拉高 */
            gpio_bit_set(tx_port, tx_pin);
            *tx_phase = -1;  /* 发送完成, 回到空闲 */
            return;
        }
        (*tx_phase)++;
    }
}

/* TIMER1 ISR 入口 — 同步处理两路 */
void SwUart_ISR(void)
{
    sw_isr_count++;

    /* ---- CH1 RX + TX ---- */
    sw_rx_process(&ch1.rx_state, &ch1.rx_byte, &ch1.rx_bit, &ch1.rx_tick,
                  &ch1.rx_prev, &ch1.rx_ready, &ch1.rx_byte_rd,
                  &sw1_edge_cnt, SW1_RX_PORT, SW1_RX_PIN);
    sw_tx_process(&ch1.tx_byte, &ch1.tx_phase, &ch1.tx_tick,
                  SW1_TX_PORT, SW1_TX_PIN);

    /* ---- CH2 RX + TX ---- */
    sw_rx_process(&ch2.rx_state, &ch2.rx_byte, &ch2.rx_bit, &ch2.rx_tick,
                  &ch2.rx_prev, &ch2.rx_ready, &ch2.rx_byte_rd,
                  &sw2_edge_cnt, SW2_RX_PORT, SW2_RX_PIN);
    sw_tx_process(&ch2.tx_byte, &ch2.tx_phase, &ch2.tx_tick,
                  SW2_TX_PORT, SW2_TX_PIN);
}
