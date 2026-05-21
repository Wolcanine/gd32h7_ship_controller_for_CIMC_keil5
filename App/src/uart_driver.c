/*******************************************************************************
 * 文件名          uart_driver.c
 * 描述            多路 UART 驱动 — 中断接收 + 软件 FIFO
 *                 支持 UART_TOF(USART1)、UART_DBG(USART0)、UART_CAM(USART2)
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-08      AI助手          添加 UART_CAM (USART2)
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

/***************************************************************
* GD32F4xx UART 驱动模块 —— 基于 GD32F4xx 标准外设库的串口收发驱动
* Copyright (c) 2025
*
* 本文件是 TOF200F 激光测距工程的一部分
*
* 文件名称          uart_driver
* 开发环境          Eclipse + ARM GCC
* 适用平台          GD32H7XXVET6
* 硬件连接
*                   UART_TOF (模块通信): USART1, PA2-TX / PA3-RX, AF7, APB2
*                   UART_DBG (调试输出): USART0, PA9-TX / PA10-RX, AF7, APB2
*                   UART_CAM (摄像头):   USART2, PD5-TX / PD6-RX, AF7, APB1
*
* 修改记录
* 日期                作者          备注
* 2025-05-06          AI助手        初始版本：中断接收 + 软件FIFO
* 2025-05-06          AI助手        修复：UART_TOF 改为 USART1(PA2/PA3)，
*                                   补充RX引脚gpio_output_options_set，
*                                   启用 USART1_IRQHandler 中断服务函数
* 2026-05-08          AI助手        添加 UART_CAM (USART2, PD5/PD6)
*                                   用于接收摄像头识别板数据
***************************************************************/

#include "uart_driver.h"
#include "gd32h7xx_rcu.h"
#include "gd32h7xx_gpio.h"
#include "gd32h7xx_usart.h"
#include "gd32h7xx_misc.h"
#include <string.h>

//=================================================== 硬件配置表 ====================================================

/* 每个UART模块的硬件参数 */
typedef struct {
    uint32_t        usart_base;     /* USART外设基地址（USART0/USART1等） */
    rcu_periph_enum tx_port_clk;    /* TX引脚所在GPIO端口的RCU时钟 */
    uint32_t        tx_port;        /* TX引脚所在GPIO端口基地址 */
    uint32_t        tx_pin;         /* TX引脚位掩码 */
    rcu_periph_enum rx_port_clk;    /* RX引脚所在GPIO端口的RCU时钟 */
    uint32_t        rx_port;        /* RX引脚所在GPIO端口基地址 */
    uint32_t        rx_pin;         /* RX引脚位掩码 */
    uint32_t        af_num;         /* GPIO复用功能编号（AF7/AF8等） */
    rcu_periph_enum usart_clk;      /* USART外设的RCU时钟 */
    IRQn_Type       irqn;           /* USART中断向量号 */
} uart_hw_cfg_t;

static const uart_hw_cfg_t uart_hw_cfg[] = {
    [UART_TOF] = {
        /* USART1: TX=PA2(AF7), RX=PA3(AF7), 挂在APB2总线 */
        .usart_base   = USART1,
        .tx_port_clk  = RCU_GPIOA,
        .tx_port      = GPIOA,
        .tx_pin       = GPIO_PIN_2,
        .rx_port_clk  = RCU_GPIOA,
        .rx_port      = GPIOA,
        .rx_pin       = GPIO_PIN_3,
        .af_num       = GPIO_AF_7,
        .usart_clk    = RCU_USART1,
        .irqn         = USART1_IRQn,
    },
    [UART_DBG] = {
        /* USART0: TX=PA9(AF7), RX=PA10(AF7), 挂在APB2总线 */
        .usart_base   = USART0,
        .tx_port_clk  = RCU_GPIOA,
        .tx_port      = GPIOA,
        .tx_pin       = GPIO_PIN_9,
        .rx_port_clk  = RCU_GPIOA,
        .rx_port      = GPIOA,
        .rx_pin       = GPIO_PIN_10,
        .af_num       = GPIO_AF_7,
        .usart_clk    = RCU_USART0,
        .irqn         = USART0_IRQn,
    },
    [UART_CAM] = {
        /* USART2: TX=PD5(AF7), RX=PD6(AF7), 挂在APB1总线 */
        .usart_base   = USART2,
        .tx_port_clk  = RCU_GPIOD,
        .tx_port      = GPIOD,
        .tx_pin       = GPIO_PIN_5,
        .rx_port_clk  = RCU_GPIOD,
        .rx_port      = GPIOD,
        .rx_pin       = GPIO_PIN_6,
        .af_num       = GPIO_AF_7,
        .usart_clk    = RCU_USART2,
        .irqn         = USART2_IRQn,
    },
};

//=================================================== 运行时数据 ====================================================

/* 每个UART模块的接收缓冲区及其管理变量 */
typedef struct {
    volatile uint8_t  rx_buf[UART_RX_BUF_SIZE]; /* 环形接收缓冲区 */
    volatile uint32_t head;                     /* 写指针（由ISR递增） */
    volatile uint32_t tail;                     /* 读指针（由用户函数递增） */
    volatile uint32_t last_rx_ms;               /* 最后一次接收到数据的时间戳(ms) */
} uart_runtime_t;

static uart_runtime_t uart_runtime[3];          /* 三个模块各自的运行时数据 */

//=================================================== 初始化函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART模块初始化
// 参数说明     module      模块索引（UART_TOF / UART_DBG / UART_CAM）
// 参数说明     baud        波特率（如 115200）
// 返回参数     void
// 备注信息     初始化流程：
//              1. 复位运行时数据结构
//              2. 使能 GPIO 和 USART 时钟
//              3. 配置 TX/RX 引脚为复用功能推挽输出（上拉）
//              4. 配置 USART 参数：波特率、8N1、无流控
//              5. 使能接收中断（RBNE），使能 NVIC
//-------------------------------------------------------------------------------------------------------------------
void uart_init(uart_module_enum module, uint32_t baud)
{
    const uart_hw_cfg_t *cfg = &uart_hw_cfg[module];
    uart_runtime_t *rt = &uart_runtime[module];

    /* ---- 第1步：复位接收缓冲区状态 ---- */
    rt->head = 0;
    rt->tail = 0;
    rt->last_rx_ms = 0;
    memset((void *)rt->rx_buf, 0, UART_RX_BUF_SIZE);

    /* ---- 第2步：使能 GPIO 和 USART 外设时钟 ---- */
    rcu_periph_clock_enable(cfg->tx_port_clk);                      // 使能TX引脚GPIO时钟
    if (cfg->rx_port_clk != cfg->tx_port_clk) {                     // 若TX/RX不在同一GPIO端口
        rcu_periph_clock_enable(cfg->rx_port_clk);                  // 额外使能RX引脚GPIO时钟
    }
    rcu_periph_clock_enable(cfg->usart_clk);                        // 使能USART外设时钟

    /* ---- 第3步：配置TX引脚为复用功能推挽输出 ---- */
    gpio_af_set(cfg->tx_port, cfg->af_num, cfg->tx_pin);          // 设置复用功能号
    gpio_mode_set(cfg->tx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                  cfg->tx_pin);                                     // 复用模式 + 上拉
    gpio_output_options_set(cfg->tx_port, GPIO_OTYPE_PP,
                           GPIO_OSPEED_60MHZ, cfg->tx_pin);         // 推挽输出 + 50MHz

    /* ---- 第4步：配置RX引脚为复用功能推挽输出 ---- */
    gpio_af_set(cfg->rx_port, cfg->af_num, cfg->rx_pin);          // 设置复用功能号
    gpio_mode_set(cfg->rx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                  cfg->rx_pin);                                     // 复用模式 + 上拉
    gpio_output_options_set(cfg->rx_port, GPIO_OTYPE_PP,
                           GPIO_OSPEED_60MHZ, cfg->rx_pin);         // 即使RX为输入，也需设置（GD32要求）

    /* ---- 第5步：配置USART通信参数 ---- */
    usart_deinit(cfg->usart_base);                                  // 复位USART到默认状态
    usart_baudrate_set(cfg->usart_base, baud);                      // 设置波特率
    usart_word_length_set(cfg->usart_base, USART_WL_8BIT);          // 8位数据位
    usart_stop_bit_set(cfg->usart_base, USART_STB_1BIT);            // 1位停止位
    usart_parity_config(cfg->usart_base, USART_PM_NONE);            // 无校验位
    usart_hardware_flow_rts_config(cfg->usart_base,
                                  USART_RTS_DISABLE);               // 禁用RTS硬件流控
    usart_hardware_flow_cts_config(cfg->usart_base,
                                  USART_CTS_DISABLE);               // 禁用CTS硬件流控
    usart_receive_config(cfg->usart_base, USART_RECEIVE_ENABLE);    // 使能接收
    usart_transmit_config(cfg->usart_base, USART_TRANSMIT_ENABLE);  // 使能发送

    /* ---- 第6步：使能USART接收中断 ---- */
    nvic_irq_enable(cfg->irqn, 1, 0);                               // NVIC使能，优先级1
    usart_interrupt_enable(cfg->usart_base, USART_INT_RBNE);        // 使能接收缓冲区非空中断
    usart_enable(cfg->usart_base);                                  // 最后使能USART外设
}

//=================================================== 发送函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送单个字节（阻塞模式）
// 参数说明     module      模块索引
// 参数说明     data        要发送的字节数据
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void uart_send_byte(uart_module_enum module, uint8_t data)
{
    uint32_t base = uart_hw_cfg[module].usart_base;                 // 获取USART基地址
    usart_data_transmit(base, data);                                // 写入发送数据寄存器
    while (RESET == usart_flag_get(base, USART_FLAG_TBE));          // 等待发送缓冲区空（TBE=1）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送字节数组（阻塞模式）
// 参数说明     module      模块索引
// 参数说明     buff        待发送数据的指针
// 参数说明     len         发送字节数
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void uart_send_buffer(uart_module_enum module, const uint8_t *buff, uint32_t len)
{
    while (len--) {
        uart_send_byte(module, *buff++);                            // 逐字节发送
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送字符串（阻塞模式）
// 参数说明     module      模块索引
// 参数说明     str         以 '\0' 结尾的字符串指针
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void uart_send_string(uart_module_enum module, const char *str)
{
    while (*str) {
        uart_send_byte(module, (uint8_t)*str++);                    // 逐字符发送，直到遇到 '\0'
    }
}

//=================================================== 接收函数（从软件FIFO读取） ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从软件FIFO中非阻塞读取一个字节
// 参数说明     module      模块索引
// 参数说明     dat         用于存储读取数据的指针
// 返回参数     uint8_t     1 = 读取成功，0 = 缓冲区为空
//-------------------------------------------------------------------------------------------------------------------
uint8_t uart_query_byte(uart_module_enum module, uint8_t *dat)
{
    uart_runtime_t *rt = &uart_runtime[module];
    if (rt->head != rt->tail) {                                     // FIFO中有数据
        *dat = rt->rx_buf[rt->tail];                                // 从读指针处取一个字节
        rt->tail = (rt->tail + 1) % UART_RX_BUF_SIZE;               // 读指针后移（环形）
        return 1;
    }
    return 0;                                                       // FIFO为空
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从软件FIFO中读取指定最大长度的数据
// 参数说明     module      模块索引
// 参数说明     buff        接收缓冲区指针
// 参数说明     max_len     最大读取字节数
// 返回参数     uint32_t    实际读取的字节数
//-------------------------------------------------------------------------------------------------------------------
uint32_t uart_read_buffer(uart_module_enum module, uint8_t *buff, uint32_t max_len)
{
    uart_runtime_t *rt = &uart_runtime[module];
    uint32_t cnt = 0;
    while (cnt < max_len && rt->head != rt->tail) {                 // 缓冲区有数据 且 未达上限
        buff[cnt++] = rt->rx_buf[rt->tail];                         // 读取一个字节
        rt->tail = (rt->tail + 1) % UART_RX_BUF_SIZE;               // 读指针后移
    }
    return cnt;                                                     // 返回实际读出的字节数
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空软件接收FIFO
// 参数说明     module      模块索引
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void uart_flush_rx(uart_module_enum module)
{
    uart_runtime_t *rt = &uart_runtime[module];
    rt->head = 0;                                                   // 写指针归零
    rt->tail = 0;                                                   // 读指针归零
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取软件FIFO中当前可读取的字节数
// 参数说明     module      模块索引
// 返回参数     uint32_t    缓冲区中已接收但未读取的字节数
//-------------------------------------------------------------------------------------------------------------------
uint32_t uart_rx_available(uart_module_enum module)
{
    uart_runtime_t *rt = &uart_runtime[module];
    if (rt->head >= rt->tail) {
        return rt->head - rt->tail;                                 // 正常情况：head 在 tail 之后
    }
    return UART_RX_BUF_SIZE - rt->tail + rt->head;                  // 环形折返：head 已绕回
}

//=================================================== USART 中断服务函数 ====================================================

#if 0  /* USART0 IRQ 在 main.c 中定义时启用，此处禁用避免冲突 */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     USART0 接收中断服务函数
//-------------------------------------------------------------------------------------------------------------------
void USART0_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(USART0);
        uart_runtime_t *rt = &uart_runtime[UART_DBG];
        uint32_t next = (rt->head + 1) % UART_RX_BUF_SIZE;
        if (next != rt->tail) {
            rt->rx_buf[rt->head] = data;
            rt->head = next;
        }
        /* 若FIFO已满，则丢弃该字节（静默丢弃，防止死锁） */
        rt->last_rx_ms = g_sys_ms;
    }
}
#endif /* 0 */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     USART1 接收中断服务函数
// 备注信息     自动被 NVIC 调用，每收到一个字节触发一次
//              将接收到的数据存入 UART_TOF 对应的软件FIFO，
//              用户程序通过 uart_query_byte / uart_read_buffer 读取
//-------------------------------------------------------------------------------------------------------------------
void USART1_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(USART1);
        uart_runtime_t *rt = &uart_runtime[UART_TOF];
        uint32_t next = (rt->head + 1) % UART_RX_BUF_SIZE;
        if (next != rt->tail) {
            rt->rx_buf[rt->head] = data;
            rt->head = next;
        }
        /* 若缓冲区满，则丢弃该字节，避免阻塞中断 */
        rt->last_rx_ms = g_sys_ms;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     USART2 接收中断服务函数（摄像头识别板数据）
// 备注信息     摄像头板识别到垃圾后通过 USART2 发送数据
//              用户程序通过 uart_query_byte / uart_read_buffer 读取 UART_CAM
//-------------------------------------------------------------------------------------------------------------------
void USART2_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(USART2);
        uart_runtime_t *rt = &uart_runtime[UART_CAM];
        uint32_t next = (rt->head + 1) % UART_RX_BUF_SIZE;
        if (next != rt->tail) {
            rt->rx_buf[rt->head] = data;
            rt->head = next;
        }
        rt->last_rx_ms = g_sys_ms;
    }
}
