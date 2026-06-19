/*******************************************************************************
 * 文件名          ps2.c
 * 描述            PS2 手柄接收驱动 — 软件 SPI 模拟
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

#include "ps2.h"
#include <stdio.h>

/* ==================== 软件模拟 SPI 宏定义 ==================== */
/* 通过宏直接操作 GPIO，模拟 SPI 时序，提升代码执行效率 */
#define DI   gpio_input_bit_get(PS2_DI_PORT, PS2_DI_PIN)   /* 读取 MISO 数据 */
#define DO_H gpio_bit_set(PS2_DO_PORT, PS2_DO_PIN)          /* MOSI 置高 */
#define DO_L gpio_bit_reset(PS2_DO_PORT, PS2_DO_PIN)        /* MOSI 置低 */
#define CS_H gpio_bit_set(PS2_CS_PORT, PS2_CS_PIN)          /* CS 置高 (取消片选) */
#define CS_L gpio_bit_reset(PS2_CS_PORT, PS2_CS_PIN)        /* CS 置低 (选中手柄) */
#define CLK_H gpio_bit_set(PS2_CLK_PORT, PS2_CLK_PIN)       /* SCK 置高 */
#define CLK_L gpio_bit_reset(PS2_CLK_PORT, PS2_CLK_PIN)     /* SCK 置低 */

/* 全局 PS2 数据结构体，存储最新读取的手柄状态 */
PS2_Data_t PS2_Data;

/* 断连检测：连续 PS2_FAIL_THRESHOLD 帧无效即判离线 */
#define PS2_FAIL_THRESHOLD  10
static uint8_t ps2_fail_cnt = 0;
static uint8_t ps2_connected = 0;

/*
 * 软件微秒延时
 * 循环次数需根据 MCU 主频调整：
 * - GD32H7XX @168MHz 时约 60 次 ≈ 1us
 * - GD32H7XX @600MHz 时约 214 次 ≈ 1us (60 × 600/168)
 */
static void PS2_DelayUs(uint32_t us)
{
    uint32_t i, j;
    for(i = 0; i < us; i++) {
        for(j = 0; j < 214; j++) {
            __NOP();            /* 空操作，消耗一个时钟周期 */
        }
    }
}

/*
 * PS2 手柄 SPI 通信核心函数 (软件模拟 SPI)
 * 在时钟上升沿发送数据，下降沿接收数据，MSB 先行
 * cmd: 要发送的命令字节
 * 返回值: 从手柄读取到的响应字节
 */
static uint8_t PS2_SendCmd(uint8_t cmd)
{
    uint8_t ref, res = 0;
    /* 逐位发送：从 bit0 到 bit7，每次左移 1 位 */
    for(ref = 0x01; ref != 0x00; ref <<= 1) {
        /* 发送当前位 */
        if(cmd & ref)
            DO_H;               /* 该位为 1，MOSI 置高 */
        else
            DO_L;               /* 该位为 0，MOSI 置低 */
        CLK_L;                  /* 时钟下降沿，手柄锁存数据 */
        PS2_DelayUs(16);        /* 等待时序稳定 */
        /* 接收当前位 */
        if(DI)
            res |= ref;         /* MISO 为高，该位记为 1 */
        CLK_H;                  /* 时钟上升沿，手柄输出下一位 */
        PS2_DelayUs(16);
    }
    PS2_DelayUs(25);
    return res;
}

/*
 * PS2 手柄初始化
 * 配置 4 个 SPI 相关引脚：DI(输入)、DO/CS/CLK(推挽输出)
 * 初始化后 CS 和 CLK 保持高电平 (SPI 空闲状态)
 */
void ps2_init(void)
{
    /* 使能 GPIOA (DI/DO) 和 GPIOB (CS/CLK) 时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    /* DI: 浮空输入模式 (PA5, CN5 Pin 50) */
    gpio_mode_set(PS2_DI_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PS2_DI_PIN);

    /* DO、CS、CLK: 推挽输出，速度 50MHz */
    /* DO: PA7 (CN5 Pin 49), CS: PB12 (CN6 Pin 75), CLK: PB13 (CN5 Pin 22) */
    gpio_mode_set(PS2_DO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PS2_DO_PIN);
    gpio_mode_set(PS2_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PS2_CS_PIN);
    gpio_mode_set(PS2_CLK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PS2_CLK_PIN);

    gpio_output_options_set(PS2_DO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, PS2_DO_PIN);
    gpio_output_options_set(PS2_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, PS2_CS_PIN);
    gpio_output_options_set(PS2_CLK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, PS2_CLK_PIN);

    /* 初始状态：CS 和 CLK 高电平，DO 高电平 (SPI 空闲) */
    CS_H;
    CLK_H;
    DO_H;
}

/*
 * 读取并解析完整的一帧 PS2 数据
 * PS2 协议每帧 9 字节：
 *   [0]=0x01  (命令头)
 *   [1]=0x42  (轮询命令)
 *   [2~8]=0x00 (填充，用于接收手柄响应)
 * 响应字节含义：
 *   buf[1] = 模式字节 (0x41=数字, 0x73=模拟)
 *   buf[3] = 按键状态字节1 (取反后解析)
 *   buf[4] = 按键状态字节2 (取反后解析)
 *   buf[5] = 右摇杆 X
 *   buf[6] = 右摇杆 Y
 *   buf[7] = 左摇杆 X
 *   buf[8] = 左摇杆 Y
 */
void ps2_read_data(void)
{
    uint8_t rcv_buf[9] = {0};
    const uint8_t cmd_buf[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t key1, key2;

    /* CS 拉低，开始通信 */
    CS_L;
    for(int i = 0; i < 9; i++) {
        rcv_buf[i] = PS2_SendCmd(cmd_buf[i]);
    }
    CS_H;                       /* CS 拉高，结束通信 */

    /* 解析模式 */
    PS2_Data.mode = rcv_buf[1];

    /* 按键字节取反：PS2 协议中按键按下为低电平，取反后 1 表示按下 */
    key1 = ~rcv_buf[3];         /* 按键字节1 */
    key2 = ~rcv_buf[4];         /* 按键字节2 */

    /* ---- 按键字节1 解析 (key1) ---- */
    PS2_Data.select   = (key1 >> 0) & 0x01;
    PS2_Data.l3       = (key1 >> 1) & 0x01;
    PS2_Data.r3       = (key1 >> 2) & 0x01;
    PS2_Data.start    = (key1 >> 3) & 0x01;
    PS2_Data.up       = (key1 >> 4) & 0x01;
    PS2_Data.right    = (key1 >> 5) & 0x01;
    PS2_Data.down     = (key1 >> 6) & 0x01;
    PS2_Data.left     = (key1 >> 7) & 0x01;

    /* ---- 按键字节2 解析 (key2) ---- */
    PS2_Data.l2       = (key2 >> 0) & 0x01;
    PS2_Data.r2       = (key2 >> 1) & 0x01;
    PS2_Data.l1       = (key2 >> 2) & 0x01;
    PS2_Data.r1       = (key2 >> 3) & 0x01;
    PS2_Data.triangle = (key2 >> 4) & 0x01;
    PS2_Data.circle   = (key2 >> 5) & 0x01;
    PS2_Data.cross    = (key2 >> 6) & 0x01;
    PS2_Data.square   = (key2 >> 7) & 0x01;

    /* ---- 摇杆模拟值 ---- */
    PS2_Data.joy_right_x = rcv_buf[5];
    PS2_Data.joy_right_y = rcv_buf[6];
    PS2_Data.joy_left_x  = rcv_buf[7];
    PS2_Data.joy_left_y  = rcv_buf[8];

    /* ---- 断连检测：模式字节有效则视为连接正常 ---- */
    if (rcv_buf[1] == PS2_MODE_DIGITAL || rcv_buf[1] == PS2_MODE_ANALOG) {
        ps2_fail_cnt = 0;
        ps2_connected = 1;
    } else {
        ps2_fail_cnt++;
        if (ps2_fail_cnt >= PS2_FAIL_THRESHOLD) {
            ps2_connected = 0;
        }
    }
}

/*
 * 获取左摇杆归一化油门值
 * 利用左摇杆 Y 轴 (上下) 控制油门/速度
 * 计算方式: (127 - 原始值) / 127
 *   - 摇杆推最上 (0)   → +1.0 (最大前进)
 *   - 摇杆中位 (127)   →  0.0 (零油门)
 *   - 摇杆拉最下 (255) → -1.0 (最大后退)
 * 返回值: float 类型 -1.0f ~ 1.0f
 */
float ps2_get_throttle(void)
{
    if (!ps2_connected) return 0.0f;

    /* 摇杆垂直方向原始值：0=最上，127≈中心，255=最下 */
    float throttle = (127.0f - (float)PS2_Data.joy_left_y) / 127.0f;

    /* 钳位到 [-1.0, 1.0]，防止计算误差 */
    if(throttle > 1.0f)  throttle = 1.0f;
    if(throttle < -1.0f) throttle = -1.0f;
    return throttle;
}

/*
 * 获取左摇杆归一化转向值
 * 利用左摇杆 X 轴 (左右) 控制转向角度
 * 计算方式: (原始值 - 128) / 127
 *   - 摇杆推最左 (0)   → -1.0 (最大左转)
 *   - 摇杆中位 (128)   →  0.0 (回正)
 *   - 摇杆推最右 (255) → +1.0 (最大右转)
 * 返回值: float 类型 -1.0f ~ 1.0f
 */
float ps2_get_steering(void)
{
    if (!ps2_connected) return 0.0f;

    /* 摇杆水平方向原始值：0=最左，128≈中心，255=最右 */
    float steering = ((float)PS2_Data.joy_left_x - 128.0f) / 127.0f;

    /* 钳位到 [-1.0, 1.0] */
    if(steering > 1.0f)  steering = 1.0f;
    if(steering < -1.0f) steering = -1.0f;
    return steering;
}

/*
 * 获取当前 PS2 手柄工作模式
 * 返回值: PS2_MODE_DIGITAL (0x41) 或 PS2_MODE_ANALOG (0x73)
 */
uint8_t ps2_get_mode(void)
{
    if (!ps2_connected) return PS2_MODE_DIGITAL;      /* 断连时默认数字模式（手动安全） */
    return PS2_Data.mode;
}

uint8_t ps2_is_connected(void)
{
    return ps2_connected;
}

/*
 * 兼容旧代码的按键检测接口
 * 通过按键索引查询对应按键的按下状态
 * key: 按键索引宏 (PSB_SELECT / PSB_START 等)
 * 返回值: 0=未按下, 1=按下
 */
uint8_t ps2_key_pressed(uint8_t key)
{
    switch(key) {
        case PSB_SELECT:    return PS2_Data.select;
        case PSB_L3:        return PS2_Data.l3;
        case PSB_R3:        return PS2_Data.r3;
        case PSB_START:     return PS2_Data.start;
        case PSB_PAD_UP:    return PS2_Data.up;
        case PSB_PAD_RIGHT: return PS2_Data.right;
        case PSB_PAD_DOWN:  return PS2_Data.down;
        case PSB_PAD_LEFT:  return PS2_Data.left;
        case PSB_L2:        return PS2_Data.l2;
        case PSB_R2:        return PS2_Data.r2;
        case PSB_L1:        return PS2_Data.l1;
        case PSB_R1:        return PS2_Data.r1;
        case PSB_TRIANGLE:  return PS2_Data.triangle;
        case PSB_CIRCLE:    return PS2_Data.circle;
        case PSB_CROSS:     return PS2_Data.cross;
        case PSB_SQUARE:    return PS2_Data.square;
        default:            return 0;  /* 未知按键，返回未按下 */
    }
}

/*******************************************************************************
 * 函数名    ps2_print_state
 * 描述      通过串口 (printf → UART7) 输出完整的 PS2 手柄状态
 *           用于遥控器模块独立测试，输出内容包括:
 *             - 连接状态 & 工作模式
 *             - 全部 16 个按键状态（按键名 + ●/○ 表示按下/松开）
 *             - 左右摇杆原始值 (0~255)
 *             - 归一化油门/转向值 (-1.0 ~ +1.0)
 * 参数      none
 * 返回值    none
 * 备注      调用前需确保 uart_init(UART_DBG, 9600) 已完成
 ******************************************************************************/
void ps2_print_state(void)
{
    /* ---- 获取当前值（避免多次读取略有偏差） ---- */
    uint8_t mode      = ps2_get_mode();
    uint8_t connected = ps2_is_connected();
    float   throttle  = ps2_get_throttle();
    float   steering  = ps2_get_steering();

    /* ---- 辅助宏：按键按下显示 #，松开显示 . ---- */
#define PS2_BTN(v)  ((v) ? "#" : ".")

    /* ==================== 第1行：连接 & 模式 ==================== */
    printf("\r\n========================================\r\n");
    printf("  PS2 Remote Controller Status\r\n");
    printf("========================================\r\n");
    printf("  Connection : %s\r\n", connected ? "ONLINE" : "OFFLINE");
    printf("  Mode       : 0x%02X (%s)\r\n",
           mode, (mode == PS2_MODE_ANALOG) ? "ANALOG"  :
                 (mode == PS2_MODE_DIGITAL) ? "DIGITAL" : "UNKNOWN");

    /* ==================== 第2行：方向键 (D-Pad) ==================== */
    printf("  ----------------------------------------\r\n");
    printf("  [D-Pad]\r\n");
    printf("         UP   : %s\r\n", PS2_BTN(PS2_Data.up));
    printf("    LEFT RIGHT: %s / %s\r\n",
           PS2_BTN(PS2_Data.left), PS2_BTN(PS2_Data.right));
    printf("        DOWN  : %s\r\n", PS2_BTN(PS2_Data.down));

    /* ==================== 第3行：动作键 (右侧4键) ==================== */
    printf("  ----------------------------------------\r\n");
    printf("  [Action Buttons]\r\n");
    printf("    ^ TRIANGLE : %s     O CIRCLE : %s\r\n",
           PS2_BTN(PS2_Data.triangle), PS2_BTN(PS2_Data.circle));
    printf("    X CROSS    : %s     # SQUARE : %s\r\n",
           PS2_BTN(PS2_Data.cross), PS2_BTN(PS2_Data.square));

    /* ==================== 第4行：肩键 & 功能键 ==================== */
    printf("  ----------------------------------------\r\n");
    printf("  [Shoulder & Function]\r\n");
    printf("    L1: %s  L2: %s  L3: %s\r\n",
           PS2_BTN(PS2_Data.l1), PS2_BTN(PS2_Data.l2), PS2_BTN(PS2_Data.l3));
    printf("    R1: %s  R2: %s  R3: %s\r\n",
           PS2_BTN(PS2_Data.r1), PS2_BTN(PS2_Data.r2), PS2_BTN(PS2_Data.r3));
    printf("    SELECT: %s      START: %s\r\n",
           PS2_BTN(PS2_Data.select), PS2_BTN(PS2_Data.start));

    /* ==================== 第5行：摇杆原始值 ==================== */
    printf("  ----------------------------------------\r\n");
    printf("  [Joysticks - Raw ADC (0~255)]\r\n");
    printf("    Left  X: %3u (128=center, >128=right)\r\n",
           PS2_Data.joy_left_x);
    printf("    Left  Y: %3u (128=center, <128=up)\r\n",
           PS2_Data.joy_left_y);
    printf("    Right X: %3u\r\n", PS2_Data.joy_right_x);
    printf("    Right Y: %3u\r\n", PS2_Data.joy_right_y);

    /* ==================== 第6行：归一化控制量 ==================== */
    printf("  ----------------------------------------\r\n");
    printf("  [Normalized Control (-1.0 ~ +1.0)]\r\n");
    printf("    Throttle : %+6.3f  (0=stop, +1=full fwd, -1=full rev)\r\n",
           (double)throttle);
    printf("    Steering : %+6.3f  (0=center, +1=right, -1=left)\r\n",
           (double)steering);

    printf("========================================\r\n");

#undef PS2_BTN
}
