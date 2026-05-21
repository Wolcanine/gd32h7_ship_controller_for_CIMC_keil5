/*******************************************************************************
 * 文件名          ps2.h
 * 描述            PS2 手柄接收驱动 — SPI 引脚定义 & API 声明
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

#ifndef PS2_H
#define PS2_H

#include "gd32h7xx.h"

/* ==================== PS2 手柄 SPI 引脚定义 ==================== */
#define PS2_DI_PORT   GPIOB      /* 数据输入 (MISO) */
#define PS2_DI_PIN    GPIO_PIN_14
#define PS2_DO_PORT   GPIOB      /* 数据输出 (MOSI) */
#define PS2_DO_PIN    GPIO_PIN_15
#define PS2_CS_PORT   GPIOB      /* 片选 */
#define PS2_CS_PIN    GPIO_PIN_12
#define PS2_CLK_PORT  GPIOB      /* 时钟 */
#define PS2_CLK_PIN   GPIO_PIN_13

/* ==================== 手柄模式常量 ==================== */
#define PS2_MODE_DIGITAL  0x41   /* 数字模式 */
#define PS2_MODE_ANALOG   0x73   /* 模拟模式（摇杆+压感） */

/* ==================== 按键索引 ==================== */
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

/* ==================== PS2 数据结构体 ==================== */
typedef struct {
    uint8_t mode;
    uint8_t select:1;
    uint8_t l3:1;
    uint8_t r3:1;
    uint8_t start:1;
    uint8_t up:1;
    uint8_t right:1;
    uint8_t down:1;
    uint8_t left:1;
    uint8_t l2:1;
    uint8_t r2:1;
    uint8_t l1:1;
    uint8_t r1:1;
    uint8_t triangle:1;
    uint8_t circle:1;
    uint8_t cross:1;
    uint8_t square:1;
    uint8_t joy_left_x;
    uint8_t joy_left_y;
    uint8_t joy_right_x;
    uint8_t joy_right_y;
} PS2_Data_t;

extern PS2_Data_t PS2_Data;

void ps2_init(void);
void ps2_read_data(void);
uint8_t ps2_key_pressed(uint8_t key);
float ps2_get_throttle(void);
float ps2_get_steering(void);
uint8_t ps2_get_mode(void);
uint8_t ps2_is_connected(void);

#endif
