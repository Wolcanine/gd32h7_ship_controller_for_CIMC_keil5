/*******************************************************************************
 * 文件名          oled.h
 * 描述            OLED SSD1306 驱动 — 软件 I2C (PD12/PD13)
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2025-03-10      Lingyu Meng     初始版本
 * 2026-05-21      CIMC            整合至水面垃圾船工程
 ******************************************************************************/

#ifndef __OLED_H
#define __OLED_H

#include "gd32h7xx.h"
#include "systick.h"

/* ==================== 引脚定义 ==================== */
#define OLED_SCL_PORT    GPIOD
#define OLED_SCL_PIN     GPIO_PIN_12
#define OLED_SDA_PORT    GPIOD
#define OLED_SDA_PIN     GPIO_PIN_13

#define OLED_SCLK_Set()     gpio_bit_set(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SCLK_Clr()     gpio_bit_reset(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SDIN_Set()     gpio_bit_set(OLED_SDA_PORT, OLED_SDA_PIN)
#define OLED_SDIN_Clr()     gpio_bit_reset(OLED_SDA_PORT, OLED_SDA_PIN)
#define IIC_READ_SDA  gpio_input_bit_get(OLED_SDA_PORT, OLED_SDA_PIN)

#define OLED_CMD  0
#define OLED_DATA 1

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_DrawPoint(uint8_t x, uint8_t y);
void OLED_ClearPoint(uint8_t x, uint8_t y);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
void OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[]);

#endif
