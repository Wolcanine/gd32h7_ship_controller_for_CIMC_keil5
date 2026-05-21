/*******************************************************************************
 * 文件名          oled.c
 * 描述            OLED SSD1306 驱动 — 软件 I2C 模拟
 *                 分辨率 128×32，I2C 地址 0x78 (7bit: 0x3C)
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2025-03-10      Lingyu Meng     初始版本
 * 2026-05-21      CIMC            整合至水面垃圾船工程
 ******************************************************************************/

#include "oled.h"
#include "oledfont.h"

#define u8  uint8_t
#define u32 uint32_t

static uint8_t OLED_GRAM[144][4];

/*******************************************************************************
 * 函数名    IIC_delay
 * 描述      I2C 时序延时（空循环）
 ******************************************************************************/
static void IIC_delay(void)
{
    __IO uint16_t t = 300;
    while (t--);
}

/*******************************************************************************
 * 函数名    I2C_Start
 * 描述      I2C 起始信号
 ******************************************************************************/
static void I2C_Start(void)
{
    gpio_mode_set(OLED_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, OLED_SDA_PIN);
    gpio_output_options_set(OLED_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100_220MHZ, OLED_SDA_PIN);

    OLED_SDIN_Set();
    OLED_SCLK_Set();
    IIC_delay();
    OLED_SDIN_Clr();
    IIC_delay();
    OLED_SCLK_Clr();
    IIC_delay();
}

/*******************************************************************************
 * 函数名    I2C_Stop
 * 描述      I2C 停止信号
 ******************************************************************************/
static void I2C_Stop(void)
{
    gpio_mode_set(OLED_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, OLED_SDA_PIN);
    gpio_output_options_set(OLED_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100_220MHZ, OLED_SDA_PIN);

    OLED_SDIN_Clr();
    OLED_SCLK_Set();
    IIC_delay();
    OLED_SDIN_Set();
}

/*******************************************************************************
 * 函数名    I2C_WaitAck
 * 描述      等待从机应答
 ******************************************************************************/
static void I2C_WaitAck(void)
{
    uint8_t t = 0;

    OLED_SDIN_Set();
    IIC_delay();
    OLED_SCLK_Set();
    IIC_delay();

    gpio_mode_set(OLED_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, OLED_SDA_PIN);
    while (IIC_READ_SDA) {
        t++;
        if (t > 254) {
            I2C_Stop();
            break;
        }
    }
    OLED_SCLK_Clr();
    IIC_delay();

    gpio_mode_set(OLED_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, OLED_SDA_PIN);
    gpio_output_options_set(OLED_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100_220MHZ, OLED_SDA_PIN);
}

/*******************************************************************************
 * 函数名    Send_Byte
 * 描述      向 I2C 总线发送一个字节
 ******************************************************************************/
static void Send_Byte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        OLED_SCLK_Clr();
        if (dat & 0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        IIC_delay();
        OLED_SCLK_Set();
        IIC_delay();
        OLED_SCLK_Clr();
        dat <<= 1;
    }
}

/*******************************************************************************
 * 函数名    OLED_WR_Byte
 * 描述      向 SSD1306 写入一个字节
 * 参数      mode    0=命令, 1=数据
 ******************************************************************************/
static void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    I2C_Start();
    Send_Byte(0x78);
    I2C_WaitAck();
    if (mode) Send_Byte(0x40);
    else      Send_Byte(0x00);
    I2C_WaitAck();
    Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

void OLED_ColorTurn(uint8_t i)
{
    if (i == 0) OLED_WR_Byte(0xA6, OLED_CMD);
    if (i == 1) OLED_WR_Byte(0xA7, OLED_CMD);
}

void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0) {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1) {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_Refresh(void)
{
    uint8_t i, n;
    for (i = 0; i < 4; i++) {
        OLED_WR_Byte(0xB0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 4; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0;
    OLED_Refresh();
}

void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    OLED_GRAM[x][i] |= n;
}

void OLED_ClearPoint(uint8_t x, uint8_t y)
{
    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    OLED_GRAM[x][i] |= n;
    OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    uint8_t i, k, k1, k2;
    if (x1 == x2) {
        for (i = 0; i < (y2 - y1); i++)
            OLED_DrawPoint(x1, y1 + i);
    } else if (y1 == y2) {
        for (i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1);
    } else {
        k1 = y2 - y1;
        k2 = x2 - x1;
        k = k1 * 10 / k2;
        for (i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1 + i * k / 10);
    }
}

void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a, b, num;
    a = 0;
    b = r;
    while (2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b);
        OLED_DrawPoint(x - a, y - b);
        OLED_DrawPoint(x - a, y + b);
        OLED_DrawPoint(x + a, y + b);
        OLED_DrawPoint(x + b, y + a);
        OLED_DrawPoint(x + b, y - a);
        OLED_DrawPoint(x - b, y - a);
        OLED_DrawPoint(x - b, y + a);
        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0) { b--; a--; }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t y0 = y;
    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';
    for (i = 0; i < size2; i++) {
        if (size1 == 12)       temp = asc2_1206[chr1][i];
        else if (size1 == 16)  temp = asc2_1608[chr1][i];
        else if (size1 == 24)  temp = asc2_2412[chr1][i];
        else return;
        for (m = 0; m < 8; m++) {
            if (temp & 0x80) OLED_DrawPoint(x, y);
            else             OLED_ClearPoint(x, y);
            temp <<= 1;
            y++;
            if ((y - y0) == size1) { y = y0; x++; break; }
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1)
{
    while ((*chr >= ' ') && (*chr <= '~')) {
        OLED_ShowChar(x, y, *chr, size1);
        x += size1 / 2;
        if (x > 128 - size1) { x = 0; y += 2; }
        chr++;
    }
}

static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1)
{
    uint8_t t, temp;
    for (t = 0; t < len; t++) {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if (temp == 0)
            OLED_ShowChar(x + (size1 / 2) * t, y, '0', size1);
        else
            OLED_ShowChar(x + (size1 / 2) * t, y, temp + '0', size1);
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1)
{
    uint8_t i, m, n = 0, temp, chr1;
    uint8_t x0 = x, y0 = y;
    uint8_t size3 = size1 / 8;
    while (size3--) {
        chr1 = num * size1 / 8 + n;
        n++;
        for (i = 0; i < size1; i++) {
            if (size1 == 16)       temp = Hzk1[chr1][i];
            else if (size1 == 24)  temp = Hzk2[chr1][i];
            else if (size1 == 32)  temp = Hzk3[chr1][i];
            else if (size1 == 64)  temp = Hzk4[chr1][i];
            else return;
            for (m = 0; m < 8; m++) {
                if (temp & 0x01) OLED_DrawPoint(x, y);
                else             OLED_ClearPoint(x, y);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == size1) { x = x0; y0 = y0 + 8; }
            y = y0;
        }
    }
}

void OLED_ScrollDisplay(uint8_t num, uint8_t space)
{
    uint8_t i, n, t = 0, m = 0, r;
    while (1) {
        if (m == 0) { OLED_ShowChinese(128, 8, t, 16); t++; }
        if (t == num) {
            for (r = 0; r < 16 * space; r++) {
                for (i = 0; i < 144; i++)
                    for (n = 0; n < 4; n++)
                        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
                OLED_Refresh();
            }
            t = 0;
        }
        m++;
        if (m == 16) m = 0;
        for (i = 0; i < 144; i++)
            for (n = 0; n < 4; n++)
                OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
        OLED_Refresh();
    }
}

static void OLED_WR_BP(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xB0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xF0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0F) | 0x01, OLED_CMD);
}

void OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[])
{
    uint32_t j = 0;
    uint8_t x, y;
    for (y = y0; y < y1; y++) {
        OLED_WR_BP(x0, y);
        for (x = x0; x < x1; x++) {
            OLED_WR_Byte(BMP[j], OLED_DATA);
            j++;
        }
    }
}

/*******************************************************************************
 * 函数名    OLED_Init
 * 描述      初始化 OLED 显示屏 (SSD1306, 128×32)
 *           引脚 PD12(SCL), PD13(SDA)，软件 I2C
 ******************************************************************************/
void OLED_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);

    gpio_mode_set(OLED_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, OLED_SCL_PIN);
    gpio_output_options_set(OLED_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100_220MHZ, OLED_SCL_PIN);
    OLED_SCLK_Set();

    gpio_mode_set(OLED_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, OLED_SDA_PIN);
    gpio_output_options_set(OLED_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100_220MHZ, OLED_SDA_PIN);
    OLED_SDIN_Set();

    delay_1ms(500);

    OLED_WR_Byte(0xAE, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x1F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
    OLED_Clear();
}
