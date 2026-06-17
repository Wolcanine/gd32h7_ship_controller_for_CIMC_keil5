/*******************************************************************************
 * 文件名          pca9685.c
 * 描述            PCA9685 I2C PWM 发生器驱动 — 16 路 12-bit PWM
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-08      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

// pca9685.c — I2C PWM 发生器驱动 (PCA9685)
// 使用现有 MyI2C 软件 I2C 驱动，与 MPU6050 共享 I2C 总线
// 参考 Adafruit PCA9685 库移植，适配 GD32H7XX + 裸机环境
#include "pca9685.h"
#include "MyI2C.h"
#include <stdio.h>

// ==================== PCA9685 寄存器定义 ====================
#define PCA9685_MODE1        0x00
#define PCA9685_MODE2        0x01
#define PCA9685_PRESCALE     0xFE

#define LED0_ON_L            0x06
#define LED0_ON_H            0x07
#define LED0_OFF_L           0x08
#define LED0_OFF_H           0x09

#define MODE1_RESTART        0x80
#define MODE1_EXTCLK         0x40
#define MODE1_AI             0x20    // Auto-Increment
#define MODE1_SLEEP          0x10
#define MODE1_ALLCALL        0x01

#define MODE2_OCH            0x08    // Output change on STOP (not ACK)
#define MODE2_OUTDRV         0x04    // Totem-pole output (not open-drain)
#define MODE2_OUTNE_HIZ      0x02    // OUTNE: high-impedance when OE=1

// ==================== 内部：I2C 读写原语 ====================

static void write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t addr_w = (dev_addr << 1) | 0;
    MyI2C_Start();
    MyI2C_SendByte(addr_w);
    MyI2C_ReceiveAck();     // 忽略应答位（下同）
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

static uint8_t read_reg(uint8_t dev_addr, uint8_t reg)
{
    uint8_t addr_w = (dev_addr << 1) | 0;
    uint8_t addr_r = (dev_addr << 1) | 1;
    uint8_t data;

    MyI2C_Start();
    MyI2C_SendByte(addr_w);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
    MyI2C_Stop();

    MyI2C_Start();
    MyI2C_SendByte(addr_r);
    MyI2C_ReceiveAck();
    data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);       // NACK — 单字节读取
    MyI2C_Stop();

    return data;
}

// ==================== 公共 API ====================

void pca9685_init(uint8_t addr)
{
    /* ---- OE 引脚：推挽输出，初始拉低使能 ---- */
    rcu_periph_clock_enable(PCA9685_OE_RCU);
    gpio_mode_set(PCA9685_OE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PCA9685_OE_PIN);
    gpio_output_options_set(PCA9685_OE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, PCA9685_OE_PIN);
    gpio_bit_reset(PCA9685_OE_PORT, PCA9685_OE_PIN);   /* OE=L → 输出使能 */

    /* MODE1: 复位到默认值（使能 AI, 清除 SLEEP） */
    write_reg(addr, PCA9685_MODE1, 0x00);

    /* MODE2: 推挽输出 + STOP 时更新（避免开漏浮空导致舵机抖振） */
    write_reg(addr, PCA9685_MODE2, MODE2_OUTDRV | MODE2_OCH);

    /* 回读 MODE2 确认配置 */
    {
        uint8_t m2 = read_reg(addr, PCA9685_MODE2);
        printf("PCA9685(0x%02X) MODE2 = 0x%02X (OUTDRV=%d OCH=%d)\r\n",
               addr, (unsigned)m2,
               (int)((m2 & MODE2_OUTDRV) ? 1 : 0),
               (int)((m2 & MODE2_OCH) ? 1 : 0));
    }

    /* 回读验证：确认 I2C 通信正常 */
    uint8_t val = read_reg(addr, PCA9685_MODE1);
    printf("PCA9685(0x%02X) MODE1 = 0x%02X", addr, (unsigned)val);
    if (val == 0x00) {
        printf(" => I2C OK\r\n");
    } else {
        printf(" => CHECK HARDWARE (OE? V+? wiring?)\r\n");
    }
}

/*******************************************************************************
 * 函数名    pca9685_output_enable
 * 描述      拉低 OE 引脚，使能所有 PWM 通道输出（舵机上电保持角度）
 ******************************************************************************/
void pca9685_output_enable(void)
{
    gpio_bit_reset(PCA9685_OE_PORT, PCA9685_OE_PIN);
}

/*******************************************************************************
 * 函数名    pca9685_output_disable
 * 描述      拉高 OE 引脚，关闭所有 PWM 通道输出（舵机断电 limp，省电 + 防抖）
 ******************************************************************************/
void pca9685_output_disable(void)
{
    gpio_bit_set(PCA9685_OE_PORT, PCA9685_OE_PIN);
}

void pca9685_set_pwm_freq(uint8_t addr, float freq)
{
    if (freq < 1.0f)  freq = 1.0f;
    if (freq > 1600.0f) freq = 1600.0f;

    /* PCA9685 内部振荡器 25MHz, 12-bit 分辨率 (4096) */
    /* prescale = round(25000000 / (4096 * freq)) - 1 */
    float prescaleval = 25000000.0f / 4096.0f / freq - 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    uint8_t oldmode = read_reg(addr, PCA9685_MODE1);
    uint8_t sleepmode = (oldmode & 0x7F) | MODE1_SLEEP;   // bit4=1 sleep

    write_reg(addr, PCA9685_MODE1, sleepmode);             // 进入 sleep 方可写 prescale
    write_reg(addr, PCA9685_PRESCALE, prescale);
    write_reg(addr, PCA9685_MODE1, oldmode);               // 恢复原模式
    /* 等待振荡器稳定 ~500us (H759@600MHz: 500 × 600/168 ≈ 1786) */
    volatile uint32_t delay = 1786;
    while (delay--);

    write_reg(addr, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void pca9685_set_pwm(uint8_t addr, uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t reg_base = LED0_ON_L + 4 * ch;
    uint8_t addr_w = (addr << 1) | 0;

    MyI2C_Start();
    MyI2C_SendByte(addr_w);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg_base);
    MyI2C_ReceiveAck();
    /* Auto-Increment 模式下连续发送 4 字节 */
    MyI2C_SendByte((uint8_t)(on & 0xFF));
    MyI2C_ReceiveAck();
    MyI2C_SendByte((uint8_t)(on >> 8));
    MyI2C_ReceiveAck();
    MyI2C_SendByte((uint8_t)(off & 0xFF));
    MyI2C_ReceiveAck();
    MyI2C_SendByte((uint8_t)(off >> 8));
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

void pca9685_set_pin(uint8_t addr, uint8_t ch, uint16_t val, uint8_t invert)
{
    if (val > 4095) val = 4095;

    if (invert) {
        if (val == 0)
            pca9685_set_pwm(addr, ch, 4096, 0);     // 全开
        else if (val == 4095)
            pca9685_set_pwm(addr, ch, 0, 4096);     // 全关
        else
            pca9685_set_pwm(addr, ch, 0, 4095 - val);
    } else {
        if (val == 4095)
            pca9685_set_pwm(addr, ch, 4096, 0);     // 全开
        else if (val == 0)
            pca9685_set_pwm(addr, ch, 0, 4096);     // 全关
        else
            pca9685_set_pwm(addr, ch, 0, val);
    }
}

uint16_t pca9685_angle_to_pulse(float angle, uint16_t servo_min, uint16_t servo_max)
{
    if (angle < 0.0f)                angle = 0.0f;
    if (angle > PCA9685_SERVO_ANGLE_MAX) angle = PCA9685_SERVO_ANGLE_MAX;
    return (uint16_t)(servo_min + (uint16_t)((servo_max - servo_min) * angle / PCA9685_SERVO_ANGLE_MAX));
}
