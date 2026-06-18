/*******************************************************************************
 * 文件名          main.c
 * 描述            水面垃圾清扫船主控程序
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-06-17      CIMC           代码整理：提取System_Init，测试模式独立函数
 ******************************************************************************/

/* ==================== 模块测试开关 ==================== */
/* 同时只启用一个。全部注释 = 正常全系统模式 */
//#define MOTOR_MODULE_TEST
//#define PCA9685_MODULE_TEST
//#define DIRECT_SERVO_SWEEP
#define SW_UART_ECHO_TEST

#include "gd32h7xx.h"
#include "systick.h"
#include <stdio.h>
#include "uart_driver.h"
#include "main.h"
#include "ps2.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "pwm_output.h"
#include "ship_controller.h"
#include "auto_nav.h"
#include "laser.h"
#include "gd32_driver_pit.h"
#include "pca9685.h"
#include "servo_arm.h"
#include "step_motor.h"
#include "sw_uart.h"

/* ==================== 全局变量 ==================== */
volatile uint8_t  timer20ms_flag = 0;   /* 50Hz 周期标志, TIMER3 ISR 置位 */
volatile uint32_t g_sys_ms      = 0;   /* 毫秒计数器, SysTick ISR 递增   */

/* ==================== printf 重定向 ==================== */
int fputc(int ch, FILE *f)
{
    while (RESET == usart_flag_get(UART4, USART_FLAG_TBE));
    usart_data_transmit(UART4, (uint8_t)ch);
    return ch;
}

/* ==================== 公共系统初始化 ==================== */
static void System_Init(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    uart_init(UART_DBG, 115200);
    systick_config();
    printf("\r\n===== Boat Control System Starting =====\r\n");
}

/* ==================== 正常全系统模式 ==================== */
#if !defined(MOTOR_MODULE_TEST) && !defined(PCA9685_MODULE_TEST) && \
    !defined(DIRECT_SERVO_SWEEP) && !defined(SW_UART_ECHO_TEST)
static void System_Run(void)
{
    /* ---- PS2 ---- */
    ps2_init();
    printf("[PS2] OK\r\n");

    /* ---- MPU6050 (内部调用 MyI2C_Init, 失败不阻塞) ---- */
    if (MPU6050_Init() != MPU6050_OK) {
        printf("[MPU6050] FAILED, continuing without gyro\r\n");
    } else {
        printf("[MPU6050] OK\r\n");
    }

    /* ---- TOF200F 激光 ---- */
    Laser_Init();
    printf("[TOF] OK\r\n");

    /* ---- 电机 PWM ---- */
    pwm_output_init();
    printf("[PWM] OK\r\n");

    /* ---- 船舶控制器 ---- */
    ShipController sc;
    ShipController_Init(&sc, 0.5f, 0.01f, 0.1f, 10.0f, 90.0f, 0.5f);
    printf("[ShipCtrl] OK\r\n");

    /* ---- 自动航行 (默认手动模式) ---- */
    AutoNav_Init();
    printf("[AutoNav] OK (manual)\r\n");

    /* ---- 机械臂舵机 ---- */
    ServoArm_Init();
    printf("[ServoArm] OK\r\n");

    /* ---- 步进电机传送带 ---- */
    Stepper_Init();
    printf("[Stepper] OK (TIMER6 2kHz)\r\n");

    /* ---- 50Hz 控制定时器 ---- */
    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz started\r\n");

    printf("===== System Ready =====\r\n\r\n");

    /* ==================== 主循环 ==================== */
    uint32_t last_loop_ms = 0;
    uint8_t  estop_flag   = 0;
    uint8_t  print_cnt    = 0;

    while (1) {
        /* 等待 50Hz 周期 (超时 500ms → 紧急停车) */
        while (!timer20ms_flag) {
            __WFI();
            if (!estop_flag && (g_sys_ms - last_loop_ms > 500)) {
                estop_flag = 1;
                pwm_set_left_duty(0.0f);
                pwm_set_right_duty(0.0f);
                printf("\r\n*** EMERGENCY STOP: loop timeout! ***\r\n");
            }
        }
        timer20ms_flag = 0;
        estop_flag     = 0;
        last_loop_ms   = g_sys_ms;

        /* 传感器 */
        ps2_read_data();
        MPU6050_Update();
        Laser_GetDistanceCm();

        /* 机械臂 */
        ServoArm_RemoteControl();
        ServoArm_HandlePresets();
        ServoArm_SmoothUpdate();

        /* 传送带 (步进电机): PS2 L1=收垃圾(正转), L2=释放(反转), 否则停止 */
        if (PS2_Data.l1) {
            Stepper_SetSpeed(1, 4);     /* 正转, 500Hz 步进 */
        } else if (PS2_Data.l2) {
            Stepper_SetSpeed(-1, 4);    /* 反转, 500Hz 步进 */
        } else {
            Stepper_SetSpeed(0, 0);     /* 停止 */
        }

        /* 1Hz 上报 */
        if (++print_cnt >= 50) {
            print_cnt = 0;
            printf("PS2:%.2f/%.2f GYRO:%+04ld/%+04ld/%+04ld "
                   "ACCEL:%+05ld/%+05ld/%+05ld LASER:%4u MODE:0x%02X\r\n",
                   (double)ps2_get_throttle(), (double)ps2_get_steering(),
                   (long)MPU6050_GetGyroX_dps100(), (long)MPU6050_GetGyroY_dps100(),
                   (long)MPU6050_GetGyroZ_dps100(),
                   (long)MPU6050_GetAccelX_mg(),   (long)MPU6050_GetAccelY_mg(),
                   (long)MPU6050_GetAccelZ_mg(),
                   (unsigned)Laser_GetDistanceCm(), (unsigned)ps2_get_mode());
            ServoArm_PrintStatus();
            printf("STEP: %ld steps\r\n", (long)Stepper_GetCurStep());
        }
    }
}
#endif /* !MOTOR_MODULE_TEST && !PCA9685_MODULE_TEST && !DIRECT_SERVO_SWEEP && !SW_UART_ECHO_TEST */

/* ==================== 电机模块测试 ==================== */
#ifdef MOTOR_MODULE_TEST
static void test_motor(void)
{
    float throttle = 0.0f, diff = 0.0f, left_duty, right_duty;
    uint8_t p_up = 0, p_down = 0, p_left = 0, p_right = 0;
    uint8_t p_tri = 0, p_cross = 0, p_square = 0, p_circle = 0, changed;

    ps2_init();
    pwm_output_init();
    pit_ms_init(PIT_TIMER3, 20);

    printf("===============================================\r\n");
    printf("  4-Motor PWM Test -- PS2 Remote Control\r\n");
    printf("  Left : D0=PC2 D1=PC3   Right: D2=PC5 D3=PC10\r\n");
    printf("  Limit: 0~100%%, startup kick 70%%x100ms\r\n");
    printf("  UP/DN throttle, LT/RT steer, SQ/CI jump, X zero, TR stop\r\n");
    printf("===============================================\r\n\r\n");

    while (1) {
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;

        ps2_read_data();
        changed = 0;

        if (PS2_Data.up    && !p_up)    { throttle += 0.01f; changed = 1; }
        if (PS2_Data.down  && !p_down)  { throttle -= 0.01f; changed = 1; }
        if (PS2_Data.square && !p_square) { throttle += 0.05f; changed = 1; }
        if (PS2_Data.circle && !p_circle) { throttle -= 0.05f; changed = 1; }
        if (PS2_Data.left  && !p_left)  { diff -= 0.01f; changed = 1; }
        if (PS2_Data.right && !p_right) { diff += 0.01f; changed = 1; }
        if (PS2_Data.cross && !p_cross) { diff = 0.0f;  changed = 1; }
        if (PS2_Data.triangle && !p_tri) { throttle = 0.0f; diff = 0.0f; changed = 1; }

        if (throttle > 1.0f) throttle = 1.0f;
        if (throttle < 0.0f) throttle = 0.0f;
        if (diff > 1.0f)     diff = 1.0f;
        if (diff < -1.0f)    diff = -1.0f;

        left_duty  = throttle + diff;
        right_duty = throttle - diff;
        if (left_duty  > 1.0f) left_duty  = 1.0f;
        if (left_duty  < 0.0f) left_duty  = 0.0f;
        if (right_duty > 1.0f) right_duty = 1.0f;
        if (right_duty < 0.0f) right_duty = 0.0f;

        pwm_set_left_duty(left_duty);
        pwm_set_right_duty(right_duty);
        pwm_throttle_tick();

        p_up = PS2_Data.up;       p_down  = PS2_Data.down;
        p_left = PS2_Data.left;   p_right = PS2_Data.right;
        p_tri = PS2_Data.triangle; p_cross = PS2_Data.cross;
        p_square = PS2_Data.square; p_circle = PS2_Data.circle;

        if (changed) printf("MOTOR: T=%+.2f D=%+.2f | L=%+.2f R=%+.2f\r\n",
                            throttle, diff, left_duty, right_duty);
    }
}
#endif

/* ==================== PCA9685 模块测试 ==================== */
#ifdef PCA9685_MODULE_TEST
static void test_pca9685(void)
{
    #define S_MIN  102
    #define S_MAX  512
    static const float angles[7] = {0,30,60,90,120,150,180};

    MyI2C_Init();
    pca9685_init(PCA9685_I2C_ADDR);
    pca9685_set_pwm_freq(PCA9685_I2C_ADDR, PCA9685_SERVO_FREQ);
    pca9685_output_enable();

    for (int i = 0; i < 7; i++) {
        uint16_t p = pca9685_angle_to_pulse(angles[i], S_MIN, S_MAX);
        pca9685_set_pwm(PCA9685_I2C_ADDR, 8+i, 0, p);
    }

    ps2_init();
    pit_ms_init(PIT_TIMER3, 20);

    printf("PCA9685 Test: ch8~14 fixed angles, ch0~5 reserved\r\n");
    while (1) {
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;
    }
}
#endif

/* ==================== 舵机扫频测试 ==================== */
#ifdef DIRECT_SERVO_SWEEP
static void test_sweep(void)
{
    #define SP_PORT  GPIOF
    #define SP_PIN   GPIO_PIN_9
    #define SP_RCU   RCU_GPIOF
    #define P_MIN    500
    #define P_MAX    2500
    #define P_STEP   40
    #define P_DIV    25

    int32_t pulse_us = 1500;
    int8_t  dir = 1;
    uint8_t cnt = 0;

    rcu_periph_clock_enable(SP_RCU);
    gpio_mode_set(SP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SP_PIN);
    gpio_output_options_set(SP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, SP_PIN);
    gpio_bit_reset(SP_PORT, SP_PIN);

    MyI2C_Init();
    pca9685_init(PCA9685_I2C_ADDR);
    pca9685_set_pwm_freq(PCA9685_I2C_ADDR, PCA9685_SERVO_FREQ);
    pca9685_output_enable();
    for (int i = 0; i < 7; i++)
        pca9685_set_pwm(PCA9685_I2C_ADDR, 8+i, 0,
            pca9685_angle_to_pulse(i*30.0f, 102, 512));

    volatile uint32_t cal = 600000;
    uint32_t t0 = g_sys_ms; while (cal--);
    uint32_t t1 = g_sys_ms;
    uint32_t lpu = (t1>t0) ? 600000/((t1-t0)*1000) : 120;
    if (!lpu) lpu = 1;
    printf("Sweep: PF9 0.5~2.5ms, %u loops/us\r\n", (unsigned)lpu);

    uint16_t pc = 0;
    while (1) {
        gpio_bit_set(SP_PORT, SP_PIN);   { volatile uint32_t d = pulse_us*lpu; while(d--); }
        gpio_bit_reset(SP_PORT, SP_PIN); { volatile uint32_t d = (20000-pulse_us)*lpu; while(d--); }

        if (++cnt >= P_DIV) {
            cnt = 0;
            pulse_us += P_STEP * dir;
            if (pulse_us >= P_MAX) { pulse_us = P_MAX; dir = -1; }
            if (pulse_us <= P_MIN) { pulse_us = P_MIN; dir =  1; }
        }
        if (++pc >= 5) {
            pc = 0;
            printf("PF9: %4ld us (%5.1f deg)\r\n", (long)pulse_us,
                   (double)(pulse_us-P_MIN)*180.0/(P_MAX-P_MIN));
        }
    }
}
#endif

/* ==================== 软串口 PC 回显测试 ==================== */
#ifdef SW_UART_ECHO_TEST
static void test_sw_uart_echo(void)
{
    uint32_t sw1_rx_cnt = 0, sw1_tx_cnt = 0;
    uint32_t sw2_rx_cnt = 0, sw2_tx_cnt = 0;
    uint32_t last_print_ms = 0;
    uint8_t  byte;

    /* 全部 printf 在 SwUart_Init 之前, 确保能看到 */
    printf("\r\n===== SW UART Echo to PC =====\r\n");
    printf("CH1: PE2/PE5  CH2: PF7/PF6  115200\r\n");

    printf("Calling SwUart_Init...\r\n");
    SwUart_Init();
    printf("SwUart_Init done.  ISR=%u\r\n", (unsigned)sw_isr_count);
    printf("Ready.  Open CH1/CH2 terminals, type to echo.\r\n\r\n");

    while (1)
    {
        /* ---- CH1 回显 ---- */
        if (SwUart1_QueryByte(&byte)) {
            sw1_rx_cnt++;
            SwUart1_SendByte(byte);
            sw1_tx_cnt++;
        }

        /* ---- CH2 回显 ---- */
        if (SwUart2_QueryByte(&byte)) {
            sw2_rx_cnt++;
            SwUart2_SendByte(byte);
            sw2_tx_cnt++;
        }

        /* ---- 每秒统计 ---- */
        if (g_sys_ms - last_print_ms >= 1000) {
            last_print_ms = g_sys_ms;
            printf("[1s] CH1 rx=%u tx=%u E=%u TO=%u | CH2 rx=%u tx=%u E=%u TO=%u | ISR=%u\r\n",
                   (unsigned)sw1_rx_cnt, (unsigned)sw1_tx_cnt,
                   (unsigned)sw1_edge_cnt, (unsigned)sw1_tx_timeout,
                   (unsigned)sw2_rx_cnt, (unsigned)sw2_tx_cnt,
                   (unsigned)sw2_edge_cnt, (unsigned)sw2_tx_timeout,
                   (unsigned)sw_isr_count);
        }
    }
}
#endif

/* ==================== 主入口 ==================== */
int main(void)
{
    System_Init();

#ifdef MOTOR_MODULE_TEST
    test_motor();
#elif defined(PCA9685_MODULE_TEST)
    test_pca9685();
#elif defined(DIRECT_SERVO_SWEEP)
    test_sweep();
#elif defined(SW_UART_ECHO_TEST)
    test_sw_uart_echo();
#else
    System_Run();
#endif
}
