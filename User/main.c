/*******************************************************************************
 * 文件名          main.c
 * 描述            水面垃圾清扫船主控程序
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-06-17      CIMC           代码整理：提取System_Init，测试模式独立函数
 * 2026-06-18      CIMC           串口机械臂角度指令: {a,b,c,d,e,f} → MoveToAngles
 * 2026-06-26      CIMC           新增 GYRO_PCA9685_TEST — 陀螺仪+PCA9685 组合测试
 ******************************************************************************/

/* ==================== 模块测试开关 ==================== */
/* 同时只启用一个。全部注释 = 正常全系统模式 */
//#define MOTOR_MODULE_TEST
//#define PCA9685_MODULE_TEST
//#define DIRECT_SERVO_SWEEP
//#define SW_UART_ECHO_TEST
////#define GPS_MODULE_TEST
#define STEPPER_MODULE_TEST
//#define SERVO_200HZ_TEST     /* ★ PCA9685 244Hz 舵机+电机测试 */
//#define ARM_TEACH_TEST
//#define GYRO_PCA9685_TEST
//#define UART_DBG_ECHO_TEST

#include "gd32h7xx.h"
#include "systick.h"
#include <stdio.h>
#include "uart_driver.h"
#include "main.h"
#include "ps2.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "pwm_output.h"
#include "motor_driver.h"
#include "ship_controller.h"
#include "auto_nav.h"
#include "laser.h"
#include "gd32_driver_pit.h"
#include "pca9685.h"
#include "servo_arm.h"
#include "step_motor.h"
#include "sw_uart.h"
#include "gps.h"

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
    uart_init(UART_DBG, 9600);
    systick_config();
    printf("\r\n===== Boat Control System Starting =====\r\n");
}

/* ==================== 正常全系统模式 ==================== */
#if !defined(MOTOR_MODULE_TEST) && !defined(PCA9685_MODULE_TEST) && \
    !defined(DIRECT_SERVO_SWEEP) && !defined(SW_UART_ECHO_TEST) && \
    !defined(GPS_MODULE_TEST) && !defined(STEPPER_MODULE_TEST) && \
    !defined(ARM_TEACH_TEST) && !defined(UART_DBG_ECHO_TEST) && \
    !defined(GYRO_PCA9685_TEST) && !defined(SERVO_200HZ_TEST)
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

    /* ---- 电机驱动 (GPIOC 软件 PWM / PCA9685 I2C, 编译时选择) ---- */
    motor_init();
    printf("[Motor] OK\r\n");

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

    /* ---- 软件串口 (两路 9600bps, TIMER1) ---- */
    SwUart_Init();
    printf("[SW UART] CH1+CH2 9600bps OK\r\n");

    /* ---- GPS 模块 (SW UART CH1) ---- */
    GPS_Init();

    /* ---- 50Hz 控制定时器 ---- */
    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz started\r\n");

    printf("===== System Ready =====\r\n");
    printf("  Green LED ON (analog)  = Manual\r\n");
    printf("  Green LED OFF (digital) = Auto (laser avoidance)\r\n\r\n");

    /* ==================== 主循环 ==================== */
    uint32_t last_loop_ms = 0;
    uint8_t  estop_flag   = 0;
    uint8_t  print_cnt    = 0;
    uint8_t  prev_mode    = 0xFF;   /* 上一次模式 (用于检测切换) */

    while (1) {
        /* 等待 50Hz 周期 (超时 500ms → 紧急停车) */
        while (!timer20ms_flag) {
            __WFI();
            if (!estop_flag && (g_sys_ms - last_loop_ms > 500)) {
                estop_flag = 1;
                motor_stop();
                Stepper_SetSpeed(0, 0);
                printf("\r\n*** EMERGENCY STOP: loop timeout! ***\r\n");
            }
        }
        timer20ms_flag = 0;
        estop_flag     = 0;
        last_loop_ms   = g_sys_ms;

        /* ---- 传感器 (所有模式共用) ---- */
        ps2_read_data();
        MPU6050_Update();
        uint16_t laser_cm = Laser_GetDistanceCm();
        GPS_Process();

        /* ---- PS2 断连安全 ---- */
        if (!ps2_is_connected()) {
            motor_set_left_duty(0.0f);
            motor_set_right_duty(0.0f);
            Stepper_SetSpeed(0, 0);
            static uint8_t disc_warn = 0;
            if (++disc_warn >= 250) {  /* 每 5 秒告警一次 */
                disc_warn = 0;
                printf("*** PS2 DISCONNECTED -- motors stopped ***\r\n");
            }
            goto arm_and_report;
        }

        /* ---- 自动航行状态机 (手动模式时内部直接 return) ---- */
        AutoNav_Process();

        /* ---- 模式切换检测 ---- */
        NavMode mode = AutoNav_GetMode();
        if (mode != prev_mode) {
            prev_mode = mode;
            /* 模式切换时重置控制器和状态机 */
            ShipController_Reset(&sc);
            motor_set_left_duty(0.0f);
            motor_set_right_duty(0.0f);
            if (mode == NAV_MODE_AUTO) {
                AutoNav_ResetState();
                printf("\r\n===== SWITCH TO AUTO MODE (laser avoidance) =====\r\n\r\n");
            } else {
                printf("\r\n===== SWITCH TO MANUAL MODE (PS2 control) =====\r\n\r\n");
            }
        }

        /* ---- 油门/转向 → PID → 电机 ---- */
        float throttle = AutoNav_GetThrottle();   /* 手动=PS2值, 自动=避障值 */
        float steering = AutoNav_GetSteering();
        ShipController_Update(&sc, throttle, steering);
        motor_throttle_tick();

        /* ---- 步进电机 (模式感知，解决 L1/L2 冲突) ---- */
        if (mode == NAV_MODE_AUTO) {
            /* 自动模式: L1/L2 直接控制步进 (超驰自动逻辑) */
            ServoArm_SetRemoteEnabled(0);         /* 禁用 arm 手动遥控 */
            if (PS2_Data.l1) {
                Stepper_SetSpeed(1, 4);
            } else if (PS2_Data.l2) {
                Stepper_SetSpeed(-1, 4);
            } else {
                /* 自动逻辑: 直行时开传送带, 转向时停 */
                if (AutoNav_GetState() == NAV_STATE_FORWARD)
                    Stepper_SetSpeed(1, 4);
                else
                    Stepper_SetSpeed(0, 0);
            }
        } else {
            /* 手动模式: SELECT + L1/L2 = 步进电机 (L1/L2 单独 = 腕旋转 CH4) */
            ServoArm_SetRemoteEnabled(1);         /* 允许 arm 手动遥控 */
            if (PS2_Data.select && PS2_Data.l1) {
                Stepper_SetSpeed(1, 4);
            } else if (PS2_Data.select && PS2_Data.l2) {
                Stepper_SetSpeed(-1, 4);
            } else {
                Stepper_SetSpeed(0, 0);
            }
        }

        /* ---- 机械臂 ---- */
    arm_and_report:
        ServoArm_RemoteControl();
        ServoArm_HandlePresets();
        ServoArm_ProcessSerialCommand();   /* 串口角度指令 {a,b,c,d,e,f} */
        ServoArm_SmoothUpdate();
        ServoArm_SequenceUpdate();          /* 采集序列自动推进 */

        /* PS2 R3 一键启动采集序列 */
        {
            static uint8_t prev_r3 = 0;
            if (PS2_Data.r3 && !prev_r3) {
                ServoArm_StartSequence();
            }
            prev_r3 = PS2_Data.r3;
        }

        /* ---- 1Hz 上报 (模式感知) ---- */
        if (++print_cnt >= 50) {
            print_cnt = 0;
            if (mode == NAV_MODE_AUTO) {
                printf("AUTO:%s THR=%.2f STR=%.2f LASER=%ucm YAW=%.1fdeg\r\n",
                       (AutoNav_GetState() == NAV_STATE_FORWARD) ? "FWD " : "TURN",
                       (double)AutoNav_GetThrottle(),
                       (double)AutoNav_GetSteering(),
                       (unsigned)laser_cm,
                       (double)AutoNav_GetYawDeg());
            } else {
                printf("MAN: PS2=%.2f/%.2f GYRO:%+04ld/%+04ld/%+04ld "
                       "ACCEL:%+05ld/%+05ld/%+05ld LASER:%4u\r\n",
                       (double)ps2_get_throttle(), (double)ps2_get_steering(),
                       (long)MPU6050_GetGyroX_dps100(), (long)MPU6050_GetGyroY_dps100(),
                       (long)MPU6050_GetGyroZ_dps100(),
                       (long)MPU6050_GetAccelX_mg(),   (long)MPU6050_GetAccelY_mg(),
                       (long)MPU6050_GetAccelZ_mg(),
                       (unsigned)laser_cm);
            }
            ServoArm_PrintStatus();
            GPS_PrintStatus();
            printf("STEP: %ld steps\r\n", (long)Stepper_GetCurStep());
        }
    }
}
#endif /* !MOTOR_MODULE_TEST && ... && !UART_DBG_ECHO_TEST */

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

/* ==================== 陀螺仪 + PCA9685 组合测试 ==================== */
#ifdef GYRO_PCA9685_TEST
static void test_gyro_pca9685(void)
{
    #define PCA_MIN  102
    #define PCA_MAX  512
    uint32_t last_print_ms = 0;
    int8_t   sweep_dir = 1;
    float    sweep_angle = 0.0f;
    uint16_t sweep_frame = 0;

    printf("\r\n===============================================\r\n");
    printf("  MPU6050 Gyro + PCA9685 Combined Test\r\n");
    printf("  I2C Bus: PE13(SCL) / PE15(SDA) shared\r\n");
    printf("===============================================\r\n\r\n");

    /* ---- 1. I2C 总线初始化 (PE13/PE15, 开漏) ---- */
    MyI2C_Init();
    printf("[I2C] PE13/PE15 OK\r\n");

    /* ---- 2. MPU6050 初始化 (WHO_AM_I + 校准) ---- */
    if (MPU6050_Init() != MPU6050_OK) {
        printf("[MPU6050] *** FAILED -- check wiring! ***\r\n");
        printf("  VCC->3.3V  GND->GND  SCL->PE13  SDA->PE15  AD0->GND\r\n");
        /* 不阻塞，继续测试 PCA9685 */
    } else {
        printf("[MPU6050] Init OK (ID=0x68, calibrated)\r\n");
    }

    /* ---- 3. PCA9685 初始化 (MODE2 推挽 + STOP更新) ---- */
    pca9685_init(PCA9685_I2C_ADDR);
    pca9685_set_pwm_freq(PCA9685_I2C_ADDR, PCA9685_SERVO_FREQ);

    printf("[PCA9685] 0x40 OK (50Hz, OE=L)\r\n");

    /* ---- 4. ch8~ch14 初始归零 ---- */
    for (int i = 0; i < 7; i++) {
        pca9685_set_pwm(PCA9685_I2C_ADDR, 8 + i, 0,
                        pca9685_angle_to_pulse(0.0f, PCA_MIN, PCA_MAX));
    }
    printf("[PCA9685] ch8~14 -> 0 deg\r\n");

    /* ---- 5. 启动 50Hz 定时器 ---- */
    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz started\r\n");

    printf("\r\n===== Combined Test Ready =====\r\n");
    printf("  PCA9685 ch8~14: slow sweep 0~180~0 deg\r\n");
    printf("  MPU6050: 1Hz serial report (Gyro/Accel/Temp)\r\n");
    printf("  Rotate sensor to verify gyro values change!\r\n");
    printf("  Send 'c' via serial to re-calibrate gyro\r\n\r\n");

    while (1)
    {
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;

        /* ---- 更新 MPU6050 数据 ---- */
        MPU6050_Update();

        /* ---- PCA9685 扫频 (ch8~ch14, 每 5 帧移动 1°) ---- */
        if (++sweep_frame >= 5) {
            sweep_frame = 0;

            /* 扫频方向反转 */
            sweep_angle += (float)sweep_dir * 1.0f;
            if (sweep_angle >= 180.0f) { sweep_angle = 180.0f; sweep_dir = -1; }
            if (sweep_angle <= 0.0f)   { sweep_angle = 0.0f;   sweep_dir =  1; }

            /* 更新 ch8~ch14 角度 */
            for (int i = 0; i < 7; i++) {
                uint16_t pulse = pca9685_angle_to_pulse(sweep_angle, PCA_MIN, PCA_MAX);
                pca9685_set_pwm(PCA9685_I2C_ADDR, 8 + i, 0, pulse);
            }
        }

        /* ---- 检查串口重新校准指令 ---- */
        {
            uint8_t ch;
            if (uart_query_byte(UART_DBG, &ch) && ch == 'c') {
                MPU6050_Recalibrate();
            }
        }

        /* ---- 1Hz 状态打印 ---- */
        if (g_sys_ms - last_print_ms >= 1000) {
            last_print_ms = g_sys_ms;
            printf("GYRO(dps): X=%+5ld.%02ld Y=%+5ld.%02ld Z=%+5ld.%02ld | "
                   "ACCEL(mg): X=%+5ld Y=%+5ld Z=%+5ld | "
                   "TEMP=%ld.%02ld C | "
                   "PCA_SWEEP=%.0f deg\r\n",
                   (long)(MPU6050_GetGyroX_dps100() / 100),
                   (long)((MPU6050_GetGyroX_dps100() >= 0
                        ? MPU6050_GetGyroX_dps100()
                        : -MPU6050_GetGyroX_dps100()) % 100),
                   (long)(MPU6050_GetGyroY_dps100() / 100),
                   (long)((MPU6050_GetGyroY_dps100() >= 0
                        ? MPU6050_GetGyroY_dps100()
                        : -MPU6050_GetGyroY_dps100()) % 100),
                   (long)(MPU6050_GetGyroZ_dps100() / 100),
                   (long)((MPU6050_GetGyroZ_dps100() >= 0
                        ? MPU6050_GetGyroZ_dps100()
                        : -MPU6050_GetGyroZ_dps100()) % 100),
                   (long)MPU6050_GetAccelX_mg(),
                   (long)MPU6050_GetAccelY_mg(),
                   (long)MPU6050_GetAccelZ_mg(),
                   (long)(MPU6050_GetTemp_c100() / 100),
                   (long)(MPU6050_GetTemp_c100() % 100),
                   (double)sweep_angle);
        }
    }
    #undef PCA_MIN
    #undef PCA_MAX
}
#endif /* GYRO_PCA9685_TEST */

/* ==================== PCA9685 200Hz 舵机测试 ==================== */
#ifdef SERVO_200HZ_TEST
static void test_servo_200hz(void)
{
    #define T_SERVO_MIN  500     /* 0.50ms @ 244Hz, 1μs/step */
    #define T_SERVO_MAX  2500    /* 2.50ms @ 244Hz, 1μs/step */

    /* 差速电机通道 (H 桥双引脚) */
    #define T_MOTOR_CH_LF  6     /* 左电机正转 */
    #define T_MOTOR_CH_LR  7     /* 左电机反转 */
    #define T_MOTOR_CH_RF  8     /* 右电机正转 */
    #define T_MOTOR_CH_RR  9     /* 右电机反转 */
    #define T_MOTOR_MAX    1024  /* 25% 占空比上限 (1024/4096) */

    /* 主驱动电机 (仅正转, 1~2ms 脉宽) */
    #define T_MAIN_CH       10    /* CH10 = 主驱动电机 */
    #define T_MAIN_MIN      1000  /* 1.00ms = 停止 */
    #define T_MAIN_MAX      2000  /* 2.00ms = 全速 */

    uint32_t last_print_ms = 0;

    printf("\r\n===============================================\r\n");
    printf("  PCA9685 244Hz Servo + Motor Test\r\n");
    printf("  I2C: PE13(SCL) / PE15(SDA)\r\n");
    printf("  Freq: 244Hz (1us/step), prescale=24\r\n");
    printf("===============================================\r\n\r\n");

    /* ---- 1. I2C + PCA9685 ---- */
    MyI2C_Init();
    printf("[I2C] PE13/PE15 OK\r\n");

    pca9685_init(PCA9685_I2C_ADDR);
    pca9685_set_pwm_freq(PCA9685_I2C_ADDR, 244.0f);

    printf("[PCA9685] @ 244Hz, OE=L\r\n");

    /* ---- 2. 舵机: CH0=0°, CH1=180° (参考) ---- */
    pca9685_set_pwm(PCA9685_I2C_ADDR, 0, 0, T_SERVO_MIN);
    pca9685_set_pwm(PCA9685_I2C_ADDR, 1, 0, T_SERVO_MAX);
    for (int i = 2; i < 6; i++)
        pca9685_set_pwm(PCA9685_I2C_ADDR, i, 0, T_SERVO_MIN);
    printf("[SERVO] CH0=0°  CH1=180°\r\n");

    /* ---- 3. 电机: CH6~CH9 初始滑行 ---- */
    pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LF, 0, 0);
    pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LR, 0, 0);
    pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RF, 0, 0);
    pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RR, 0, 0);
    printf("[MOTOR] CH6~9 -> coast\r\n");

    /* ---- 3b. 主驱动电机: CH10=1000us (停止) ---- */
    pca9685_set_pwm(PCA9685_I2C_ADDR, T_MAIN_CH, 0, T_MAIN_MIN);
    printf("[MAIN] CH10=%uus (stop)\r\n", (unsigned)T_MAIN_MIN);

    /* ---- 4. TOF200F 激光 (USART1 PA2/PD6) ---- */
    printf("[TOF] init USART1 PA2/PD6...\r\n");
    Laser_Init();
    printf("[TOF] USART1 PA2/PD6 @115200 OK\r\n");

    /* ---- 5. 启动 PS2 + 50Hz 定时器 ---- */
    ps2_init();
    printf("[PS2] OK\r\n");
    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz started\r\n");

    printf("\r\n===== Servo+Motor Test Ready =====\r\n");
    printf("  CH0=0° CH1=180° (servo reference)\r\n");
    printf("  CH6~9: motor auto-ramp test\r\n");
    printf("  PS2: UP=+throttle DOWN=-throttle X=brake\r\n\r\n");

    while (1)
    {
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;

        ps2_read_data();

        float throttle = ps2_get_throttle();
        float steering = ps2_get_steering();
        uint8_t brake = ps2_key_pressed(PSB_CROSS);

        if (brake) {
            /* ── 刹车: 所有电机短路制动 ── */
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LF, 4096, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LR, 4096, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RF, 4096, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RR, 4096, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MAIN_CH, 0, T_MAIN_MIN);

        } else if (throttle > 0.01f) {
            /* ── 前进: 主电机油门直驱 + 差速电机正转转向 ── */
            uint16_t main_pulse = T_MAIN_MIN +
                (uint16_t)(throttle * (float)(T_MAIN_MAX - T_MAIN_MIN));
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MAIN_CH, 0, main_pulse);

            float base = throttle;
            float diff = steering * throttle * 0.5f;  /* 转向分量 (50%强度) */
            float left  = base - diff;
            float right = base + diff;

            /* 钳位 0~1, 仅正转, 最小 1% 避免频繁启停 */
            if (left  < 0.0f)  left  = 0.0f;
            if (left  > 0.0f && left  < 0.01f) left  = 0.01f;
            if (left  > 1.0f)  left  = 1.0f;
            if (right < 0.0f)  right = 0.0f;
            if (right > 0.0f && right < 0.01f) right = 0.01f;
            if (right > 1.0f)  right = 1.0f;

            uint16_t dL = (uint16_t)(left  * (float)T_MOTOR_MAX);
            uint16_t dR = (uint16_t)(right * (float)T_MOTOR_MAX);

            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LF, 0, dL);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LR, 0, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RF, 0, dR);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RR, 0, 0);

        } else if (throttle < -0.01f) {
            /* ── 倒车: 主电机停转, 差速电机固定速度反转 ── */
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MAIN_CH, 0, T_MAIN_MIN);

            uint16_t d_rev = T_MOTOR_MAX / 3;  /* 固定 ~8% 反转 */
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LF, 0, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LR, 0, d_rev);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RF, 0, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RR, 0, d_rev);

        } else {
            /* ── 停止: 全部滑行 ── */
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MAIN_CH, 0, T_MAIN_MIN);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LF, 0, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_LR, 0, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RF, 0, 0);
            pca9685_set_pwm(PCA9685_I2C_ADDR, T_MOTOR_CH_RR, 0, 0);
        }

        /* TOF200F 激光测距 */
        uint16_t dist_cm = Laser_GetDistanceCm();

        /* 1Hz 上报 */
        if (g_sys_ms - last_print_ms >= 1000) {
            last_print_ms = g_sys_ms;
            uint16_t mp = T_MAIN_MIN;
            if (throttle > 0.0f)
                mp = T_MAIN_MIN + (uint16_t)(throttle * (float)(T_MAIN_MAX - T_MAIN_MIN));
            printf("[244Hz] THR=%.2f STR=%.2f | Main=%uuS | TOF=%ucm brake=%u\r\n",
                   (double)throttle, (double)steering,
                   (unsigned)mp, (unsigned)dist_cm, (unsigned)brake);
        }
    }
    #undef T_SERVO_MIN
    #undef T_SERVO_MAX
    #undef T_MOTOR_CH_LF
    #undef T_MOTOR_CH_LR
    #undef T_MOTOR_CH_RF
    #undef T_MOTOR_CH_RR
    #undef T_MOTOR_MAX
}
#endif /* SERVO_200HZ_TEST */

/* ==================== 机械臂示教测试 ==================== */
#ifdef ARM_TEACH_TEST
static void test_arm_teach(void)
{
    uint32_t last_print_ms = 0;

    printf("\r\n===== Arm Teach Mode =====\r\n");
    printf("Modules: PS2 + PCA9685 + 50Hz timer ONLY\r\n");

    ServoArm_SetSmoothSpeed(1);     /* 步进 1°/帧 */
    ServoArm_SetSmoothDivider(2);   /* 分频 2 → 25°/s */

    /* 仅初始化机械臂依赖的模块 */
    ps2_init();
    printf("[PS2] OK\r\n");

    ServoArm_Init();         /* 内部调用 MyI2C_Init + pca9685_init + 默认归位 */
    printf("[ServoArm] OK\r\n");

    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz started\r\n");

    printf("\r\n===== Ready =====\r\n");
    printf("LT/RT=Base  UP/DN=Shld  SQ/CI=Elbw  TRI/X=WrstP\r\n");
    printf("L1/L2=WrstR  R1/R2=Grip  START=Capture  SELECT=PARK\r\n");
    printf("R3=AutoSeq(6-step)  SERIAL: send { a,b,c,d,e,f } to move arm\r\n\r\n");

    while (1)
    {
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;

        /* 读取遥控器 */
        ps2_read_data();

        /* 机械臂控制 */
        ServoArm_RemoteControl();
        ServoArm_HandlePresets();
        ServoArm_ProcessSerialCommand();   /* 串口角度指令 {a,b,c,d,e,f} */
        ServoArm_SequenceUpdate();          /* 采集序列自动推进 (6阶段) */
        ServoArm_SmoothUpdate();

        /* PS2 R3 一键启动采集序列 */
        {
            static uint8_t prev_r3 = 0;
            if (PS2_Data.r3 && !prev_r3) {
                ServoArm_StartSequence();
            }
            prev_r3 = PS2_Data.r3;
        }

        /* 每秒打印状态 — 调试时取消注释 */
        //if (g_sys_ms - last_print_ms >= 1000) {
        //    last_print_ms = g_sys_ms;
        //    ServoArm_PrintStatus();
        //}
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

/* ==================== 步进电机传送带测试 ==================== */
#ifdef STEPPER_MODULE_TEST
static void test_stepper(void)
{
    uint32_t last_print_ms = 0;

    printf("\r\n===== Stepper Motor Test (Conveyor Belt) =====\r\n");
    printf("Pins: STEP=PG3 DIR=PD4 EN=PH6  TIMER6 @ 2kHz\r\n");

    ps2_init();
    printf("[PS2] OK\r\n");

    Stepper_Init();
    printf("[Stepper] OK\r\n");

    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz started\r\n");

    printf("PS2: L1=fwd(collect) L2=rev(release) idle=stop\r\n");
    printf("     speed_div=4 (500Hz step rate)\r\n\r\n");

    /* 上电即持续正转 (speed_div=1 → 2000Hz 最快) */
    Stepper_SetSpeed(1, 1);

    while (1)
    {
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;

        /* 每秒打印步数 */
        if (g_sys_ms - last_print_ms >= 1000) {
            last_print_ms = g_sys_ms;
            printf("STEP: %ld steps\r\n", (long)Stepper_GetCurStep());
        }
    }
}
#endif

/* ==================== GPS 模块测试 ==================== */
#ifdef GPS_MODULE_TEST
static void test_gps(void)
{
    uint32_t last_print_ms = 0;

    printf("\r\n===== GPS Module Test (ATGM336H-5N) =====\r\n");
    printf("SW UART CH1: PE2/PE5 @ 9600bps\r\n");

    SwUart_Init();
    printf("SwUart_Init OK, ISR=%u\r\n", (unsigned)sw_isr_count);

    GPS_Init();
    printf("Waiting for GPS fix (may take 30s~several min)...\r\n");
    printf("LED on GPS module blinks = fix OK.\r\n\r\n");

    while (1)
    {
        /* 处理 GPS 接收数据 */
        GPS_Process();

        /* 每秒打印状态 */
        if (g_sys_ms - last_print_ms >= 1000) {
            last_print_ms = g_sys_ms;
            GPS_PrintStatus();
            printf("  [SW ISR=%u E1=%u E2=%u]\r\n",
                   (unsigned)sw_isr_count,
                   (unsigned)sw1_edge_cnt, (unsigned)sw2_edge_cnt);
        }
    }
}
#endif

/* ==================== 调试串口回显测试 (UART_DBG RX验证) ==================== */
#ifdef UART_DBG_ECHO_TEST
static void test_uart_dbg_echo(void)
{
    uint32_t rx_cnt = 0, tx_cnt = 0;
    uint32_t last_print_ms = 0;
    uint8_t  byte;

    printf("\r\n===== UART_DBG (UART4) Echo Test =====\r\n");
    printf("Port: PB5-TX / PB13-RX @ 9600 -> CH340 -> USB\r\n");
    printf("Type anything, MCU prints RX char + hex.\r\n\r\n");

    while (1)
    {
        /* 收到一个字节 → 打印一行明确消息 */
        if (uart_query_byte(UART_DBG, &byte)) {
            rx_cnt++;
            printf("RX[%u]: '%c' (0x%02X)\r\n",
                   (unsigned)rx_cnt, (char)byte, (unsigned)byte);
            tx_cnt++;
        }

        /* 每秒心跳 */
        if (g_sys_ms - last_print_ms >= 1000) {
            last_print_ms = g_sys_ms;
            printf("[1s] alive rx=%u\r\n", (unsigned)rx_cnt);
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
    printf("CH1: PE2/PE5  CH2: PF7/PF6  9600\r\n");

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
#elif defined(SERVO_200HZ_TEST)
    test_servo_200hz();
#elif defined(ARM_TEACH_TEST)
    test_arm_teach();
#elif defined(UART_DBG_ECHO_TEST)
    test_uart_dbg_echo();
#elif defined(DIRECT_SERVO_SWEEP)
    test_sweep();
#elif defined(SW_UART_ECHO_TEST)
    test_sw_uart_echo();
#elif defined(GPS_MODULE_TEST)
    test_gps();
#elif defined(STEPPER_MODULE_TEST)
    test_stepper();
#elif defined(GYRO_PCA9685_TEST)
    test_gyro_pca9685();
#else
    System_Run();
#endif
}
