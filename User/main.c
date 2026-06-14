/*******************************************************************************
 * 文件名          main.c
 * 描述            水面垃圾清扫船主控程序
 *                 集成 PS2 遥控、MPU6050 陀螺仪、PWM 电机输出、激光测距、PID 闭环控制
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-15      AI助手          安全修复：PS2断连检测/MPU6050不死锁/看门狗
 * 2026-05-19      AI助手          机械臂标定+逆运动学
 * 2026-05-21      CIMC            GD32F407->GD32H759 移植
 * 2026-06-03      CIMC            引脚重分配：解决摄像头/LCD/SDRAM 冲突
 * 2026-06-10      CIMC            TOF串口改为UART3 PA0/PA1；剔除模块测试代码
 * 2026-06-11      CIMC            新增 PCA9685_MODULE_TEST + MOTOR_MODULE_TEST
 * 2026-06-12      CIMC            双MCU->单H7：UART_CAM->Vision 同板接口，预留视觉API
 ******************************************************************************/

/* ==================== 模块测试开关 ==================== */
/* 同时只启用一个测试宏。注释掉所有测试宏 = 正常全系统模式 */
#define MOTOR_MODULE_TEST       /* 四电机 PWM + PS2 遥控独立测试 (限幅20%, 同侧并联) */
//#define PCA9685_MODULE_TEST    /* 舵机 PCA9685 + PS2 遥控独立测试 */

#include "gd32h7xx.h"
#include "systick.h"
#include <stdio.h>
#include "uart_driver.h"
#include "main.h"
#include "gd32h759i_eval.h"
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
#include "oled.h"
#include "vision.h"

/* 50Hz 控制周期标志 — TIMER3 ISR 置位，主循环消费 */
volatile uint8_t timer20ms_flag = 0;

/* 毫秒计数器 — SysTick ISR 递增，供 TOF 帧超时检测等使用 */
volatile uint32_t g_sys_ms = 0;

/*******************************************************************************
 * 函数名    fputc
 * 描述      printf 重定向到 UART4 (PB5 TX / PB13 RX, AF14, 115200 8N1)
 *           经跳线帽连板载 CH340 -> USB 直连电脑串口助手
 *           配合 Keil MicroLIB: 勾选 "Use MicroLIB" 即可
 * 参数      ch    待发送字符
 * 参数      f     文件指针 (MicroLIB 忽略)
 * 返回值    发送的字符
 ******************************************************************************/
int fputc(int ch, FILE *f)
{
    while (RESET == usart_flag_get(UART4, USART_FLAG_TBE));
    usart_data_transmit(UART4, (uint8_t)ch);
    return ch;
}

/*******************************************************************************
 * 函数名    main
 * 描述      系统主入口：依次初始化各外设模块，进入 50Hz 主循环
 *             主循环内容：
 *               1. 等待 TIMER3 50Hz 周期（带看门狗超时保护）
 *               2. 读取 PS2、MPU6050、TOF 激光测距
 *               3. PS2 遥控调节机械臂舵机
 *               4. 机械臂缓冲插值
 *               5. 传感器数据周期性打印
 * 参数      none
 * 返回值    none
 ******************************************************************************/
#ifdef MOTOR_MODULE_TEST
/* ==================== 四电机 PWM 独立测试模式 ==================== */
int main(void)
{
    float throttle  = 0.0f;   /* 油门 [0.00, 0.20] */
    float diff      = 0.0f;   /* 差速 [-0.20, 0.20] */
    float left_duty, right_duty;  /* 左侧双电机组 / 右侧双电机组 */

    /* 按键上升沿检测：前次状态 */
    uint8_t p_up = 0, p_down = 0, p_left = 0, p_right = 0;
    uint8_t p_tri = 0, p_cross = 0, p_square = 0, p_circle = 0;
    uint8_t changed = 0;

    /* ---- 启用 CPU Cache ---- */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* ---- 板级外设 ---- */
    gd_eval_led_init(LED1);
    uart_init(UART_DBG, 115200);
    systick_config();

    printf("\r\n");
    printf("===============================================\r\n");
    printf("  4-Motor PWM Test -- PS2 Remote Control\r\n");
    printf("  Left  (x2): D0=PC2 D1=PC3  (J4-32/33) -> Board A\r\n");
    printf("  Right (x2): D2=PC5 D3=PC10 (J4-34/51) -> Board B\r\n");
    printf("  Board: 2x dual H-bridge, inputs paralleled per side\r\n");
    printf("  All 4 motors at stern, paired left/right\r\n");
    printf("  Limit: ±20%% (0.20)\r\n");
    printf("===============================================\r\n\r\n");

    /* ---- PS2 遥控器 ---- */
    ps2_init();
    printf("[PS2] DI=PA5 DO=PA7 CS=PB12 CLK=PB10\r\n");

    /* ---- 电机 PWM（上电滑行，4路全低 -> 四电机断电） ---- */
    pwm_output_init();
    printf("[PWM] 4-motor outputs initialized -> COAST (all LOW)\r\n");

    /* ---- 50Hz 控制定时器 ---- */
    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz control timer started\r\n");

    printf("\r\n");
    printf("-----------------------------------------------\r\n");
    printf("  Controls (edge-triggered, +-0.01 per press):\r\n");
    printf("  UP         throttle +1%%\r\n");
    printf("  DN         throttle -1%%\r\n");
    printf("  LT         steer left  (left -, right +)\r\n");
    printf("  RT         steer right (left +, right -)\r\n");
    printf("  TR         EMERGENCY STOP (throttle=0 steer=0)\r\n");
    printf("  X         zero steer\r\n");
    printf("  SQ         throttle +0.05 (jump)\r\n");
    printf("  CI         throttle -0.05 (jump)\r\n");
    printf("-----------------------------------------------\r\n");
    printf("  Throttle         [0.00 ~ 0.20]  forward only\r\n");
    printf("  Steer            [-0.20 ~ 0.20] differential\r\n");
    printf("  Left  (x2)  = clamp(throttle + steer, 0, 0.20)\r\n");
    printf("  Right (x2)  = clamp(throttle - steer, 0, 0.20)\r\n");
    printf("  LED1 (PF10) = heartbeat 2.5Hz\r\n");
    printf("===============================================\r\n\r\n");

    printf("MOTOR: T=0.00 D=0.00 | L(x2)=0.00 R(x2)=0.00 [COAST]\r\n");

    uint8_t led_cnt = 0;

    while (1) {
        /* 等待 50Hz 周期 */
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;

        /* 读取 PS2 */
        ps2_read_data();

        changed = 0;

        /* — 油门 — */
        if (PS2_Data.up    && !p_up)    { throttle += 0.01f; changed = 1; }
        if (PS2_Data.down  && !p_down)  { throttle -= 0.01f; changed = 1; }
        if (PS2_Data.square && !p_square) { throttle += 0.05f; changed = 1; }
        if (PS2_Data.circle && !p_circle) { throttle -= 0.05f; changed = 1; }

        /* — 差速 — */
        if (PS2_Data.left  && !p_left)  { diff -= 0.01f; changed = 1; }
        if (PS2_Data.right && !p_right) { diff += 0.01f; changed = 1; }
        if (PS2_Data.cross && !p_cross) { diff = 0.0f;  changed = 1; }

        /* — 急停 — */
        if (PS2_Data.triangle && !p_tri) {
            throttle = 0.0f; diff = 0.0f; changed = 1;
        }

        /* — 钳位 — */
        if (throttle > 0.20f) throttle = 0.20f;
        if (throttle < 0.00f) throttle = 0.00f;
        if (diff > 0.20f)     diff = 0.20f;
        if (diff < -0.20f)    diff = -0.20f;

        /* 计算左右双电机组 duty */
        left_duty  = throttle + diff;
        right_duty = throttle - diff;
        if (left_duty  > 0.20f) left_duty  = 0.20f;
        if (left_duty  < 0.00f) left_duty  = 0.00f;
        if (right_duty > 0.20f) right_duty = 0.20f;
        if (right_duty < 0.00f) right_duty = 0.00f;

        /* 输出到四电机 */
        pwm_set_left_duty(left_duty);
        pwm_set_right_duty(right_duty);

        /* 保存按键状态 */
        p_up = PS2_Data.up;       p_down  = PS2_Data.down;
        p_left = PS2_Data.left;   p_right = PS2_Data.right;
        p_tri = PS2_Data.triangle; p_cross = PS2_Data.cross;
        p_square = PS2_Data.square; p_circle = PS2_Data.circle;

        /* 有变化时打印 */
        if (changed) {
            printf("MOTOR: T=%+0.2f D=%+0.2f | L(x2)=%+0.2f R(x2)=%+0.2f\r\n",
                   throttle, diff, left_duty, right_duty);
        }

        /* 心跳 LED */
        if (++led_cnt >= 20) {
            led_cnt = 0;
            gd_eval_led_toggle(LED1);
        }
    }
}

#elif defined(PCA9685_MODULE_TEST)
/* ==================== PCA9685 舵机独立测试模式 ==================== */
int main(void)
{
    /* ---- 启用 CPU Cache ---- */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* ---- 板级外设初始化 ---- */
    gd_eval_led_init(LED1);
    uart_init(UART_DBG, 115200);
    systick_config();

    printf("\r\n");
    printf("===============================================\r\n");
    printf("  PCA9685 Servo Test — PS2 Remote Control\r\n");
    printf("  I2C: PE13(SCL) / PE15(SDA)\r\n");
    printf("  PCA9685: 0x40, 50Hz, 6-channel servo\r\n");
    printf("===============================================\r\n\r\n");

    /* ---- I2C 总线初始化 (PE13/PE15) ---- */
    MyI2C_Init();
    printf("[I2C] PE13(SCL) PE15(SDA) init OK\r\n");

    /* ---- PCA9685 初始化 ---- */
    pca9685_init(PCA9685_I2C_ADDR);
    pca9685_set_pwm_freq(PCA9685_I2C_ADDR, PCA9685_SERVO_FREQ);
    printf("[PCA9685] PWM freq = %d Hz\r\n", (int)PCA9685_SERVO_FREQ);

    /* ---- 所有舵机归中 (90°) ---- */
    {
        uint8_t i;
        for (i = 0; i < ARM_JOINT_COUNT; i++) {
            ServoArm_SetAngle(i, 90.0f);
        }
        printf("[Servo] all 6 joints -> 90° (center)\r\n");
    }

    /* ---- PS2 遥控器 ---- */
    ps2_init();
    printf("[PS2] DI=PA5 DO=PA7 CS=PB12 CLK=PB10\r\n");

    /* ---- 50Hz 控制定时器 ---- */
    pit_ms_init(PIT_TIMER3, 20);
    printf("[TIMER3] 50Hz control timer started\r\n");

    printf("\r\n");
    printf("-----------------------------------------------\r\n");
    printf("  PS2 Remote Servo Control Mapping:\r\n");
    printf("  LT / RT    Base rotate     (ch0)\r\n");
    printf("  UP / DN    Shoulder        (ch1)\r\n");
    printf("  SQ / CI    Elbow           (ch2)\r\n");
    printf("  TR / X    Wrist pitch     (ch3)\r\n");
    printf("  L1 / L2  Wrist rotate    (ch4)\r\n");
    printf("  R1 / R2  Gripper         (ch5)\r\n");
    printf("-----------------------------------------------\r\n");
    printf("  LED1 (PF10) = heartbeat (2.5Hz)\r\n");
    printf("  Serial output = angle changes only\r\n");
    printf("===============================================\r\n\r\n");

    /* ---- 主循环 ---- */
    uint8_t led_cnt = 0;

    while (1) {
        /* 等待 50Hz 周期 */
        while (!timer20ms_flag) __WFI();
        timer20ms_flag = 0;

        /* 读取 PS2 遥控器 */
        ps2_read_data();

        /* PS2 遥控调节舵机角度 */
        ServoArm_RemoteControl();

        /* 心跳 LED (~2.5Hz = 50Hz / 20) */
        if (++led_cnt >= 20) {
            led_cnt = 0;
            gd_eval_led_toggle(LED1);
        }
    }
}

#else  /* 正常全系统模式 (无测试宏定义) */
/* ==================== 正常全系统模式 ==================== */
int main(void)
{
    /* ---- 启用 CPU Cache — SystemInit() 关了 D-Cache，必须重新打开 ---- */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* ---- 板级外设初始化 ---- */
    gd_eval_led_init(LED1);
    uart_init(UART_DBG, 115200);
    systick_config();

    printf("\r\n===== Boat Control System Starting =====\r\n");

    /* ---- PS2 遥控器 ---- */
    ps2_init();
    printf("PS2 remote: OK\r\n");

    /* ---- MPU6050 六轴传感器（内部调用 MyI2C_Init；失败不阻塞） ---- */
    if (MPU6050_Init() != MPU6050_OK) {
        printf("MPU6050 init FAILED! Continuing without gyro (yaw hold & auto-nav disabled)\r\n");
    } else {
        printf("MPU6050: OK\r\n");
    }

    /* ---- TOF200F 激光测距 (UART3 PA0/PA1, 内部完成握手+高精度配置) ---- */
    Laser_Init();
    printf("TOF laser: OK\r\n");

    /* ---- 四电机 PWM 输出 ---- */
    pwm_output_init();
    printf("PWM output: OK\r\n");

    /* ---- 船舶控制器 ---- */
    ShipController sc;
    ShipController_Init(&sc,
        0.5f,     /* yaw_kp               */
        0.01f,    /* yaw_ki               */
        0.1f,     /* yaw_kd               */
        10.0f,    /* yaw_integral_limit   */
        90.0f,    /* max_yaw_rate_cmd     */
        0.5f      /* max_diff_duty        */
    );
    printf("Ship controller: OK\r\n");

    /* ---- 自动航行（默认手动模式） ---- */
    AutoNav_Init();
    printf("AutoNav: OK (manual mode)\r\n");

    /* ---- 50Hz 控制定时器 (TIMER3) ---- */
    pit_ms_init(PIT_TIMER3, 20);
    printf("Control timer (50Hz): OK\r\n");

    /* ---- 视觉识别模块 (同板集成，替代原 UART_CAM 双MCU方案) ---- */
    Vision_Init();
    printf("Vision: OK (placeholder, awaiting camera interface)\r\n");

    /* ---- 机械臂舵机 (PCA9685 I2C PWM) ---- */
    ServoArm_Init();
    printf("Servo arm: OK\r\n");

    /* ---- OLED 显示屏 (SSD1306, PD12/PD13 软件 I2C) ---- */
    OLED_Init();
    OLED_ShowString(0, 0, (uint8_t *)"Boat Control", 16);
    OLED_ShowString(0, 16, (uint8_t *)"System Ready", 16);
    OLED_Refresh();
    printf("OLED: OK\r\n");

    printf("===== System Ready =====\r\n\r\n");

    delay_1ms(500);

    /* ---- 主循环变量 ---- */
    uint32_t last_loop_ms = 0;
    uint8_t  emergency_stopped = 0;
    uint8_t  print_cnt = 0;

    while (1) {
        /* — 看门狗：等待 50Hz 周期（带超时检测） — */
        while (!timer20ms_flag) {
            __WFI();
            if (!emergency_stopped && (g_sys_ms - last_loop_ms > 500)) {
                emergency_stopped = 1;
                pwm_set_left_duty(0.0f);
                pwm_set_right_duty(0.0f);
                printf("\r\n*** EMERGENCY STOP: control loop timeout! ***\r\n");
            }
        }
        timer20ms_flag = 0;
        emergency_stopped = 0;
        last_loop_ms = g_sys_ms;

        /* — 传感器数据读取 — */
        ps2_read_data();
        MPU6050_Update();                           /* 失败不阻塞，仅跳过本次数据 */
        Laser_GetDistanceCm();                      /* 内部完成 drain + 周期查询 */

        /* — 视觉识别更新 — */
        Vision_Update();                            /* 每 N 帧触发一次检测，其余帧跳过 */

        /* — 机械臂控制 — */
        ServoArm_RemoteControl();
        ServoArm_SmoothUpdate();

        /* — 传感器数据输出 (~1Hz, 每 50 帧打印一次) — */
        if (++print_cnt >= 50) {
            print_cnt = 0;
            int32_t gx = MPU6050_GetGyroX_dps100();
            int32_t gy = MPU6050_GetGyroY_dps100();
            int32_t gz = MPU6050_GetGyroZ_dps100();
            int32_t ax = MPU6050_GetAccelX_mg();
            int32_t ay = MPU6050_GetAccelY_mg();
            int32_t az = MPU6050_GetAccelZ_mg();
            uint16_t dist = Laser_GetDistanceCm();
            uint8_t  vis_cnt = 0;
            const VisionFrame *vf = Vision_GetFrame();
            if (vf) vis_cnt = vf->count;
            printf("PS2:%.2f/%.2f GYRO:%+04ld/%+04ld/%+04ld "
                   "ACCEL:%+05ld/%+05ld/%+05ld LASER:%4u VIS:%u MODE:0x%02X\r\n",
                   (double)ps2_get_throttle(), (double)ps2_get_steering(),
                   (long)gx, (long)gy, (long)gz,
                   (long)ax, (long)ay, (long)az,
                   (unsigned)dist, (unsigned)vis_cnt, (unsigned)ps2_get_mode());
        }

        /* — 视觉引导抓取 (自动模式 + 检测到目标时触发) — */
        if (Vision_HasTarget() && ps2_get_mode() == PS2_MODE_DIGITAL) {
            VisionTarget t = Vision_GetClosestTarget();
            if (t.confidence > 50) {
                ServoArm_CollectTarget(&t);
            }
        }
    }
}
#endif /* MOTOR_MODULE_TEST / PCA9685_MODULE_TEST / normal */
