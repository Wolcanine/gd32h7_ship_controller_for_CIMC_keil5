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
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 * 2026-06-03      CIMC            引脚重分配：解决摄像头/LCD/SDRAM 冲突
 * 2026-06-10      CIMC            TOF串口改为UART3 PA0/PA1；剔除模块测试代码
 ******************************************************************************/

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

/* 50Hz 控制周期标志 — TIMER3 ISR 置位，主循环消费 */
volatile uint8_t timer20ms_flag = 0;

/* 毫秒计数器 — SysTick ISR 递增，供 TOF 帧超时检测等使用 */
volatile uint32_t g_sys_ms = 0;

/*******************************************************************************
 * 函数名    fputc
 * 描述      printf 重定向到 UART4 (PB5 TX / PB13 RX, AF14, 115200 8N1)
 *           经跳线帽连板载 CH340 → USB 直连电脑串口助手
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

    /* ---- 电机 PWM 输出 ---- */
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

    /* ---- 摄像头识别板串口 (UART7 PC10/PC11) ---- */
    uart_init(UART_CAM, 115200);
    printf("Camera UART (UART7): OK\r\n");

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

        /* — 机械臂控制 — */
        ServoArm_RemoteControl();
        ServoArm_SmoothUpdate();

        /* — 传感器数据输出 (~10Hz, 每 5 帧打印一次) — */
        if (++print_cnt >= 5) {
            print_cnt = 0;
            int32_t gx = MPU6050_GetGyroX_dps100();
            int32_t gy = MPU6050_GetGyroY_dps100();
            int32_t gz = MPU6050_GetGyroZ_dps100();
            int32_t ax = MPU6050_GetAccelX_mg();
            int32_t ay = MPU6050_GetAccelY_mg();
            int32_t az = MPU6050_GetAccelZ_mg();
            uint16_t dist = Laser_GetDistanceCm();
            printf("PS2:%.2f/%.2f GYRO:%+04ld/%+04ld/%+04ld "
                   "ACCEL:%+05ld/%+05ld/%+05ld LASER:%4u MODE:0x%02X\r\n",
                   (double)ps2_get_throttle(), (double)ps2_get_steering(),
                   (long)gx, (long)gy, (long)gz,
                   (long)ax, (long)ay, (long)az,
                   (unsigned)dist, (unsigned)ps2_get_mode());
        }

#if 0   /* TODO: 传感器调试结束后启用摄像头指令处理 */
        uint8_t cam_cmd;
        while (uart_query_byte(UART_CAM, &cam_cmd)) {
            ServoArm_ProcessCmd(cam_cmd);
        }
#endif
    }
}
