/*******************************************************************************
 * 文件名          auto_nav.c
 * 描述            自动航行避障模块 — 激光测距 + 陀螺仪角度闭环
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-07      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

// auto_nav.c — 自动航行模块（避障实现）
// =============================================
// 自动 / 手动由 PS2 手柄 mode 变量决定：
//   PS2_MODE_ANALOG (0x73, 红灯亮) = 手动
//   PS2_MODE_DIGITAL(0x41, 红灯灭) = 自动
//
// 自动模式：激光测距避障
//   直行 → 遇障 → 陀螺仪角度闭环转向 → 确认安全 → 恢复直行
//   转向方向交替切换，避免原地绕圈
// =============================================

#include "auto_nav.h"
#include "ps2.h"
#include "laser.h"
#include "MPU6050.h"
#include <math.h>
#include <stdio.h>

// ==================== 可调参数 ====================
#define NAV_SAFE_DIST_CM    200         // 安全距离 (cm)，≤ 此值视为有障碍
#define NAV_TURN_ANGLE_DEG  45.0f       // 每次转向的目标角度 (deg)
#define NAV_FORWARD_THR     0.8f        // 直行油门
#define NAV_TURN_THR        0.4f        // 转向时油门
#define NAV_CONFIRM_CYCLES  3           // 确认障碍的连续周期数

// ==================== 内部状态 ====================
static NavMode    current_mode  = NAV_MODE_MANUAL;
static NavState   nav_state     = NAV_STATE_FORWARD;
static uint8_t    obstacle_cnt  = 0;        // 连续检测到障碍的周期计数
static uint8_t    turn_dir_flag = 0;        // 0=左转, 1=右转 (交替)
static float      nav_yaw       = 0.0f;     // 累计偏航角 (deg)，由陀螺仪积分
static float      turn_start_yaw = 0.0f;    // 进入转向时的偏航角基准

// ==================== 自动模式输出值 ====================
static float auto_throttle = 0.0f;
static float auto_steering = 0.0f;

// ==================== 初始化 ====================
void AutoNav_Init(void)
{
    current_mode    = NAV_MODE_MANUAL;
    nav_state       = NAV_STATE_FORWARD;
    obstacle_cnt    = 0;
    turn_dir_flag   = 0;
    nav_yaw         = 0.0f;
    turn_start_yaw  = 0.0f;
    auto_throttle   = 0.0f;
    auto_steering   = 0.0f;

    Laser_Init();
}

// ==================== 陀螺仪偏航角积分（每周期 20ms 调用一次） ====================
static void integrate_yaw(void)
{
    // 陀螺仪失效时不积分（偏航保持上次值，自动转向降级为盲转）
    if (!MPU6050_IsHealthy()) return;

    // MPU6050_GetGyroZ_dps100() 返回 0.01°/s 单位的有符号值
    float rate_dps = (float)MPU6050_GetGyroZ_dps100() / 100.0f;
    nav_yaw += rate_dps * 0.02f;    // dt = 20ms = 0.02s
}

// ==================== 主处理函数 ====================
void AutoNav_Process(void)
{
    // ---- 1. 根据 PS2 模式切换手动/自动 ----
    current_mode = (ps2_get_mode() == PS2_MODE_ANALOG) ? NAV_MODE_MANUAL : NAV_MODE_AUTO;

    // 手动模式不执行任何自动航行逻辑
    if (current_mode == NAV_MODE_MANUAL)
        return;

    // ---- 2. 偏航角积分（在自动模式下始终运行） ----
    integrate_yaw();

    // ---- 3. 读取激光测距 ----
    uint16_t dist_cm = Laser_GetDistanceCm();

    // ---- 4. 避障状态机 ----
    switch (nav_state) {

    // ============================
    // 状态 A：直行
    // ============================
    case NAV_STATE_FORWARD:
        auto_throttle = NAV_FORWARD_THR;
        auto_steering = 0.0f;

        // 重置偏航积分（直行段漂移不会累积到下次转向）
        nav_yaw = 0.0f;

        // 激光读数有效且小于安全距离 → 计数
        if (dist_cm < NAV_SAFE_DIST_CM && dist_cm < 5000) {
            obstacle_cnt++;
            if (obstacle_cnt >= NAV_CONFIRM_CYCLES) {
                // 确认有障碍 → 进入转向
                nav_state      = NAV_STATE_TURNING;
                turn_start_yaw = 0.0f;      // 刚重置过 nav_yaw=0
                turn_dir_flag  = !turn_dir_flag;  // 交替方向
                printf("NAV: obstacle %dcm, turn %s\r\n",
                       (int)dist_cm,
                       turn_dir_flag ? "RIGHT" : "LEFT");
            }
        } else {
            obstacle_cnt = 0;
        }
        break;

    // ============================
    // 状态 B：转向避障（角度闭环）
    // ============================
    case NAV_STATE_TURNING:
        // 转向输出
        auto_throttle = NAV_TURN_THR;
        auto_steering = turn_dir_flag ? 1.0f : -1.0f;

        // 检查是否转够了目标角度
        float yaw_delta = (float)fabs(nav_yaw - turn_start_yaw);
        if (yaw_delta >= NAV_TURN_ANGLE_DEG) {
            // 转到位了 → 检查障碍是否已避开
            uint16_t check_dist = Laser_GetDistanceCm();
            if (check_dist >= NAV_SAFE_DIST_CM || check_dist >= 5000) {
                // 安全 → 恢复直行
                nav_state = NAV_STATE_FORWARD;
                obstacle_cnt = 0;
                printf("NAV: clear (%dcm), resume forward\r\n",
                       (int)check_dist);
            } else {
                // 障碍仍在 → 继续同方向转一个角度
                turn_start_yaw = nav_yaw;
                printf("NAV: still blocked (%dcm), turn again\r\n",
                       (int)check_dist);
            }
        }
        break;
    }
}

// ==================== 查询函数 ====================
NavMode AutoNav_GetMode(void)
{
    return current_mode;
}

NavState AutoNav_GetState(void)
{
    return nav_state;
}

float AutoNav_GetYawDeg(void)
{
    return nav_yaw;
}

float AutoNav_GetThrottle(void)
{
    if (current_mode == NAV_MODE_MANUAL)
        return ps2_get_throttle();
    else
        return auto_throttle;
}

float AutoNav_GetSteering(void)
{
    if (current_mode == NAV_MODE_MANUAL)
        return ps2_get_steering();
    else
        return auto_steering;
}
