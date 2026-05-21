/*******************************************************************************
 * 文件名          servo_arm.c
 * 描述            6 舵机机械臂控制 — PCA9685 驱动 + PS2 遥控 + 逆运动学
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-08      AI助手          初始版本
 * 2026-05-19      AI助手          脉宽标定 + 逆运动学缓冲移动
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

#include "servo_arm.h"
#include "pca9685.h"
#include "ps2.h"
#include "arm_ik.h"
#include <stdio.h>

/* ==================== 舵机参数 ==================== */
#define PULSE_0     102     /* 0° 脉宽计数值 (0.5ms) */
#define PULSE_FULL  512     /* 180° 脉宽计数值 (2.5ms) */
#define PULSE_RNG   (PULSE_FULL - PULSE_0)

/* 各关节对正脉宽（经实测标定）*/
static const uint16_t joint_center_pulse[5] = {325, 334, 323, 337, 328};
#define GRIPPER_CLOSE_PULSE  382
#define GRIPPER_OPEN_PULSE   229

/* ==================== 预设动作角度表 ==================== */
/*           [BASE] [SHOULDER] [ELBOW] [WRIST_P] [WRIST_R] [GRIPPER] */
/* PARK      90       0        180      90        90         0     */
/* READY     90      90         90      90        90         0     */
/* COLLECT   90      135        45      90        90       180     */
/* DROP     180      90         90      90        90         0     */

#define ANGLE_PARK      { 90,   0,   180,  90,  90,  0   }
#define ANGLE_READY     { 90,   90,  90,   90,  90,  0   }
#define ANGLE_COLLECT   { 90,   135, 45,   90,  90,  180 }
#define ANGLE_DROP      { 180,  90,  90,   90,  90,  0   }

static const uint16_t angle_table[4][6] = {
    [ARM_PARK]    = ANGLE_PARK,
    [ARM_READY]   = ANGLE_READY,
    [ARM_COLLECT] = ANGLE_COLLECT,
    [ARM_DROP]    = ANGLE_DROP,
};

#define PCA9685_ADDR  PCA9685_I2C_ADDR

static uint16_t arm_target[ARM_JOINT_COUNT];
static uint8_t  smooth_active = 0;
#define SMOOTH_SPEED_DEG  2   /* 每帧步进 (°)，50Hz → 100°/s */

/*******************************************************************************
 * 函数名    ServoArm_Init
 * 描述      初始化 PCA9685，所有关节恢复 90° 对正位
 ******************************************************************************/
void ServoArm_Init(void)
{
    pca9685_init(PCA9685_ADDR);
    pca9685_set_pwm_freq(PCA9685_ADDR, PCA9685_SERVO_FREQ);

    uint8_t i;
    smooth_active = 0;
    for (i = 0; i < ARM_JOINT_COUNT; i++) {
        arm_target[i] = 90;
        ServoArm_SetAngle(i, 90.0f);
    }
    printf("ServoArm: initialized, center\r\n");
}

/*******************************************************************************
 * 函数名    ServoArm_SetAngle
 * 描述      设置单个舵机角度（含对正脉宽校准）
 * 参数      ch      通道号 (0~5)
 *           angle   角度 (0~180°)
 * 公式      脉宽 = 对正脉宽 + (角度 - 90°) × 410/180°，钳位 [102, 512]
 ******************************************************************************/
void ServoArm_SetAngle(uint8_t ch, float angle)
{
    if (ch >= ARM_JOINT_COUNT) return;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f)  angle = 180.0f;

    uint16_t pulse;
    if (ch == ARM_CH_GRIPPER) {
        pulse = GRIPPER_OPEN_PULSE + (uint16_t)(angle * (GRIPPER_CLOSE_PULSE - GRIPPER_OPEN_PULSE) / 180.0f);
    } else {
        int32_t p = (int32_t)joint_center_pulse[ch]
                  + (int32_t)((angle - 90.0f) * PULSE_RNG / 180.0f);
        if (p < PULSE_0)   p = PULSE_0;
        if (p > PULSE_FULL) p = PULSE_FULL;
        pulse = (uint16_t)p;
    }
    pca9685_set_pwm(PCA9685_ADDR, ch, 0, pulse);
}

/*******************************************************************************
 * 函数名    ServoArm_SetAction
 * 描述      执行预设动作（走缓冲插值）
 * 参数      action    动作枚举 (PARK/READY/COLLECT/DROP)
 ******************************************************************************/
void ServoArm_SetAction(ArmAction action)
{
    uint8_t i;
    if (action > ARM_DROP) return;

    for (i = 0; i < ARM_JOINT_COUNT; i++) {
        arm_target[i] = angle_table[action][i];
    }
    smooth_active = 1;
    printf("Arm: action %d\r\n", (int)action);
}

/*******************************************************************************
 * 函数名    ServoArm_ProcessCmd
 * 描述      解析摄像头 1 字节指令执行预设动作
 * 参数      cmd    bit7=使能, bit[2:0]=动作类型
 ******************************************************************************/
void ServoArm_ProcessCmd(uint8_t cmd)
{
    if (!(cmd & CAM_CMD_ENABLE)) return;

    ArmAction action = (ArmAction)(cmd & CAM_CMD_ACTION);
    if (action <= ARM_DROP) {
        ServoArm_SetAction(action);
    }
}

/*******************************************************************************
 * 函数名    ServoArm_RemoteControl
 * 描述      PS2 遥控独立调关节（连续动作模式，~50°/s）
 *           ←/→ 底座  ↑/↓ 大臂  □/○ 小臂
 *           △/× 腕俯仰  L1/L2 腕旋转  R1/R2 夹爪
 ******************************************************************************/
static uint16_t arm_angle[ARM_JOINT_COUNT] = {90, 90, 90, 90, 90, 90};

void ServoArm_RemoteControl(void)
{
    uint16_t prev[6];
    uint8_t changed = 0;
    static uint8_t print_cnt = 0;

    if (PS2_Data.left || PS2_Data.right || PS2_Data.up || PS2_Data.down ||
        PS2_Data.square || PS2_Data.circle || PS2_Data.triangle || PS2_Data.cross ||
        PS2_Data.l1 || PS2_Data.l2 || PS2_Data.r1 || PS2_Data.r2) {
        smooth_active = 0;
    }

    prev[0] = arm_angle[0];
    if (PS2_Data.left)  { if (arm_angle[0] > 0)   arm_angle[0]--; }
    if (PS2_Data.right) { if (arm_angle[0] < 180)  arm_angle[0]++; }
    if (arm_angle[0] != prev[0]) { ServoArm_SetAngle(0, (float)arm_angle[0]); changed = 1; }

    prev[1] = arm_angle[1];
    if (PS2_Data.up)    { if (arm_angle[1] < 180)  arm_angle[1]++; }
    if (PS2_Data.down)  { if (arm_angle[1] > 0)    arm_angle[1]--; }
    if (arm_angle[1] != prev[1]) { ServoArm_SetAngle(1, (float)arm_angle[1]); changed = 1; }

    prev[2] = arm_angle[2];
    if (PS2_Data.square){ if (arm_angle[2] > 0)    arm_angle[2]--; }
    if (PS2_Data.circle){ if (arm_angle[2] < 180)  arm_angle[2]++; }
    if (arm_angle[2] != prev[2]) { ServoArm_SetAngle(2, (float)arm_angle[2]); changed = 1; }

    prev[3] = arm_angle[3];
    if (PS2_Data.triangle){ if (arm_angle[3] > 0)  arm_angle[3]--; }
    if (PS2_Data.cross) { if (arm_angle[3] < 180)  arm_angle[3]++; }
    if (arm_angle[3] != prev[3]) { ServoArm_SetAngle(3, (float)arm_angle[3]); changed = 1; }

    prev[4] = arm_angle[4];
    if (PS2_Data.l1)    { if (arm_angle[4] > 0)    arm_angle[4]--; }
    if (PS2_Data.l2)    { if (arm_angle[4] < 180)  arm_angle[4]++; }
    if (arm_angle[4] != prev[4]) { ServoArm_SetAngle(4, (float)arm_angle[4]); changed = 1; }

    prev[5] = arm_angle[5];
    if (PS2_Data.r1)    { if (arm_angle[5] > 0)    arm_angle[5]--; }
    if (PS2_Data.r2)    { if (arm_angle[5] < 180)  arm_angle[5]++; }
    if (arm_angle[5] != prev[5]) { ServoArm_SetAngle(5, (float)arm_angle[5]); changed = 1; }

    if (changed) {
        if (++print_cnt % 5 == 1) {
            printf("Arm: B%03u S%03u E%03u WP%03u WR%03u G%03u\r\n",
                   (unsigned)arm_angle[0], (unsigned)arm_angle[1],
                   (unsigned)arm_angle[2], (unsigned)arm_angle[3],
                   (unsigned)arm_angle[4], (unsigned)arm_angle[5]);
        }
    }
}

/*******************************************************************************
 * 函数名    ServoArm_MoveToXY
 * 描述      XY 逆运动学 + 缓冲移动
 *           给定地面坐标 (x,y,z=0)，IK 解算后设为目标角度走平滑插值
 * 参数      x, y    地面坐标 (mm)

 * 符号说明  IK_xxx_SIGN: 若关节转向相反，修改对应宏的 ± 号即可
 ******************************************************************************/
#define IK_BASE_SIGN     (+1)
#define IK_SHOULDER_SIGN (+1)
#define IK_ELBOW_SIGN    (+1)

void ServoArm_MoveToXY(float x, float y)
{
    double t1, t2, t3, t4;
    if (ArmIK_Solve((double)x, (double)y, &t1, &t2, &t3, &t4) != 0) {
        printf("Arm IK: unreachable (x=%.0f, y=%.0f)\r\n", x, y);
        return;
    }

    arm_target[ARM_CH_BASE]     = (uint16_t)(90.0 + IK_BASE_SIGN     * t1 + 0.5);
    arm_target[ARM_CH_SHOULDER] = (uint16_t)(90.0 + IK_SHOULDER_SIGN * t2 + 0.5);
    arm_target[ARM_CH_ELBOW]    = (uint16_t)(90.0 + IK_ELBOW_SIGN    * t3 + 0.5);
    arm_target[ARM_CH_WRIST_P]  = (uint16_t)(90.0 + t4 + 0.5);

    for (int i = 0; i < 4; i++) {
        if (arm_target[i] > 180) arm_target[i] = 180;
    }
    smooth_active = 1;
    printf("Arm IK: target B%u S%u E%u WP%u (from (%.0f, %.0f))\r\n",
           (unsigned)arm_target[ARM_CH_BASE],
           (unsigned)arm_target[ARM_CH_SHOULDER],
           (unsigned)arm_target[ARM_CH_ELBOW],
           (unsigned)arm_target[ARM_CH_WRIST_P], x, y);
}

/*******************************************************************************
 * 函数名    ServoArm_SmoothUpdate
 * 描述      缓冲插值推进器 — 每帧将 arm_angle[] 朝 arm_target[] 逼近 2°
 *           50Hz 主循环中调用
 ******************************************************************************/
void ServoArm_SmoothUpdate(void)
{
    if (!smooth_active) return;

    uint8_t moving = 0;
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        uint16_t cur = arm_angle[i];
        uint16_t tgt = arm_target[i];
        if (cur == tgt) continue;

        moving = 1;
        if (cur < tgt) {
            cur += SMOOTH_SPEED_DEG;
            if (cur > tgt) cur = tgt;
        } else {
            if (cur > tgt + SMOOTH_SPEED_DEG)
                cur -= SMOOTH_SPEED_DEG;
            else
                cur = tgt;
        }
        arm_angle[i] = cur;
        ServoArm_SetAngle(i, (float)cur);
    }

    if (!moving) smooth_active = 0;
}

/*******************************************************************************
 * 函数名    ServoArm_CancelMove
 * 描述      取消缓冲移动
 ******************************************************************************/
void ServoArm_CancelMove(void)
{
    smooth_active = 0;
}
