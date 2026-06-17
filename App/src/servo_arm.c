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
 * 2026-06-12      CIMC            新增 CollectTarget (已移除)
 * 2026-06-17      CIMC            重构校准为安装偏移；新增预设XY；270°舵机适配
 ******************************************************************************/

#include "servo_arm.h"
#include "pca9685.h"
#include "ps2.h"
#include "arm_ik.h"
#include <stdio.h>

/* =================================================================================
 *  舵机硬件参数
 * =================================================================================
 *  PWM 50Hz, 0°→0.5ms(102count), max°→2.5ms(512count)
 *  脉宽公式: pulse = 102 + angle × 410 / joint_max_angle[ch]
 */

#define PULSE_0      102
#define PULSE_FULL   512
#define PULSE_RNG    (PULSE_FULL - PULSE_0)   /* 410 */

/* 各通道舵机最大行程 (度) — ch0 为 270° 舵机，其余 180° */
static const float joint_max_angle[ARM_JOINT_COUNT] = {
    270.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f
};

/* =================================================================================
 *  安装角度偏移校准
 * =================================================================================
 *  公式: servo = physical - offset
 *  校准: 遥控器调关节到基准位置 → 记串口显示值 → offset = 默认角度 - 显示值
 */

static const float joint_offset[ARM_JOINT_COUNT] = {
     7.0f,   /* ch0 底座  默认135, 显示128 → 135-128=+7  */
    -6.0f,   /* ch1 大臂  默认 90, 显示 96 → 90-96=-6   */
    -7.0f,   /* ch2 小臂  默认 90, 显示 97 → 90-97=-7   */
     3.0f,   /* ch3 腕俯仰 默认 30, 显示 27 → 30-27=+3   */
    -8.0f,   /* ch4 腕旋转 默认 90, 显示 98 → 90-98=-8   */
     0.0f    /* ch5 夹爪  不校准                         */
};

/* =================================================================================
 *  关节限位
 * =================================================================================
 *  一般关节: [0, joint_max_angle[ch]]
 *  夹爪:     机械安全限位，防止堵转
 */

#define GRIPPER_ANGLE_MIN  51
#define GRIPPER_ANGLE_MAX  141

/* =================================================================================
 *  预设动作
 * ================================================================================= */
/*            [BASE] [SHOULDER] [ELBOW] [WRIST_P] [WRIST_R] [GRIPPER]   */

#define ACT_PARK    { 135,  90,  90,  30,  90, 141 }  /* 收船 / 上电默认  */
#define ACT_READY   {  90,  90,  90,  90,  90,  90 }  /* 准备抓取         */
#define ACT_COLLECT {  90, 135,  45,  90,  90, 141 }  /* 闭合夹爪 + 抬起  */
#define ACT_DROP    { 180,  90,  90,  90,  90,  51 }  /* 伸出 + 张开夹爪   */

static const uint16_t action_angle[4][ARM_JOINT_COUNT] = {
    [ARM_PARK]    = ACT_PARK,
    [ARM_READY]   = ACT_READY,
    [ARM_COLLECT] = ACT_COLLECT,
    [ARM_DROP]    = ACT_DROP,
};

/* =================================================================================
 *  预设 XY 坐标 (PS2 START 循环触发)
 * ================================================================================= */
static const float preset_xy[][2] = {
    {170,   0},      /* 正前方 */
    {150,  80},      /* 右前方 — 实测可达 */
    {150, -80},      /* 左前方 — 实测可达 */
    {130,   0},      /* 近处   */
};
#define PRESET_COUNT  4

/* =================================================================================
 *  模块内部状态
 * ================================================================================= */

#define PCA9685_ADDR       PCA9685_I2C_ADDR
#define SMOOTH_SPEED_DEG   2          /* 缓冲插值步进 (°/帧)，50Hz→100°/s */

static uint16_t arm_angle[ARM_JOINT_COUNT] = {135, 90, 90, 30, 90, 141};
static uint16_t arm_target[ARM_JOINT_COUNT];
static uint8_t  smooth_active = 0;

/* =================================================================================
 *  ServoArm_Init — 上电初始化，所有关节归默认位置
 * ================================================================================= */
void ServoArm_Init(void)
{
    pca9685_init(PCA9685_ADDR);
    pca9685_set_pwm_freq(PCA9685_ADDR, PCA9685_SERVO_FREQ);

    /* 上电默认: 底座135 / 大臂90 / 小臂90 / 腕俯仰30 / 腕旋转90 / 夹爪141 */
    static const uint16_t defaults[ARM_JOINT_COUNT] = {135, 90, 90, 30, 90, 141};

    smooth_active = 0;
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        arm_target[i] = defaults[i];
        ServoArm_SetAngle(i, (float)defaults[i]);
    }

    pca9685_output_enable();
    printf("ServoArm: ready, defaults=[135,90,90,30,90,141]\r\n");
}

/* =================================================================================
 *  ServoArm_SetAngle — 单关节角度输出 (物理角度 → 偏移补偿 → PWM)
 * =================================================================================
 *  ch          : 0~5
 *  phys_angle  : 目标物理角度, ch0 范围 0~270°, ch1~5 范围 0~180°
 *  内部流程    : 限位 → 减 offset → 转为舵机角度 → 查 joint_max_angle → PWM
 */
void ServoArm_SetAngle(uint8_t ch, float phys_angle)
{
    if (ch >= ARM_JOINT_COUNT) return;

    /* 物理角度限位 */
    float min_phys = 0.0f;
    float max_phys = joint_max_angle[ch];
    if (ch == ARM_CH_GRIPPER) { min_phys = GRIPPER_ANGLE_MIN; max_phys = GRIPPER_ANGLE_MAX; }
    if (phys_angle < min_phys) phys_angle = min_phys;
    if (phys_angle > max_phys) phys_angle = max_phys;

    /* 安装偏移补偿 → 舵机角度 */
    float servo_angle = phys_angle - joint_offset[ch];
    if (servo_angle < 0.0f)               servo_angle = 0.0f;
    if (servo_angle > joint_max_angle[ch]) servo_angle = joint_max_angle[ch];

    /* 舵机角度 → PWM 脉宽 */
    int32_t p = (int32_t)(PULSE_0 + servo_angle * PULSE_RNG / joint_max_angle[ch]);
    if (p < PULSE_0)   p = PULSE_0;
    if (p > PULSE_FULL) p = PULSE_FULL;

    pca9685_set_pwm(PCA9685_ADDR, ch, 0, (uint16_t)p);
}

/* =================================================================================
 *  ServoArm_SetAction — 执行预设动作 (缓冲插值)
 * ================================================================================= */
void ServoArm_SetAction(ArmAction action)
{
    if (action > ARM_DROP) return;

    pca9685_output_enable();
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        arm_target[i] = action_angle[action][i];
    }
    smooth_active = 1;
    printf("Arm: action %d\r\n", (int)action);
}

/* =================================================================================
 *  ServoArm_RemoteControl — PS2 手动调关节 (分频 ~2.5°/s)
 * =================================================================================
 *  ←/→ 底座   ↑/↓ 大臂   □/○ 小臂   △/× 腕俯仰   L1/L2 腕旋转   R1/R2 夹爪
 */
#define REMOTE_DIVIDER  1    /* 临时: 全速 50°/s，校准完改回 20 */

void ServoArm_RemoteControl(void)
{
    uint16_t prev[ARM_JOINT_COUNT];
    uint8_t  changed = 0;
    static uint8_t tick = 0;
    static uint8_t print_cnt = 0;

    if (++tick < REMOTE_DIVIDER) return;
    tick = 0;

    /* 有按键时取消缓冲移动 + 使能输出 */
    if (PS2_Data.left  || PS2_Data.right || PS2_Data.up   || PS2_Data.down ||
        PS2_Data.square|| PS2_Data.circle|| PS2_Data.triangle||PS2_Data.cross||
        PS2_Data.l1    || PS2_Data.l2    || PS2_Data.r1    || PS2_Data.r2) {
        smooth_active = 0;
        pca9685_output_enable();
    }

    /* ch0 底座 — ← → */
    prev[0] = arm_angle[0];
    if (PS2_Data.left)  { if (arm_angle[0] > 0)                             arm_angle[0]--; }
    if (PS2_Data.right) { if (arm_angle[0] < (uint16_t)joint_max_angle[0]) arm_angle[0]++; }
    if (arm_angle[0] != prev[0]) { ServoArm_SetAngle(0, (float)arm_angle[0]); changed = 1; }

    /* ch1 大臂 — ↑ ↓ */
    prev[1] = arm_angle[1];
    if (PS2_Data.up)    { if (arm_angle[1] < (uint16_t)joint_max_angle[1]) arm_angle[1]++; }
    if (PS2_Data.down)  { if (arm_angle[1] > 0)                             arm_angle[1]--; }
    if (arm_angle[1] != prev[1]) { ServoArm_SetAngle(1, (float)arm_angle[1]); changed = 1; }

    /* ch2 小臂 — □ ○ */
    prev[2] = arm_angle[2];
    if (PS2_Data.square){ if (arm_angle[2] > 0)                             arm_angle[2]--; }
    if (PS2_Data.circle){ if (arm_angle[2] < (uint16_t)joint_max_angle[2]) arm_angle[2]++; }
    if (arm_angle[2] != prev[2]) { ServoArm_SetAngle(2, (float)arm_angle[2]); changed = 1; }

    /* ch3 腕俯仰 — △ × */
    prev[3] = arm_angle[3];
    if (PS2_Data.triangle){ if (arm_angle[3] > 0)                             arm_angle[3]--; }
    if (PS2_Data.cross)  { if (arm_angle[3] < (uint16_t)joint_max_angle[3]) arm_angle[3]++; }
    if (arm_angle[3] != prev[3]) { ServoArm_SetAngle(3, (float)arm_angle[3]); changed = 1; }

    /* ch4 腕旋转 — L1 L2 */
    prev[4] = arm_angle[4];
    if (PS2_Data.l1)    { if (arm_angle[4] > 0)                             arm_angle[4]--; }
    if (PS2_Data.l2)    { if (arm_angle[4] < (uint16_t)joint_max_angle[4]) arm_angle[4]++; }
    if (arm_angle[4] != prev[4]) { ServoArm_SetAngle(4, (float)arm_angle[4]); changed = 1; }

    /* ch5 夹爪 — R1 R2 (独立安全限位) */
    prev[5] = arm_angle[5];
    if (PS2_Data.r1)    { if (arm_angle[5] > GRIPPER_ANGLE_MIN) arm_angle[5]--; }
    if (PS2_Data.r2)    { if (arm_angle[5] < GRIPPER_ANGLE_MAX) arm_angle[5]++; }
    if (arm_angle[5] != prev[5]) { ServoArm_SetAngle(5, (float)arm_angle[5]); changed = 1; }

    /* 变化时偶尔打印 */
    if (changed && ++print_cnt % 5 == 1) {
        printf("Arm: B%03u S%03u E%03u WP%03u WR%03u G%03u\r\n",
               (unsigned)arm_angle[0], (unsigned)arm_angle[1],
               (unsigned)arm_angle[2], (unsigned)arm_angle[3],
               (unsigned)arm_angle[4], (unsigned)arm_angle[5]);
    }
}

/* =================================================================================
 *  ServoArm_MoveToXY — IK 解算 → 缓冲移动到目标 XY
 * =================================================================================
 *  调用链: HandlePresets → MoveToXY → ArmIK_Solve → arm_target → SmoothUpdate
 */
#define IK_BASE_SIGN      (+1)
#define IK_SHOULDER_SIGN  (+1)
#define IK_ELBOW_SIGN     (+1)

void ServoArm_MoveToXY(float x, float y)
{
    double t1, t2, t3, t4;
    if (ArmIK_Solve((double)x, (double)y, &t1, &t2, &t3, &t4) != 0) {
        printf("Arm IK: unreachable (x=%.0f y=%.0f)\r\n", x, y);
        return;
    }

    arm_target[ARM_CH_BASE]     = (uint16_t)(135.0 + IK_BASE_SIGN     * t1 + 0.5);
    arm_target[ARM_CH_SHOULDER] = (uint16_t)( 90.0 + IK_SHOULDER_SIGN * t2 + 0.5);
    arm_target[ARM_CH_ELBOW]    = (uint16_t)( 90.0 + IK_ELBOW_SIGN    * t3 + 0.5);
    arm_target[ARM_CH_WRIST_P]  = (uint16_t)( 30.0 + t4 + 0.5);

    for (int i = 0; i < 4; i++) {
        if (arm_target[i] > (uint16_t)joint_max_angle[i])
            arm_target[i] = (uint16_t)joint_max_angle[i];
    }

    pca9685_output_enable();
    smooth_active = 1;
    printf("Arm IK: B%u S%u E%u WP%u <- (%.0f, %.0f)\r\n",
           (unsigned)arm_target[ARM_CH_BASE], (unsigned)arm_target[ARM_CH_SHOULDER],
           (unsigned)arm_target[ARM_CH_ELBOW],  (unsigned)arm_target[ARM_CH_WRIST_P], x, y);
}

/* =================================================================================
 *  ServoArm_SmoothUpdate — 缓冲插值推进 (50Hz 调用)
 * ================================================================================= */
void ServoArm_SmoothUpdate(void)
{
    if (!smooth_active) return;

    uint8_t moving = 0;
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        if (arm_angle[i] == arm_target[i]) continue;

        moving = 1;
        if (arm_angle[i] < arm_target[i]) {
            arm_angle[i] += SMOOTH_SPEED_DEG;
            if (arm_angle[i] > arm_target[i]) arm_angle[i] = arm_target[i];
        } else {
            if (arm_angle[i] > arm_target[i] + SMOOTH_SPEED_DEG)
                arm_angle[i] -= SMOOTH_SPEED_DEG;
            else
                arm_angle[i] = arm_target[i];
        }
        ServoArm_SetAngle(i, (float)arm_angle[i]);
    }

    if (!moving) {
        smooth_active = 0;
        pca9685_output_disable();
    }
}

/* =================================================================================
 *  ServoArm_HandlePresets — PS2 START/SELECT 预设 XY
 * ================================================================================= */
void ServoArm_HandlePresets(void)
{
    static uint8_t preset_idx     = 0;
    static uint8_t p_start_prev   = 0;
    static uint8_t p_select_prev  = 0;

    if (PS2_Data.start && !p_start_prev) {
        p_start_prev = 1;
        ServoArm_MoveToXY(preset_xy[preset_idx][0], preset_xy[preset_idx][1]);
        preset_idx = (preset_idx + 1) % PRESET_COUNT;
    }
    if (!PS2_Data.start) p_start_prev = 0;

    if (PS2_Data.select && !p_select_prev) {
        p_select_prev = 1;
        ServoArm_SetAction(ARM_PARK);
    }
    if (!PS2_Data.select) p_select_prev = 0;
}

/* =================================================================================
 *  ServoArm_PrintStatus — 串口打印 6 路角度 + 脉宽 (1Hz)
 * ================================================================================= */
void ServoArm_PrintStatus(void)
{
    printf("SERVO: ");
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        float sa = (float)arm_angle[i] - joint_offset[i];
        if (sa < 0.0f)               sa = 0.0f;
        if (sa > joint_max_angle[i]) sa = joint_max_angle[i];

        int32_t  p   = (int32_t)(PULSE_0 + sa * PULSE_RNG / joint_max_angle[i]);
        if (p < PULSE_0)   p = PULSE_0;
        if (p > PULSE_FULL) p = PULSE_FULL;
        uint16_t pus = (uint16_t)((uint32_t)p * 20000UL / 4096UL);

        printf("ch%u:%3ud/%4uus ", (unsigned)i, (unsigned)arm_angle[i], (unsigned)pus);
    }
    printf("\r\n");
}

/* =================================================================================
 *  ServoArm_CancelMove — 取消缓冲移动
 * ================================================================================= */
void ServoArm_CancelMove(void)
{
    smooth_active = 0;
    pca9685_output_disable();
}
