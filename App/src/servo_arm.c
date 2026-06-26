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
 * 2026-06-18      CIMC            示教模式: 遥控速度可调 + START捕获姿态
 *                                  IK/HandlePresets 保留但暂停调用
 * 2026-06-18      CIMC            新增串口角度指令: MoveToAngles + ProcessSerialCommand
 * 2026-06-24      CIMC            移除 joint_offset 机制，角度直接即舵机角度；换大扭力舵机后重新校准
 ******************************************************************************/

#include "servo_arm.h"
#include "pca9685.h"
#include "ps2.h"
#include "arm_ik.h"
#include "uart_driver.h"
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
 *  舵机默认位置校准
 * =================================================================================
 *  角度直接即为舵机角度 (0~joint_max_angle[ch])，无中间 offset 层
 *  校准: 遥控器调关节到目标姿态 → PS2 START 捕获 → 填入 action_angle 表
 */

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
/*  ch1=新大扭力舵机, 其他沿用旧标定值                                       */

#define ACT_PARK    { 128,  88,  97,  27,  98, 141 }  /* 收船 / 上电默认  */
#define ACT_READY   {  83,  88,  97,  87,  98,  90 }  /* 准备抓取         */
#define ACT_COLLECT {  83, 133,  52,  87,  98, 141 }  /* 闭合夹爪 + 抬起  */
#define ACT_DROP    { 173,  88,  97,  87,  98,  51 }  /* 伸出 + 张开夹爪   */

static const uint16_t action_angle[4][ARM_JOINT_COUNT] = {
    [ARM_PARK]    = ACT_PARK,
    [ARM_READY]   = ACT_READY,
    [ARM_COLLECT] = ACT_COLLECT,
    [ARM_DROP]    = ACT_DROP,
};

#if 0  /* ---- 预设 XY 坐标 (IK 模式用, 2026-06-18 暂停) ---- */
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
#endif

/* =================================================================================
 *  模块内部状态
 * ================================================================================= */

#define PCA9685_ADDR       PCA9685_I2C_ADDR

static uint16_t arm_angle[ARM_JOINT_COUNT]  = {128, 88, 97, 27, 98, 141};
static uint16_t arm_target[ARM_JOINT_COUNT];
static uint8_t  smooth_active   = 0;
static uint8_t  remote_divider  = 10;    /* 遥控速度: 1=50°/s, 10=5°/s, 20=2.5°/s */
static uint8_t  smooth_speed    = 2;     /* 缓冲步进 (°/帧), 1~10               */
static uint8_t  smooth_divider  = 1;     /* 缓冲分频: 1=50Hz, 2=25Hz, 4=12.5Hz  */
static uint8_t  capture_slot    = 0;     /* 示教捕获序号 */

/* =================================================================================
 *  采集序列 — 一键自动执行 (按一次 R3 跑完全部 6 阶段)
 * =================================================================================
 *  seq_phase: 0=空闲, 1~6=当前执行阶段号
 *  StartSequence 设 seq_phase=1 → SequenceUpdate 自动推进,
 *  每阶段等 smooth_active==0 后自动进入下一阶段
 */
#define SEQ_IDLE  0
#define SEQ_STEPS 6
static uint8_t  seq_phase = SEQ_IDLE;     /* 0=空闲, 1~6=当前阶段 */

/* =================================================================================
 *  ServoArm_Init — 上电初始化，所有关节归默认位置
 * ================================================================================= */
void ServoArm_Init(void)
{
    pca9685_init(PCA9685_ADDR);
    pca9685_set_pwm_freq(PCA9685_ADDR, PCA9685_SERVO_FREQ);

    /* 上电默认: 底座128 / 大臂88(新舵机) / 小臂97 / 腕俯仰27 / 腕旋转98 / 夹爪141 */
    static const uint16_t defaults[ARM_JOINT_COUNT] = {128, 88, 97, 27, 98, 141};

    smooth_active = 0;
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        arm_target[i] = defaults[i];
        ServoArm_SetAngle(i, (float)defaults[i]);
    }

    pca9685_output_enable();
    printf("ServoArm: ready, defaults=[128,88,97,27,98,141]\r\n");
}

/* =================================================================================
 *  ServoArm_SetAngle — 单关节角度输出 (舵机角度 → PWM)
 * =================================================================================
 *  ch         : 0~5
 *  angle      : 舵机角度, ch0 范围 0~270°, ch1~5 范围 0~180°
 *  内部流程   : 限位 → PWM 脉宽
 */
void ServoArm_SetAngle(uint8_t ch, float angle)
{
    if (ch >= ARM_JOINT_COUNT) return;

    /* 舵机角度限位 */
    float min_a = 0.0f;
    float max_a = joint_max_angle[ch];
    if (ch == ARM_CH_GRIPPER) { min_a = GRIPPER_ANGLE_MIN; max_a = GRIPPER_ANGLE_MAX; }
    if (angle < min_a) angle = min_a;
    if (angle > max_a) angle = max_a;

    /* 舵机角度 → PWM 脉宽 */
    int32_t p = (int32_t)(PULSE_0 + angle * PULSE_RNG / joint_max_angle[ch]);
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
 *  ServoArm_RemoteControl — PS2 手动调关节 (速度 = 50Hz / remote_divider)
 * =================================================================================
 *  ←/→ 底座   ↑/↓ 大臂   □/○ 小臂   △/× 腕俯仰   L1/L2 腕旋转   R1/R2 夹爪
 *  速度: divider=1→50°/s, 10→5°/s, 20→2.5°/s, 50→1°/s
 */
void ServoArm_RemoteControl(void)
{
    uint16_t prev[ARM_JOINT_COUNT];
    uint8_t  changed = 0;
    static uint8_t tick = 0;
    static uint8_t print_cnt = 0;

    if (seq_phase != SEQ_IDLE) return;  /* 序列自动执行中，忽略手动遥控 */

    if (++tick < remote_divider) return;
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

    /* 变化时偶尔打印 — 调试时取消注释 */
    //if (changed && ++print_cnt % 5 == 1) {
    //    printf("Arm: B%03u S%03u E%03u WP%03u WR%03u G%03u\r\n",
    //           (unsigned)arm_angle[0], (unsigned)arm_angle[1],
    //           (unsigned)arm_angle[2], (unsigned)arm_angle[3],
    //           (unsigned)arm_angle[4], (unsigned)arm_angle[5]);
    //}
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

    arm_target[ARM_CH_BASE]     = (uint16_t)(128.0 + IK_BASE_SIGN     * t1 + 0.5);
    arm_target[ARM_CH_SHOULDER] = (uint16_t)( 88.0 + IK_SHOULDER_SIGN * t2 + 0.5);
    arm_target[ARM_CH_ELBOW]    = (uint16_t)( 97.0 + IK_ELBOW_SIGN    * t3 + 0.5);
    arm_target[ARM_CH_WRIST_P]  = (uint16_t)( 27.0 + t4 + 0.5);

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
    static uint8_t tick = 0;
    if (!smooth_active) return;
    if (++tick < smooth_divider) return;
    tick = 0;

    uint8_t moving = 0;
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        if (arm_angle[i] == arm_target[i]) continue;

        moving = 1;
        if (arm_angle[i] < arm_target[i]) {
            arm_angle[i] += smooth_speed;
            if (arm_angle[i] > arm_target[i]) arm_angle[i] = arm_target[i];
        } else {
            if (arm_angle[i] > arm_target[i] + smooth_speed)
                arm_angle[i] -= smooth_speed;
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
 *  ServoArm_HandlePresets — [示教模式] PS2 START=捕获姿态, SELECT=回PARK
 * =================================================================================
 *  原 IK 预设 XY 功能已暂停 (代码保留, 见下方注释块)
 */
void ServoArm_HandlePresets(void)
{
    static uint8_t p_start_prev  = 0;
    static uint8_t p_select_prev = 0;

    /* START: 捕获当前姿态 → 串口输出 (供填入动作表) */
    if (PS2_Data.start && !p_start_prev) {
        p_start_prev = 1;
        ServoArm_CapturePosition(capture_slot);
        capture_slot++;
    }
    if (!PS2_Data.start) p_start_prev = 0;

    /* SELECT: 回 PARK */
    if (PS2_Data.select && !p_select_prev) {
        p_select_prev = 1;
        ServoArm_SetAction(ARM_PARK);
    }
    if (!PS2_Data.select) p_select_prev = 0;
}

#if 0  /* ---- 原 IK 预设 XY 功能 (2026-06-18 暂停, 代码保留) ---- */
/*
void ServoArm_HandlePresets_IK(void)
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
*/
#endif

/* =================================================================================
 *  ServoArm_PrintStatus — 串口打印 6 路角度 + 脉宽 (1Hz)
 * ================================================================================= */
void ServoArm_PrintStatus(void)
{
    printf("SERVO: ");
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        float sa = (float)arm_angle[i];
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
 *  ServoArm_SetSpeed — 设置遥控手动速度
 * =================================================================================
 *  divider: 1(50°/s) ~ 50(1°/s), 默认 10(5°/s)
 *  公式: 实际速度 = 50Hz / divider °/s
 */
void ServoArm_SetSpeed(uint8_t divider)
{
    if (divider < 1)  divider = 1;
    if (divider > 50) divider = 50;
    remote_divider = divider;
    printf("Arm: remote speed = %u°/s (divider=%u)\r\n",
           (unsigned)(50 / divider), (unsigned)divider);
}

/* =================================================================================
 *  ServoArm_SetSmoothSpeed — 设置预设动作缓冲速度
 * =================================================================================
 *  deg_per_frame: 每帧(20ms)移动度数, 1=50°/s, 2=100°/s, 默认2
 */
void ServoArm_SetSmoothSpeed(uint8_t deg_per_frame)
{
    if (deg_per_frame < 1)  deg_per_frame = 1;
    if (deg_per_frame > 10) deg_per_frame = 10;
    smooth_speed = deg_per_frame;
    printf("Arm: smooth speed = %u°/s (%u deg/frame)\r\n",
           (unsigned)(deg_per_frame * 50), (unsigned)deg_per_frame);
}

/* =================================================================================
 *  ServoArm_SetSmoothDivider — 设置预设动作缓冲分频
 * =================================================================================
 *  div: 1=全速50Hz, 2=25Hz, 4=12.5Hz... 越大越慢
 *  实际速度 = smooth_speed × 50 / div °/s
 */
void ServoArm_SetSmoothDivider(uint8_t div)
{
    if (div < 1)  div = 1;
    if (div > 20) div = 20;
    smooth_divider = div;
    printf("Arm: smooth divider = %u (%.1f°/s)\r\n",
           (unsigned)div, (double)(smooth_speed * 50.0f / div));
}

/* =================================================================================
 *  ServoArm_CapturePosition — 捕获当前姿态 → 串口输出动作表格式
 * =================================================================================
 *  slot: 示教序号, 仅用于标注
 *  输出格式可直接填入 action_angle[][] 表
 */
void ServoArm_CapturePosition(uint8_t slot)
{
    printf("\r\n===== CAPTURE #%u =====\r\n", (unsigned)slot);
    printf("/* slot %u */ { %3u, %3u, %3u, %3u, %3u, %3u },\r\n",
           (unsigned)slot,
           (unsigned)arm_angle[0], (unsigned)arm_angle[1],
           (unsigned)arm_angle[2], (unsigned)arm_angle[3],
           (unsigned)arm_angle[4], (unsigned)arm_angle[5]);
    printf("// B=%3u° S=%3u° E=%3u° WP=%3u° WR=%3u° G=%3u°\r\n\r\n",
           (unsigned)arm_angle[0], (unsigned)arm_angle[1],
           (unsigned)arm_angle[2], (unsigned)arm_angle[3],
           (unsigned)arm_angle[4], (unsigned)arm_angle[5]);
}

/* =================================================================================
 *  ServoArm_CancelMove — 取消缓冲移动
 * ================================================================================= */
void ServoArm_CancelMove(void)
{
    smooth_active = 0;
    pca9685_output_disable();
}

/* =================================================================================
 *  ServoArm_MoveToAngles — 设置6关节目标角度 → 缓冲移动
 * =================================================================================
 *  angles[6]   : 6个舵机角度值, 直接赋给 arm_target, 启动 smooth 插值
 *  使用示例     uint16_t pos[] = {128, 88, 97, 27, 98, 141};
 *              ServoArm_MoveToAngles(pos);
 */
void ServoArm_MoveToAngles(const uint16_t angles[6])
{
    pca9685_output_enable();
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        arm_target[i] = angles[i];
    }
    smooth_active = 1;
    printf("Arm: move to { %u, %u, %u, %u, %u, %u }\r\n",
           (unsigned)angles[0], (unsigned)angles[1], (unsigned)angles[2],
           (unsigned)angles[3], (unsigned)angles[4], (unsigned)angles[5]);
}

/* =================================================================================
 *  ServoArm_ProcessSerialCommand — 检查调试串口(UART_DBG)角度指令
 * =================================================================================
 *  格式: { 128, 88, 97, 27, 98, 141 }
 *  从 '{' 开始, '}' 结束, 内部逗号/空格分隔6个整数
 *  每次调用逐字节消费 UART_DBG FIFO, 非阻塞
 *  解析成功 → 调用 ServoArm_MoveToAngles 启动缓冲移动
 *
 *  状态机: IDLE → 等待'{' → ACTIVE → 收字符到缓冲区 → '}' → 解析执行 → IDLE
 *  超时保护: 帧内 1 秒未收到 '}' 自动复位
 */
#define SERIAL_PARSE_IDLE    0
#define SERIAL_PARSE_ACTIVE  1
#define SERIAL_PARSE_BUF_MAX 48

void ServoArm_ProcessSerialCommand(void)
{
    uint8_t byte;
    static uint8_t  state          = SERIAL_PARSE_IDLE;
    static char     buf[SERIAL_PARSE_BUF_MAX];
    static uint8_t  idx            = 0;
    static uint32_t frame_start_ms = 0;

    /* 帧超时保护: 1秒内未闭合 → 丢弃 */
    if (state == SERIAL_PARSE_ACTIVE && (g_sys_ms - frame_start_ms > 1000)) {
        state = SERIAL_PARSE_IDLE;
        idx   = 0;
    }

    /* 逐字节消费 FIFO (一次可能收到多个字节) */
    while (uart_query_byte(UART_DBG, &byte)) {
        if (byte == '{') {
            /* 帧起始 — 复位缓冲区 */
            state = SERIAL_PARSE_ACTIVE;
            idx   = 0;
            frame_start_ms = g_sys_ms;
        } else if (byte == '}') {
            /* 帧结束 — 解析并执行 */
            if (state == SERIAL_PARSE_ACTIVE && idx > 0) {
                buf[idx] = '\0';

                /* 手动解析数字 (避免 sscanf, MicroLIB 兼容) */
                uint16_t angles[6];
                uint8_t  num_count = 0;
                uint16_t val       = 0;
                uint8_t  in_number = 0;
                uint8_t  i;

                for (i = 0; i <= idx; i++) {
                    char c = buf[i];   /* idx 位置存放了 '\0' */
                    if (c >= '0' && c <= '9') {
                        val = val * 10 + (uint16_t)(c - '0');
                        in_number = 1;
                    } else {
                        if (in_number && num_count < 6) {
                            angles[num_count++] = val;
                            val       = 0;
                            in_number = 0;
                        }
                    }
                }

                if (num_count == 6) {
                    ServoArm_MoveToAngles(angles);
                } else {
                    printf("Arm: serial err — got %u values, need 6\r\n",
                           (unsigned)num_count);
                }
            }
            state = SERIAL_PARSE_IDLE;
            idx   = 0;
        } else if (state == SERIAL_PARSE_ACTIVE) {
            /* 帧内字符: 数字 / 逗号 / 空格 — 全部存入 */
            if (idx < SERIAL_PARSE_BUF_MAX - 1) {
                buf[idx++] = (char)byte;
            }
            /* 缓冲区满: 丢弃 (避免溢出) */
        }
        /* else: 帧外字符 → 直接丢弃 */
    }
}

/* =================================================================================
 *  ServoArm_StartSequence — 一键启动采集序列 (R3 按下调用)
 * =================================================================================
 *  6 阶段自动执行:
 *    1) 移其他关节 (大臂不动)  2) 大臂最后动    3) 闭合夹爪
 *    4) 返回: 大臂先动         5) 返回: 其余关节  6) 张开爪子 (投放)
 *
 *  防重入: 序列已在运行中则忽略
 */
void ServoArm_StartSequence(void)
{
    if (seq_phase != SEQ_IDLE) return;  /* 已在运行中 */

    seq_phase     = 1;
    smooth_active = 0;
    printf("\r\nArm: SEQ start — 6-phase auto sequence\r\n");
}

/* =================================================================================
 *  ServoArm_SequenceUpdate — 序列自动推进 (50Hz, 在 SmoothUpdate 之前调用)
 * =================================================================================
 *  空闲时返回; smooth_active=1 表示当前阶段还在移动;
 *  smooth_active=0 表示阶段完成, 自动构建下一阶段目标并启动 MoveToAngles
 */
void ServoArm_SequenceUpdate(void)
{
    uint16_t tgt[ARM_JOINT_COUNT];
    uint8_t  i;

    if (seq_phase == SEQ_IDLE) return;   /* 空闲 */

    if (smooth_active) return;           /* 当前阶段还在移动中 */

    /* ---- 当前阶段完成, 执行下一阶段 ---- */
    switch (seq_phase) {

    case 1:  /* 移其他关节 (大臂不动) */
        for (i = 0; i < ARM_JOINT_COUNT; i++) tgt[i] = arm_angle[i];
        tgt[0] = 128; tgt[2] = 145; tgt[3] = 119; tgt[4] = 99; tgt[5] = 109;
        printf("Arm: SEQ [1/6] others -> {128, —, 145, 119, 99, 109}\r\n");
        ServoArm_MoveToAngles(tgt);
        seq_phase = 2;
        break;

    case 2:  /* 大臂最后动 */
        for (i = 0; i < ARM_JOINT_COUNT; i++) tgt[i] = arm_angle[i];
        tgt[1] = 119;
        printf("Arm: SEQ [2/6] shoulder -> 119\r\n");
        ServoArm_MoveToAngles(tgt);
        seq_phase = 3;
        break;

    case 3:  /* 闭合夹爪 */
        for (i = 0; i < ARM_JOINT_COUNT; i++) tgt[i] = arm_angle[i];
        tgt[5] = 141;
        printf("Arm: SEQ [3/6] close claw -> 141\r\n");
        ServoArm_MoveToAngles(tgt);
        seq_phase = 4;
        break;

    case 4:  /* 返回 — 大臂先动 */
        for (i = 0; i < ARM_JOINT_COUNT; i++) tgt[i] = arm_angle[i];
        tgt[1] = 88;
        printf("Arm: SEQ [4/6] return shoulder first -> 88\r\n");
        ServoArm_MoveToAngles(tgt);
        seq_phase = 5;
        break;

    case 5:  /* 返回 — 其余关节归 PARK */
        for (i = 0; i < ARM_JOINT_COUNT; i++) tgt[i] = arm_angle[i];
        tgt[2] = 97; tgt[3] = 27; tgt[4] = 98;
        printf("Arm: SEQ [5/6] return others -> PARK\r\n");
        ServoArm_MoveToAngles(tgt);
        seq_phase = 6;
        break;

    case 6:  /* 张开爪子 (投放) */
        for (i = 0; i < ARM_JOINT_COUNT; i++) tgt[i] = arm_angle[i];
        tgt[5] = 51;
        printf("Arm: SEQ [6/6] open claw (drop)\r\n");
        ServoArm_MoveToAngles(tgt);
        seq_phase = SEQ_IDLE;
        printf("Arm: SEQ done!\r\n");
        break;
    }
}

/* =================================================================================
 *  ServoArm_IsSequenceBusy — 查询序列是否运行中
 * =================================================================================
 *  返回值: 0=空闲, 1=运行中 (等待移动完成 或 正在推进)
 */
uint8_t ServoArm_IsSequenceBusy(void)
{
    return (seq_phase != SEQ_IDLE) ? 1 : 0;
}
