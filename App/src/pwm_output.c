/*******************************************************************************
 * 文件名          pwm_output.c
 * 描述            四电机 PWM 输出 — 双引脚 H 桥控制，两板并联
 *                 左侧双电机(D0/D1) + 右侧双电机(D2/D3)，同侧并联共享控制信号
 *                 正反转切换自动插入 100ms 刹车保护
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 驱动板控制逻辑 (每路电机 2 个引脚，同侧双电机并联共享):
 *                 正转调速:  Dx = 1/PWM, Dy = 0
 *                 反转调速:  Dx = 0,     Dy = 1/PWM
 *                 自由滑行:  Dx = 0,     Dy = 0
 *                 快速刹车:  Dx = 1,     Dy = 1   (绕组短路制动)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-07      AI助手          初始版本 (IN1/IN2/ENA 三线制)
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 * 2026-06-11      CIMC            PWM 极性修复 + 大功率电机限幅
 * 2026-06-12      CIMC            更换驱动板：三线制→双线H桥，4引脚全在GPIOC
 ******************************************************************************/

#include "pwm_output.h"
#include "gd32_driver_pwm.h"

/* ==================== 常量 ==================== */
/* 映射系数：duty 1.0 → PWM_DUTY_MAX */
#define DUTY_SCALE  ((float)PWM_DUTY_MAX)

/* 方向切换刹车保持周期数（50Hz × 5 = 100ms） */
#define BRAKE_CYCLES 5

/* 启动突加参数: 油门 >10% 时先给 70% 持续 0.1s 克服静摩擦 */
#define START_THRESHOLD   0.10f
#define START_KICK_DUTY   0.70f
#define START_KICK_FRAMES 5        /* 50Hz × 5 = 100ms */

/* ==================== 电机方向切换状态 ==================== */
typedef struct {
    int8_t  brake_cnt;      /* >0: 正在刹车倒计时 (50Hz 帧)    */
    float   last_duty;      /* 上次正常输出时的 duty，用于方向突变检测 */
} MotorSwitchState;

static MotorSwitchState left_sw  = {0, 0.0f};
static MotorSwitchState right_sw = {0, 0.0f};

/* ==================== 油门映射 ==================== */
/*
 *  target    = 0  → internal = 0
 *  target    > 0  → internal = 0.10 + target × 0.90
 *
 *  用户设置 1% → 内部 10.9%  (刚能维持转动)
 *  用户设置 100% → 内部 100%
 */

typedef struct {
    float  target;
    float  internal;
    int8_t kick_cnt;    /* >0 = 突加倒计时 */
} MotorThrottle;

#define THR_MIN_RUN    0.10f   /* 电机最小运行油门    */
#define THR_KICK_DUTY  0.70f   /* 启动突加油门        */
#define THR_KICK_TICKS 5       /* 突加帧数 (50Hz×5=100ms) */

static MotorThrottle left_thr  = {0.0f, 0.0f, 0};
static MotorThrottle right_thr = {0.0f, 0.0f, 0};

static float motor_throttle_map(float t)
{
    if (t <= 0.0f) return 0.0f;
    return THR_MIN_RUN + t * (1.0f - THR_MIN_RUN);
}

/* ==================== 内部：双引脚 H 桥控制 ==================== */
/*!
 *  \brief   设置一组电机的 H 桥输出（同侧双电机并联共享信号）
 *  \param   pin_a_port / pin_a_pin   正转 PWM 引脚 (D0/D2)
 *  \param   pin_b_port / pin_b_pin   反转 PWM 引脚 (D1/D3)
 *  \param   duty                     占空比 -1.0~1.0 (正=前进, 负=后退, 0=停止)
 *  \param   sw                       方向切换状态 (刹车计数 + last_duty)
 *
 *  控制逻辑:
 *    duty > 0  → pin_a = PWM(|duty|), pin_b = 0   (正转)
 *    duty < 0  → pin_a = 0,          pin_b = PWM(|duty|) (反转)
 *    duty = 0  → pin_a = 0,          pin_b = 0   (滑行)
 *    刹车中    → pin_a = 1,          pin_b = 1   (快速制动)
 *
 *  电平实现 (经 gd32_driver_pwm 软件 PWM):
 *    pwm_set_duty(port, pin, 0)           → 引脚 100% 高电平 (逻辑 1)
 *    pwm_set_duty(port, pin, PWM_DUTY_MAX) → 引脚 ~0% 高电平 (逻辑 0)
 *    pwm_set_duty(port, pin, PWM_DUTY_MAX - raw) → 引脚 duty% 高电平 (PWM)
 */
static void pwm_set_motor_duty(
    uint32_t pin_a_port, uint16_t pin_a_pin,
    uint32_t pin_b_port, uint16_t pin_b_pin,
    float duty, MotorSwitchState *sw)
{
    uint16_t pwm_a, pwm_b;

    /* ---- 钳位 ---- */
    if (duty > PWM_MAX_DUTY)   duty = PWM_MAX_DUTY;
    if (duty < -PWM_MAX_DUTY)  duty = -PWM_MAX_DUTY;

    if (sw->brake_cnt > 0) {
        /* 刹车保护倒计时：两引脚均置高 → 电机绕组短路制动 */
        sw->brake_cnt--;
        if (sw->brake_cnt == 0) {
            sw->last_duty = duty;   /* 刹车结束，同步方向防止重复触发 */
        }
        pwm_a = 0;                  /* 0 → 100% HIGH = 逻辑 1 */
        pwm_b = 0;
    }
    else if (duty == 0.0f) {
        /* 停止：两引脚均置低 → 电机断电，惯性滑行 */
        pwm_a = PWM_DUTY_MAX;       /* PWM_DUTY_MAX → ~0% HIGH = 逻辑 0 */
        pwm_b = PWM_DUTY_MAX;
    }
    else {
        /* ---- 检测方向突变（正 ↔ 反） ---- */
        if ((sw->last_duty > 0 && duty < 0) || (sw->last_duty < 0 && duty > 0)) {
            sw->brake_cnt = BRAKE_CYCLES;
            pwm_a = 0;              /* 先刹车 100ms 再切换方向 */
            pwm_b = 0;
        }
        else {
            /* 正常方向输出 */
            float abs_duty = (duty > 0) ? duty : -duty;
            uint16_t raw = (uint16_t)(abs_duty * DUTY_SCALE);
            if (raw > PWM_DUTY_MAX) raw = PWM_DUTY_MAX;
            uint16_t pwm_active = PWM_DUTY_MAX - raw;   /* PWM 占空比对应的计数值 */

            if (duty > 0) {
                /* 正转: pin_a = PWM, pin_b = 0 */
                pwm_a = pwm_active;
                pwm_b = PWM_DUTY_MAX;
            } else {
                /* 反转: pin_a = 0, pin_b = PWM */
                pwm_a = PWM_DUTY_MAX;
                pwm_b = pwm_active;
            }
        }
    }

    /* ---- 写入软件 PWM ---- */
    pwm_set_duty((uint32_t)pin_a_port, pin_a_pin, pwm_a);
    pwm_set_duty((uint32_t)pin_b_port, pin_b_pin, pwm_b);

    /* ---- 更新 last_duty（刹车期间不更新，保持原方向记录） ---- */
    if (sw->brake_cnt == 0) {
        sw->last_duty = duty;
    }
}

/* ==================== 公共 API ==================== */

/*******************************************************************************
 * 函数名    pwm_output_init
 * 描述      初始化 4 个电机 PWM 引脚 (全部 GPIOC，驱动四电机，同侧并联)
 *           初始状态：全部输出低电平 → 四电机断电滑行
 *           软件 PWM 时基 TIMER4 首次调用 pwm_init 时自动配置
 ******************************************************************************/
void pwm_output_init(void)
{
    /* 4 个引脚均初始化为软件 PWM 输出 (全部 GPIOC) */
    pwm_init(MOTOR_LEFT_D0_PORT,  MOTOR_LEFT_D0_PIN);
    pwm_init(MOTOR_LEFT_D1_PORT,  MOTOR_LEFT_D1_PIN);
    pwm_init(MOTOR_RIGHT_D2_PORT, MOTOR_RIGHT_D2_PIN);
    pwm_init(MOTOR_RIGHT_D3_PORT, MOTOR_RIGHT_D3_PIN);

    /* 初始状态：全部低电平 = 滑行 (coast) */
    pwm_set_duty(MOTOR_LEFT_D0_PORT,  MOTOR_LEFT_D0_PIN,  (uint16_t)PWM_DUTY_MAX);
    pwm_set_duty(MOTOR_LEFT_D1_PORT,  MOTOR_LEFT_D1_PIN,  (uint16_t)PWM_DUTY_MAX);
    pwm_set_duty(MOTOR_RIGHT_D2_PORT, MOTOR_RIGHT_D2_PIN, (uint16_t)PWM_DUTY_MAX);
    pwm_set_duty(MOTOR_RIGHT_D3_PORT, MOTOR_RIGHT_D3_PIN, (uint16_t)PWM_DUTY_MAX);

    /* 复位状态机 */
    left_sw.brake_cnt  = 0;
    left_sw.last_duty  = 0.0f;
    right_sw.brake_cnt = 0;
    right_sw.last_duty = 0.0f;

    left_thr  = (MotorThrottle){0.0f, 0.0f, 0};
    right_thr = (MotorThrottle){0.0f, 0.0f, 0};
}

/* ==================== 油门更新 (映射 + 启动突加) ==================== */
static void motor_throttle_update(MotorThrottle *thr)
{
    float mapped = motor_throttle_map(thr->target);

    /* 归零: 立即停止 */
    if (mapped == 0.0f) {
        thr->internal = 0.0f;
        thr->kick_cnt = 0;
        return;
    }

    /* 突加进行中 */
    if (thr->kick_cnt > 0) {
        thr->kick_cnt--;
        if (thr->kick_cnt == 0) {
            thr->internal = mapped;          /* 突加结束，降到映射值 */
        }
        /* 突加期间不变 (保持 THR_KICK_DUTY) */
        return;
    }

    /* 检测启动: 之前停止 → 现在有油门 */
    if (thr->internal == 0.0f) {
        thr->kick_cnt = THR_KICK_TICKS;
        thr->internal = THR_KICK_DUTY;       /* 突加启动 */
        return;
    }

    /* 已在转动: 平滑跟随映射值 */
    thr->internal = mapped;
}

/*******************************************************************************
 * 函数名    pwm_set_left_duty / pwm_set_right_duty
 * 描述      设置目标油门。实际输出由 motor_throttle_update 每帧管理。
 * 参数      duty    -1.0~1.0, 正=前进, 负=后退, 0=停止
 ******************************************************************************/
void pwm_set_left_duty(float duty)
{
    left_thr.target = duty;
}

void pwm_set_right_duty(float duty)
{
    right_thr.target = duty;
}

/*******************************************************************************
 * 函数名    pwm_throttle_tick
 * 描述      50Hz 调用: 更新油门状态机 + 输出到硬件
 ******************************************************************************/
void pwm_throttle_tick(void)
{
    motor_throttle_update(&left_thr);
    motor_throttle_update(&right_thr);

    pwm_set_motor_duty(
        MOTOR_LEFT_D0_PORT,  MOTOR_LEFT_D0_PIN,
        MOTOR_LEFT_D1_PORT,  MOTOR_LEFT_D1_PIN,
        left_thr.internal, &left_sw);

    pwm_set_motor_duty(
        MOTOR_RIGHT_D2_PORT,  MOTOR_RIGHT_D2_PIN,
        MOTOR_RIGHT_D3_PORT,  MOTOR_RIGHT_D3_PIN,
        right_thr.internal, &right_sw);
}
