/*******************************************************************************
 * 文件名          step_motor.c
 * 描述            步进电机驱动 — 传送带控制
 *                 3 线接口 (EN/DIR/STEP)，通过 TIMER6 @ 2kHz 中断产生步进脉冲
 *                 驱动板型号: 与 STEP 工程相同 (通用 3 线步进驱动模块)
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 硬件连接:
 *   PG3 (J4-57) → STEP  步进脉冲 (上升沿触发)
 *   PD4 (J4-22) → DIR   方向控制 (H=正向, L=反向)
 *   PH6 (J4-37) → EN    使能控制 (H=使能, L=禁用)
 *
 * 步进参数 (与驱动板 DIP 开关设置一致):
 *   步距角 1.8°, 1/16 微步 → 3200 步/圈
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-06-18      CIMC           初始版本，适配 GD32H7
 ******************************************************************************/

#include "step_motor.h"
#include "gd32_driver_pit.h"

/* ==================== 内部状态变量 ==================== */
static int8_t   g_dir        = 0;    /* 方向: 1=正转, -1=反转, 0=停止     */
static uint32_t g_speed_div  = STEPPER_DEFAULT_SPEED_DIV;  /* 速度分频  */
static uint32_t g_speed_cnt  = 0;    /* 中断计数器                       */
static int32_t  g_cur_step   = 0;    /* 绝对步数累计 (调试)              */
static uint8_t  g_step_phase = 0;    /* 脉冲相位: 0=低电平, 1=高电平    */

/* ==================== 初始化 ==================== */

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   Stepper_Init
// 描述:     初始化 GPIO + TIMER6 PIT
//---------------------------------------------------------------------------------------------------------------------
void Stepper_Init(void)
{
    /* ---- 1. 使能 GPIO 时钟 ---- */
    rcu_periph_clock_enable(STEPPER_RCU_STEP);  /* PG3 */
    rcu_periph_clock_enable(STEPPER_RCU_DIR);   /* PD4 */
    rcu_periph_clock_enable(STEPPER_RCU_EN);    /* PH6 */

    /* ---- 2. 配置 GPIO 为推挽输出 ---- */
    /* STEP 引脚 PG3 */
    gpio_mode_set(STEPPER_STEP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, STEPPER_STEP_PIN);
    gpio_output_options_set(STEPPER_STEP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, STEPPER_STEP_PIN);
    gpio_bit_reset(STEPPER_STEP_PORT, STEPPER_STEP_PIN);     /* 初始低电平 */

    /* DIR 引脚 PD4 */
    gpio_mode_set(STEPPER_DIR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, STEPPER_DIR_PIN);
    gpio_output_options_set(STEPPER_DIR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, STEPPER_DIR_PIN);
    gpio_bit_reset(STEPPER_DIR_PORT, STEPPER_DIR_PIN);       /* 初始正向 */

    /* EN 引脚 PH6 — 高电平使能 (与 STEP 工程一致) */
    gpio_mode_set(STEPPER_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, STEPPER_EN_PIN);
    gpio_output_options_set(STEPPER_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, STEPPER_EN_PIN);
    gpio_bit_set(STEPPER_EN_PORT, STEPPER_EN_PIN);            /* EN=H → 使能 */

    /* ---- 3. 初始化 TIMER6 为 500μs (2kHz) 中断 ---- */
    pit_us_init(PIT_TIMER6, 500);

    /* ---- 4. 复位内部状态 ---- */
    g_dir        = 0;
    g_speed_div  = STEPPER_DEFAULT_SPEED_DIV;
    g_speed_cnt  = 0;
    g_cur_step   = 0;
    g_step_phase = 0;
}

/* ==================== 速度控制 ==================== */

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   Stepper_SetSpeed
// 描述:     设置转速和方向
//           dir>0 正转 → DIR=H, dir<0 反转 → DIR=L, dir=0 停止 → 禁用 EN
//---------------------------------------------------------------------------------------------------------------------
void Stepper_SetSpeed(int8_t dir, uint32_t speed_div)
{
    int8_t norm_dir;

    if (dir == 0 || speed_div == 0)
    {
        /* 停止：方向置零，ISR 不产生脉冲 */
        g_dir = 0;
        return;
    }

    /* 限幅 speed_div */
    if (speed_div < STEPPER_MIN_SPEED_DIV)
        speed_div = STEPPER_MIN_SPEED_DIV;
    if (speed_div > STEPPER_MAX_SPEED_DIV)
        speed_div = STEPPER_MAX_SPEED_DIV;

    norm_dir = (dir > 0) ? 1 : -1;

    /* 参数未变 → 不重置计数器，保持脉冲序列连续 */
    if (g_dir == norm_dir && g_speed_div == speed_div)
        return;

    g_dir       = norm_dir;
    g_speed_div = speed_div;
    g_speed_cnt = 0;  /* 仅在参数变化时重置，使速度变更即时生效 */

    /* 设置方向引脚 */
    if (g_dir > 0)
        gpio_bit_set(STEPPER_DIR_PORT, STEPPER_DIR_PIN);    /* 正向: DIR=H */
    else
        gpio_bit_reset(STEPPER_DIR_PORT, STEPPER_DIR_PIN);  /* 反向: DIR=L */
}

/* ==================== 调试接口 ==================== */

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   Stepper_GetCurStep
// 描述:     返回累计步数（正转+1, 反转-1），用于调试
//---------------------------------------------------------------------------------------------------------------------
int32_t Stepper_GetCurStep(void)
{
    return g_cur_step;
}

/* ==================== 中断服务 ==================== */

//---------------------------------------------------------------------------------------------------------------------
// 函数名:   Stepper_Step_IRQHandler
// 描述:     TIMER6 中断回调 (2kHz)
//           通过软件分频控制步进速率，产生 STEP 脉冲
//           脉冲时序: 置 DIR → 等待 → STEP↑ → 等待 → STEP↓
//           当前简化: 每次中断只做一步 (SETUP_HOLD→PULSE→HOLD_OFF 在连续中断中完成)
//---------------------------------------------------------------------------------------------------------------------
void Stepper_Step_IRQHandler(void)
{
    /* 停止状态不产生脉冲 */
    if (g_dir == 0)
        return;

    /* 速度分频: 每 g_speed_div 次中断产生一个脉冲 */
    g_speed_cnt++;
    if (g_speed_cnt < g_speed_div)
        return;
    g_speed_cnt = 0;

    /*
     * 脉冲生成 (两阶段状态机):
     *   阶段0: STEP 当前为低 → 拉高 (上升沿触发驱动板)
     *   阶段1: STEP 当前为高 → 拉低 (准备下一次上升沿)
     * 每个中断周期只执行一个阶段，实现完整的 H→L 脉冲
     */
    if (g_step_phase == 0)
    {
        /* 上升沿: 驱动板在上升沿采样 DIR 并执行一步 */
        gpio_bit_set(STEPPER_STEP_PORT, STEPPER_STEP_PIN);
        g_step_phase = 1;

        /* 更新累计步数 */
        g_cur_step += (g_dir > 0) ? 1 : -1;
    }
    else
    {
        /* 下降沿: 复位 STEP 线，准备下一次脉冲 */
        gpio_bit_reset(STEPPER_STEP_PORT, STEPPER_STEP_PIN);
        g_step_phase = 0;
    }
}
