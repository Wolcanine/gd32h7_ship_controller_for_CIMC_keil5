/*********************************************************************************************************************
* 水面垃圾收集船主控程序 — 6 舵机机械臂控制接口
* Copyright (c) 2025-2026
*
* 文件名称          servo_arm
* 目标平台          GD32H759IMK6
* 编译环境          Keil MDK5 (uVision5)
*
* 功能              通过 PCA9685 (I2C) 驱动 6 路舵机:
*                   - 安装偏移校准 (joint_offset)
*                   - PS2 遥控手动调关节
*                   - 逆运动学 XY → 关节角度
*                   - 缓冲平滑插值
*                   - PS2 START/SELECT 预设 XY 坐标
*
* 修改记录
* 日期                作者          备注
* 2026-05-07          AI助手        初始版本
* 2026-05-19          AI助手        机械臂标定 + 逆运动学
* 2026-05-21          CIMC          GD32F407→GD32H759 移植
* 2026-06-12          CIMC          移除 CAM_CMD 协议，新增视觉引导接口
* 2026-06-17          CIMC          重构校准；移除视觉依赖；270°舵机适配
*********************************************************************************************************************/

#ifndef SERVO_ARM_H
#define SERVO_ARM_H

#include "gd32h7xx.h"

/* ==================== 舵机通道映射 ==================== */
#define ARM_JOINT_COUNT  6

#define ARM_CH_BASE       0    /* 底座旋转 (270°舵机, 0~270°) */
#define ARM_CH_SHOULDER   1    /* 大臂 */
#define ARM_CH_ELBOW      2    /* 小臂 */
#define ARM_CH_WRIST_P    3    /* 腕部俯仰 */
#define ARM_CH_WRIST_R    4    /* 腕部旋转 */
#define ARM_CH_GRIPPER    5    /* 夹爪 */

/* ==================== 预设动作 ==================== */
typedef enum {
    ARM_PARK    = 0,    /* 收船 — 与上电默认一致                   */
    ARM_READY   = 1,    /* 准备 — 展开到预抓取姿态                 */
    ARM_COLLECT = 2,    /* 抓取 — 闭合夹爪 + 抬起                  */
    ARM_DROP    = 3,    /* 投放 — 伸到船舷外 + 张开夹爪             */
} ArmAction;

/* ==================== API ==================== */
void ServoArm_Init(void);                                 /* 初始化 PCA9685, 归默认位置   */
void ServoArm_SetAngle(uint8_t ch, float angle);          /* 单关节角度输出 (含偏移补偿)   */
void ServoArm_SetAction(ArmAction action);                /* 执行预设动作 (缓冲插值)       */
void ServoArm_RemoteControl(void);                        /* PS2 手动调关节 (~2.5°/s)     */
void ServoArm_HandlePresets(void);                        /* PS2 START/SELECT 预设 XY     */
void ServoArm_MoveToXY(float x, float y);                 /* IK 解算 → 缓冲移动到目标 XY  */
void ServoArm_SmoothUpdate(void);                         /* 缓冲插值推进 (50Hz 调用)     */
void ServoArm_CancelMove(void);                           /* 取消缓冲移动                 */
void ServoArm_PrintStatus(void);                          /* 串口打印 6 路角度 + 脉宽     */

#endif
