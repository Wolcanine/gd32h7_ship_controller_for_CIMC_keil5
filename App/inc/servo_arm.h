/*********************************************************************************************************************
* 水面垃圾收集船主控程序 — 6 舵机机械臂控制接口
* Copyright (c) 2025-2026
*
* 本文件是水面垃圾收集船主控程序的一部分
*
* 文件名称          servo_arm
* 目标平台          GD32H759IMK6
* 编译环境          Keil MDK5 (uVision5)
*
* 功能              通过 PCA9685 (I2C) 驱动 6 路舵机，执行抓取/投放/巡边等动作
*
* 架构说明
*                   原方案通过 UART_CAM 接收摄像头板串口指令，现已废弃
*                   现方案为同板视觉模块 (vision.h) 直接输出目标坐标，
*                   机械臂通过 ServoArm_CollectTarget() 接收视觉引导的抓取坐标
*
* 修改记录
* 日期                作者          备注
* 2026-05-07          AI助手        初始版本
* 2026-05-19          AI助手        机械臂标定 + 逆运动学
* 2026-05-21          CIMC          GD32F407→GD32H759 移植
* 2026-06-12          CIMC          移除 CAM_CMD 板间协议，新增视觉引导接口
*********************************************************************************************************************/

#ifndef SERVO_ARM_H
#define SERVO_ARM_H

#include "gd32h7xx.h"
#include "vision.h"

// ==================== 舵机通道映射 ====================
#define ARM_JOINT_COUNT 6       // 关节总数

#define ARM_CH_BASE     0       // 底座旋转 (0~180°)
#define ARM_CH_SHOULDER 1       // 大臂
#define ARM_CH_ELBOW    2       // 小臂
#define ARM_CH_WRIST_P  3       // 腕部俯仰
#define ARM_CH_WRIST_R  4       // 腕部旋转
#define ARM_CH_GRIPPER  5       // 夹爪

// ==================== 预设动作 ====================
typedef enum {
    ARM_PARK    = 0,    // 待机/收船 — 折叠收回，整体降低重心
    ARM_READY   = 1,    // 准备 — 展开到预抓取姿态
    ARM_COLLECT = 2,    // 抓取 — 闭合夹爪 + 抬起
    ARM_DROP    = 3,    // 投放 — 伸到船舷外 + 张开夹爪
} ArmAction;

// ==================== API ====================
void ServoArm_Init(void);
void ServoArm_SetAngle(uint8_t ch, float angle);        // 0~180°，直接输出
void ServoArm_SetAction(ArmAction action);               // 执行预设动作（缓冲移动）
void ServoArm_RemoteControl(void);                       // PS2 遥控单关节调节
void ServoArm_MoveToXY(float x, float y);                // XY 坐标逆运动学（缓冲移动）
void ServoArm_SmoothUpdate(void);                        // 50Hz 插值推进（主循环调用）
void ServoArm_CancelMove(void);                          // 取消缓冲移动

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     ServoArm_CollectTarget
// 函数说明     根据视觉检测目标坐标，执行抓取动作
//             结合逆运动学将夹爪移动到目标上方 → 闭合夹爪 → 抬起到投放姿态
// 参数         target      视觉检测目标（来自 Vision_GetClosestTarget 等）
// 返回值       void
// 使用示例     VisionTarget t = Vision_GetClosestTarget();
//              if (t.confidence > 50) { ServoArm_CollectTarget(&t); }
// 备注信息     TODO: 待视觉接口数据稳定后实现完整的抓取序列
//-------------------------------------------------------------------------------------------------------------------
void ServoArm_CollectTarget(const VisionTarget *target);

#endif
