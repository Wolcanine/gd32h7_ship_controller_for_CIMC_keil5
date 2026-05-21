// servo_arm.h — 6 舵机机械臂控制接口
// 通过 PCA9685 (I2C) 驱动，用于水面垃圾收集
// 接收摄像头识别板的串口指令，执行抓取/投放动作
#ifndef SERVO_ARM_H
#define SERVO_ARM_H

#include "gd32h7xx.h"

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

// ==================== 摄像头协议常量 ====================
// 摄像头板通过 UART_CAM (USART2) 发送 1 字节指令
// Bit[7]: 是否启用（0x80）
// Bit[2:0]: 动作类型
#define CAM_CMD_MASK     0x87
#define CAM_CMD_ACTION   0x07    // 低 3 位 = action (ArmAction)
#define CAM_CMD_ENABLE   0x80    // bit7 = 1 表示有效指令

// ==================== API ====================
void ServoArm_Init(void);
void ServoArm_SetAngle(uint8_t ch, float angle);        // 0~180°，直接输出
void ServoArm_SetAction(ArmAction action);               // 执行预设动作（缓冲移动）
void ServoArm_ProcessCmd(uint8_t cmd);                   // 处理摄像头协议指令
void ServoArm_RemoteControl(void);                       // PS2 遥控单关节调节
void ServoArm_MoveToXY(float x, float y);                // XY 坐标逆运动学（缓冲移动）
void ServoArm_SmoothUpdate(void);                        // 50Hz 插值推进（主循环调用）
void ServoArm_CancelMove(void);                          // 取消缓冲移动

#endif
