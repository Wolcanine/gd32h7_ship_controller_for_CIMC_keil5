// auto_nav.h — 自动航行模块接口
// 自动 / 手动由 PS2 手柄 mode 变量决定：
//   模拟模式 (0x73, 红灯亮) = 手动，数字模式 (0x41, 红灯灭) = 自动
// 自动模式：激光测距避障（直行 → 遇障转向 → 恢复直行）
#ifndef AUTO_NAV_H
#define AUTO_NAV_H

#include "gd32h7xx.h"

typedef enum {
    NAV_MODE_MANUAL = 0,
    NAV_MODE_AUTO   = 1
} NavMode;

typedef enum {
    NAV_STATE_FORWARD = 0,      // 直行
    NAV_STATE_TURNING = 1,      // 转向避障
} NavState;

/* ---------- 生命周期 ---------- */
void AutoNav_Init(void);
void AutoNav_Process(void);     // 每个控制周期调用一次

/* ---------- 模式 / 状态 查询 ---------- */
NavMode  AutoNav_GetMode(void);
NavState AutoNav_GetState(void);
float    AutoNav_GetYawDeg(void);      // 调试用：当前累计偏航角

/* ---------- 油门 / 转向设定值 ---------- */
float AutoNav_GetThrottle(void);
float AutoNav_GetSteering(void);

#endif
