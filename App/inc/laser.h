// laser.h — 激光测距模块接口
// 当前为占位实现，接入实际硬件后替换 laser.c 即可
#ifndef LASER_H
#define LASER_H

#include "gd32h7xx.h"

#define LASER_READ_ERROR   0xFFFF      // 读错误返回值
#define LASER_INVALID      0           // 无效读数

void     Laser_Init(void);             // 初始化激光传感器（GPIO/UART/I2C 等）
uint16_t Laser_GetDistanceCm(void);    // 返回距离 (cm)，出错返回 LASER_READ_ERROR

#endif
