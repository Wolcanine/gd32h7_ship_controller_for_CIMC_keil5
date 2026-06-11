/*******************************************************************************
 * 文件名          MPU6050.c
 * 描述            MPU6050 六轴传感器驱动 — I2C 接口
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

#include "MyI2C.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "systick.h"
#include <stdio.h>

//-------------------------------------------------------------------------------------------------------------------
// 模块内部静态变量
//-------------------------------------------------------------------------------------------------------------------

// 灵敏度跟踪 —— 由 MPU6050_HardwareConfig() 根据写入硬件的量程配置同步更新
static uint16_t Accel_LSB_G = 16384;    // 默认 ±2g → 16384 LSB/g
static uint16_t Gyro_LSB_DPS = 131;     // 默认 ±250dps → 131 LSB/°/s

// 陀螺仪三轴零偏值（原始 ADC 计数值），由 MPU6050_CalibrateGyro() 计算得到
static int16_t GyroBiasX = 0;
static int16_t GyroBiasY = 0;
static int16_t GyroBiasZ = 0;

// 健康标志：init 成功置 1，连续 MPU6050_MAX_CONSECUTIVE_FAIL 次 Update 失败清 0
#define MPU6050_MAX_CONSECUTIVE_FAIL  10
static uint8_t mpu6050_healthy = 0;
static uint8_t consecutive_fail = 0;


//================================================== I2C 通信底层 ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     CheckAck
// 函数说明     检查 I2C 从机应答 —— 收到 NACK 时立即停止总线并返回错误
//              本函数为 MPU6050_WriteReg / ReadReg / ReadRegs 的内部共用函数
// 返回值       MPU6050_OK = 从机应答正常 (ACK)
//             MPU6050_ERROR = 从机无应答 (NACK)，可能原因：器件离线、地址错误、总线短路
//-------------------------------------------------------------------------------------------------------------------
static MPU6050_Status CheckAck(void)
{
    if(MyI2C_ReceiveAck() != 0U) {
        MyI2C_Stop();                   // 收到 NACK → 立即释放总线
        return MPU6050_ERROR;
    }
    return MPU6050_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_WriteReg
// 函数说明     通过 I2C 向 MPU6050 指定寄存器写入一字节数据
// 参数         RegAddress      目标寄存器地址
// 参数         Data            待写入的 8 位数据
// 返回值       MPU6050_OK / MPU6050_ERROR（任一步骤收到 NACK 即返回错误）
// 使用示例     MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
//-------------------------------------------------------------------------------------------------------------------
MPU6050_Status MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MPU6050_Status status;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);    // 发送器件地址 + 写方向位 (bit0 = 0)
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    MyI2C_SendByte(RegAddress);         // 发送目标寄存器地址
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    MyI2C_SendByte(Data);               // 发送待写入数据
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    MyI2C_Stop();
    return MPU6050_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_ReadReg
// 函数说明     通过 I2C 从 MPU6050 指定寄存器读取一字节数据
//              使用 "写寄存器地址 → 重复起始 → 读数据" 的 I2C 复合格式
// 参数         RegAddress      目标寄存器地址
// 参数         Data            读出数据存放指针
// 返回值       MPU6050_OK / MPU6050_ERROR
// 使用示例     uint8_t id;  MPU6050_ReadReg(MPU6050_WHO_AM_I, &id);
//-------------------------------------------------------------------------------------------------------------------
MPU6050_Status MPU6050_ReadReg(uint8_t RegAddress, uint8_t *Data)
{
    MPU6050_Status status;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);    // 发送器件地址 + 写方向位
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    MyI2C_SendByte(RegAddress);         // 发送目标寄存器地址
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    MyI2C_Start();                      // 重复起始条件（不释放总线）

    MyI2C_SendByte(MPU6050_ADDRESS | 0x01); // 发送器件地址 + 读方向位 (bit0 = 1)
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    *Data = MyI2C_ReceiveByte();        // 接收一字节数据
    MyI2C_SendAck(1);                   // 主机发送 NACK，通知从机停止发送

    MyI2C_Stop();
    return MPU6050_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_ReadRegs
// 函数说明     从 MPU6050 指定起始寄存器连续读取多个字节（地址自动递增）
//              用于一次性读取加速度、温度、陀螺仪等连续寄存器块
// 参数         RegAddress      起始寄存器地址
// 参数         Data            数据缓冲区指针（调用者需确保缓冲区足够大）
// 参数         Length          要读取的字节数
// 返回值       MPU6050_OK / MPU6050_ERROR
// 使用示例     uint8_t buf[14];  MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, 14);
//-------------------------------------------------------------------------------------------------------------------
MPU6050_Status MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length)
{
    uint8_t i;
    MPU6050_Status status;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);    // 发送器件地址 + 写方向位
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    MyI2C_SendByte(RegAddress);         // 发送起始寄存器地址
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    MyI2C_Start();                      // 重复起始条件

    MyI2C_SendByte(MPU6050_ADDRESS | 0x01); // 发送器件地址 + 读方向位
    status = CheckAck();
    if(status != MPU6050_OK) return status;

    for(i = 0; i < Length; i++) {
        Data[i] = MyI2C_ReceiveByte();  // 逐个字节接收
        if(i == Length - 1) {
            MyI2C_SendAck(1);           // 最后一个字节 → 主机发 NACK，结束传输
        } else {
            MyI2C_SendAck(0);           // 非最后字节 → 主机发 ACK，通知从机继续
        }
    }

    MyI2C_Stop();
    return MPU6050_OK;
}


//================================================== 硬件寄存器配置（内部使用） ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_HardwareConfig
// 函数说明     配置 MPU6050 硬件寄存器：复位 → 唤醒 → 时钟源 → 采样率 → 低通滤波 → 量程
//              同时更新灵敏度跟踪变量 Accel_LSB_G / Gyro_LSB_DPS
// 返回值       MPU6050_OK / MPU6050_ERROR
// 备注信息     如需更改量程，修改本函数中 GYRO_CONFIG 和 ACCEL_CONFIG 的写入值即可，
//              灵敏度变量会自动同步，换算函数无需手动修改
//-------------------------------------------------------------------------------------------------------------------
static MPU6050_Status MPU6050_HardwareConfig(void)
{
    MPU6050_Status status;

    // 复位 MPU6050（bit7 = 1 触发复位，硬件自动清除）
    status = MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
    if(status != MPU6050_OK) return status;
    delay_1ms(100);

    // 唤醒器件，选择 X 轴陀螺仪 PLL 作为时钟源（比内部 RC 振荡器更精确）
    status = MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    if(status != MPU6050_OK) return status;
    delay_1ms(10);

    // 开启加速度计和陀螺仪的全部三轴（写 0x00 表示全部使能）
    status = MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    if(status != MPU6050_OK) return status;

    // 采样率分频：输出频率 = 1kHz / (1 + 9) = 100 Hz
    status = MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    if(status != MPU6050_OK) return status;

    // 数字低通滤波 DLPF_CFG = 6 → 加速度带宽 5Hz，陀螺仪带宽 5Hz
    // 值越大截止频率越低、输出越平滑但响应越慢
    status = MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    if(status != MPU6050_OK) return status;

    // 陀螺仪量程 FS_SEL = 0x00 → ±250 °/s，灵敏度 131 LSB/(°/s)
    status = MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);
    if(status != MPU6050_OK) return status;
    Gyro_LSB_DPS = 131;                 // 同步更新灵敏度变量

    // 加速度量程 AFS_SEL = 0x00 → ±2g，灵敏度 16384 LSB/g
    status = MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
    if(status != MPU6050_OK) return status;
    Accel_LSB_G = 16384;                // 同步更新灵敏度变量

    return MPU6050_OK;
}


//================================================== 高层 API ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_Init
// 函数说明     完整初始化流程，一次性完成所有传感器准备工作
//              流程：I2C 初始化 → 硬件寄存器配置 → WHO_AM_I 校验 → 陀螺仪零偏校准 → 低通滤波初始化
// 返回值       MPU6050_OK    传感器已就绪，可以开始读取数据
//             MPU6050_ERROR  通信失败（I2C NACK）或 WHO_AM_I 校验失败（ID 不匹配）
// 使用示例     if(MPU6050_Init() != MPU6050_OK) { printf("Init failed\r\n"); while(1); }
// 备注信息     调用前必须确保 MPU6050 已正确上电且接线无误
//              校准期间模块必须保持完全静止，否则零偏值不准确
//-------------------------------------------------------------------------------------------------------------------
MPU6050_Status MPU6050_Init(void)
{
    uint8_t id;

    MyI2C_Init();                       // 初始化 I2C GPIO (PB10/PB11 开漏输出)
    delay_1ms(100);

    // ---- 步骤 1：配置硬件寄存器 ----
    if(MPU6050_HardwareConfig() != MPU6050_OK) {
        printf("MPU6050: hardware config failed (I2C NACK)\r\n");
        return MPU6050_ERROR;
    }

    // ---- 步骤 2：验证 WHO_AM_I 器件 ID ----
    if(MPU6050_GetID(&id) != MPU6050_OK) {
        printf("MPU6050: read WHO_AM_I failed (I2C NACK)\r\n");
        return MPU6050_ERROR;
    }

    printf("MPU6050 ID = 0x%02X\r\n", id);

    if(id != 0x68) {
        // ID 不匹配：可能是接线问题 (VCC/GND/SCL/SDA/AD0) 或芯片损坏
        printf("MPU6050 ERROR: unexpected ID (expected 0x68)\r\n");
        printf("Check wiring: VCC 3.3V, GND GND, SCL PB10, SDA PB11, AD0 GND\r\n");
        return MPU6050_ERROR;
    }

    // ---- 步骤 3：陀螺仪零偏校准（采集 200 帧 ≈ 1 秒） ----
    printf("MPU6050 OK, calibrating gyro... keep device still\r\n");
    MPU6050_CalibrateGyro(200);
    {
        int16_t bx, by, bz;
        MPU6050_GetGyroBias(&bx, &by, &bz);
        printf("Gyro bias (raw): X=%d, Y=%d, Z=%d\r\n", bx, by, bz);
        printf("Gyro bias (dps): X=%ld.%02ld, Y=%ld.%02ld, Z=%ld.%02ld\r\n",
               (long)(MPU6050_GyroToDps100(bx) / 100),
               (long)((MPU6050_GyroToDps100(bx) >= 0 ? MPU6050_GyroToDps100(bx) : -MPU6050_GyroToDps100(bx)) % 100),
               (long)(MPU6050_GyroToDps100(by) / 100),
               (long)((MPU6050_GyroToDps100(by) >= 0 ? MPU6050_GyroToDps100(by) : -MPU6050_GyroToDps100(by)) % 100),
               (long)(MPU6050_GyroToDps100(bz) / 100),
               (long)((MPU6050_GyroToDps100(bz) >= 0 ? MPU6050_GyroToDps100(bz) : -MPU6050_GyroToDps100(bz)) % 100));
    }

    // ---- 步骤 4：初始化一阶低通滤波器（alpha=0.3 兼顾平滑度与响应速度） ----
    MPU6050_FilterInit(0.3f);

    printf("Calibration done. Send 'c' via serial to re-calibrate\r\n\r\n");
    mpu6050_healthy = 1;
    consecutive_fail = 0;
    return MPU6050_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_Recalibrate
// 函数说明     陀螺仪重新校准 —— 串口收到字符 'c' 时由 main.c 调用
//              重新采集 200 帧静止数据，更新三轴零偏值并打印结果
// 返回值       void
// 使用示例     if(ch == 'c') MPU6050_Recalibrate();
// 备注信息     调用前必须确保模块完全静止，且本次校准只更新零偏，不重新配置滤波
//-------------------------------------------------------------------------------------------------------------------
void MPU6050_Recalibrate(void)
{
    printf("\r\nRe-calibrating gyro... keep device still\r\n");
    MPU6050_CalibrateGyro(200);

    int16_t bx, by, bz;
    MPU6050_GetGyroBias(&bx, &by, &bz);
    printf("Gyro bias (raw): X=%d, Y=%d, Z=%d\r\n", bx, by, bz);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_GetID
// 函数说明     读取 MPU6050 WHO_AM_I 寄存器（地址 0x75）
// 参数         ID          存放读出 ID 的指针
// 返回值       MPU6050_OK / MPU6050_ERROR
// 使用示例     uint8_t id;  MPU6050_GetID(&id);  // 正常返回值为 0x68
//-------------------------------------------------------------------------------------------------------------------
MPU6050_Status MPU6050_GetID(uint8_t *ID)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I, ID);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_GetData
// 函数说明     一次性从 MPU6050 读取全部 14 字节数据（加速度×6 + 温度×2 + 陀螺仪×6）
//              陀螺仪原始值自动减去已标定的零偏量
// 参数         Data        存放读出数据的结构体指针
// 返回值       MPU6050_OK / MPU6050_ERROR（I2C 通信失败时返回错误）
// 使用示例     MPU6050_Data_t data;  MPU6050_GetData(&data);
// 备注信息     读取寄存器块：ACCEL_XOUT_H(0x3B) 起连续 14 字节
//-------------------------------------------------------------------------------------------------------------------
MPU6050_Status MPU6050_GetData(MPU6050_Data_t *Data)
{
    uint8_t Buffer[14];
    MPU6050_Status status;

    // 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 字节
    status = MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Buffer, 14);
    if(status != MPU6050_OK) return status;

    // 拼接高/低字节为大端序 16 位有符号整数
    Data->AccX  = (int16_t)((Buffer[0]  << 8) | Buffer[1]);
    Data->AccY  = (int16_t)((Buffer[2]  << 8) | Buffer[3]);
    Data->AccZ  = (int16_t)((Buffer[4]  << 8) | Buffer[5]);
    Data->Temp  = (int16_t)((Buffer[6]  << 8) | Buffer[7]);

    // 陀螺仪原始值减去标定好的零偏（静止时输出趋近于零）
    Data->GyroX = (int16_t)((Buffer[8]  << 8) | Buffer[9])  - GyroBiasX;
    Data->GyroY = (int16_t)((Buffer[10] << 8) | Buffer[11]) - GyroBiasY;
    Data->GyroZ = (int16_t)((Buffer[12] << 8) | Buffer[13]) - GyroBiasZ;

    return MPU6050_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_CalibrateGyro
// 函数说明     陀螺仪零偏校准算法：采集若干帧静止数据，各轴分别取算术平均作为零偏值
//              校准完成后，后续所有 GetData 调用会自动减去该零偏
// 参数         samples     采集帧数，建议 100~200（耗时约 samples × 5ms）
// 返回值       void
// 使用示例     MPU6050_CalibrateGyro(200);  // 采集 200 帧，约 1 秒
// 备注信息     调用前模块必须处于完全静止状态，否则校准值不准、后续数据会引入常值误差
//-------------------------------------------------------------------------------------------------------------------
void MPU6050_CalibrateGyro(uint8_t samples)
{
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    uint8_t i;

    for(i = 0; i < samples; i++) {
        uint8_t buf[6];
        MPU6050_ReadRegs(MPU6050_GYRO_XOUT_H, buf, 6);     // 直接读取陀螺仪（不经零偏修正）
        sumX += (int16_t)((buf[0] << 8) | buf[1]);
        sumY += (int16_t)((buf[2] << 8) | buf[3]);
        sumZ += (int16_t)((buf[4] << 8) | buf[5]);
        delay_1ms(5);                                       // 5ms 间隔，避免连续读取过快
    }

    // 算术平均 → 零偏值
    GyroBiasX = (int16_t)(sumX / samples);
    GyroBiasY = (int16_t)(sumY / samples);
    GyroBiasZ = (int16_t)(sumZ / samples);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_GetGyroBias
// 函数说明     获取当前已标定的陀螺仪三轴零偏值（原始 ADC 计数值）
//              可用于调试、验证校准效果，或将零偏值保存到 Flash 以便下次开机直接加载
// 参数         x, y, z     分别存放 X/Y/Z 轴零偏值的指针
// 返回值       void
//-------------------------------------------------------------------------------------------------------------------
void MPU6050_GetGyroBias(int16_t *x, int16_t *y, int16_t *z)
{
    *x = GyroBiasX;
    *y = GyroBiasY;
    *z = GyroBiasZ;
}


//================================================== 一阶低通滤波（指数移动平均） ==================================================
//   filtered = alpha × raw + (1 - alpha) × filtered_prev
//   alpha 越接近 1 越灵敏（噪声保留更多），越接近 0 越平滑（但响应变慢）
//   当前默认 alpha = 0.3，适合大多数使用场景

static MPU6050_Data_t FilterPrev;       // 上一帧滤波后的值
static float FilterAlpha;               // 平滑系数 (0 ~ 1)
static uint8_t FilterReady = 0;         // 首帧标记：第一帧不滤波，直接初始化

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_FilterInit
// 函数说明     初始化一阶低通滤波器 —— 设置平滑系数，复位首帧标记
// 参数         alpha       平滑系数，取值范围 0~1
//                         0.1 = 极平滑（响应很慢），0.5 = 中等，0.9 = 接近原始值
// 返回值       void
// 使用示例     MPU6050_FilterInit(0.3f);  // 默认值，兼顾平滑与响应速度
//-------------------------------------------------------------------------------------------------------------------
void MPU6050_FilterInit(float alpha)
{
    FilterAlpha = alpha;
    FilterReady = 0;                    // 标记为首帧，下次 FilterApply 将直接赋值
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_FilterApply
// 函数说明     对六轴数据（加速度 + 陀螺仪）应用一阶低通滤波，原地修改数据
//              在 MPU6050_GetData() 之后、物理量换算之前调用
// 参数         Data        待滤波数据结构体指针（原地修改，输入输出共用）
// 返回值       void
// 使用示例     MPU6050_FilterApply(&data);
// 备注信息     温度数据不参与滤波（温度变化极其缓慢，滤波无意义）
//-------------------------------------------------------------------------------------------------------------------
void MPU6050_FilterApply(MPU6050_Data_t *Data)
{
    float a = FilterAlpha;
    float b = 1.0f - a;

    if(!FilterReady) {
        // 首帧数据直接作为初始值，不做滤波（避免从零开始收敛）
        FilterPrev = *Data;
        FilterReady = 1;
        return;
    }

    // filtered = alpha × raw + (1 - alpha) × prevFiltered
    Data->AccX  = (int16_t)(a * Data->AccX  + b * FilterPrev.AccX);
    Data->AccY  = (int16_t)(a * Data->AccY  + b * FilterPrev.AccY);
    Data->AccZ  = (int16_t)(a * Data->AccZ  + b * FilterPrev.AccZ);
    Data->GyroX = (int16_t)(a * Data->GyroX + b * FilterPrev.GyroX);
    Data->GyroY = (int16_t)(a * Data->GyroY + b * FilterPrev.GyroY);
    Data->GyroZ = (int16_t)(a * Data->GyroZ + b * FilterPrev.GyroZ);

    FilterPrev = *Data;                 // 更新上一帧值，供下次滤波使用
}


//================================================== 数据缓存 & 对外读取接口 ==================================================
//  调用 MPU6050_Update() 后数据存入缓存，外部模块通过各 Get 函数独立获取

static int32_t CachedAccelX_mg = 0;     // 加速度 X 轴，单位 mg
static int32_t CachedAccelY_mg = 0;     // 加速度 Y 轴，单位 mg
static int32_t CachedAccelZ_mg = 0;     // 加速度 Z 轴，单位 mg
static int32_t CachedGyroX_dps100 = 0;  // 陀螺仪 X 轴，单位 0.01 °/s
static int32_t CachedGyroY_dps100 = 0;  // 陀螺仪 Y 轴，单位 0.01 °/s
static int32_t CachedGyroZ_dps100 = 0;  // 陀螺仪 Z 轴，单位 0.01 °/s
static int32_t CachedTemp_c100 = 0;     // 温度，单位 0.01 °C

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_Update
// 函数说明     执行一次完整的传感器数据采集流程：
//              GetData（读原始值）→ FilterApply（低通滤波）→ 物理量换算 → 写入缓存
//              之后外部模块可通过 MPU6050_GetAccelX_mg() 等函数独立获取各值
// 返回值       MPU6050_OK    数据更新成功，缓存值已刷新
//             MPU6050_ERROR  I2C 通信失败，缓存保持上次有效值不变
// 使用示例     if(MPU6050_Update() == MPU6050_OK) { x = MPU6050_GetAccelX_mg(); }
// 备注信息     建议以固定频率（如 100Hz = 每 10ms）在主循环中调用
//-------------------------------------------------------------------------------------------------------------------
MPU6050_Status MPU6050_Update(void)
{
    MPU6050_Data_t data;
    MPU6050_Status status;

    // 读取原始数据
    status = MPU6050_GetData(&data);
    if(status != MPU6050_OK) {
        consecutive_fail++;
        if (consecutive_fail >= MPU6050_MAX_CONSECUTIVE_FAIL) {
            mpu6050_healthy = 0;
        }
        return status;
    }
    consecutive_fail = 0;

    // 低通滤波
    MPU6050_FilterApply(&data);

    // 换算为物理单位并写入缓存
    CachedAccelX_mg    = MPU6050_AccelToMg(data.AccX);
    CachedAccelY_mg    = MPU6050_AccelToMg(data.AccY);
    CachedAccelZ_mg    = MPU6050_AccelToMg(data.AccZ);
    CachedGyroX_dps100 = MPU6050_GyroToDps100(data.GyroX);
    CachedGyroY_dps100 = MPU6050_GyroToDps100(data.GyroY);
    CachedGyroZ_dps100 = MPU6050_GyroToDps100(data.GyroZ);
    CachedTemp_c100    = MPU6050_TempToC100(data.Temp);

    return MPU6050_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 加速度 getter 函数 —— 返回最近一次 Update 后缓存的值
//-------------------------------------------------------------------------------------------------------------------
int32_t MPU6050_GetAccelX_mg(void) { return CachedAccelX_mg; }
int32_t MPU6050_GetAccelY_mg(void) { return CachedAccelY_mg; }
int32_t MPU6050_GetAccelZ_mg(void) { return CachedAccelZ_mg; }

//-------------------------------------------------------------------------------------------------------------------
// 陀螺仪 getter 函数 —— 返回最近一次 Update 后缓存的值（已减零偏）
//-------------------------------------------------------------------------------------------------------------------
int32_t MPU6050_GetGyroX_dps100(void) { return CachedGyroX_dps100; }
int32_t MPU6050_GetGyroY_dps100(void) { return CachedGyroY_dps100; }
int32_t MPU6050_GetGyroZ_dps100(void) { return CachedGyroZ_dps100; }

//-------------------------------------------------------------------------------------------------------------------
// 温度 getter 函数 —— 返回最近一次 Update 后缓存的值
//-------------------------------------------------------------------------------------------------------------------
int32_t MPU6050_GetTemp_c100(void) { return CachedTemp_c100; }

uint8_t MPU6050_IsHealthy(void) { return mpu6050_healthy; }


//================================================== 物理量换算 ==================================================
//  换算系数由 MPU6050_HardwareConfig() 中配置的量程自动决定
//  修改量程后系数会自动同步，无需手动修改以下函数

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_AccelToMg
// 函数说明     将加速度原始 ADC 值换算为毫克 (mg)，系数与当前量程自动匹配
// 参数         raw         加速度原始值（带符号 int16_t）
// 返回值       换算后的加速度值，单位 mg（例如 1000 = 1g 重力加速度）
// 使用示例     int32_t mg = MPU6050_AccelToMg(data.AccX);  // Z 轴静止时约 1000mg
//-------------------------------------------------------------------------------------------------------------------
int32_t MPU6050_AccelToMg(int16_t raw)
{
    return (int32_t)raw * 1000 / (int32_t)Accel_LSB_G;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_GyroToDps100
// 函数说明     将陀螺仪原始 ADC 值换算为 0.01 °/s（返回值 ÷ 100 = 实际 °/s）
// 参数         raw         陀螺仪原始值（已减零偏，带符号 int16_t）
// 返回值       换算后的角速度值，单位 0.01 °/s
// 使用示例     int32_t val = MPU6050_GyroToDps100(data.GyroX);
//             printf("GyroX=%ld.%02ld dps", val/100, val%100);
//-------------------------------------------------------------------------------------------------------------------
int32_t MPU6050_GyroToDps100(int16_t raw)
{
    return (int32_t)raw * 100 / (int32_t)Gyro_LSB_DPS;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     MPU6050_TempToC100
// 函数说明     将温度传感器原始 ADC 值换算为 0.01 °C（返回值 ÷ 100 = 实际 °C）
// 参数         raw         温度传感器原始值
// 返回值       换算后的温度值，单位 0.01 °C
// 使用示例     int32_t val = MPU6050_TempToC100(data.Temp);
//             printf("Temp=%ld.%02ld C", val/100, val%100);
// 备注信息     计算公式：Temperature(°C) = raw / 340 + 36.53
//              此值为芯片内部温度，比环境温度高 2~5°C 属正常现象
//-------------------------------------------------------------------------------------------------------------------
int32_t MPU6050_TempToC100(int16_t raw)
{
    return (int32_t)raw * 100 / 340 + 3653;
}
