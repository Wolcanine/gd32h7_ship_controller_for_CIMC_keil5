/*******************************************************************************
 * 文件名          laser.c
 * 描述            TOF200F 激光测距模块 — Modbus-RTU 主动轮询 + 流式帧解析 + CRC 校验
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 通信接口        UART3, PA0-TX / PA1-RX, AF8, 115200 8N1
 * 查询方式        主机主动轮询 (200ms 间隔)
 * 协议格式        Modbus-RTU: 地址(1B) + 功能码(1B) + 数据(NB) + CRC16(2B)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-06      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 * 2026-06-10      CIMC            TOF 串口改为 UART3 PA0/PA1 (原 USART1 通信失败)
 * 2026-06-10      CIMC            精简调试输出，仅保留异常告警
 ******************************************************************************/

#include "laser.h"
#include "uart_driver.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

//================================================== 协议常量 ==================================================

#define TOF_ADDR            0x01U       // TOF200F 模块 Modbus 地址
#define FRAME_TIMEOUT_MS    10U         // 帧间隔超时 (ms)，超时后丢弃不完整帧
#define QUERY_INTERVAL_MS   200U        // 主动查询间隔 (ms)
#define RX_FRAME_MAX        64U         // 接收帧缓冲区最大字节数

//================================================== Modbus 命令 ==================================================

// Modbus-RTU 预计算命令（含 CRC16 校验），无需运行时计算
static const uint8_t CMD_QUERY_DISTANCE[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};   // 读距离寄存器
static const uint8_t CMD_HANDSHAKE[]      = {0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD8, 0x0A};   // 握手 (写寄存器 0x0001=0)
static const uint8_t CMD_HIGH_PRECISION[] = {0x01, 0x06, 0x00, 0x04, 0x00, 0x01, 0x09, 0xCB};   // 高精度模式 (写寄存器 0x0004=1)

//================================================== 模块内部状态 ==================================================

static uint16_t last_distance_mm = 0xFFFFU;     // 最近一次有效距离值 (mm)，0xFFFF = 无效
static uint8_t  rx_frame[RX_FRAME_MAX];          // 接收帧拼接缓冲区
static uint32_t rx_len = 0;                      // 当前帧缓冲区中已有字节数
static uint32_t last_rx_ms = 0;                  // 最后一次收到字节的时间戳 (ms)
static uint32_t last_query_ms = 0;               // 最后一次发送查询的时间戳 (ms)
static uint32_t query_count = 0;                 // 查询计数器 (调试用)
static uint32_t frame_count = 0;                 // 成功帧计数器 (调试用)
static uint32_t crc_error_count = 0;             // CRC 错误计数器

//================================================== Modbus CRC16 校验 ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     modbus_crc16
// 函数说明     计算 Modbus-RTU CRC16 校验值（多项式 0xA001，初始值 0xFFFF）
// 参数         data        待计算数据的指针
// 参数         len         数据字节数 (不含 CRC 字段本身)
// 返回值       uint16_t    16 位 CRC 校验值 (低字节在前)
// 使用示例     uint16_t crc = modbus_crc16(buf, 6);  // 计算前 6 字节的 CRC
//-------------------------------------------------------------------------------------------------------------------
static uint16_t modbus_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;

    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8U; i++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (crc >> 1) ^ 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

//================================================== 帧缓冲管理 ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     consume_frame_bytes
// 函数说明     从接收帧缓冲区头部丢弃 n 个字节（将剩余数据前移）
//              用于解析完成一帧或发现无效字节后清理缓冲区
// 参数         n           要丢弃的字节数；若 n >= rx_len 则清空整个缓冲区
// 返回值       void
//-------------------------------------------------------------------------------------------------------------------
static void consume_frame_bytes(uint32_t n)
{
    if (n >= rx_len) {
        rx_len = 0;
        return;
    }

    memmove(rx_frame, &rx_frame[n], rx_len - n);
    rx_len -= n;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     check_frame_crc
// 函数说明     校验单帧 Modbus 数据的 CRC16
// 参数         frame       完整帧数据指针（包含 CRC 字段）
// 参数         len         完整帧总长度（含 2 字节 CRC）
// 返回值       uint8_t     1 = CRC 校验通过, 0 = CRC 错误
// 备注信息     CRC 计算结果与帧末尾 2 字节（低字节在前）对比
//-------------------------------------------------------------------------------------------------------------------
static uint8_t check_frame_crc(const uint8_t *frame, uint32_t len)
{
    uint16_t calc = modbus_crc16(frame, len - 2U);
    uint16_t recv = (uint16_t)frame[len - 2U] | ((uint16_t)frame[len - 1U] << 8);
    return (calc == recv) ? 1U : 0U;
}

//================================================== 帧处理（按功能码分类） ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     handle_read_response
// 函数说明     处理功能码 0x03 的响应帧（读寄存器响应）
//              校验 CRC → 提取距离值（寄存器 0x0010 的 2 字节数据，单位 mm）
// 参数         frame_len   完整帧长度（含 CRC）
// 返回值       void
// 备注信息     CRC 错误时仅递增错误计数，不更新距离值；正常解析后存入 last_distance_mm
//-------------------------------------------------------------------------------------------------------------------
static void handle_read_response(uint32_t frame_len)
{
    if (!check_frame_crc(rx_frame, frame_len)) {
        crc_error_count++;
        printf("[Laser CRC ERR] count=%lu\r\n", (unsigned long)crc_error_count);
        return;
    }

    frame_count++;
    if (rx_frame[2] >= 2U) {
        last_distance_mm = ((uint16_t)rx_frame[3] << 8) | rx_frame[4];
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     handle_write_ack
// 函数说明     处理功能码 0x06 的确认帧（写寄存器响应）
//              仅校验 CRC，正常情况下不做额外处理
// 参数         frame_len   完整帧长度 (固定 8 字节)
// 返回值       void
//-------------------------------------------------------------------------------------------------------------------
static void handle_write_ack(uint32_t frame_len)
{
    if (!check_frame_crc(rx_frame, frame_len)) {
        crc_error_count++;
        printf("[Laser ACK CRC ERR] count=%lu\r\n", (unsigned long)crc_error_count);
        return;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     handle_exception
// 函数说明     处理异常响应帧（功能码最高位 = 1）
//              CRC 通过时打印异常码供调试，CRC 失败时计入错误计数
// 参数         frame_len   完整帧长度 (固定 5 字节)
// 返回值       void
//-------------------------------------------------------------------------------------------------------------------
static void handle_exception(uint32_t frame_len)
{
    if (check_frame_crc(rx_frame, frame_len)) {
        printf("[Laser EXCEPTION] func=0x%02X code=0x%02X\r\n", rx_frame[1], rx_frame[2]);
    } else {
        crc_error_count++;
        printf("[Laser EXCEPTION CRC ERR] count=%lu\r\n", (unsigned long)crc_error_count);
    }
}

//================================================== 流式帧解析 ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     try_parse_frames
// 函数说明     尝试从接收缓冲区中解析完整的 Modbus 帧
//              从缓冲区头部开始，检查地址 → 识别功能码 → 计算帧长 → 调用对应处理函数
//              解析成功一帧后 consume 掉已解析字节，继续尝试下一帧
//              数据不足（未收齐完整帧）时直接返回，等待下次收到数据后再尝试
// 返回值       void
//-------------------------------------------------------------------------------------------------------------------
static void try_parse_frames(void)
{
    while (rx_len >= 2U) {
        if (rx_frame[0] != TOF_ADDR) {
            printf("[Laser DROP] 0x%02X\r\n", rx_frame[0]);
            consume_frame_bytes(1U);
            continue;
        }

        uint8_t func = rx_frame[1];
        uint32_t frame_len = 0;

        if (func == 0x03U) {
            if (rx_len < 3U) {
                return;
            }

            uint8_t byte_count = rx_frame[2];
            if (byte_count == 0U || byte_count > (RX_FRAME_MAX - 5U)) {
                printf("[Laser DROP] bad byte_count=%u\r\n", (unsigned)byte_count);
                consume_frame_bytes(1U);
                continue;
            }

            frame_len = 3U + (uint32_t)byte_count + 2U;
            if (rx_len < frame_len) {
                return;
            }

            handle_read_response(frame_len);
            consume_frame_bytes(frame_len);
        } else if (func == 0x06U) {
            frame_len = 8U;
            if (rx_len < frame_len) {
                return;
            }

            handle_write_ack(frame_len);
            consume_frame_bytes(frame_len);
        } else if ((func & 0x80U) != 0U) {
            frame_len = 5U;
            if (rx_len < frame_len) {
                return;
            }

            handle_exception(frame_len);
            consume_frame_bytes(frame_len);
        } else {
            printf("[Laser DROP] unknown func=0x%02X\r\n", func);
            consume_frame_bytes(1U);
        }
    }
}

//================================================== 接收数据处理 ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     process_rx_byte
// 函数说明     处理一个接收到的字节：帧超时检查 → 缓冲区溢出检查 → 追加到缓冲区 → 尝试解析
// 返回值       void
// 备注信息     帧超时：距上一字节超过 FRAME_TIMEOUT_MS 则丢弃当前不完整帧重新开始
//-------------------------------------------------------------------------------------------------------------------
static void process_rx_byte(uint8_t data)
{
    if (rx_len > 0U && (g_sys_ms - last_rx_ms) > FRAME_TIMEOUT_MS) {
        printf("[Laser PARTIAL DROP] %lu bytes timed out\r\n", (unsigned long)rx_len);
        rx_len = 0;
    }
    last_rx_ms = g_sys_ms;

    if (rx_len >= RX_FRAME_MAX) {
        printf("[Laser OVERFLOW DROP] buffer full\r\n");
        rx_len = 0;
    }

    rx_frame[rx_len++] = data;
    try_parse_frames();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     drain_tof_rx
// 函数说明     排空 UART_TOF 接收 FIFO 中的所有字节，逐个送入帧解析器
//              在每次发送命令后调用，以读取模块的响应数据
// 返回值       void
//-------------------------------------------------------------------------------------------------------------------
static void drain_tof_rx(void)
{
    uint8_t ch;

    while (uart_query_byte(UART_TOF, &ch)) {
        process_rx_byte(ch);
    }
}

//================================================== TOF200F 初始化 & 运行时 ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     tof200f_config
// 函数说明     TOF200F 上电初始化序列：握手 → 高精度模式配置
//              每条命令发送后等待 80ms 确保模块处理完成，然后排空应答数据
// 返回值       void
// 备注信息     仅在上电时由 Laser_Init() 调用一次
//-------------------------------------------------------------------------------------------------------------------
static void tof200f_config(void)
{
    printf("[Laser] UART3 PA0/PA1 init, baud 115200\r\n");
    printf("[Laser] Sending handshake...\r\n");
    uart_send_buffer(UART_TOF, CMD_HANDSHAKE, sizeof(CMD_HANDSHAKE));
    delay_1ms(80);
    drain_tof_rx();

    printf("[Laser] Sending high precision config...\r\n");
    uart_send_buffer(UART_TOF, CMD_HIGH_PRECISION, sizeof(CMD_HIGH_PRECISION));
    delay_1ms(80);
    drain_tof_rx();

    uart_flush_rx(UART_TOF);
    rx_len = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     tof200f_process
// 函数说明     TOF200F 运行时处理：排空接收数据 → 帧超时清理 → 按周期发送查询命令
//              由 Laser_GetDistanceCm() 在每次主循环迭代中调用
// 返回值       void
// 备注信息     查询间隔由 QUERY_INTERVAL_MS (200ms) 控制
//-------------------------------------------------------------------------------------------------------------------
static void tof200f_process(void)
{
    drain_tof_rx();

    if ((g_sys_ms - last_rx_ms) > FRAME_TIMEOUT_MS && rx_len > 0U) {
        printf("[Laser PARTIAL DROP] %lu bytes timed out\r\n", (unsigned long)rx_len);
        rx_len = 0;
    }

    if ((g_sys_ms - last_query_ms) >= QUERY_INTERVAL_MS) {
        uart_send_buffer(UART_TOF, CMD_QUERY_DISTANCE, sizeof(CMD_QUERY_DISTANCE));
        last_query_ms = g_sys_ms;
        query_count++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     Laser_Init
// 函数说明     初始化 TOF200F 激光测距模块
//              流程：重置内部状态 → UART3 初始化 (PA0/PA1, 115200) → 握手配置
// 返回值       void
// 使用示例     Laser_Init();  // 系统启动时调用一次
//-------------------------------------------------------------------------------------------------------------------
void Laser_Init(void)
{
    last_distance_mm = 0xFFFFU;
    rx_len = 0;
    last_rx_ms = g_sys_ms;
    last_query_ms = g_sys_ms;
    query_count = 0;
    frame_count = 0;
    crc_error_count = 0;

    uart_init(UART_TOF, 115200);
    uart_flush_rx(UART_TOF);
    tof200f_config();

    printf("TOF200F laser parser: OK\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     Laser_GetDistanceCm
// 函数说明     获取最近一次有效的激光测距值，同时驱动 TOF 运行时（排空接收 + 周期查询）
//              将内部存储的毫米值四舍五入转换为厘米后返回
// 返回值       uint16_t    距离值 (cm)；LASER_READ_ERROR (0xFFFF) 表示无有效数据
// 使用示例     uint16_t dist = Laser_GetDistanceCm();  // 主循环中每次迭代调用
//-------------------------------------------------------------------------------------------------------------------
uint16_t Laser_GetDistanceCm(void)
{
    tof200f_process();

    if (last_distance_mm >= 0xFF00U) {
        return LASER_READ_ERROR;
    }

    return (last_distance_mm + 5U) / 10U;
}
