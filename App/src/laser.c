/*******************************************************************************
 * 文件名          laser.c
 * 描述            TOF200F 激光测距模块 — Modbus-RTU 主动查询
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-05-07      AI助手          初始版本
 * 2026-05-21      CIMC            GD32F407→GD32H759 移植
 ******************************************************************************/

// laser.c — TOF200F 激光测距模块
// =========================================
// USART1 (PA2-TX / PA3-RX) 115200 8N1
// Modbus-RTU 主动查询，200ms 间隔
// 公开接口：Laser_Init() / Laser_GetDistanceCm()
// =========================================

#include "laser.h"
#include "uart_driver.h"
#include "systick.h"
#include <stdio.h>

// ==================== 可配置参数 ====================
#define FRAME_TIMEOUT_MS    10          // 帧间隔超时(ms)，用于丢弃残帧
#define QUERY_INTERVAL_MS   200         // 主动查询间隔(ms)

// ==================== Modbus-RTU 指令 ====================
static const uint8_t CMD_QUERY_DISTANCE[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};
static const uint8_t CMD_HANDSHAKE[]      = {0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD8, 0x0A};
static const uint8_t CMD_HIGH_PRECISION[] = {0x01, 0x06, 0x00, 0x04, 0x00, 0x01, 0x09, 0xCB};

// ==================== 内部状态 ====================
static uint16_t last_distance_mm = 0xFFFF;

// ==================== TOF200F 初始化 ====================
static void tof200f_config(void)
{
    uint8_t resp[32];
    uint32_t len;

    printf("[Laser] Handshake: 01 06 00 01 00 00 D8 0A\r\n");
    uart_send_buffer(UART_TOF, CMD_HANDSHAKE, sizeof(CMD_HANDSHAKE));
    delay_1ms(50);
    len = uart_read_buffer(UART_TOF, resp, sizeof(resp));
    if (len > 0) {
        printf("  -> RX(%ld): ", (long)len);
        for (uint32_t i = 0; i < len; i++) printf("%02X ", resp[i]);
        printf("\r\n");
    } else {
        printf("  -> NO RESPONSE! Check PA2(TX)->TOF(RX), PA3(RX)->TOF(TX)\r\n");
    }

    printf("[Laser] High precision mode: 01 06 00 04 00 01 09 CB\r\n");
    uart_send_buffer(UART_TOF, CMD_HIGH_PRECISION, sizeof(CMD_HIGH_PRECISION));
    delay_1ms(50);
    len = uart_read_buffer(UART_TOF, resp, sizeof(resp));
    if (len > 0) {
        printf("  -> RX(%ld): ", (long)len);
        for (uint32_t i = 0; i < len; i++) printf("%02X ", resp[i]);
        printf("\r\n");
    } else {
        printf("  -> NO RESPONSE!\r\n");
    }

    uart_flush_rx(UART_TOF);
}

// ==================== 查询 + 解析 ====================
static void tof200f_process(void)
{
    static uint32_t last_query = 0;
    static uint8_t  query_count = 0;

    if ((g_sys_ms - last_query) >= QUERY_INTERVAL_MS) {
        uart_send_buffer(UART_TOF, CMD_QUERY_DISTANCE, sizeof(CMD_QUERY_DISTANCE));
        last_query = g_sys_ms;

        query_count++;
        if (query_count <= 3)
            printf("[Query #%d] sent\r\n", (int)query_count);
    }

    uint32_t avail = uart_rx_available(UART_TOF);
    if (avail >= 7) {
        uint8_t recv[32];
        uint32_t len = uart_read_buffer(UART_TOF, recv, sizeof(recv));

        for (uint32_t offset = 0; offset + 6 < len; offset++) {
            if (recv[offset] == 0x01 && recv[offset+1] == 0x03 && recv[offset+2] == 0x02) {
                if (offset + 6 < len) {
                    uint16_t d = ((uint16_t)recv[offset+3] << 8) | recv[offset+4];
                    last_distance_mm = d;
                }
            }
        }
    }
}

// ==================== 公开 API ====================

void Laser_Init(void)
{
    uart_init(UART_TOF, 115200);            // 初始化 USART1 (PA2/PA3)
    uart_flush_rx(UART_TOF);
    tof200f_config();
    printf("TOF200F laser: OK\r\n");
}

uint16_t Laser_GetDistanceCm(void)
{
    tof200f_process();

    if (last_distance_mm >= 0xFF00)
        return LASER_READ_ERROR;

    return (last_distance_mm + 5) / 10;    // mm → cm，四舍五入
}
