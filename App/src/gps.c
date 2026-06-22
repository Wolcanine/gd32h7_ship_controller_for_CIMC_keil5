/*******************************************************************************
 * 文件名          gps.c
 * 描述            ATGM336H-5N GPS/北斗双模定位模块 — NMEA-0183 解析
 *                SW UART CH1 (PE2-TX/PE5-RX) 驱动, 9600 bps
 * MCU             GD32H759IMK6
 * IDE             Keil MDK5 (uVision5)
 *
 * 修改记录
 * 日期            作者            备注
 * 2026-06-18      CIMC           初始版本, $GNRMC/$GNGGA 解析
 ******************************************************************************/

#include "gps.h"
#include "sw_uart.h"
#include "systick.h"
#include "uart_driver.h"       /* g_sys_ms */
#include <stdio.h>
#include <string.h>

/* ==================== 内部常量 ==================== */
#define GPS_BUF_SIZE        128     /* NMEA 语句缓冲区最大字节 */
#define GPS_SENTENCE_TIMEOUT_MS  2000   /* 语句间超时 (ms), 超时则丢弃未完成语句 */

/* ==================== 全局 GPS 数据 ==================== */
GPS_Data gps;

/* ==================== NMEA 解析内部状态 ==================== */
static uint8_t  nmea_buf[GPS_BUF_SIZE];  /* 当前语句缓冲区 */
static uint16_t nmea_idx;                /* 缓冲区写入位置 */
static uint8_t  nmea_dollar;             /* 是否已收到 '$' 开始新语句 */
static uint32_t last_byte_ms;            /* 最后收到字节的时间戳 */
static uint32_t last_rmc_ms;             /* 最后收到有效 RMC 的时间戳 */

/* ==================== 内部辅助函数 ==================== */

/*---------------------------------------------------------------------
 * 函数名称     nmea_reset
 * 函数说明     清空 NMEA 解析状态，准备接收下一条语句
 *---------------------------------------------------------------------*/
static void nmea_reset(void)
{
    nmea_idx   = 0;
    nmea_dollar = 0;
}

/*---------------------------------------------------------------------
 * 函数名称     nmea_atof
 * 函数说明     将 NMEA 字段字符串转换为 float (简化版, 不依赖 MicroLIB atof)
 *              支持格式: 整数 + 可选小数点 + 可选小数部分
 *              如 "2236.9453" → 2236.9453, "0.5" → 0.5, "" → 0
 *---------------------------------------------------------------------*/
static float nmea_atof(const char *s)
{
    float result = 0.0f;
    float frac   = 0.0f;
    float div    = 1.0f;
    uint8_t dot  = 0;

    if (!s || !*s) return 0.0f;

    while (*s) {
        char c = *s++;
        if (c >= '0' && c <= '9') {
            if (!dot) {
                result = result * 10.0f + (float)(c - '0');
            } else {
                div *= 10.0f;
                frac += (float)(c - '0') / div;
            }
        } else if (c == '.') {
            dot = 1;
        } else {
            break;  /* 非法字符, 停止解析 */
        }
    }

    return result + frac;
}

/*---------------------------------------------------------------------
 * 函数名称     nmea_atoi
 * 函数说明     将 NMEA 字段字符串转换为整数 (简化版, 不依赖 MicroLIB atoi)
 *---------------------------------------------------------------------*/
static int nmea_atoi(const char *s)
{
    int result = 0;
    if (!s || !*s) return 0;
    while (*s >= '0' && *s <= '9') {
        result = result * 10 + (*s - '0');
        s++;
    }
    return result;
}

/*---------------------------------------------------------------------
 * 函数名称     nmea_field_copy
 * 函数说明     从逗号分隔的 NMEA 语句中提取第 field_idx 个字段
 *              将字段内容拷贝到 dest，最多拷贝 dest_len-1 字节 (留 '\0')
 * 参数         dest        目标缓冲区
 *              dest_len    目标缓冲区大小
 *              field_idx   要提取的字段序号 (0-based)
 * 返回值       uint8_t     实际拷贝的字节数 (不含 '\0')
 *---------------------------------------------------------------------*/
static uint8_t nmea_field_copy(char *dest, uint8_t dest_len, uint8_t field_idx)
{
    uint16_t i  = 0;          /* nmea_buf 索引 */
    uint8_t  fi = 0;          /* 当前字段号 */
    uint8_t  dl = 0;          /* dest 写入长度 */

    /* 跳过 $ 头: $xxYYY, */
    while (i < nmea_idx && nmea_buf[i] != ',') i++;
    if (i < nmea_idx) i++;    /* 跳过第一个 ',' 之后的第一个字段才是 field 0 */

    /* 找到目标字段起始 */
    while (fi < field_idx && i < nmea_idx) {
        if (nmea_buf[i] == ',') fi++;
        i++;
    }
    if (fi != field_idx || i >= nmea_idx) {
        dest[0] = '\0';
        return 0;
    }

    /* 拷贝字段内容直到 ',' 或 '*' 或 '\r' 或 '\n' */
    while (i < nmea_idx && dl < dest_len - 1) {
        char c = (char)nmea_buf[i];
        if (c == ',' || c == '*' || c == '\r' || c == '\n') break;
        dest[dl++] = c;
        i++;
    }
    dest[dl] = '\0';
    return dl;
}

/*---------------------------------------------------------------------
 * 函数名称     nmea_parse_rmc
 * 函数说明     解析 $xxRMC 语句，提取定位信息
 *              RMC 字段: $xxRMC,time,status,lat,NS,lon,EW,speed,course,date,mag,magEW,mode*cs
 *              只解析前10个字段（0~9），数据手册规定的必含字段
 * 返回值       void
 *---------------------------------------------------------------------*/
static void nmea_parse_rmc(void)
{
    char field[16];
    float val_f;

    /* ---- 字段1: UTC时间 hhmmss.ss ---- */
    nmea_field_copy(field, sizeof(field), 0);
    if (strlen(field) >= 6) {
        gps.utc_hour = (uint8_t)((field[0]-'0')*10 + (field[1]-'0'));
        gps.utc_min  = (uint8_t)((field[2]-'0')*10 + (field[3]-'0'));
        gps.utc_sec  = (uint8_t)((field[4]-'0')*10 + (field[5]-'0'));
    }

    /* ---- 字段2: 定位状态 A=有效 V=无效 ---- */
    nmea_field_copy(field, sizeof(field), 1);
    gps.fix_valid = (field[0] == 'A') ? 1 : 0;

    /* ---- 字段3: 纬度 ddmm.mmmm ---- */
    nmea_field_copy(field, sizeof(field), 2);
    if (strlen(field) >= 4) {
        val_f = nmea_atof(field);
        uint16_t deg = (uint16_t)(val_f / 100.0f);     /* 度的整数部分 */
        float    min = val_f - (float)deg * 100.0f;    /* 分的部分 */
        gps.latitude = (float)deg + min / 60.0f;
    }

    /* ---- 字段4: 纬度方向 N/S ---- */
    nmea_field_copy(field, sizeof(field), 3);
    if (field[0] == 'S') gps.latitude = -gps.latitude;

    /* ---- 字段5: 经度 dddmm.mmmm ---- */
    nmea_field_copy(field, sizeof(field), 4);
    if (strlen(field) >= 5) {
        val_f = nmea_atof(field);
        uint16_t deg = (uint16_t)(val_f / 100.0f);
        float    min = val_f - (float)deg * 100.0f;
        gps.longitude = (float)deg + min / 60.0f;
    }

    /* ---- 字段6: 经度方向 E/W ---- */
    nmea_field_copy(field, sizeof(field), 5);
    if (field[0] == 'W') gps.longitude = -gps.longitude;

    /* ---- 字段7: 对地航速 (节) ---- */
    nmea_field_copy(field, sizeof(field), 6);
    val_f = nmea_atof(field);
    gps.speed_ms = val_f * 0.514444f;   /* 节 → m/s */

    /* ---- 字段8: 对地航向 (度) ---- */
    nmea_field_copy(field, sizeof(field), 7);
    gps.course_deg = nmea_atof(field);

    /* ---- 字段9: UTC日期 ddmmyy ---- */
    nmea_field_copy(field, sizeof(field), 8);
    if (strlen(field) >= 6) {
        gps.utc_day   = (uint8_t)((field[0]-'0')*10 + (field[1]-'0'));
        gps.utc_month = (uint8_t)((field[2]-'0')*10 + (field[3]-'0'));
        gps.utc_year  = (uint8_t)((field[4]-'0')*10 + (field[5]-'0'));
    }

    /* ---- 状态更新 ---- */
    gps.data_updated = 1;
    gps.last_fix_ms  = last_rmc_ms;
    gps.rmc_count++;
}

/*---------------------------------------------------------------------
 * 函数名称     nmea_parse_gga
 * 函数说明     解析 $xxGGA 语句，提取卫星数和定位质量
 *              GGA 字段: $xxGGA,time,lat,NS,lon,EW,quality,sats,hdop,alt,...*cs
 * 返回值       void
 *---------------------------------------------------------------------*/
static void nmea_parse_gga(void)
{
    char field[8];

    /* ---- 字段6: 定位质量 0=无效 1/2/3=有效 ---- */
    nmea_field_copy(field, sizeof(field), 5);
    if (field[0] >= '1' && field[0] <= '3') {
        /* GGA 定位有效 — 辅助确认 */
    }

    /* ---- 字段7: 使用卫星数量 ---- */
    nmea_field_copy(field, sizeof(field), 6);
    if (field[0] != '\0') {
        gps.satellites = (uint8_t)nmea_atoi(field);
    }
}

/*---------------------------------------------------------------------
 * 函数名称     nmea_dispatch
 * 函数说明     根据 NMEA 语句类型分派到对应解析函数
 *              支持 GP/GN/BD 前缀的 RMC 和 GGA 语句
 *              语句格式: $TALKER+SENTENCE,field1,field2,...*CS\r\n
 * 返回值       void
 *---------------------------------------------------------------------*/
static void nmea_dispatch(void)
{
    /* nmea_buf[0]='$', nmea_buf[1..5]=talker+sentence type identifier
     * e.g. "$GNRMC" → buf[1]='G', buf[2]='N', buf[3]='R', buf[4]='M', buf[5]='C'
     * e.g. "$GPGGA" → buf[1]='G', buf[2]='P', buf[3]='G', buf[4]='G', buf[5]='A' */
    if (nmea_idx < 6) return;

    /* 检查是否为 RMC 语句: 第4-6字节 = "RMC"
     * 兼容: GPRMC, GNRMC, BDRMC, GARMC, GLRMC */
    if (nmea_buf[3] == 'R' && nmea_buf[4] == 'M' && nmea_buf[5] == 'C') {
        nmea_parse_rmc();
    }
    /* 检查是否为 GGA 语句: 第4-6字节 = "GGA" */
    else if (nmea_buf[3] == 'G' && nmea_buf[4] == 'G' && nmea_buf[5] == 'A') {
        nmea_parse_gga();
    }
    /* 其它语句 (GSV, GSA, VTG, GLL, TXT 等) 暂不处理 */
}

/*---------------------------------------------------------------------
 * 函数名称     nmea_feed_byte
 * 函数说明     向 NMEA 解析器喂入一个字节
 *              状态机: 等待'$' → 收集字符直到'\n' → 分发解析 → 复位
 *              超时保护: 收 '$' 后超过 GPS_SENTENCE_TIMEOUT_MS 无 '\n' 则丢弃
 * 参数         byte    接收到的字节
 * 返回值       void
 *---------------------------------------------------------------------*/
static void nmea_feed_byte(uint8_t byte)
{
    /* ---- 超时丢弃不完整语句 ---- */
    if (nmea_dollar && (g_sys_ms - last_byte_ms) > GPS_SENTENCE_TIMEOUT_MS) {
        nmea_reset();
    }
    last_byte_ms = g_sys_ms;

    /* ---- 等待 '$' 开始新语句 ---- */
    if (byte == '$') {
        nmea_reset();
        nmea_dollar = 1;
        nmea_buf[nmea_idx++] = byte;
        return;
    }

    if (!nmea_dollar) return;  /* 未收到 '$', 丢弃 */

    /* ---- 缓冲区溢出保护 ---- */
    if (nmea_idx >= GPS_BUF_SIZE - 1) {
        nmea_reset();
        gps.parse_errors++;
        return;
    }

    /* ---- 收集字符 ---- */
    nmea_buf[nmea_idx++] = byte;

    /* ---- 收到换行 = 语句结束 ---- */
    if (byte == '\n') {
        nmea_buf[nmea_idx] = '\0';
        gps.sentence_count++;

        nmea_dispatch();

        /* RMC 内部已更新 last_fix_ms, 这里再记一次全局 */
        if (nmea_buf[3] == 'R' && nmea_buf[4] == 'M' && nmea_buf[5] == 'C') {
            last_rmc_ms = g_sys_ms;
        }

        nmea_reset();
    }
}

/* ==================== 公开接口 ==================== */

/*---------------------------------------------------------------------
 * 函数名称     GPS_Init
 * 函数说明     初始化 GPS 模块：清空数据结构 + 清空 SW UART1 残留数据
 * 备注         假定 SwUart_Init() 已在此之前调用 (TIMER1 @ 28.8kHz)
 *---------------------------------------------------------------------*/
void GPS_Init(void)
{
    /* 清空 GPS 数据结构 */
    memset(&gps, 0, sizeof(gps));
    gps.fix_valid = 0;
    gps.satellites = 0;

    /* 清空 NMEA 解析状态 */
    nmea_reset();
    last_byte_ms = g_sys_ms;
    last_rmc_ms  = 0;

    /* 排空 SW UART CH1 上电残余数据 */
    {
        uint8_t dummy;
        uint16_t drain = 0;
        while (SwUart1_QueryByte(&dummy) && drain < 256) drain++;
    }

    printf("[GPS] ATGM336H-5N, SW UART1 (PE2/PE5), 9600bps\r\n");
}

/*---------------------------------------------------------------------
 * 函数名称     GPS_Process
 * 函数说明     GPS 运行时处理：从 SW UART CH1 读取所有可用字节并送入解析器
 *---------------------------------------------------------------------*/
void GPS_Process(void)
{
    uint8_t byte;

    /* 排空 SW UART CH1 接收缓冲区 */
    while (SwUart1_QueryByte(&byte)) {
        nmea_feed_byte(byte);
    }
}

/*---------------------------------------------------------------------
 * 函数名称     GPS_IsFixValid
 * 函数说明     检查 GPS 定位是否有效 (RMC status='A')
 *---------------------------------------------------------------------*/
uint8_t GPS_IsFixValid(void)
{
    return gps.fix_valid;
}

/*---------------------------------------------------------------------
 * 函数名称     GPS_PrintStatus
 * 函数说明     打印 GPS 当前状态单行摘要到调试串口 (UART4)
 *---------------------------------------------------------------------*/
void GPS_PrintStatus(void)
{
    if (gps.fix_valid) {
        /* 北京时间 = UTC + 8 */
        uint8_t bj_hour = (gps.utc_hour + 8) % 24;
        printf("GPS: FIX OK | BJ %02u:%02u:%02u | Lat=%.6f Lon=%.6f | "
               "Spd=%.1fm/s Cse=%.0f deg | Sats=%u | RMC=%u\r\n",
               (unsigned)bj_hour, (unsigned)gps.utc_min, (unsigned)gps.utc_sec,
               (double)gps.latitude, (double)gps.longitude,
               (double)gps.speed_ms, (double)gps.course_deg,
               (unsigned)gps.satellites, (unsigned)gps.rmc_count);
    } else {
        /* 无有效定位时显示已收语句数和卫星数 */
        printf("GPS: NO FIX | Sats=%u | Sentences=%u RMC=%u\r\n",
               (unsigned)gps.satellites, (unsigned)gps.sentence_count,
               (unsigned)gps.rmc_count);
    }
}
