#ifndef __USART_H
#define __USART_H

#include "main.h"

#define ENABLE_USART   1

/* Bootloader配置 */
#define BOOTLOADER_START_ADDR       0x08000000
#define BOOTLOADER_SIZE             (8*1024)    // 8KB
#define APP_START_ADDR              (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE)
#define APP_SIZE                    (23*1024)     // 23KB
#define PARAM_START_ADDR            (APP_START_ADDR + APP_SIZE)
#define PARAM_SIZE                  (1*1024)      // 1KB

/* 各指令的数据长度定义 */
#define CMD_FLOW_ADJUST_DATA_LEN    2    // 调流量指令数据长度：方向(1) + 流量(1)
#define CMD_SYSTEM_UPGRADE_DATA_LEN 4    // 系统升级指令数据长度：文件大小(2) + 校验和(2)
#define CMD_PACKET_NUM_SIZE         2    // 包序号大小
#define CMD_RESPONSE_MAX_DATA_LEN   8    // 响应数据最大长度

/* 固件标志 */
#define APP_VALID_FLAG              0xAA55
#define APP_INVALID_FLAG            0xFFFF
#define APP_SUCCESS_FLAG            0xCD12

/* 协议常量 */
#define PROTOCOL_FRAME_HEAD         0xBB
#define PROTOCOL_FRAME_TAIL1        0x55
#define PROTOCOL_FRAME_TAIL2        0x0A
#define PROTOCOL_MIN_FRAME_SIZE     8    // 帧头(1) + 指令(1) + 长度(2) + CRC(2) + 帧尾(2)
#define PROTOCOL_MAX_DATA_SIZE      (CMD_PACKET_NUM_SIZE+1024) // 最大数据长度

/* 升级状态有效码 */
#define UPGRADE_VALID_FLAG         0xCDEF1234
#define UPGRADE_INVALID_FLAG       0x00000000

/* 指令定义 */
typedef enum {
    CMD_FLOW_ADJUST       = 0x01,  // 调流量
    CMD_SYSTEM_UPGRADE    = 0xF0,  // 系统升级
    CMD_SYSTEM_DATA       = 0xF1,  // 系统数据
    CMD_DATA_COMPLETE     = 0xF2,  // 系统数据传输完成
    CMD_UPGRADE_SUCCESS   = 0xF3   // 系统升级成功
} protocol_cmd_t;

/* 错误码定义 */
typedef enum {
    ERR_SUCCESS         = 0x00,  // 成功
    ERR_INVALID_CMD     = 0x01,  // 无效命令
    ERR_INVALID_PARAM   = 0x02,  // 无效参数
    ERR_INVALID_LENGTH  = 0x03,  // 无效长度
    ERR_INVALID_CRC     = 0x04,  // CRC校验错误
    ERR_FLASH_ERROR     = 0x05,  // Flash操作错误
    ERR_FILE_TOO_LARGE  = 0x06,  // 文件过大
    ERR_VERIFY_FAILED   = 0x07,  // 验证失败
    ERR_UPDATE_FAILED   = 0x08   // 升级失败
} protocol_error_t;

/* 协议帧结构 */
typedef struct {
    uint8_t frame_head;     // 帧头 0xBB
    uint8_t cmd;            // 指令
    uint16_t data_len;      // 数据长度
    uint8_t data[PROTOCOL_MAX_DATA_SIZE]; // 数据
    uint16_t crc16;         // CRC16校验
    uint8_t frame_tail[2];  // 帧尾 0x55 0x0A
} __attribute__((packed)) protocol_frame_t;

/* 升级状态 */
typedef struct {
    uint16_t firmware_size;     // 固件总大小
    uint16_t firmware_checksum; // 固件校验和(16bit)
    uint16_t current_packet;    // 当前包序号
    uint32_t received_size;     // 已接收大小
    uint32_t upgrade_valid;     // 升级有效标志
    uint8_t upgrade_active;     // 升级激活标志
} upgrade_state_t;


void USART_PrintfConfigure(uint32_t Baudrate);
void USART_Configure(uint32_t Baudrate);
void usart_callback(void);

void VectorTable_Init(void);

#endif

