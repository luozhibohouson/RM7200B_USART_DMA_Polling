#ifndef __USART_H
#define __USART_H

#include "main.h"

/* Bootloader配置 */
#define BOOTLOADER_START_ADDR       0x08000000
#define BOOTLOADER_SIZE             (5*1024)    // 5KB
#define APP_START_ADDR              (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE)
#define APP_SIZE                    (26*1024)     // 26KB
#define PARAM_START_ADDR            (APP_START_ADDR + APP_SIZE)
#define PARAM_SIZE                  (1*1024)      // 1KB

/* 各指令的数据长度定义 */
#define CMD_GET_VERSION_DATA_LEN    0    // 获取版本指令数据长度
#define CMD_FLOW_ADJUST_DATA_LEN    1    // 调流量指令数据长度：流量等级(1)
#define CMD_DEEP_SLEEP_DATA_LEN     1    // 深睡眠指令数据长度：休眠标志(1)
#define CMD_FAULT_REPORT_DATA_LEN   1    // 故障上报指令数据长度：故障代码(1)
#define CMD_PUMP_STATUS_DATA_LEN    0    // 气泵状态查询指令数据长度
#define CMD_SYSTEM_UPGRADE_DATA_LEN 4    // 系统升级指令数据长度：文件大小(2) + 校验和(2)
#define CMD_PACKET_NUM_SIZE         2    // 包序号大小
#define CMD_RESPONSE_MAX_DATA_LEN   13    // 响应数据最大长度

/* 固件标志 */
#define APP_VALID_FLAG              0xAA55
#define APP_INVALID_FLAG            0xFFFF
#define APP_SUCCESS_FLAG            0xCD12

/* 协议常量 */
#define PROTOCOL_FRAME_HEAD         0x5A
#define PROTOCOL_FRAME_TAIL1        0x55
#define PROTOCOL_FRAME_TAIL2        0xAA
#define PROTOCOL_FRAME_HEAD_REPLY   0xA5
#define PROTOCOL_FRAME_TAIL1_REPLY  0xAA
#define PROTOCOL_FRAME_TAIL2_REPLY  0x55
#define PROTOCOL_FRAME_HEAD_TAIL_SIZE 3
#define PROTOCOL_MIN_FRAME_SIZE     8    // 帧头(1) + 指令(1) + 长度(2) + CRC(2) + 帧尾(2)
#define PROTOCOL_MAX_DATA_SIZE      (CMD_PACKET_NUM_SIZE+1024) // 最大数据长度

/* 升级状态有效码 */
#define UPGRADE_VALID_FLAG          0xCDEF1234
#define UPGRADE_INVALID_FLAG        0x00000000

/* 休眠标志 */
#define DEEP_SLEEP_FLAG_SLEEP       0x01
#define DEEP_SLEEP_FLAG_WAKEUP      0x00

/* 流量等级 */
#define FLOW_LEVEL_50_PERCENT       0x01 //50%流量
#define FLOW_LEVEL_60_PERCENT       0x02 //60%流量
#define FLOW_LEVEL_70_PERCENT       0x03 //70%流量
#define FLOW_LEVEL_80_PERCENT       0x04 //80%流量
#define FLOW_LEVEL_90_PERCENT       0x05 //90%流量
#define FLOW_LEVEL_100_PERCENT      0x06 //100%流量
#define FLOW_QUERY_CMD              0xFF // 查询指令


/* 指令定义 */
typedef enum {
    CMD_GET_VERSION       = 0x01,  // 读取固件版本
    CMD_FLOW_ADJUST       = 0x02,  // 流量档位调整/查询
    CMD_DEEP_SLEEP        = 0x03,  // 休眠/唤醒
    CMD_FAULT_REPORT      = 0x04,  // 上报故障代码
    CMD_PUMP_STATUS       = 0x05,  // 查询气泵状态
    CMD_SYSTEM_UPGRADE    = 0xF0,  // 升级固件准备
    CMD_SYSTEM_DATA       = 0xF1,  // 升级固件数据
    CMD_DATA_COMPLETE     = 0xF2,  // 固件数据传输完成
    CMD_UPGRADE_SUCCESS   = 0xF3   // 升级成功
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

/* 故障代码定义 */
typedef enum {
    FAULT_NORMAL          = 0x00,  // 正常
    FAULT_OVER_VOLTAGE    = 0x01,  // 过压（主动上报）
    FAULT_OVER_CURRENT    = 0x02,  // 过流（主动上报）
    FAULT_SETTING_FAILED  = 0x03,  // 调档失败（主动上报）
    FAULT_NOT_LOAD        = 0x04,  // 空载（主动上报）
    FAULT_QUERY_CMD       = 0xFF   // 查询指令
} protocol_fault_t;

/* 协议帧结构 */
typedef struct {
    uint8_t frame_head;     // 帧头 0x5A（发送）或 0xA5（返回）
    uint8_t cmd;            // 指令
    uint16_t data_len;      // 数据长度（不包含帧头、指令和数据长度）
    uint8_t data[PROTOCOL_MAX_DATA_SIZE]; // 数据
    uint16_t crc16;         // CRC16校验
    uint8_t frame_tail[2];  // 帧尾 发送：0x55 0xAA，返回：0xAA 0x55
} __attribute__((packed)) protocol_frame_t;

/* 升级状态 */
typedef struct {
    uint16_t firmware_size;     // 固件总大小
    uint16_t firmware_crc16; // 固件校验和(16bit)
    uint16_t current_packet;    // 当前包序号
    uint16_t received_size;     // 已接收大小
    uint32_t upgrade_valid;     // 升级有效标志
    uint8_t upgrade_active;     // 升级激活标志
} upgrade_state_t;


void USART_PrintfConfigure(uint32_t Baudrate);
void USART_Configure(uint32_t Baudrate, uint8_t delay_enable);
void usart_callback(void);

/* 协议处理函数声明 */
uint16_t crc16(uint8_t* buff, uint32_t len);
uint8_t usart_send_frame(uint8_t cmd, uint8_t *data, uint8_t data_len);
void usart_handle_protocol_command(uint8_t *frame_data, uint16_t frame_len);
void uart_cmd_process(void);
void usart_transmit(uint8_t *buf, uint32_t len);

/* 故障处理函数声明 */
void fault_report_active(protocol_fault_t fault_code);

/* 全局变量声明 */
extern uint8_t current_flow_level;

#endif

