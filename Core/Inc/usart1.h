#ifndef __USART1_H
#define __USART1_H

#include "main.h"

/* 各指令的数据长度定义 */
#define QUERY_CMD_GET_VERSION_DATA_LEN            0    // 获取版本指令数据长度
#define QUERY_CMD_FLOW_ADJUST_DATA_LEN            1    // 调流量指令数据长度：流量等级(1)
#define QUERY_CMD_QUERY_MAX_VOL_CUR_DATA_LEN      0    // 气泵历史最大电压/最大电流查询指令数据长度
#define QUERY_CMD_QUERY_CURRENT_VOL_CUR_DATA_LEN  0    // 当前电压/电流查询指令数据长度
#define QUERY_CMD_PACKET_NUM_SIZE                 2    // 包序号大小
#define QUERY_CMD_RESPONSE_MAX_DATA_LEN           9    // 响应数据最大长度

/* 协议常量 */
#define QUERY_FRAME_HEAD            0x5B
#define QUERY_FRAME_TAIL1           0x55
#define QUERY_FRAME_TAIL2           0xAA
#define QUERY_FRAME_HEAD_REPLY      0xB5
#define QUERY_FRAME_TAIL1_REPLY     0xAA
#define QUERY_FRAME_TAIL2_REPLY     0x55
#define QUERY_FRAME_HEAD_TAIL_SIZE  3
#define QUERY_MIN_FRAME_SIZE        8    // 帧头(1) + 指令(1) + 长度(2) + CRC(2) + 帧尾(2)

/* 指令定义 */
typedef enum {
    QUERY_CMD_GET_VERSION           = 0x01,  // 读取固件版本
    QUERY_CMD_FLOW_ADJUST           = 0x02,  // 调流量
    QUERY_CMD_QUERY_MAX_VOL_CUR     = 0x03,  // 气泵历史最大电压/最大电流查询
    QUERY_CMD_QUERY_CURRENT_VOL_CUR = 0x04,  // 当前电压/电流查询
    QUERY_CMD_FAULT_REPORT          = 0x05,  // 上报故障代码
    QUERY_CMD_QUERY_FAULT_REPORT    = 0xFF,  // 查询故障代码
} query_cmd_t;

/* 错误码定义 */
typedef enum {
    QUERY_ERR_SUCCESS        = 0x00,  // 成功
    QUERY_ERR_INVALID_CMD    = 0x01,  // 无效命令
    QUERY_ERR_INVALID_PARAM  = 0x02,  // 无效参数
    QUERY_ERR_INVALID_LENGTH = 0x03,  // 无效长度
    QUERY_ERR_INVALID_CRC    = 0x04,  // CRC校验错误
} query_error_t;

#endif

