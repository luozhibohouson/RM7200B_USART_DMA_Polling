#include "usart.h"
#include "usart1.h"
#include "main.h"
#include "adc.h"
#include "controller.h"

#ifndef ENABLE_QUERY_CMD
  #define ENABLE_QUERY_CMD 0
#endif

uint32_t uart_rx_flag = 0;
uint32_t uart_rx_len = 0;
uint8_t  uart_rxbuffer[30] = {0};
uint8_t  uart_cmd[30] = {0};

static __IO uint32_t vector_table[48] __attribute__((at(0x20000000)));
static __IO upgrade_state_t upgrade_state __attribute__((at(0x200000C0)));

void USART_DMA_Configure(uint8_t *Buffer, uint8_t Length);


/* 故障上报状态机 */
typedef enum {
    FAULT_REPORT_STATE_IDLE,
    FAULT_REPORT_STATE_WAIT_ACK,
} fault_report_state_t;

static fault_report_state_t fault_report_state = FAULT_REPORT_STATE_IDLE;
static uint8_t fault_report_retry_count = 0;
static uint32_t fault_report_timeout = 0;
static protocol_fault_t fault_code_to_report = FAULT_NORMAL;

#define FAULT_REPORT_MAX_RETRIES 3
#define FAULT_REPORT_TIMEOUT_MS 1000

void usart_fault_report_process(void);
void clear_fault_report(void);
/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USART_PrintfConfigure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    sys_delayms(500); // 禁用SWD接口

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART1, ENABLE);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Tx; // USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    // USART_DMACmd(USART1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_6); //RX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_6); //TX

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_14;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    USART_Cmd(USART1, ENABLE);

    // USART_DMA_Configure(uart_rxbuffer, sizeof(uart_rxbuffer));
}

void USART_Configure(uint32_t Baudrate, uint8_t delay_enable)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

//    if( delay_enable != 0 ) {
//        sys_delayms(500); // 禁用SWD接口
//    }

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART1, ENABLE);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    // USART_DMACmd(USART1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_6); //RX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_6); //TX

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_14;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_ITConfig(USART1, USART_IT_IDLE | USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);

    // USART_DMA_Configure(uart_rxbuffer, sizeof(uart_rxbuffer));
}

void USART_DMA_Configure(uint8_t *Buffer, uint8_t Length)
{
    DMA_InitTypeDef  DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);

    DMA_DeInit(DMA1_Channel2);

    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)Buffer;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize         = Length;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_Auto_Reload        = DMA_Auto_Reload_Enable;
    DMA_Init(DMA1_Channel2, &DMA_InitStruct);

    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    DMA_Cmd(DMA1_Channel2, ENABLE);
}

void usart_transmit(uint8_t *buf, uint32_t len)
{
    for(int i = 0; i < len; i++) {
        while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC))
        {
        }

        USART_SendData(USART1, (uint8_t)buf[i]);
    }
}


void usart_callback(void)
{
#if 0
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        DMA_Cmd(DMA1_Channel2, DISABLE);

        uart_rx_len = sizeof(uart_rxbuffer) - DMA_GetCurrDataCounter(DMA1_Channel2);

        if( uart_rx_len > 0 ) {
            uart_rx_flag = 1;
        }

        USART_ReceiveData(USART1); //通过读取DR寄存器，清除IDLE标志位

        DMA_Cmd(DMA1_Channel2, ENABLE);
    }
#else
    uint8_t rxdata = 0;

    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        if( uart_rx_len > 0 ) {
            uart_rx_flag = 1;
        }

        USART_ReceiveData(USART1); //通过读取DR寄存器，清除IDLE标志位
    } else if (RESET != USART_GetITStatus(USART1, USART_IT_RXNE))  {
        rxdata = USART_ReceiveData(USART1);

        if( uart_rx_len < sizeof(uart_rxbuffer) ) {
            uart_rxbuffer[uart_rx_len++] = rxdata;
        }
    }
#endif
}

uint16_t crc16(uint8_t* buff, uint32_t len)
{
    uint16_t crc = 0;

    while(len--)
    {
        crc ^= (uint16_t) (*(buff++)) << 8;

        for(int i = 0; i < 8; i++)
        {
            if(crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }

    return crc;
}


/**
 * @brief 发送协议帧
 * @param cmd 指令
 * @param data 数据指针
 * @param data_len 数据长度
 * @return 0:成功 1:失败
 */
uint8_t usart_send_frame(uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    uint8_t tx_buffer[20];  // 足够容纳任何响应帧：帧头(1)+指令(1)+长度(1)+数据(3)+CRC(2)+帧尾(2)=11字节
    uint16_t frame_len = 0;

    // 检查数据长度，bootloader响应数据应该很小
    if (data_len > CMD_RESPONSE_MAX_DATA_LEN) {
        return 1;  // 响应数据过长
    }

    // 构建协议帧: 帧头(1) + 指令(1) + 数据长度(1) + 数据(N) + CRC16(2) + 帧尾(2)
    tx_buffer[frame_len++] = PROTOCOL_FRAME_HEAD_REPLY;     // 帧头
    tx_buffer[frame_len++] = cmd;                     // 指令
    // tx_buffer[frame_len++] = (data_len >> 8) & 0xFF; // 数据长度高字节
    tx_buffer[frame_len++] = data_len & 0xFF;         // 数据长度低字节

    // 添加数据
    if (data != NULL && data_len > 0) {
        memcpy(&tx_buffer[frame_len], data, data_len);
        frame_len += data_len;
    }

    // 计算CRC16 (对指令+数据长度+数据进行校验)
    uint16_t crc = crc16(&tx_buffer[1], 1 + 1 + data_len);
    tx_buffer[frame_len++] = (crc >> 8) & 0xFF;      // CRC16高字节
    tx_buffer[frame_len++] = crc & 0xFF;             // CRC16低字节

    // 帧尾
    tx_buffer[frame_len++] = PROTOCOL_FRAME_TAIL1_REPLY;   // 帧尾
    tx_buffer[frame_len++] = PROTOCOL_FRAME_TAIL2_REPLY;   // 帧尾

    // 发送数据
    usart_transmit(tx_buffer, frame_len);

    return 0;
}

/**
 * @brief 处理获取版本指令
 * @param data 数据指针
 * @param data_len 数据长度
 */
static void handle_get_version_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t response_data[4] = {0};
    uint8_t error_code = ERR_SUCCESS;

    // 验证数据长度
    if (data_len != CMD_GET_VERSION_DATA_LEN) {
        error_code = ERR_INVALID_LENGTH;
    } else {
        error_code = ERR_SUCCESS;

        extern char* get_app_version(void);
        extern char* get_hardware_version(void);

        char* app_version = get_app_version();
        char* hardware_version = get_hardware_version();
        response_data[0] = hardware_version[0]; // 硬件版本只有1个字节
        response_data[1] = app_version[0];
        response_data[2] = app_version[1];
        response_data[3] = error_code;
    }

    // 发送响应
    usart_send_frame(CMD_GET_VERSION, response_data, sizeof(response_data));
}


/**
 * @brief 处理调流量指令
 * @param data 数据指针
 * @param data_len 数据长度
 */
static void handle_flow_adjust_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = ERR_SUCCESS;
    uint8_t response_data[2] = {0};
    extern uint8_t current_flow_level;

    // 验证数据长度（应该是1字节：流量等级）
    if (data_len != CMD_FLOW_ADJUST_DATA_LEN) {
        error_code = ERR_INVALID_LENGTH;
        // 发送错误响应
        usart_send_frame(CMD_FLOW_ADJUST, &error_code, 1);
        return;
    }

    uint8_t flow_level = data[0];

    if (flow_level == FLOW_QUERY_CMD) {
        // 查询当前档位
        response_data[0] = current_flow_level;
        response_data[1] = ERR_SUCCESS;
        usart_send_frame(CMD_FLOW_ADJUST, response_data, 2);
    } else {
        // 设置流量档位
        if (flow_level >= FLOW_LEVEL_50_PERCENT && flow_level <= FLOW_LEVEL_100_PERCENT) {
            error_code = ERR_SUCCESS;
            extern void magic_cool_set_target_vol_by_flow(uint8_t flow_level);
            magic_cool_set_target_vol_by_flow(flow_level);
            current_flow_level = flow_level;
            clear_fault_report();
        } else {
            error_code = ERR_INVALID_PARAM;
        }

        response_data[0] = current_flow_level;
        response_data[1] = error_code;

        // 发送响应
        usart_send_frame(CMD_FLOW_ADJUST, response_data, 2);
    }
}

/**
 * @brief 处理深睡眠指令
 * @param data 数据指针
 * @param data_len 数据长度
 */
static void handle_deep_sleep_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = ERR_SUCCESS;
    uint8_t response_data[2] = {0};
    extern uint8_t deep_sleep_flag;

    // 验证数据长度（应该是2字节：方向+流量）
    if (data_len != CMD_DEEP_SLEEP_DATA_LEN) {
        error_code = ERR_INVALID_LENGTH;
    } else {
        deep_sleep_flag = data[0];

        if( deep_sleep_flag != DEEP_SLEEP_FLAG_SLEEP ) {
            error_code = ERR_INVALID_PARAM;
        } else {
            error_code = ERR_SUCCESS;
            extern void close_all_output(void);
            close_all_output();
            extern void set_stop_delay_ms(bool enable);
            set_stop_delay_ms(true);
        }
    }

    response_data[0] = deep_sleep_flag;
    response_data[1] = error_code;

    // 发送响应
    usart_send_frame(CMD_DEEP_SLEEP, response_data, sizeof(response_data));
}

/**
 * @brief 处理查询气泵状态指令
 * @param data 数据指针
 * @param data_len 数据长度
 */
#if ENABLE_PUMP_STATUS_CMD
static void handle_pump_status_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t response_data[11] = {0};

    // 验证数据长度
    if (data_len != CMD_PUMP_STATUS_DATA_LEN) {
        uint8_t error_code = ERR_INVALID_LENGTH;
        usart_send_frame(CMD_PUMP_STATUS, &error_code, 1);
        return;
    }

    // 获取气泵状态数据
    extern uint16_t magic_cool_vpp;
    extern int32_t pwm1_duty_out;
    extern float voltage_offset;
    extern float voltage_gain;
    extern float adc_dc_hvol_avg;
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    extern float adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
    extern float adc_dc_hcur_avg;
#endif
    extern bool scan_freq_enable;

    // 1. 获取气泵频率 (Hz)
    uint32_t freq = pwm_get_freq();

    // 2. 计算Vpp电压*100 (如40.22V -> 4022)
    float vpp_real = (float)(magic_cool_vpp + voltage_offset) / voltage_gain;
    uint16_t vpp_x100 = (uint16_t)(vpp_real * 100.0f);

    // 3. 计算dac电压*100 (如2.13V -> 213)
    float dac_real = (float)pwm1_duty_out * MCU_VDD_GAIN / (float)(HSI_VALUE / PWM1_FREQ);
    uint16_t dac_x100 = (uint16_t)(dac_real * 100.0f);

    // 4. 计算功率*100 (如662.02mW -> 66202)
    // ADC采样获取当前电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    // adc_hvli_input_conv(ADC_CH_SIZE);
    float pwr_mw = POWER_CAL(adc_dc_hvol_avg, adc_dc_lcur_avg);
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
    // adc_hv_input_conv(ADC_CH_SIZE);
    float pwr_mw = POWER_CAL(adc_dc_hvol_avg, adc_dc_hcur_avg);
#endif
    uint32_t pwr_x100 = (uint32_t)(pwr_mw * 100.0f);

    // 填充响应数据 (大端序)
    response_data[0] = (freq >> 8) & 0xFF;       // 气泵频率高8位
    response_data[1] = freq & 0xFF;              // 气泵频率低8位
    response_data[2] = (vpp_x100 >> 8) & 0xFF;   // Vpp电压*100高8位
    response_data[3] = vpp_x100 & 0xFF;          // Vpp电压*100低8位
    response_data[4] = (dac_x100 >> 8) & 0xFF;   // dac电压*100高8位
    response_data[5] = dac_x100 & 0xFF;          // dac电压*100低8位
    response_data[6] = (pwr_x100 >> 24) & 0xFF;  // PWR*100 HH_8bit
    response_data[7] = (pwr_x100 >> 16) & 0xFF;  // PWR*100 HL_8bit
    response_data[8] = (pwr_x100 >> 8) & 0xFF;   // PWR*100 LH_8bit
    response_data[9] = pwr_x100 & 0xFF;          // PWR*100 LL_8bit
    response_data[10] = scan_freq_enable;        // 扫频使能标志

    // 发送响应
    usart_send_frame(CMD_PUMP_STATUS, response_data, sizeof(response_data));
}
#endif

/**
 * @brief 处理故障报告指令
 * @param data 数据指针
 * @param data_len 数据长度
 */
static void handle_fault_report_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = ERR_SUCCESS;
    uint8_t response_data[2] = {0};

    // 验证数据长度（应该是1字节：故障查询或故障代码）
    if (data_len != CMD_FAULT_REPORT_DATA_LEN) {
        error_code = ERR_INVALID_LENGTH;
        usart_send_frame(CMD_FAULT_REPORT, &error_code, 1);
        return;
    }

    uint8_t fault_query = data[0];

    if (fault_query == FAULT_QUERY_CMD) {
        // 查询故障代码
        response_data[0] = fault_code_to_report;
        response_data[1] = ERR_SUCCESS;
        usart_send_frame(CMD_FAULT_REPORT, response_data, 2);
    } else {
        // 这个函数主要是主动上报故障，通过别的函数调用，这里只处理查询
        // 收到主机的响应，说明主机已经知道故障了，就不再上报了
        fault_report_state = FAULT_REPORT_STATE_IDLE;
        fault_report_retry_count = 0;
    }
}

/**
 * @brief 处理系统升级指令
 * @param data 数据指针
 * @param data_len 数据长度
 */
static void handle_system_upgrade_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = ERR_SUCCESS;

    // 验证数据长度（应该是4字节：固件大小+校验和）
    if (data_len != CMD_SYSTEM_UPGRADE_DATA_LEN) {
        error_code = ERR_INVALID_LENGTH;
    } else {
        uint16_t firmware_size = (data[0] << 8) | data[1];     // 固件总大小
        uint16_t firmware_crc16 = (data[2] << 8) | data[3]; // 固件校验和

        // 验证固件大小
        if (firmware_size == 0 || firmware_size > APP_SIZE) {
            error_code = ERR_FILE_TOO_LARGE;
        } else {
            // 初始化升级状态
            upgrade_state.firmware_size = firmware_size;
            upgrade_state.firmware_crc16 = firmware_crc16;
            upgrade_state.current_packet = 0;
            upgrade_state.received_size = 0;
            upgrade_state.upgrade_active = 1;
            upgrade_state.upgrade_valid = UPGRADE_VALID_FLAG;

            for(uint16_t i=0xfff; i>0; i--) {
                __nop();
            }

            __disable_irq();

            NVIC_SystemReset();
            while(1);
        }
    }
}

/**
 * @brief 处理usart命令
 * @param frame_data 帧数据（指令+数据长度+数据）
 * @param frame_len 帧数据长度
 */
void usart_handle_protocol_command(uint8_t *frame_data, uint16_t frame_len)
{
    if (frame_data == NULL || frame_len < 3) {
        return;
    }

    uint8_t cmd = frame_data[0];
    uint16_t data_len = (frame_data[1] << 8) | frame_data[2];
    uint8_t *data = &frame_data[3];

    // 验证数据长度一致性
    if (frame_len != (PROTOCOL_FRAME_HEAD_TAIL_SIZE + data_len)) {
        uint8_t error_code = ERR_INVALID_LENGTH;
        usart_send_frame(cmd, &error_code, 1);
        return;
    }

    // APP程序处理所有指令
    switch (cmd) {
        case CMD_GET_VERSION:
            handle_get_version_cmd(data, data_len);
            break;

        case CMD_FLOW_ADJUST:
            handle_flow_adjust_cmd(data, data_len);
            break;

        case CMD_DEEP_SLEEP:
            handle_deep_sleep_cmd(data, data_len);
            break;

        case CMD_FAULT_REPORT:
            handle_fault_report_cmd(data, data_len);
            break;

#if ENABLE_PUMP_STATUS_CMD
        case CMD_PUMP_STATUS:
            handle_pump_status_cmd(data, data_len);
            break;
#endif

        case CMD_SYSTEM_UPGRADE:
            handle_system_upgrade_cmd(data, data_len);
            break;

        default:
            {
                uint8_t error_code = ERR_INVALID_CMD;
                usart_send_frame(cmd, &error_code, 1);
            }
            break;
    }
}

bool protocol_cmd_process(void)
{
    // 检查最小帧长度: 帧头(1) + 指令(1) + 长度(2) + CRC(2) + 帧尾(2) = 8字节
    if (uart_rx_len < PROTOCOL_MIN_FRAME_SIZE) {
        return false;
    }

    // 检查帧头和帧尾
    if ((uart_cmd[0] == PROTOCOL_FRAME_HEAD) &&
        (uart_cmd[uart_rx_len-2] == PROTOCOL_FRAME_TAIL1) &&
        (uart_cmd[uart_rx_len-1] == PROTOCOL_FRAME_TAIL2)) {

        // 解析协议: 帧头(1) + 指令(1) + 数据长度(2) + 数据(N) + CRC16(2) + 帧尾(2)
        uint8_t cmd = uart_cmd[1];
        uint16_t data_len = (uart_cmd[2] << 8) | uart_cmd[3];  // 大端序

        // 验证数据长度是否合理
        if (uart_rx_len != (PROTOCOL_MIN_FRAME_SIZE + data_len)) {  // 帧头+指令+长度+数据+CRC+帧尾
            uint8_t error_code = ERR_INVALID_LENGTH;
            usart_send_frame(cmd, &error_code, 1);
            return false;
        }

        // 获取CRC16校验值 (在帧尾前2字节)
        uint16_t received_crc = (uart_cmd[uart_rx_len-4] << 8) | uart_cmd[uart_rx_len-3];

        // 计算CRC16: 对指令+数据长度+数据进行校验
        uint8_t *crc_data = &uart_cmd[1];  // 从指令开始
        uint16_t crc_len = 1 + 2 + data_len;  // 指令(1) + 长度(2) + 数据(data_len)
        uint16_t calculated_crc = crc16(crc_data, crc_len);

        if (calculated_crc == received_crc) {
            // 校验正确，处理命令
            // 传递给bootloader处理: 指令+数据长度+数据
            usart_handle_protocol_command(&uart_cmd[1], crc_len);
        } else {
            // CRC校验错误，发送错误应答
            uint8_t error_code = ERR_INVALID_CRC;
            usart_send_frame(cmd, &error_code, 1);
            return false;
        }

        return true;
    }

    return false;
}

#if ENABLE_QUERY_CMD

static uint8_t usart_send_query_frame(uint8_t cmd, uint8_t *data, uint16_t data_len)
{
    uint8_t tx_buffer[QUERY_MIN_FRAME_SIZE + 9];  // 足够容纳任何响应帧：帧头(1)+指令(1)+长度(1)+数据(3)+CRC(2)+帧尾(2)=11字节
    uint16_t frame_len = 0;

    // 检查数据长度，bootloader响应数据应该很小
    if (data_len > QUERY_CMD_RESPONSE_MAX_DATA_LEN) {
        return 1;  // 响应数据过长
    }

    // 构建协议帧: 帧头(1) + 指令(1) + 数据长度(1) + 数据(N) + CRC16(2) + 帧尾(2)
    tx_buffer[frame_len++] = QUERY_FRAME_HEAD_REPLY;     // 帧头
    tx_buffer[frame_len++] = cmd;                     // 指令
    // tx_buffer[frame_len++] = (data_len >> 8) & 0xFF; // 数据长度高字节
    tx_buffer[frame_len++] = data_len & 0xFF;         // 数据长度低字节

    // 添加数据
    if (data != NULL && data_len > 0) {
        memcpy(&tx_buffer[frame_len], data, data_len);
        frame_len += data_len;
    }

    // 计算CRC16 (对指令+数据长度+数据进行校验)
    uint16_t crc = crc16(&tx_buffer[1], 1 + 1 + data_len);
    tx_buffer[frame_len++] = (crc >> 8) & 0xFF;      // CRC16高字节
    tx_buffer[frame_len++] = crc & 0xFF;             // CRC16低字节

    // 帧尾
    tx_buffer[frame_len++] = QUERY_FRAME_TAIL1_REPLY;   // 帧尾
    tx_buffer[frame_len++] = QUERY_FRAME_TAIL2_REPLY;   // 帧尾

    // 发送数据
    usart_transmit(tx_buffer, frame_len);

    return 0;
}

static void handle_query_get_version_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t response_data[4] = {0};
    uint8_t error_code = QUERY_ERR_SUCCESS;

    // 验证数据长度
    if (data_len != QUERY_CMD_GET_VERSION_DATA_LEN) {
        error_code = QUERY_ERR_INVALID_LENGTH;
    } else {
        error_code = QUERY_ERR_SUCCESS;

        extern char* get_app_version(void);
        extern char* get_hardware_version(void);

        char* app_version = get_app_version();
        char* hardware_version = get_hardware_version();
        response_data[0] = hardware_version[0] - '0'; // 硬件版本只有1个字节
        response_data[1] = app_version[0] - '0';
        response_data[2] = app_version[1] - '0';
        response_data[3] = error_code;
    }

    // 发送响应
    usart_send_query_frame(QUERY_CMD_GET_VERSION, response_data, sizeof(response_data));
}


/**
 * @brief 处理调流量指令
 * @param data 数据指针
 * @param data_len 数据长度
 */
static void handle_query_flow_adjust_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = QUERY_ERR_SUCCESS;
    uint8_t response_data[2] = {0};
    extern uint8_t current_flow_level;

    // 验证数据长度（应该是1字节：流量等级）
    if (data_len != QUERY_CMD_FLOW_ADJUST_DATA_LEN) {
        error_code = QUERY_ERR_INVALID_LENGTH;
        // 发送错误响应
        usart_send_query_frame(QUERY_CMD_FLOW_ADJUST, &error_code, 1);
        return;
    }

    uint8_t flow_level = data[0];

    if (flow_level == FLOW_QUERY_CMD) {
        // 查询当前档位
        response_data[0] = current_flow_level;
        response_data[1] = QUERY_ERR_SUCCESS;
        usart_send_query_frame(QUERY_CMD_FLOW_ADJUST, response_data, 2);
    } else {
        // 设置流量档位
        if (flow_level >= FLOW_LEVEL_50_PERCENT && flow_level <= FLOW_LEVEL_100_PERCENT) {
            error_code = QUERY_ERR_SUCCESS;
            extern void magic_cool_set_target_vol_by_flow(uint8_t flow_level);
            magic_cool_set_target_vol_by_flow(flow_level);
            current_flow_level = flow_level;
            clear_fault_report();
        }else {
            error_code = QUERY_ERR_INVALID_PARAM;
        }

        response_data[0] = current_flow_level;
        response_data[1] = error_code;

        if( flow_level != 0xfe ) { // fe指令只有内部测试时才有，方便关闭输出
            // 发送响应
            usart_send_query_frame(QUERY_CMD_FLOW_ADJUST, response_data, 2);
        } else {
            extern void close_all_output(void);
            close_all_output();
        }
    }
}

static void handle_query_max_vol_cur_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = QUERY_ERR_SUCCESS;
    uint8_t response_data[9] = {0};
    uint32_t max_vol = 0;
    uint32_t max_cur = 0;

    extern uint32_t get_max_vol(void);
    extern uint32_t get_max_cur(void);

    max_vol = get_max_vol();
    max_cur = get_max_cur();

    response_data[0] = (max_vol >> 24) & 0xFF;
    response_data[1] = (max_vol >> 16) & 0xFF;
    response_data[2] = (max_vol >> 8) & 0xFF;
    response_data[3] = max_vol & 0xFF;
    response_data[4] = (max_cur >> 24) & 0xFF;
    response_data[5] = (max_cur >> 16) & 0xFF;
    response_data[6] = (max_cur >> 8) & 0xFF;
    response_data[7] = max_cur & 0xFF;
    response_data[8] = error_code;

    usart_send_query_frame(QUERY_CMD_QUERY_MAX_VOL_CUR, response_data, sizeof(response_data));
}

static void handle_query_current_vol_cur_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = QUERY_ERR_SUCCESS;
    uint8_t response_data[9] = {0};
    uint32_t current_vol = 0;
    uint32_t current_cur = 0;

    extern uint32_t get_current_vol(void);
    extern uint32_t get_current_cur(void);

    current_vol = get_current_vol();
    current_cur = get_current_cur();

    response_data[0] = (current_vol >> 24) & 0xFF;
    response_data[1] = (current_vol >> 16) & 0xFF;
    response_data[2] = (current_vol >> 8) & 0xFF;
    response_data[3] = current_vol & 0xFF;
    response_data[4] = (current_cur >> 24) & 0xFF;
    response_data[5] = (current_cur >> 16) & 0xFF;
    response_data[6] = (current_cur >> 8) & 0xFF;
    response_data[7] = current_cur & 0xFF;
    response_data[8] = error_code;

    usart_send_query_frame(QUERY_CMD_QUERY_CURRENT_VOL_CUR, response_data, sizeof(response_data));
}

static void handle_query_fault_report_cmd(uint8_t *data, uint16_t data_len)
{
    uint8_t error_code = ERR_SUCCESS;
    uint8_t response_data[2] = {0};

    // 验证数据长度（应该是1字节：故障查询或故障代码）
    if (data_len != CMD_FAULT_REPORT_DATA_LEN) {
        error_code = ERR_INVALID_LENGTH;
        usart_send_query_frame(QUERY_CMD_FAULT_REPORT, &error_code, 1);
        return;
    }

    uint8_t fault_query = data[0];

    if (fault_query == QUERY_CMD_QUERY_FAULT_REPORT) {
        // 查询故障代码
        response_data[0] = fault_code_to_report;
        response_data[1] = ERR_SUCCESS;
        usart_send_query_frame(QUERY_CMD_FAULT_REPORT, response_data, 2);
    } else {
        // 这个函数主要是主动上报故障，通过别的函数调用，这里只处理查询
        // 收到主机的响应，说明主机已经知道故障了，就不再上报了
        fault_report_state = FAULT_REPORT_STATE_IDLE;
        fault_report_retry_count = 0;
    }
}

static void usart_handle_query_command(uint8_t *frame_data, uint16_t frame_len)
{
    if (frame_data == NULL || frame_len < 3) {
        return;
    }

    uint8_t cmd = frame_data[0];
    uint16_t data_len = (frame_data[1] << 8) | frame_data[2];
    uint8_t *data = &frame_data[3];

    // 验证数据长度一致性
    if (frame_len != (PROTOCOL_FRAME_HEAD_TAIL_SIZE + data_len)) {
        uint8_t error_code = ERR_INVALID_LENGTH;
        usart_send_query_frame(cmd, &error_code, 1);
        return;
    }

    // 上位机程序处理所有指令
    switch (cmd) {
        case QUERY_CMD_GET_VERSION:
            handle_query_get_version_cmd(data, data_len);
            break;

        case QUERY_CMD_FLOW_ADJUST:
            handle_query_flow_adjust_cmd(data, data_len);
            break;

        case QUERY_CMD_QUERY_MAX_VOL_CUR:
            handle_query_max_vol_cur_cmd(data, data_len);
            break;

        case QUERY_CMD_QUERY_CURRENT_VOL_CUR:
            handle_query_current_vol_cur_cmd(data, data_len);
            break;

        case QUERY_CMD_FAULT_REPORT:
            handle_query_fault_report_cmd(data, data_len);
            break;

        default: {
                uint8_t error_code = QUERY_ERR_INVALID_CMD;
                usart_send_query_frame(cmd, &error_code, 1);
            }
            break;
    }
}

bool query_cmd_process(void)
{
    // 检查最小帧长度: 帧头(1) + 指令(1) + 长度(2) + CRC(2) + 帧尾(2) = 8字节
    if (uart_rx_len < QUERY_MIN_FRAME_SIZE) {
        return false;
    }

    // 检查帧头和帧尾
    if ((uart_cmd[0] == QUERY_FRAME_HEAD) &&
        (uart_cmd[uart_rx_len-2] == QUERY_FRAME_TAIL1) &&
        (uart_cmd[uart_rx_len-1] == QUERY_FRAME_TAIL2)) {

        // 解析协议: 帧头(1) + 指令(1) + 数据长度(2) + 数据(N) + CRC16(2) + 帧尾(2)
        uint8_t cmd = uart_cmd[1];
        uint16_t data_len = (uart_cmd[2] << 8) | uart_cmd[3];  // 大端序

        // 验证数据长度是否合理
        if (uart_rx_len != (QUERY_MIN_FRAME_SIZE + data_len)) {  // 帧头+指令+长度+数据+CRC+帧尾
            uint8_t error_code = ERR_INVALID_LENGTH;
            usart_send_query_frame(cmd, &error_code, 1);
            return false;
        }

        // 获取CRC16校验值 (在帧尾前2字节)
        uint16_t received_crc = (uart_cmd[uart_rx_len-4] << 8) | uart_cmd[uart_rx_len-3];

        // 计算CRC16: 对指令+数据长度+数据进行校验
        uint8_t *crc_data = &uart_cmd[1];  // 从指令开始
        uint16_t crc_len = 1 + 2 + data_len;  // 指令(1) + 长度(2) + 数据(data_len)
        uint16_t calculated_crc = crc16(crc_data, crc_len);

        if (calculated_crc == received_crc) {
            // 校验正确，处理命令
            // 传递给上位机处理: 指令+数据长度+数据
            usart_handle_query_command(&uart_cmd[1], crc_len);
        } else {
            // CRC校验错误，发送错误应答
            uint8_t error_code = ERR_INVALID_CRC;
            usart_send_query_frame(cmd, &error_code, 1);
            return false;
        }

        return true;
    }

    return false;
}
#endif

void uart_cmd_process(void)
{
    static uint32_t update_tick = 0;

    if (uart_rx_flag == 1) {
        memcpy(uart_cmd, uart_rxbuffer, uart_rx_len);

        protocol_cmd_process();

        #if ENABLE_QUERY_CMD
          query_cmd_process();
        #endif

        uart_rx_flag = 0;
        uart_rx_len = 0;
    } else {
        usart_fault_report_process();
    }
}

/**
 * @brief 主动上报故障代码
 * @param fault_code 故障代码
 */
void fault_report_active(protocol_fault_t fault_code)
{
    // 如果当前正在上报，则忽略新的上报请求
    if (fault_report_state != FAULT_REPORT_STATE_IDLE) {
        return;
    }

    if( fault_code_to_report == fault_code ) {
        return;
    }

    fault_code_to_report = fault_code;
    fault_report_state = FAULT_REPORT_STATE_WAIT_ACK;
    fault_report_retry_count = 0;

    // 立即发送第一次
#if !ENABLE_QUERY_CMD
    usart_send_frame(CMD_FAULT_REPORT, (uint8_t*)&fault_code_to_report, 1);
#else
    usart_send_query_frame(QUERY_CMD_FAULT_REPORT, (uint8_t*)&fault_code_to_report, 1);
#endif
    extern uint32_t get_systick(void);
    fault_report_timeout = get_systick() + FAULT_REPORT_TIMEOUT_MS;
    fault_report_retry_count++;
}

void usart_fault_report_process(void)
{
    if (fault_report_state == FAULT_REPORT_STATE_WAIT_ACK) {
        extern uint32_t get_systick(void);
        if (get_systick() >= fault_report_timeout) {
            if (fault_report_retry_count < FAULT_REPORT_MAX_RETRIES) {
                // 超时，重试
                #if !ENABLE_QUERY_CMD
                  usart_send_frame(CMD_FAULT_REPORT, (uint8_t*)&fault_code_to_report, 1);
                #else
                  usart_send_query_frame(QUERY_CMD_FAULT_REPORT, (uint8_t*)&fault_code_to_report, 1);
                #endif
                fault_report_timeout = get_systick() + FAULT_REPORT_TIMEOUT_MS;
                fault_report_retry_count++;
            } else {
                // 达到最大重试次数，停止上报
                fault_report_state = FAULT_REPORT_STATE_IDLE;
                fault_report_retry_count = 0;
            }
        }
    }
}

void clear_fault_report(void)
{
    fault_code_to_report = FAULT_NORMAL;
}












