#include "usart.h"
#include "main.h"

uint32_t uart_rx_flag = 0;
uint32_t uart_rx_len = 0;
uint8_t  uart_rxbuffer[30] = {0};
uint8_t  uart_cmd[30] = {0};

static __IO uint32_t vector_table[48] __attribute__((at(0x20000000)));
static __IO upgrade_state_t upgrade_state __attribute__((at(0x200000C0)));

void USART_DMA_Configure(uint8_t *Buffer, uint8_t Length);
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

    if( delay_enable != 0 ) {
        sys_delayms(500); // 禁用SWD接口
    }

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
uint8_t usart_send_frame(uint8_t cmd, uint8_t *data, uint16_t data_len)
{
    uint8_t tx_buffer[16];  // 足够容纳任何响应帧：帧头(1)+指令(1)+长度(1)+数据(3)+CRC(2)+帧尾(2)=11字节
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
    #ifndef CHUAN_YIN_VERSION  // TODO:传音的版本，发送的ASCII码。其他客户发送的为十进制数
        response_data[0] -= '0';
        response_data[1] -= '0';
        response_data[2] -= '0';
    #endif
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

    // 验证数据长度（应该是1字节：流量等级）
    if (data_len != CMD_FLOW_ADJUST_DATA_LEN) {
        error_code = ERR_INVALID_LENGTH;
    } else {
        uint8_t flow_level = data[0];

        // 验证参数
        if (flow_level > FLOW_LEVEL_100_PERCENT || flow_level < FLOW_LEVEL_50_PERCENT) {
            error_code = ERR_INVALID_PARAM;
        } else {
            error_code = ERR_SUCCESS;
            extern void magic_cool_set_target_vol_by_flow(uint8_t flow_level);
            magic_cool_set_target_vol_by_flow(flow_level);
        }
    }

    // 发送响应
    usart_send_frame(CMD_FLOW_ADJUST, &error_code, 1);
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
            extern uint8_t magic_cool_mode;
            magic_cool_mode = 0;
        }
    }

    response_data[0] = deep_sleep_flag;
    response_data[1] = error_code;

    // 发送响应
    usart_send_frame(CMD_DEEP_SLEEP, response_data, sizeof(response_data));
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
void usart_handle_command(uint8_t *frame_data, uint16_t frame_len)
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

    // APP程序只接收调流量和系统升级指令
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

void uart_cmd_process(void)
{
    static uint32_t update_tick = 0;

    if (uart_rx_flag == 1) {
        memcpy(uart_cmd, uart_rxbuffer, uart_rx_len);

        // 检查最小帧长度: 帧头(1) + 指令(1) + 长度(2) + CRC(2) + 帧尾(2) = 8字节
        if (uart_rx_len < PROTOCOL_MIN_FRAME_SIZE) {
            uart_rx_flag = 0;
            uart_rx_len = 0;
            return;
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
                uart_rx_flag = 0;
                uart_rx_len = 0;
                uint8_t error_code = ERR_INVALID_LENGTH;
                usart_send_frame(cmd, &error_code, 1);
                return;
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
                usart_handle_command(&uart_cmd[1], crc_len);
            } else {
                // CRC校验错误，发送错误应答
                uint8_t error_code = ERR_INVALID_CRC;
                usart_send_frame(cmd, &error_code, 1);
            }
        }

        uart_rx_flag = 0;
        uart_rx_len = 0;
    }
}













