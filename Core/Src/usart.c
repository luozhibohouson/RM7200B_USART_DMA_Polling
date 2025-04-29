#include "usart.h"
#include "main.h"

uint32_t uart_rx_flag = 0;
uint32_t uart_rx_len = 0;
uint8_t  uart_rxbuffer[30] = {0};
uint8_t  uart_cmd[30] = {0};

void USART_DMA_Configure(uint8_t *Buffer, uint8_t Length);
/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USART_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    sys_delayms(100); // 禁用SWD接口

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART1, ENABLE);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    USART_DMACmd(USART1, ENABLE);

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

    USART_DMA_Configure(uart_rxbuffer, sizeof(uart_rxbuffer));
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
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        DMA_Cmd(DMA1_Channel2, DISABLE);

        uart_rx_len = sizeof(uart_rxbuffer) - DMA_GetCurrDataCounter(DMA1_Channel2);

        if( uart_rx_len > 0 ) {
            uart_rx_flag = 1;
        }

        USART_ReceiveData(USART1); //通过读取DR寄存器，清除IDLE标志位

        DMA_Cmd(DMA1_Channel2, ENABLE);
    }
}

void uart_cmd_process(void)
{
    static uint32_t update_tick = 0;

    if (uart_rx_flag == 1) {
        memcpy(uart_cmd, uart_rxbuffer, uart_rx_len);

        // 去掉最后一个字节"\n"
        if( uart_cmd[uart_rx_len-1] == '\n' ) {
            uart_rx_len--;
        }

        if ((uart_cmd[0] == 0xAA) && (uart_cmd[uart_rx_len-1] == 0x55)) {
            uint8_t checkSum = 0;
            for(int i=1; i<uart_rx_len-2; i++) {
                checkSum ^= uart_cmd[i];
            }
            if (checkSum != uart_cmd[uart_rx_len-2]) {
                printf("checkSum:%X, error\r\n", checkSum);
                uart_rx_flag = 0;
                return;
            }

            // magic_cool_parse_cmd((uint8_t*)usb_recv_buf, DATA_TYPE_UART);
        }
    }
    // else if( is_magic_cool_enable() ) {
    //     if( (get_systick() - update_tick) > 500 ) {
    //         update_tick = get_systick();
    //         uint8_t uart_tx_len = magic_cool_update_uploadData(usb_send_buf, DATA_TYPE_UART);
    //         // printf("usb state: %d\r\n", CDC_Transmit_FS(usb_send_buf, uart_tx_len));
    //         usart_transmit(usb_send_buf, uart_tx_len);
    //     }
    // }
}













