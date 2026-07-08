/***********************************************************************************************************************
    @file    main.c
    @author  FAE Team
    @date    15-Mar-2023
    @brief   THIS FILE PROVIDES ALL THE SYSTEM FUNCTIONS.
  **********************************************************************************************************************
    @attention

    <h2><center>&copy; Copyright(c) <2023> <MindMotion></center></h2>

      Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
    following conditions are met:
    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
       the following disclaimer in the documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
       promote products derived from this software without specific prior written permission.

      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************************************************************/

/* Define to prevent recursive inclusion */
#define _MAIN_C_

/* Files include */
#include "platform.h"
#include "usart.h"
#include "main.h"
#include "mm32spin0230_it.h"
#include "opa.h"
#include "magic_cool.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "controller.h"
#include "flash_ops.h"
#include "comp.h"


void hardware_init(uint8_t delay_enable);
void hardware_deinit(void);
void deep_sleep(void);
/**
  * @addtogroup MM32SPIN0230_LibSamples
  * @{
  */

typedef struct {
    uint16_t app_valid;
    uint16_t app_success;
    uint16_t app_size;
    uint16_t app_crc16;
    char bootloader_version[2];    // 由Bootloader写入
    char app_version[2];           // 由App写入
    char hardware_version[2];      // 由App写入
} app_t;

static app_t app_info;
char* get_app_version(void)
{
    return app_info.app_version;
}

char* get_hardware_version(void)
{
    return app_info.hardware_version;
}

// 增加升级成功标志，防止烧错固件，导致bootloader无法启动
void app_upgrade_success(void)
{
    flash_read_bytes(PARAM_START_ADDR, (uint8_t*)&app_info, sizeof(app_info));

    if( app_info.app_valid == APP_VALID_FLAG && app_info.app_success == APP_SUCCESS_FLAG ) {
        return;
    }

    app_info.app_success = APP_SUCCESS_FLAG;
    // strncpy(app_info.app_version, APP_VERSION, sizeof(app_info.app_version));
    // strncpy(app_info.hardware_version, HARDWARE_VERSION, sizeof(app_info.hardware_version));
    app_info.app_version[0] = APP_VERSION[0]-'0';
    app_info.app_version[1] = APP_VERSION[1]-'0';
    app_info.hardware_version[0] = HARDWARE_VERSION[0]-'0';
    app_info.hardware_version[1] = 0;

    flash_erase_page((uint16_t)(PARAM_START_ADDR / FLASH_PAGE_SIZE));
    flash_write_halfword(PARAM_START_ADDR, (uint16_t*)&app_info, sizeof(app_info));
#if defined(ERASE_FLOW_CFG) && (ENABLE_WRITE_FREQ == 1)
    //NOTE: 每次升级成功后擦除保留的频率值
    extern void erase_flow_freq_cfg(void);
    erase_flow_freq_cfg();
#endif
}

int a=5;

void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_EXTI, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SYSCFG, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    // 串口RX引脚配置唤醒
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource13);

    EXTI_StructInit(&EXTI_InitStruct);
    EXTI_InitStruct.EXTI_Line    = EXTI_Line13;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

#if (Magic_Cool_Customer != CY_ChuanYi)
    // 按键唤醒配置
    #if (HARDWARE_VERSION_CODE == HW_VER_1_0_INT)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
        GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);

        EXTI_StructInit(&EXTI_InitStruct);
        EXTI_InitStruct.EXTI_Line    = EXTI_Line3;
        EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStruct);

        NVIC_InitStruct.NVIC_IRQChannel = EXTI2_3_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
    #else
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
        GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);

        EXTI_StructInit(&EXTI_InitStruct);
        EXTI_InitStruct.EXTI_Line    = EXTI_Line3;
        EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStruct);

        NVIC_InitStruct.NVIC_IRQChannel = EXTI2_3_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
    #endif
#endif
}

void EXTI_Configure_deinit(void)
{
    RCC->APB1RSTR |= RCC_APB1Periph_EXTI;
    RCC->APB1RSTR &= ~(RCC_APB1Periph_EXTI);
}
/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/
uint8_t sys_delayms(int ms)
{
    int tickx = sys_tick + ms;

    while(tickx > sys_tick) {
#if defined(ENABLE_USART)
        uart_cmd_process();
        extern bool get_stop_delay_ms(void);
        if( get_stop_delay_ms() ) {
            return 1;
        }
#endif
    }

    return 0;
}

uint32_t get_systick(void)
{
    return sys_tick;
}
/***********************************************************************************************************************
  * @brief  This function is main entrance
  * @note   main
  * @param  none
  * @retval none
  *********************************************************************************************************************/
int main(void)
{
    app_upgrade_success();

    PLATFORM_Init();

    hardware_init(ENABLE);

    rm_magic_config();

    while (1)
    {
        #if UART_DEBUG
        ProcessDebugUartData();

        if(DebugMode)
        continue;
        #endif

        
        // key_scan();
        rm_magic_run();
#if defined(ENABLE_USART)
        uart_cmd_process();
#endif
        deep_sleep();
    }
}

/**
  * @}
  */
void hardware_init(uint8_t delay_enable)
{
#if defined(ENABLE_USART)
    // 从boot跳转，需要延时。从休眠唤醒不需要延时
    USART_Configure(115200, delay_enable);
#elif defined(ENABLE_PRINTF)
    USART_PrintfConfigure(1000000);
#endif

    GPIO_Configure();
    ADC_Configure();
    // TIM13_Configure();
    TIM1_Configure();
    OPAMP_Configure();
#if (HARDWARE_VERSION_CODE == HW_VER_2_0_INT)
    COMP_Configure();
#endif
    // EXTI_Configure();
}

void hardware_deinit(void)
{

}

extern volatile bool t10ms_f;
uint8_t deep_sleep_flag = 0;
void deep_sleep(void)
{
    static uint8_t sleep_time = 0;
#if 0
    static uint32_t deep_sleep_tick = 0;
    extern uint8_t get_magic_cool_mode(void);
    if( get_magic_cool_mode() == 0 && get_systick() - deep_sleep_tick >= 5*1000 ) {
        deep_sleep_tick = get_systick();
        deep_sleep_flag = DEEP_SLEEP_FLAG_SLEEP;
    }
#endif
    if( deep_sleep_flag == DEEP_SLEEP_FLAG_SLEEP ) 
    {

        extern void close_all_output(void);
        close_all_output();

        if( t10ms_f ) {
            t10ms_f = 0;
            sleep_time++;
        }

        if( sleep_time >= 2 )
        {
            #if ENABLE_WRITE_FREQ
                extern void write_final_freq_to_flash(void);
                write_final_freq_to_flash();
            #endif

        #if defined(ENABLE_USART)
            // 等待USART1发送完成
            while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC));
        #endif

            // 复位各个模块
            RCC->APB1RSTR |= RCC_APB1Periph_OPA1 | \
                            RCC_APB1Periph_OPA2 | \
                            RCC_APB1Periph_ADC1 | \
                            RCC_APB1Periph_USART1 | \
                            RCC_APB1Periph_TIM13 | \
                            RCC_APB1Periph_TIM1 | \
                            RCC_APB1Periph_COMP;

            RCC->APB1RSTR &= ~(RCC_APB1Periph_OPA1 | \
                            RCC_APB1Periph_OPA2 | \
                            RCC_APB1Periph_ADC1 | \
                            RCC_APB1Periph_USART1 | \
                            RCC_APB1Periph_TIM13 | \
                            RCC_APB1Periph_TIM1 | \
                            RCC_APB1Periph_COMP);

            RCC->AHBRSTR |= RCC_AHBPeriph_DMA;
            RCC->AHBRSTR &= ~(RCC_AHBPeriph_DMA);

            // 配置GPIO为模拟输入
            GPIO_InitTypeDef  GPIO_InitStruct;

            GPIO_StructInit(&GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_All;
            GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
            GPIO_Init(GPIOA, &GPIO_InitStruct);

            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_All;
            GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
            GPIO_Init(GPIOB, &GPIO_InitStruct);
        #if (HARDWARE_VERSION_CODE == HW_VER_1_0_INT)
            GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_RESET);  //PIN4-->PWM
            GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET); //VIN分压检测引脚 20PIN
            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_15|GPIO_Pin_9;
            GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
            GPIO_Init(GPIOA, &GPIO_InitStruct);
        #elif (HARDWARE_VERSION_CODE == HW_VER_2_0_INT)
            GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_RESET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET); //VIN分压检测引脚
            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_15;
            GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
            GPIO_Init(GPIOA, &GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
            GPIO_Init(GPIOB, &GPIO_InitStruct);
        #endif
            uint32_t rcc_cfgr_value = RCC->CFGR;
            // AHB配置为8分频 60M/8=7.5M
            RCC->CFGR = 0x000000A0;
            // 关闭FLASH预取缓存(AHB时钟必须低于30MHz才能开启or关闭预取缓存)
            FLASH->ACR &=  ~(0x01U << FLASH_ACR_PRFTBE_Pos);

            EXTI_Configure();

            __nop();__nop();__nop();
            __nop();__nop();__nop();

            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            __WFI();

            __nop();__nop();__nop();
            __nop();__nop();__nop();

            FLASH->ACR |=  (0x01U << FLASH_ACR_PRFTBE_Pos);
            RCC->CFGR = rcc_cfgr_value;

            EXTI_Configure_deinit();

            hardware_init(DISABLE);

            // sys_delayms(5);

            sleep_time = 0;

            deep_sleep_flag = DEEP_SLEEP_FLAG_WAKEUP;

            extern void set_stop_delay_ms(bool enable);
            set_stop_delay_ms(false);

            // 发送唤醒标志给上位机
            extern uint8_t usart_send_frame(uint8_t cmd, uint8_t *data, uint8_t data_len);
            uint8_t response_data[2] = {0};
            response_data[0] = deep_sleep_flag;
            response_data[1] = 0;
            usart_send_frame(CMD_DEEP_SLEEP, response_data, sizeof(response_data));
        }
    }
}

/**
  * @}
  */

/**
  * @}
  */

/********************************************** (C) Copyright MindMotion **********************************************/

