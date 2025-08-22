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
    strncpy(app_info.app_version, APP_VERSION, sizeof(app_info.app_version));
    strncpy(app_info.hardware_version, HARDWARE_VERSION, sizeof(app_info.hardware_version));

    flash_erase_page((uint16_t)(PARAM_START_ADDR / FLASH_PAGE_SIZE));
    flash_write_halfword(PARAM_START_ADDR, (uint16_t*)&app_info, sizeof(app_info));
}


void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_EXTI, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SYSCFG, ENABLE);

    // 按键引脚配置唤醒
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_8|GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource13);

    EXTI_StructInit(&EXTI_InitStruct);
    EXTI_InitStruct.EXTI_Line    = EXTI_Line8|EXTI_Line13;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
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
void sys_delayms(int ms)
{
    int tickx = sys_tick + ms;

    while(tickx > sys_tick) {
#if ENABLE_USART
        uart_cmd_process();
#endif
    }
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
        // key_scan();
        rm_magic_run();
#if ENABLE_USART
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
#if ENABLE_USART
    // 从boot跳转，需要延时。从休眠唤醒不需要延时
    USART_Configure(115200, delay_enable);
#elif ENABLE_PRINTF
    USART_PrintfConfigure(1000000);
#endif

    GPIO_Configure();
    ADC_Configure();
    TIM13_Configure();
    TIM1_Configure();
    OPAMP_Configure();
    // EXTI_Configure();
}

void hardware_deinit(void)
{

}

extern volatile bool t1s_f;
extern uint8_t  magic_cool_mode;
// void deep_sleep(void)
// {
//     static uint8_t sleep_time = 0;
//     if( t1s_f ) {
//         t1s_f = 0;
//         sleep_time++;
//     }
//     if( !magic_cool_mode ) {
//         if( sleep_time > 5 ) {

//             // 复位各个模块
//             RCC->APB1RSTR |= RCC_APB1Periph_OPA1 | \
//                             RCC_APB1Periph_OPA2 | \
//                             RCC_APB1Periph_ADC1 | \
//                             RCC_APB1Periph_USART1 | \
//                             RCC_APB1Periph_TIM13 | \
//                             RCC_APB1Periph_TIM1;

//             RCC->APB1RSTR &= ~(RCC_APB1Periph_OPA1 | \
//                             RCC_APB1Periph_OPA2 | \
//                             RCC_APB1Periph_ADC1 | \
//                             RCC_APB1Periph_USART1 | \
//                             RCC_APB1Periph_TIM13 | \
//                             RCC_APB1Periph_TIM1);

//             RCC->AHBRSTR |= RCC_AHBPeriph_DMA;
//             RCC->AHBRSTR &= ~(RCC_AHBPeriph_DMA);

//             // 配置GPIO为模拟输入
//             GPIO_InitTypeDef  GPIO_InitStruct;

//             GPIO_StructInit(&GPIO_InitStruct);
//             GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_All;
//             GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_High;
//             GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
//             GPIO_Init(GPIOA, &GPIO_InitStruct);

//             GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_All;
//             GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_High;
//             GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
//             GPIO_Init(GPIOB, &GPIO_InitStruct);

//             GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_SET);
//             GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET);
//             GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_15|GPIO_Pin_9;
//             GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
//             GPIO_Init(GPIOA, &GPIO_InitStruct);

//             EXTI_Configure();

//             __nop();__nop();__nop();
//             __nop();__nop();__nop();

//             SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//             __WFI();

//             __nop();__nop();__nop();
//             __nop();__nop();__nop();

//             sleep_time = 0;

//             hardware_init(DISABLE);
//         }
//     } else {
//         sleep_time = 0;
//     }
// }

extern volatile bool t10ms_f;
uint8_t deep_sleep_flag = 0;
void deep_sleep(void)
{
    static uint8_t sleep_time = 0;
    if( deep_sleep_flag == DEEP_SLEEP_FLAG_SLEEP ) {

        magic_cool_mode = 0;

        if( t10ms_f ) {
            t10ms_f = 0;
            sleep_time++;
        }

        if( sleep_time >= 5 ) {
            // 复位各个模块
            RCC->APB1RSTR |= RCC_APB1Periph_OPA1 | \
                            RCC_APB1Periph_OPA2 | \
                            RCC_APB1Periph_ADC1 | \
                            RCC_APB1Periph_USART1 | \
                            RCC_APB1Periph_TIM13 | \
                            RCC_APB1Periph_TIM1;

            RCC->APB1RSTR &= ~(RCC_APB1Periph_OPA1 | \
                            RCC_APB1Periph_OPA2 | \
                            RCC_APB1Periph_ADC1 | \
                            RCC_APB1Periph_USART1 | \
                            RCC_APB1Periph_TIM13 | \
                            RCC_APB1Periph_TIM1);

            RCC->AHBRSTR |= RCC_AHBPeriph_DMA;
            RCC->AHBRSTR &= ~(RCC_AHBPeriph_DMA);

            // 配置GPIO为模拟输入
            GPIO_InitTypeDef  GPIO_InitStruct;

            GPIO_StructInit(&GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_All;
            GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_High;
            GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
            GPIO_Init(GPIOA, &GPIO_InitStruct);

            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_All;
            GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_High;
            GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
            GPIO_Init(GPIOB, &GPIO_InitStruct);

            GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_SET);
            GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET);
            GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_15|GPIO_Pin_9;
            GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
            GPIO_Init(GPIOA, &GPIO_InitStruct);

            EXTI_Configure();

            __nop();__nop();__nop();
            __nop();__nop();__nop();

            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            __WFI();

            __nop();__nop();__nop();
            __nop();__nop();__nop();

            EXTI_Configure_deinit();

            hardware_init(DISABLE);

            sys_delayms(10);

            sleep_time = 0;

            deep_sleep_flag = DEEP_SLEEP_FLAG_WAKEUP;

            // 发送唤醒标志给上位机
            extern uint8_t usart_send_frame(uint8_t cmd, uint8_t *data, uint16_t data_len);
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

