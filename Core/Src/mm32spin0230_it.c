/***********************************************************************************************************************
    @file    mm32spin0230_it.c
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
#define _MM32SPIN0230_IT_C_

/* Files include */
#include "platform.h"
#include "usart.h"
#include "mm32spin0230_it.h"
#include "define.h"

/**
  * @addtogroup MM32SPIN0230_LibSamples
  * @{
  */

/**
  * @addtogroup USART
  * @{
  */

/**
  * @addtogroup USART_DMA_Polling
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
  * @brief  This function handles NMI exception
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void NMI_Handler(void)
{
}

/***********************************************************************************************************************
  * @brief  This function handles Hard Fault exception
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    pwm_enable(DISABLE);
    while (1)
    {
    }
}

/***********************************************************************************************************************
  * @brief  This function handles SVCall exception
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void SVC_Handler(void)
{
}

/***********************************************************************************************************************
  * @brief  This function handles PendSVC exception
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void PendSV_Handler(void)
{
}

/***********************************************************************************************************************
  * @brief  This function handles SysTick Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
volatile uint32_t sys_tick = 0;
volatile bool t1s_f = 0;
volatile bool t10ms_f = 0;
extern uint8_t deep_sleep_flag;
void SysTick_Handler(void)
{
    if (0 != PLATFORM_DelayTick)
    {
        PLATFORM_DelayTick--;
    }

    sys_tick += 1;

    if( sys_tick % 1000 == 0 ) {
        t1s_f = 1;
    }

    if( (deep_sleep_flag == DEEP_SLEEP_FLAG_SLEEP) && sys_tick % 10 == 0 ) {
        t10ms_f = 1;
    } else {
        t10ms_f = 0;
    }
#if !ENABLE_USART
    extern void key_scan();
    key_scan();
#endif

#if ENABLE_KEY_VOL_CFG
    extern void magic_cool_led_control(uint32_t tick);
    magic_cool_led_control(sys_tick);
#endif
}

/***********************************************************************************************************************
  * @brief  This function handles DMA1_Channel1 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
// void DMA1_Channel1_IRQHandler(void)
// {
//     if (RESET != DMA_GetITStatus(DMA1_IT_TC1))
//     {
//         DMA_Cmd(DMA1_Channel1, DISABLE);

//         DMA_ClearITPendingBit(DMA1_IT_TC1);
//     }
// }

/***********************************************************************************************************************
  * @brief  This function handles DMA1_Channel2 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
    if (RESET != DMA_GetITStatus(DMA1_IT_TC2))
    {
        DMA_Cmd(DMA1_Channel2, DISABLE);

        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }
}

/***********************************************************************************************************************
  * @brief  This function handles USART1 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USART1_IRQHandler(void)
{
    usart_callback();
}


/***********************************************************************************************************************
  * @brief  This function handles EXTI4_15 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void EXTI4_15_IRQHandler(void)
{
    if (RESET != EXTI_GetITStatus(EXTI_Line8))
    {
        EXTI_ClearITPendingBit(EXTI_Line8);
    }

    if (RESET != EXTI_GetITStatus(EXTI_Line13))
    {
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/********************************************** (C) Copyright MindMotion **********************************************/

