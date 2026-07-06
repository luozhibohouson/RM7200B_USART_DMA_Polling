#include "opa.h"

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void OPAMP_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* OPA1 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStruct);
    #if 1
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    #else
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;
    #endif
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_OPA1, ENABLE);

    OPAMP_ModeConfig(OPAMP1, OPAMP_Mode_LowPower);

    OPAMP_Cmd(OPAMP1, ENABLE);

    /* OPA2 */
#if 0
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_OPA2, ENABLE);

    OPAMP_ModeConfig(OPAMP2, OPAMP_Mode_LowPower);

    OPAMP_Cmd(OPAMP2, ENABLE);

#endif
}

void OPA_Enable(void)
{
    OPAMP_Cmd(OPAMP1, ENABLE);
    // OPAMP_Cmd(OPAMP2, ENABLE);
}

void OPA_Disable(void)
{
    OPAMP_Cmd(OPAMP1, DISABLE);
    // OPAMP_Cmd(OPAMP2, DISABLE);
}



