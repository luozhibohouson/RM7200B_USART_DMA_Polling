#include "comp.h"

void COMP_Configure(void)
{
    COMP_InitTypeDef COMP_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // --- 硬件初始化 ---
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SYSCFG, ENABLE); // EXTI 需要 SYSCFG 时钟

    // 配置 PA8 为比较器输入引脚
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 配置 COMP2
    COMP_StructInit(&COMP_InitStruct);
    COMP_InitStruct.COMP_Invert     = COMP_InvertingInput_3; // 反相输入连接到内部参考电压CRV
    COMP_InitStruct.COMP_NonInvert  = COMP_NonInvertingInput_1;  // 正相输入连接到 PB2
    COMP_InitStruct.COMP_Output     = COMP_Output_None;
    COMP_InitStruct.COMP_OutputPol  = COMP_Pol_NonInvertedOut;
    COMP_InitStruct.COMP_Hysteresis = COMP_Hysteresis_No;
    COMP_InitStruct.COMP_Mode       = COMP_Mode_MediumPower;
    COMP_InitStruct.COMP_OFLT       = COMP_Filter_4_Period;
    COMP_Init(COMP2, &COMP_InitStruct);

    // 设置内部参考电压 CRV 为 VDDA/2
    COMP_SetCrv(COMP_CRV_SRC_VDDA, (0xFF*20/33));
    COMP_CrvCmd(ENABLE);

    // 使能 COMP2
    COMP_Cmd(COMP2, ENABLE);

    // 配置 EXTI 中断线 (COMP2 对应 EXTI Line 20)
    EXTI_StructInit(&EXTI_InitStruct);
    EXTI_InitStruct.EXTI_Line    = EXTI_Line20;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; // 上升沿和下降沿都触发
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    // 配置 NVIC
    NVIC_InitStruct.NVIC_IRQChannel = COMP1_2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

// 在中断服务程序中调用
void COMP_IRQHandler(void)
{
    // 检查是否是 COMP2 (EXTI Line 20) 触发的中断
    if (EXTI_GetITStatus(EXTI_Line20) != RESET)
    {
        // 判断当前比较器输出电平来决定是上升沿还是下降沿
        // 注意：这里的逻辑依赖于 COMP_OutputPol 的配置
        // 如果是 COMP_Pol_NonInvertedOut:
        //   - 输出高电平 (1) -> 发生了上升沿 (In+ > In-)
        //   - 输出低电平 (0) -> 发生了下降沿 (In+ < In-)
        // if (COMP_GetOutputLevel(COMP2) == 1) {

        // } else {

        // }



        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line20);
    }
}
