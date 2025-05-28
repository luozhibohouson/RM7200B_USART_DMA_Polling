#include "tim.h"

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TIM13_Configure(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct;
    RCC_ClocksTypeDef       RCC_Clocks;
    TIM_OCInitTypeDef       TIM_OCInitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    uint32_t TimerPeriod;

    RCC_GetClocksFreq(&RCC_Clocks);

    /* Compute the value to be set in ARR regiter to generate signal frequency at 50 Khz */
    TimerPeriod = (RCC_Clocks.PCLK1_Frequency / PWM1_FREQ ) - 1;

    /* TIM13 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Prescaler         = 0;
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period            = TimerPeriod;
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_Div1;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStruct);

    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse        = (TimerPeriod*60/100);
    TIM_OCInitStruct.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState  = TIM_OCIdleState_Set;

    TIM_OC1Init(TIM13, &TIM_OCInitStruct);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_6);    /* TIM13_CH1 */

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_Cmd(TIM13, ENABLE);

    TIM_CtrlPWMOutputs(TIM13, DISABLE);
}

void tim13_set_duty(uint32_t duty)
{
    TIM_SetCompare1(TIM13, duty);
}

// uint32_t pwm1_min_duty = PWM1_FREQ*2/11; //最小为0.6V
// uint32_t pwm1_max_duty = PWM1_FREQ*10/11; //最大为3.0V
void pwm1_set_duty(uint32_t duty)
{
    // uint32_t temp = 0;

    // if( duty > pwm1_max_duty ) {
    //     duty = pwm1_max_duty;
    // } else if( duty < pwm1_min_duty ) {
    //     duty = pwm1_min_duty;
    // }

    tim13_set_duty(duty);
}

void TIM1_Configure(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct;
    TIM_BDTRInitTypeDef     TIM_BDTRInitStruct;
    TIM_OCInitTypeDef       TIM_OCInitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM1, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Prescaler         = 0;
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period            = 10000 - 1;
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_Div1;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStruct.TIM_Pulse        = 5000-1;
    TIM_OCInitStruct.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState  = TIM_OCIdleState_Set;
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Set;

    TIM_OC3Init(TIM1, &TIM_OCInitStruct);

    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_BDTRStructInit(&TIM_BDTRInitStruct);
    TIM_BDTRInitStruct.TIM_OSSRState       = TIM_OSSRState_Enable;
    TIM_BDTRInitStruct.TIM_OSSIState       = TIM_OSSIState_Enable;
    TIM_BDTRInitStruct.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStruct.TIM_DeadTime        = 50;
    TIM_BDTRInitStruct.TIM_Break           = TIM_Break_Enable;
    TIM_BDTRInitStruct.TIM_BreakPolarity   = TIM_BreakPolarity_High;
    TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);

    // TIM_BreakInputFilterConfig(TIM1, TIM_IOBKIN_BKIN5, TIM_BKINF_16);
    // TIM_BreakInputFilterCmd(TIM1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_6);  /* TIM1_CH3N  */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);  /* TIM1_CH3  */

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_Cmd(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
}

uint32_t tim1_min_freq = 0;
uint32_t tim1_max_freq = 0;
uint32_t tim1_freq = 0;
uint32_t tim1_reload = 0;
uint32_t tim1_duty = 0;
uint32_t tim1_duty_dt = 0;

void timer1_set_output(uint32_t freq, uint32_t duty)
{
    uint32_t compare = 0;
    uint32_t autoreload = SystemCoreClock / freq;
    TIM_SetAutoreload(TIM1, autoreload);
    compare = autoreload * duty / 100;
    TIM_SetCompare3(TIM1, compare);
    tim1_freq = freq;
    tim1_reload = autoreload;
    tim1_duty = compare;
}

void timer1_set_freq(uint32_t freq)
{
    uint32_t autoreload = SystemCoreClock / freq;
    if (tim1_duty > (autoreload >> 1)) {  /// 占空比保护
        tim1_duty = (autoreload >> 1) - 1;
        TIM_SetCompare3(TIM1, tim1_duty);
    }
    TIM_SetAutoreload(TIM1, autoreload);
    tim1_freq = freq;
    tim1_reload = autoreload;
}

void timer1_set_reload(uint32_t autoreload)
{
    TIM_SetAutoreload(TIM1, autoreload);
    tim1_freq = SystemCoreClock / autoreload;
    tim1_reload = autoreload;
}

void timer1_set_duty(uint32_t duty)
{
    TIM_SetCompare3(TIM1, duty);
    tim1_duty = duty;
}


void tim1_set_deadtime(TIM_TypeDef *TIMX, uint32_t deadtime)
{
    TIMX->BDTR &= 0xFFFFFF00;
    TIMX->BDTR |= deadtime;
}

/*
模式1：与时钟相同7bit数据[6:0]， 0-127个时钟
模式2：时钟2分频6bit数据(64 + [5:0])*2， 128-255个时钟 步进2
模式2：时钟8分频6bit数据(32 + [4:0])*8， 256-511个时钟 步进8
模式2：时钟16分频6bit数据(32 + [4:0])*16，512-1024个时钟 步进16
*/
void timer1_set_duty_bydt(uint32_t duty_dt)
{
    uint32_t time = 0;
    uint32_t temp = duty_dt;

    tim1_duty_dt = temp;
    if (temp <= 127) {
        tim1_set_deadtime(TIM1, temp);
    } else if (temp > 127 && temp <= 255) { // 步进2
        time = temp >> 1;
        temp = time - 64;
        temp |= 4 << 5;
        tim1_set_deadtime(TIM1, temp);
    }  else if (temp > 255 && temp <= 511) {  // 步进8
        time = temp >> 3;
        temp = time - 32;
        temp |= 6 << 5;
        tim1_set_deadtime(TIM1, temp);
    }  else if (temp > 511 && temp <= 1024) {  // 步进16
        time = temp >> 4;
        temp = time - 32;
        temp |= 7 << 5;
        tim1_set_deadtime(TIM1, temp);
    } else {
        temp = 0xff;
        tim1_set_deadtime(TIM1, temp);
    }
}



void pwm_set_config(uint32_t freq, uint32_t duty)
{
    timer1_set_output(freq, duty);
}

void pwm_set_freq(uint32_t freq)
{
    timer1_set_output(freq, 50);
}

uint32_t pwm_get_freq(void)
{
    return tim1_freq;
}

uint32_t pwm_get_reload(void)
{
    return tim1_reload;
}


void pwm_set_freq_limt(uint32_t min, uint32_t max)
{
    tim1_max_freq = max;
    tim1_min_freq = min;
}

void pwm_set_dt(uint32_t dt)
{
    uint32_t temp = 0;

    temp = tim1_reload * dt / 100;
    timer1_set_duty_bydt(temp);
}

void pwm_freq_increase(uint32_t step)
{
    tim1_freq = tim1_freq + step;
    if (tim1_freq >= tim1_max_freq)
        tim1_freq = tim1_max_freq;
    //timer1_set_freq(tim1_freq);
    timer1_set_output(tim1_freq, 50);
}

void pwm_freq_decrease(uint32_t step)
{
    tim1_freq = tim1_freq - step;
    if (tim1_freq <= tim1_min_freq)
        tim1_freq = tim1_min_freq;
    //timer1_set_freq(tim1_freq);
    timer1_set_output(tim1_freq, 50);
}

int pwm_duty_increase(uint32_t step)
{
    int ret = 0;
    tim1_duty = tim1_duty + step;
    if (tim1_duty > (tim1_reload >> 1)) {
        tim1_duty = tim1_reload >> 1;
        ret = 1;
    }
    timer1_set_duty(tim1_duty);
//    printf("ch%d %d, %d\r\n",  tim1_reload, tim1_duty);
    return ret;
}

int pwm_duty_bydt_increase(uint32_t step)
{
    int ret = 0;
    if (tim1_duty_dt < (step + 10)) {
        tim1_duty_dt = 10;
        ret = 1;
    } else {
        tim1_duty_dt = tim1_duty_dt - step;
    }
    timer1_set_duty_bydt(tim1_duty_dt);
    return ret;
}

int pwm_duty_decrease(uint32_t step)
{
    int ret = 0;

    if (tim1_duty < step) {
        tim1_duty = 10;
        ret = 1;
    } else {
        tim1_duty = tim1_duty - step;
    }
    timer1_set_duty(tim1_duty);
    return ret;
}

int pwm_duty_set_pid(uint32_t duty)
{
    int ret = 0;
    if (duty <= 150) {
        duty = 150;
        tim1_duty = duty;
        ret = 1;
    } else if (duty > (tim1_reload >> 1)) {
        tim1_duty = tim1_reload >> 1;
        ret = 1;
    } else {
        tim1_duty = duty;
    }
    timer1_set_duty(tim1_duty);
//    timer1_set_duty_bydt(tim1_duty);
    return ret;
}

int pwm_duty_set_pid_dt(uint32_t duty)
{
    int ret = 0;
    int temp = 0;

    temp = tim1_duty - duty;

    if (temp > (int)tim1_duty) {
        tim1_duty_dt = tim1_duty - 20;
        ret = 1;
    } else if (temp < 20) {
        tim1_duty_dt = 20;
        ret = 1;
    } else {
        tim1_duty_dt = tim1_duty - duty;
    }
    timer1_set_duty_bydt(tim1_duty_dt);
    return ret;
}

int pwm_duty_bydt_decrease(uint32_t step)
{
    int ret = 0;

    tim1_duty_dt = tim1_duty_dt + step;
    if (tim1_duty_dt > tim1_duty) {
        tim1_duty_dt = tim1_duty - 10;
        ret = 1;
    }
    timer1_set_duty_bydt(tim1_duty_dt);
    return ret;
}

void pwm_enable(uint32_t enable)
{
    if (enable) {
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
    } else {
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
    }
}

