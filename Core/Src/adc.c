#include "adc.h"
#include "rm_math.h"
#include "tim.h"
#include <math.h>

uint32_t adc_freq = 1000000;
uint16_t adc_data[ADC_BUFFER_SIZE] = {0};
#if 1//MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
uint16_t adc_voltage_data_i[ADC_CH_SIZE] = {0};
uint16_t adc_current_data_i[ADC_CH_SIZE] = {0};
#endif
float adc_voltage_data[ADC_CH_SIZE] = {0};
float adc_current_data[ADC_CH_SIZE] = {0};
int adc_freq_Level;

float adc_vpp = 0.0;  // 交流电压峰峰值
float adc_ipp = 0.0;  // 交流电流峰峰值
float adc_vol_avg = 0;
float adc_cur_avg = 0;


#if MAGIC_COOL_VPP_RMS && MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
float adc_vrms = 0.0;  // 交流电压有效值
float adc_irms = 0.0;  // 交流电流有效值
#endif

float adc_dc_hvol_avg = 0.0;  // 直流高压电压
float adc_dc_hcur_avg = 0.0;  // 直流高压电流
float adc_dc_lcur_avg = 0.0;  // 直流低压电流
float adc_vocurp_avg = 0.0;   // VOCurP 电流检测平均值（高端总电流）

bool firstPowerOn = true;
bool DefineDebugCurrentWave =false;
bool DefineDebugCurrentException =true;//false;


// #if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
uint16_t adc_dc_hcur_offset = 0;
// #elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
uint16_t adc_dc_lcur_offset = 0;
// #endif

#define _TEST_CURRENT_WAVE 1
void SetPowerOnFlag(bool flag) 
{
    firstPowerOn = flag;
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void ADC_Configure(void)
{
    ADC_InitTypeDef  ADC_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC1, ENABLE);

    ADC_DeInit(ADC1);

    ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_Prescaler  = ADC_Prescaler_4;    //60M/4=15M
    ADC_InitStruct.ADC_Mode       = ADC_Mode_Continue;
    ADC_InitStruct.ADC_DataAlign  = ADC_DataAlign_Right;
    ADC_Init(ADC1, &ADC_InitStruct);

    // ADC_SampleTimeConfig(ADC1, ADC_Channel_0, ADC_SampleTime_240_5);
    // ADC_SampleTimeConfig(ADC1, ADC_Channel_2, ADC_SampleTime_240_5);
    // ADC_SampleTimeConfig(ADC1, ADC_Channel_3, ADC_SampleTime_240_5);

    // ADC_AnyChannelNumCfg(ADC1, 2);
    // ADC_AnyChannelSelect(ADC1, 0, ADC_Channel_0);
    // ADC_AnyChannelSelect(ADC1, 1, ADC_Channel_2);
    // ADC_AnyChannelSelect(ADC1, 2, ADC_Channel_3);
    ADC_AnyChannelCmd(ADC1, ENABLE);

#if (HARDWARE_VERSION_CODE == HW_VER_1_0_INT)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStruct);
    // GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2;
    // GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_2;

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;

    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
#elif (HARDWARE_VERSION_CODE == HW_VER_2_0_INT)
    // OPA对应通道可直连ADC对应通道，此处只初始化VIN通道
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

    ADC_Cmd(ADC1, ENABLE);


    // adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VHIN, ADC_CHANNEL_IHIN, ADC_BUFFER_SIZE);

}

void ADC_DMA_Configure(uint32_t srcaddr, uint32_t num)
{
    DMA_InitTypeDef  DMA_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);

    DMA_DeInit(DMA1_Channel1);

    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->ADDATA);
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)srcaddr;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize         = num;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_Auto_Reload        = DMA_Auto_Reload_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_DMACmd(ADC1, ENABLE);
}

void adc_set_conv_freq(int freq_Level)
{
    adc_freq_Level = freq_Level;
}

uint32_t adc_get_conv_freq(void)
{
    uint32_t SamplingTime = 0;
    switch (adc_freq_Level) {
        case ADC_15MHZ_CONV_FREQ_1MHZ:
            SamplingTime = ADC_SampleTime_2_5;//(2.5+12.5)个ADC时钟周期=一次转换需要周期//ADC_SampleTime_240_5;//;//;//;//;//;
            adc_freq = 1000000;//1us转换一次
            break;
        case ADC_15MHZ_CONV_FREQ_714KHZ:
            SamplingTime = ADC_SampleTime_8_5;
            adc_freq = 714000;
            break;
        case ADC_15MHZ_CONV_FREQ_556KHZ:
            SamplingTime = ADC_SampleTime_14_5;
            adc_freq = 556000;
            break;
        case ADC_15MHZ_CONV_FREQ_357KHZ:
            SamplingTime = ADC_SampleTime_29_5;
            adc_freq = 357000;
            break;
        case ADC_15MHZ_CONV_FREQ_273KHZ:
            SamplingTime = ADC_SampleTime_42_5;
            adc_freq = 273000;
            break;
        case ADC_15MHZ_CONV_FREQ_217KHZ:
            SamplingTime = ADC_SampleTime_56_5;
            adc_freq = 217000;
            break;
        case ADC_15MHZ_CONV_FREQ_176KHZ:
            SamplingTime = ADC_SampleTime_72_5;
            adc_freq = 176000;
            break;
        case ADC_15MHZ_CONV_FREQ_59KHZ :
            SamplingTime = ADC_SampleTime_240_5;
            adc_freq = 59000;
            break;
        case ADC_15MHZ_CONV_FREQ_938KHZ :
            SamplingTime = ADC_SampleTime_3_5;
            adc_freq = 938000;
            break;
        case ADC_15MHZ_CONV_FREQ_882KHZ :
            SamplingTime = ADC_SampleTime_4_5;
            adc_freq = 882000;
            break;
        case ADC_15MHZ_CONV_FREQ_833KHZ :
            SamplingTime = ADC_SampleTime_5_5;
            adc_freq = 833000;
            break;
        case ADC_15MHZ_CONV_FREQ_789KHZ :
            SamplingTime = ADC_SampleTime_6_5;
            adc_freq = 789000;
            break;
        case ADC_15MHZ_CONV_FREQ_750KHZ :
            SamplingTime = ADC_SampleTime_7_5;
            adc_freq = 750000;
            break;
        default:
            SamplingTime = ADC_SampleTime_2_5;
            adc_freq = 1000000;
            break;
    }

    return SamplingTime;
}


void adc_set_channel_dma(uint32_t srcaddr, uint32_t ch0, uint32_t ch1, uint32_t num)
{
    DMA_InitTypeDef  DMA_InitStruct;

    uint32_t SamplingTime = 0;
    DMA_Cmd(DMA1_Channel1, DISABLE);
    ADC_Cmd(ADC1, DISABLE);

    ADC_Configure();

    SamplingTime = adc_get_conv_freq();
    ADC_SampleTimeConfig(ADC1, ch0, SamplingTime);
    ADC_SampleTimeConfig(ADC1, ch1, SamplingTime);

    ADC_AnyChannelNumCfg(ADC1, 1);
    ADC_AnyChannelSelect(ADC1, 0, ch0);
    ADC_AnyChannelSelect(ADC1, 1, ch1);

    ADC_DMA_Configure(srcaddr, num);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// void adc_voltage_current_get(int num)
// {
//     int i = 0;
//     int count = num << 1; // 2通道数据

//     adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VOUT, ADC_CHANNEL_IOUT, count);

//     while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
//     {
//     }

//     DMA_ClearFlag(DMA1_FLAG_TC1);

//     for (i = 0; i < num; i++) {
//         adc_voltage_data_i[i] = adc_data[i * 2];
//         adc_voltage_data_i[i] = adc_data[i * 2 + 1];
//     }
// }

void adc_voltage_get_vpp(int num)
{
    int i = 0;
    int count = num << 1; // 2通道数据
    int period_sample_cnt = 0;      // 周期采样个数
    float vol_avg_sum = 0.0;

    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VOUT, ADC_CHANNEL_IOUT, count);
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);

    period_sample_cnt = adc_freq / pwm_get_freq();  // 周期采样个数
    period_sample_cnt = 1 + num - (num % period_sample_cnt);  // 完整周期采样个数

    vol_avg_sum = 0.0;
    for (i = 0; i < period_sample_cnt; i++) {
        vol_avg_sum += (float)adc_data[i * 2];
    }
    // 计算平均值
    adc_vol_avg = vol_avg_sum / period_sample_cnt;   // 完整周期电压平均值
//    printf("period_sample_cnt:%d, vsum:%f, csum:%f vavg:%f, cavg%f\r\n", period_sample_cnt, vol_avg_sum, cur_avg_sum, adc_vol_avg, adc_cur_avg);

    // 中心对称
    for (i = 0; i < num; i++) {
        adc_voltage_data[i] = (float)adc_data[i * 2] - adc_vol_avg;
        // printf("%.5f, %.5f, %d, %d\r\n", adc_voltage_data[i], adc_current_data[i], adc_data[i * 2], adc_data[i * 2 + 1]);
    }

    // 3点中值滤波，去除毛刺
    median_filter_3(adc_voltage_data, num);

#if  MAGIC_COOL_VPP_DEFAULT == MAGIC_COOL_VPP_MAXMIN
    adc_vpp = find_peak_to_peak(adc_voltage_data, num);
//    printf("vpp:%f ipp:%f\r\n", adc_vpp, adc_ipp);
#elif MAGIC_COOL_VPP_DEFAULT == MAGIC_COOL_VPP_AVG
    adc_vpp = get_peak_to_peak(adc_voltage_data, num);
//    printf("vpp:%f ipp:%f\r\n", adc_vpp, adc_ipp);
#else // 其他
    adc_vpp = 0.0;
#endif
}


void adc_current_get_ipp(int num)
{
    int i = 0;
    int count = num << 1; // 2通道数据
    int period_sample_cnt = 0;      // 周期采样个数
    float cur_avg_sum = 0.0;

    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VOUT, ADC_CHANNEL_IOUT, count);
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);

    period_sample_cnt = adc_freq / pwm_get_freq();  // 周期采样个数
    period_sample_cnt = 1 + num - (num % period_sample_cnt);  // 完整周期采样个数
    cur_avg_sum = 0.0;
    for (i = 0; i < period_sample_cnt; i++) {
        cur_avg_sum += (float)adc_data[i * 2 + 1];
    }
    adc_cur_avg = cur_avg_sum / period_sample_cnt;   // 完整周期电流平均值

    for (i = 0; i < num; i++) {
        adc_current_data[i] = (float)adc_data[i * 2 + 1] - adc_cur_avg;
//        printf("%.5f, %.5f, %d, %d\r\n", adc_voltage_data[i], adc_current_data[i], adc_data[i * 2], adc_data[i * 2 + 1]);
    }

#if  MAGIC_COOL_VPP_DEFAULT == MAGIC_COOL_VPP_MAXMIN
    adc_ipp = find_peak_to_peak(adc_current_data, num);
//    printf("vpp:%f ipp:%f\r\n", adc_vpp, adc_ipp);
#elif MAGIC_COOL_VPP_DEFAULT == MAGIC_COOL_VPP_AVG
    adc_ipp = get_peak_to_peak(adc_current_data, num);
//    printf("vpp:%f ipp:%f\r\n", adc_vpp, adc_ipp);
#else // 其他
    adc_ipp = 0.0;
#endif
}


#if 1
void adc_output_conv(int num)  // 输出交流电压电流
{
    int i = 0;
    int count = 0; // 2通道数据
    int period_sample_cnt = 0;      // 周期采样个数
    float vol_avg_sum = 0.0;
    float cur_avg_sum = 0.0;

    count = num << 1;

    #if 0
    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VOUT, ADC_CHANNEL_IOUT, count);
    #else
    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VHIN, ADC_CHANNEL_IHIN, count);
    #endif
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);

//    printf("  %d-%d-%d-\r\n", adc_freq, pwm_get_freq(), period_sample_cnt);

   for (i = 0; i < num; i++) {
       adc_voltage_data[i] = adc_data[i * 2];
       adc_current_data[i] = adc_data[i * 2 + 1];
//        printf("%.5f, %.5f, %d, %d\r\n", adc_voltage_data[i], adc_current_data[i], adc_data[i * 2], adc_data[i * 2 + 1]);
   }
#if MAGIC_COOL_ADC_CENTER
    period_sample_cnt = adc_freq / pwm_get_freq();  // 周期采样个数
    period_sample_cnt = 1 + num - (num % period_sample_cnt);  // 完整周期采样个数
    vol_avg_sum = 0.0;
    cur_avg_sum = 0.0;
    for (i = 0; i < period_sample_cnt; i++) {
        vol_avg_sum += (float)adc_data[i * 2];
        cur_avg_sum += (float)adc_data[i * 2 + 1];
//        printf("%d  %f  %f  %d  %d\r\n", i, vol_avg_sum, cur_avg_sum, adc_data[i * 2], adc_data[i * 2 + 1]);
    }
    // 计算平均值
    adc_vol_avg = vol_avg_sum / period_sample_cnt;   // 完整周期电压平均值
    adc_cur_avg = cur_avg_sum / period_sample_cnt;   // 完整周期电流平均值
//    printf("period_sample_cnt:%d, vsum:%f, csum:%f vavg:%f, cavg%f\r\n", period_sample_cnt, vol_avg_sum, cur_avg_sum, adc_vol_avg, adc_cur_avg);

    // 中心对称
    for (i = 0; i < num; i++) {
        adc_voltage_data[i] = (float)adc_data[i * 2] - adc_vol_avg;
        adc_current_data[i] = (float)adc_data[i * 2 + 1] - adc_cur_avg;
//        printf("%.5f, %.5f, %d, %d\r\n", adc_voltage_data[i], adc_current_data[i], adc_data[i * 2], adc_data[i * 2 + 1]);
    }

    // 3点中值滤波，去除毛刺
    median_filter_3(adc_voltage_data, num);
#endif

#if  MAGIC_COOL_VPP_DEFAULT == MAGIC_COOL_VPP_MAXMIN
    adc_vpp = find_peak_to_peak(adc_voltage_data, num);
    adc_ipp = find_peak_to_peak(adc_current_data, num);
//    printf("vpp:%f ipp:%f\r\n", adc_vpp, adc_ipp);
#elif MAGIC_COOL_VPP_DEFAULT == MAGIC_COOL_VPP_AVG
    adc_vpp = get_peak_to_peak(adc_voltage_data, num);
    adc_ipp = get_peak_to_peak(adc_current_data, num);
//    printf("vpp:%f ipp:%f\r\n", adc_vpp, adc_ipp);
#else // 其他
    adc_vpp = 0.0;
    adc_ipp = 0.0;
#endif

#if MAGIC_COOL_VPP_RMS && MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
    // 不分开计算的原因是电压值比较稳定，电流值比较小，误差大，在判断过零时会出错
    // 如果电压电流信号稳定，也可以分开计算
    get_vol_cur_rms(adc_voltage_data, adc_current_data, num, &adc_vrms, &adc_irms);
//    printf("num:%d vrms:%f irms%f\r\n", num, adc_vrms, adc_irms);
#endif
}
#endif


// #if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH

#if FIND_PEAK
#define PEAK_THRESHOLD  500//400
#define MAX_OUT         128


// 从 start 往后找，自动找到尖峰最高点
int find_first_peak_max(int start)
{
    int max_pos = start;
    for (int i = start; i < 127; i++)
    {
        // 还在上升 → 更新最高点
        if (adc_current_data_i[i] > adc_current_data_i[max_pos])
        {
            max_pos = i;
        }
        // 开始下降 → 已经过了最高点，直接返回
        else if (adc_current_data_i[i] > adc_current_data_i[i+1])
        {
            return max_pos;
        }
    }
    return max_pos;
}

// 从 end 往前找，自动找到尖峰最高点
int find_last_peak_max(int end)
{
    int max_pos = end;
    for (int i = end; i > 0; i--)
    {
        if (adc_current_data_i[i] > adc_current_data_i[max_pos])
        {
            max_pos = i;
        }
        else if (adc_current_data_i[i] > adc_current_data_i[i-1])
        {
            return max_pos;
        }
    }
    return max_pos;
}

uint32_t extract_between_real_peaks(void)
{
    int i;
    int first_valid = -1;
    int last_valid = -1;
    int first_peak = -1;
    int last_peak = -1;
    uint32_t cur_sum = 0;

    uint16_t final_data[MAX_OUT] = {0};
    uint16_t final_len = 0;


    // final_len = 0;

    // 1. 找第一个 >400
    for (i = 0; i < 128; i++)
    {
        if (adc_current_data_i[i] > PEAK_THRESHOLD)
        {
            first_valid = i;
            break;
        }
    }

    // 2. 找最后一个 >400
    for (i = 127; i >= 0; i--)
    {
        if (adc_current_data_i[i] > PEAK_THRESHOLD)
        {
            last_valid = i;
            break;
        }
    }

    if (first_valid == -1 || last_valid == -1) 
    {
        printf("no valid peak\r\n");
        return 0;
    }

    // 3. 自动找第一个尖峰最高点
    first_peak = find_first_peak_max(first_valid);

    // 4. 自动找最后一个尖峰最高点
    last_peak = find_last_peak_max(last_valid);

    // 5. 复制两个最高点之间所有数据
    if (first_peak <= last_peak)
    {
        for (i = first_peak; i <= last_peak && final_len < MAX_OUT; i++)
        {
            final_data[final_len++] = adc_current_data_i[i];
            cur_sum += adc_current_data_i[i];

            printf("%d\r\n", final_data[final_len-1]);
        }
    }

    return (cur_sum / final_len);

}
#else

#if 1
#define THRESHOLD_LOW     (180+adc_dc_hcur_offset) //400   //一般是180最新的泵基准最小太大要400  // 小于100为谷底区 1296是电流偏置值，实际电流值=（adc值-1296）*3.3/4096/510
#define MAX_OUT           128


// ========================= 工具函数 =========================
// 从start往后找，找到【真正的最低点】（谷底）
static int find_real_min_forward(int start)
{
    int min_pos = start;
    for (int i = start; i < 127; i++)
    {
        if (adc_current_data_i[i] < adc_current_data_i[min_pos])
        {
            min_pos = i;
        }
        // 开始上升 → 最低点已过
        if (adc_current_data_i[i] < adc_current_data_i[i+1])
        {
            return min_pos;
        }
    }
    return min_pos;
}

// 从end往前找，找到【真正的最低点】（谷底）
static int find_real_min_backward(int end)
{
    int min_pos = end;
    for (int i = end; i > 0; i--)
    {
        if (adc_current_data_i[i] < adc_current_data_i[min_pos])
        {
            min_pos = i;
        }
        // 开始上升 → 最低点已过
        if (adc_current_data_i[i] < adc_current_data_i[i-1])
        {
            return min_pos;
        }
    }
    return min_pos;
}


uint32_t extract_between_real_peaks(void)
{
    int i;
    int first_low = -1;   // 第一个 <100
    int last_low = -1;    // 最后一个 <100
    int first_min = -1;   // 第一个真正最低点
    int last_min = -1;    // 最后一个真正最低点
    uint32_t cur_sum = 0;

// 输出数组（全局，方便外部查看提取到的数据）
// ========================= 主函数 =========================
// 输出数组（全局，方便外部查看提取到的数据）
    uint16_t final_data[MAX_OUT] = {0};
    uint16_t final_len = 0;

    // 1. 找第一个 < 100
    for (i = 0; i < MAX_OUT; i++)
    {
        if (adc_current_data_i[i] < THRESHOLD_LOW)
        {
            first_low = i;
            break;
        }
    }

    // 2. 找最后一个 < 100
    for (i = MAX_OUT - 1; i >= 0; i--)
    {
        if (adc_current_data_i[i] < THRESHOLD_LOW)
        {
            last_low = i;
            break;
        }
    }

    // 无有效谷底
    if (first_low == -1 || last_low == -1)
    {
        if(DefineDebugCurrentException)
        printf("no valid valley\r\n");
        
        for(i=0; i<MAX_OUT; i++)
        {
            // printf("%d\r\n", adc_current_data_i[i]);
            cur_sum += adc_current_data_i[i];

        }
        // printf("cur_sum:%d\r\n", cur_sum/MAX_OUT);
        return cur_sum / MAX_OUT;
    }

    // 3. 找第一个真正最低点
    first_min = find_real_min_forward(first_low);

    // 4. 找最后一个真正最低点
    last_min = find_real_min_backward(last_low);

    // 5. 复制两个最低点之间所有数据
    if (first_min <= last_min)
    {   
        cur_sum = 0;
        for (i = first_min; i <= last_min && final_len < MAX_OUT; i++)
        {
            final_data[final_len++] = adc_current_data_i[i];

            cur_sum += adc_current_data_i[i];

            #if _TEST_CURRENT_WAVE
            if(DefineDebugCurrentWave)
            printf("%d\r\n", final_data[final_len-1]);
            #endif

        }
    }

    // 防除0
    if (final_len == 0) return 0;

    // 返回平均值
    return cur_sum / final_len;
}
#endif
#endif

#if 0
float calculate_rms(uint16_t *samples, int num_samples) 
{ 
    float sum_of_squares = 0.0f, CurrentIvalue=0.0f; // 1. 对所有采样点平方后求和 

    for (int i = 0; i < num_samples; i++) 
    { 
        CurrentIvalue = CUR_CAL(samples[i]); // 转换为实际电流值
        // printf("%d,%.4f\r\n", samples[i], CurrentIvalue);
        sum_of_squares += CurrentIvalue * CurrentIvalue; // 或 pow(samples[i], 2) 
    } // 2. 求平均值 
        
    float mean_of_squares = sum_of_squares / num_samples; // 3. 开平方得到有效值 

    float rms = sqrt(mean_of_squares); 
    // printf("RMS1: %.4fmA\r\n", rms*1000);
    return rms; 
} 

#endif

void adc_hv_input_conv(int num)  // 直流高压输入电压电流
{
    int i = 0;
    uint32_t vol_sum = 0;
    uint32_t cur_sum = 0;
    uint32_t  cur_peak = 0;


    int count = num << 1; // 2通道数据

    // GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);

    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VHIN, ADC_CHANNEL_IHIN, count);
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);
    // GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
#if 1
    for (i = 0; i < num; i++) 
    {
        adc_voltage_data_i[i] = adc_data[i * 2];
        adc_current_data_i[i] = adc_data[i * 2 + 1];
        vol_sum += adc_voltage_data_i[i];
        #if _NO_FILTER_PEAK
        cur_sum += adc_current_data_i[i];
        #else
        if(firstPowerOn) 
        {
            cur_sum += adc_current_data_i[i];
        }
        #endif
       #if 0
        if (adc_current_data_i[i] > cur_peak) 
        {
          cur_peak = adc_current_data_i[i];  // 记录最大值 ✅
        }
       #endif
      

       #if _TEST_CURRENT_WAVE
        if(DefineDebugCurrentWave)
       printf("%4d  %d\r\n", adc_data[i * 2], adc_data[i * 2 + 1]);
       #endif
    }

    // cur_sum =extract_between_real_peaks();  



    adc_dc_hvol_avg = vol_sum / num;

    #if _NO_FILTER_PEAK
    adc_dc_hcur_avg = cur_sum / num;
    #else
    if(firstPowerOn) 
    {
        adc_dc_hcur_avg = cur_sum / num;
    }
    else
    {
        // printf("filtering...\r\n");
        adc_dc_hcur_avg = extract_between_real_peaks();
       
    }
    // adc_dc_hcur_avg = cur_peak;
    #endif
    // adc_dc_hcur_avg -=  1271;
#else

    for (i = 0; i < num; i++) 
    {
        adc_current_data_i[i] = adc_data[i * 2 + 1];

        vol_sum += adc_data[i * 2];

    //    printf("%d %d\r\n", adc_voltage_data_i[i], adc_current_data_i[i]);
    }

    adc_dc_hcur_avg = extract_between_real_peaks();

  

  


   float rms_value = calculate_rms(adc_current_data_i, num); 
    // get_vol_cur_rms(adc_voltage_data, adc_current_data, num, &adc_vrms, &adc_irms);
    // printf("RMS2: %.4fmA\r\n", rms_value*1000);
    adc_dc_hvol_avg = vol_sum / num;;
    // adc_dc_hcur_avg = rms_value;
#endif

    if( !adc_dc_hcur_offset ) 
    {
        adc_dc_hcur_offset = (uint16_t)adc_dc_hcur_avg;
        printf("adc_dc_hcur_offset:%d\r\n", adc_dc_hcur_offset);
    }
    adc_dc_hcur_avg -= adc_dc_hcur_offset;

    if( firstPowerOn ) 
    {
        firstPowerOn = 0;
    }
    // printf("cur_peak=%d\n",cur_peak);
    // printf("adc:%f adc_dc_hcur_avg:%f\r\n", adc_dc_hvol_avg, adc_dc_hcur_avg);
}
// #elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
#if 0
uint16_t find_valid_data(void)
{
// 筛选条件
    const int MIN_VAL = adc_dc_lcur_offset;//1658;
    uint16_t selected[200] = {0};
    int count = 0;
    int len = 128;
    uint32_t sum = 0;

    // ===================== 核心查找算法 =====================
    // 规则：
    // 1. 大于 1658
    // 2. 剔除最开始的下降趋势段（2650往下掉的那段）
    // 3. 只保留红框内有效数据
    // 自动找到第一个低点后，才开始收集 >1658 的数据
    int find_first_low = 0;
    int i = 0;
    for (i = 1; i < len - 1; i++) 
    {
        // 先找到第一个波谷低点
        if (!find_first_low && adc_current_data_i[i] < adc_current_data_i[i-1] && adc_current_data_i[i] < adc_current_data_i[i+1]) 
        {
            find_first_low = 1;
            // continue;
            break;
        }

       #if 0
        // 找到第一个低点之后，再收集 >1658 的数据
        if (find_first_low && adc_current_data_i[i] > MIN_VAL) 
        {
            selected[count++] = adc_current_data_i[i];
            sum += adc_current_data_i[i];
        }
       #endif
    }

    for(;i<len; i++) 
    {
        if (adc_current_data_i[i] > MIN_VAL) 
        {
            selected[count++] = adc_current_data_i[i];
            sum += adc_current_data_i[i];
        }
    }


    // 计算平均值dd
    uint16_t avg = sum / count;

    // ===================== 打印结果 =====================
    // printf("===== 筛选后的数据（大于1658 + 剔除开头下降段）=====\n");
    for (int i = 0; i < count; i++) 
    {
        printf("第%3d个： %d\n", i+1, selected[i]);
    }

    printf("\n===== 统计结果 =====\n");
    printf("符合条件的数据总数：%d\n", count);
    printf("数据总和：%lld\n", sum);
    printf("平均值：%d\n", avg);

    return avg;
}
#else

#if 1

#define THRESHOLD_LOW_2     (150+adc_dc_lcur_offset)     // 小于100为谷底区 1296是电流偏置值，实际电流值=（adc值-1296）*3.3/4096/510
// #define MAX_OUT           128


// ========================= 工具函数 =========================
// 从start往后找，找到【真正的最低点】（谷底）
static int find_real_min_forward_2(uint16_t *filtered, int start)
{
    int min_pos = start;
    for (int i = start; i < 127; i++)
    {
        if (filtered[i] < filtered[min_pos])
        {
            min_pos = i;
        }
        // 开始上升 → 最低点已过
        if (filtered[i] < filtered[i+1])
        {
            return min_pos;
        }
    }
    return min_pos;
}

// 从end往前找，找到【真正的最低点】（谷底）
static int find_real_min_backward_2(uint16_t *filtered, int end)
{
    int min_pos = end;
    for (int i = end; i > 0; i--)
    {
        if (filtered[i] < filtered[min_pos])
        {
            min_pos = i;
        }
        // 开始上升 → 最低点已过
        if (filtered[i] < filtered[i-1])
        {
            return min_pos;
        }
    }
    return min_pos;
}


uint16_t extract_between_real_peaks_2(uint16_t *filtered, int count)
{
    int i;
    int first_low = -1;   // 第一个 <100
    int last_low = -1;    // 最后一个 <100
    int first_min = -1;   // 第一个真正最低点
    int last_min = -1;    // 最后一个真正最低点
    uint32_t cur_sum = 0;

// 输出数组（全局，方便外部查看提取到的数据）
// ========================= 主函数 =========================
// 输出数组（全局，方便外部查看提取到的数据）
    uint16_t final_data[MAX_OUT] = {0};
    uint16_t final_len = 0;

    // 1. 找第一个 < 100
    for (i = 0; i < count; i++)
    {
        if (filtered[i] < THRESHOLD_LOW_2)
        {
            first_low = i;
            break;
        }
    }

    // 2. 找最后一个 < 100
    for (i = (count-1); i >= 0; i--)
    {
        if (filtered[i] < THRESHOLD_LOW_2)
        {
            last_low = i;
            break;
        }
    }

    // 无有效谷底
    if (first_low == -1 || last_low == -1)
    {
        printf("no valid valley\r\n");
        for(i=0; i<count; i++)
        {
            // printf("%d\r\n", adc_current_data_i[i]);
            cur_sum += filtered[i];

        }
        printf("cur_sum:%d\r\n", cur_sum/count);
        return cur_sum / count;
    }

    // 3. 找第一个真正最低点
    first_min = find_real_min_forward_2(filtered, first_low);

    // 4. 找最后一个真正最低点
    last_min = find_real_min_backward_2(filtered, last_low);

    // 5. 复制两个最低点之间所有数据
    #if _TEST_CURRENT_WAVE
    if(DefineDebugCurrentWave)
    printf("第二次筛选出的数据点:\n");
    #endif

    if (first_min <= last_min)
    {   
        cur_sum = 0;
        for (i = first_min; i <= last_min && final_len < count/*MAX_OUT*/; i++)
        {
            final_data[final_len++] = filtered[i];

            cur_sum += filtered[i];

            #if _TEST_CURRENT_WAVE
            if(DefineDebugCurrentWave)
            printf("%d\r\n", final_data[final_len-1]);
            #endif
        }
    }

    // 防除0
    if (final_len == 0) return 0;

    // 返回平均值
    return cur_sum / final_len;
}



uint16_t find_valid_data(void)
{
    uint16_t filtered[128]; // 存储筛选后的点
    int count = 0;            // 筛选出的数量
    long sum = 0;             // 总和（用long避免溢出）
    uint16_t average;           // 平均值

    // 1. 遍历查找大于1657的数据点
    for (int i = 0; i < 128; i++)
    {
        if (adc_current_data_i[i] > adc_dc_lcur_offset)
        {
            filtered[count] = adc_current_data_i[i];
            sum += adc_current_data_i[i];
            count++;
        }
    }

    // 2. 计算平均值
    if (count > 0)
    {
        average = sum / count;
    }
    else
    {
        average = 0;
    }

    // 3. 打印结果
#if 0
    printf("筛选出的点数量: %d\n", count);
    printf("所有点的和: %ld\n", sum);
    printf("平均值: %.2f\n", average);

    // 可选：打印筛选出的数组内容
    printf("筛选出的数据点:\n");

#endif   
     #if _TEST_CURRENT_WAVE
     if(DefineDebugCurrentWave)
     printf("首次筛选出的数据点:\n");
     #endif

    for (int i = 0; i < count; i++)
    {
        #if _TEST_CURRENT_WAVE
        if(DefineDebugCurrentWave)
        printf("第%3d个： %d\n", i+1, filtered[i]);
        #endif

    }
    // printf("\n");

    average = extract_between_real_peaks_2(filtered,count);
    // printf("average:%d\r\n", average);
    return average;
}

#endif
#endif
uint16_t cur_min_peak = 0,cur_max_peak = 0;

void adc_hvli_input_conv(int num)  // 直流高压输入电压,低端电流
{
    int i = 0;
    uint32_t vol_sum = 0;
    uint32_t cur_sum = 0;
    int count = num << 1; // 2通道数据

    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VHIN, ADC_CHANNEL_IGND, count);
    
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);

    for (i = 0; i < num; i++) 
    {
        // adc_voltage_data[i] = (float)adc_data[i * 2];
        adc_current_data_i[i] = adc_data[i * 2 + 1];

        // vol_sum += adc_voltage_data_i[i];
        // cur_sum += adc_current_data_i[i];
        vol_sum += adc_data[i * 2];
        
        #if _NO_FILTER_PEAK
        cur_sum += adc_current_data_i[i];
        #else
        if(firstPowerOn) 
        {
            cur_sum += adc_current_data_i[i];
        }
        else
        {
           #if 0
            if (adc_current_data_i[i] > cur_max_peak)
            {
                cur_max_peak = adc_current_data_i[i]; // 记录最大值
            }

            if (adc_current_data_i[i] < cur_min_peak)
            {
                cur_min_peak = adc_current_data_i[i]; // 记录最小值
            }
           #endif
        }
        #endif

       #if _TEST_CURRENT_WAVE
        if(DefineDebugCurrentWave)
       printf("%4d  %d\r\n", adc_data[i * 2], adc_data[i * 2 + 1]);
       #endif


    }

    

    adc_dc_hvol_avg = vol_sum / num;

    #if _NO_FILTER_PEAK
    adc_dc_lcur_avg = cur_sum / num;
    #else
    if(firstPowerOn) 
    {
        adc_dc_lcur_avg = cur_sum / num;
    }
    else
    {
        // printf("filtering...\r\n");
        adc_dc_lcur_avg = find_valid_data();
        #if _TEST_CURRENT_WAVE
        if(DefineDebugCurrentWave)
        printf("adc_dc_lcur_avg:%d\r\n", (uint16_t)adc_dc_lcur_avg);
        // printf("cur_min_peak:%d, cur_max_peak:%d,vpp:%d\r\n", cur_min_peak, cur_max_peak, cur_max_peak - cur_min_peak);
        // adc_dc_lcur_avg =cur_max_peak - cur_min_peak;
        cur_max_peak = adc_dc_lcur_offset;
        cur_min_peak = adc_dc_lcur_offset;
        
        
        #endif
    }
    // adc_dc_hcur_avg = cur_peak;
    #endif

    // adc_dc_lcur_avg -= 1024;
    // adc_dc_lcur_avg -= 1886; //减去偏置电流

    if( !adc_dc_lcur_offset ) 
    {
        adc_dc_lcur_offset = (uint16_t)adc_dc_lcur_avg;
    #if (HARDWARE_VERSION_CODE == HW_VER_1_0_INT)
        printf("adc_dc_lcur_offset:%d\r\n", adc_dc_lcur_offset);
    #elif (HARDWARE_VERSION_CODE == HW_VER_2_0_INT)
        #define Over_Current_mA         80      //限流
        #define Sampling_Resistance_mR  110     //采样电阻
        #define OpAmp_Gain              101     //运算放大倍数
        #define Vdd_V                   MCU_VDD_GAIN_10X // MCU供电电压

        uint16_t vol_dc_lcur = adc_dc_lcur_offset * Vdd_V*100 / 4096; // 总共放大1000倍
        // printf("vol_dc_lcur:%d \r\n", vol_dc_lcur);
        uint16_t over_current_threshold = Over_Current_mA * Sampling_Resistance_mR * OpAmp_Gain / 1000; // 总共放大1000倍
        // printf("over_current_threshold:%d \r\n", over_current_threshold);
        uint16_t register_value = (vol_dc_lcur + over_current_threshold) * 255 / Vdd_V / 100;
        if( register_value > 0xFF ) {
            register_value = 0xFF;
        }

        COMP_SetCrv(COMP_CRV_SRC_VDDA, register_value);
        printf("adc_dc_lcur_offset:%d register_value:%d Over_Current_mA:%d\r\n", adc_dc_lcur_offset, register_value, Over_Current_mA);
    #endif
    }
    adc_dc_lcur_avg -= adc_dc_lcur_offset;

    if( firstPowerOn ) 
    {
        firstPowerOn = 0;
        cur_min_peak = adc_dc_lcur_offset;
        cur_max_peak = adc_dc_lcur_offset;
    }
}
// #endif

// float voltage_get_vpp_sum(int num)  // 单个通道电压处理速度快
// {
//     int i = 0;
//     float vpp = 0;
//     for (i = 0; i < num; i++) {
//         adc_voltage_current_get(128);
//         vpp += get_peak_to_peak(adc_voltage_data, num);
//     }
//     return vpp;
// }

// float current_get_ipp_sum(int num)  // 单个通道电流处理速度快
// {
//     int i = 0;
//     float ipp = 0;
//     for (i = 0; i < num; i++) {
//         adc_voltage_current_get(128);
//         ipp += get_peak_to_peak(adc_current_data, num);
//     }
//     return ipp;
// }
#if 0

/***********************************************************************************************************************
  * @brief  VOCurP 高端总电流检测
  * @note   根据公式：I = (VOCurP - 1V) / 510
  *         其中：VOCurP 通过 ADC 采样，1V 为偏置，510 为总增益
  * @param  num: 采样点数
  * @retval None
  *********************************************************************************************************************/
void adc_vocurp_conv(int num)
{
    int i = 0;
    uint32_t vocurp_sum = 0;
    int count = num;

    // 配置 VOCurP ADC 通道
    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VOCURP, ADC_CHANNEL_VOCURP, count);
    
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);
    
    // 累加 VOCurP ADC 值
    for (i = 0; i < num; i++) {
        vocurp_sum += adc_data[i];
    }
    
    // 计算平均值（ADC 原始值）
    adc_vocurp_avg = (float)vocurp_sum / num;
    
    // 注意：实际电流计算在应用层使用 VOCURP_CAL 宏进行
    // I = (adc_vocurp_avg * VOCURP_COEFFICIENT - VOCURP_OFFSET) / VOCURP_GAIN
}

void adc_init(void)
{
    ADC_InitTypeDef ADC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC1, ENABLE);

    ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_Prescaler  = ADC_Prescaler_4;    //60M/4=15M
    ADC_InitStruct.ADC_Mode       = ADC_Mode_Continue;
    ADC_InitStruct.ADC_DataAlign  = ADC_DataAlign_Right;
    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_VrefSensorCmd(ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    ADC_SampleTimeConfig(ADC1, ADC_Channel_VoltTempSensor, ADC_SampleTime_2_5);
    // ADC_SampleTimeConfig(ADC1, ADC_Channel_2, ADC_SampleTime_240_5);
    // ADC_SampleTimeConfig(ADC1, ADC_Channel_3, ADC_SampleTime_240_5);

    ADC_AnyChannelNumCfg(ADC1, 0);
    ADC_AnyChannelSelect(ADC1, 0, ADC_Channel_VoltTempSensor);
    // ADC_AnyChannelSelect(ADC1, 1, ADC_Channel_2);
    // ADC_AnyChannelSelect(ADC1, 2, ADC_Channel_3);
    ADC_AnyChannelCmd(ADC1, ENABLE);

    ADC_DMA_Configure((uint32_t)&adc_data, 128);
}

float vol = 0.0;
void ADC_InternalVoltageSensor_Sample(void)
{
    uint16_t ConversionValue = 0;
    uint16_t CalibrationData = *(uint16_t *)(0x1FFFF7E0);
    float    VrefCalculation = (float)CalibrationData * (float)3.3 / (float)4096.0;

    // printf("\r\nTest %s, 0x%x, %0.2f", __FUNCTION__, CalibrationData, VrefCalculation);

    while (1)
    {
        DMA_Cmd(DMA1_Channel1, DISABLE);
        ADC_Cmd(ADC1, DISABLE);

        adc_init();

        ADC_DMA_Configure((uint32_t)&adc_data, 128);

        ADC_SoftwareStartConvCmd(ADC1, ENABLE);

        while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
        {
        }

        DMA_ClearFlag(DMA1_FLAG_TC1);

        ConversionValue = adc_data[128/2];
        vol = (float)4096.0 * (float)VrefCalculation / (float)ConversionValue;

        // printf("\r\nVDDA = %0.2fV", (float)4096.0 * (float)VrefCalculation / (float)ConversionValue);

        sys_delayms(500);
    }
}

void adc_test(void)
{
    ADC_InternalVoltageSensor_Sample();

    int i = 128;
    ADC_VrefSensorCmd(ENABLE);
    sys_delayms(100);
    while(1) {
        adc_output_conv(128);
        for (i = 0; i < 128; i++) {
            // printf("[%d] %f %f\r\n", i, adc_voltage_data[i], adc_current_data[i]);
            printf("%d\r\n", adc_data[i*2]);
        }
        sys_delayms(100);
    }
}
#endif

