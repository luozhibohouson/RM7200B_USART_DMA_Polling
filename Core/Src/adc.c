#include "adc.h"
#include "rm_math.h"
#include "tim.h"


uint32_t adc_freq = 1000000;
uint16_t adc_data[ADC_BUFFER_SIZE] = {0};
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
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

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
uint16_t adc_dc_hcur_offset = 0;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
uint16_t adc_dc_lcur_offset = 0;
#endif

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
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2;
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
            SamplingTime = ADC_SampleTime_2_5;
            adc_freq = 1000000;
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



void adc_output_conv(int num)  // 输出交流电压电流
{
    int i = 0;
    int count = 0; // 2通道数据
    int period_sample_cnt = 0;      // 周期采样个数
    float vol_avg_sum = 0.0;
    float cur_avg_sum = 0.0;

    count = num << 1;

    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VOUT, ADC_CHANNEL_IOUT, count);
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);

//    printf("  %d-%d-%d-\r\n", adc_freq, pwm_get_freq(), period_sample_cnt);

//    for (i = 0; i < num; i++) {
//        adc_voltage_data[i] = adc_data[i * 2];
//        adc_current_data[i] = adc_data[i * 2 + 1];
////        printf("%.5f, %.5f, %d, %d\r\n", adc_voltage_data[i], adc_current_data[i], adc_data[i * 2], adc_data[i * 2 + 1]);
//    }
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

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
void adc_hv_input_conv(int num)  // 直流高压输入电压电流
{
    int i = 0;
    uint32_t vol_sum = 0;
    uint32_t cur_sum = 0;
    int count = num << 1; // 2通道数据

    adc_set_channel_dma((uint32_t)&adc_data, ADC_CHANNEL_VHIN, ADC_CHANNEL_IHIN, count);
    while (RESET == DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
    }

    DMA_ClearFlag(DMA1_FLAG_TC1);
    for (i = 0; i < num; i++) {
        adc_voltage_data_i[i] = adc_data[i * 2];
        adc_current_data_i[i] = adc_data[i * 2 + 1];
        vol_sum += adc_voltage_data_i[i];
        cur_sum += adc_current_data_i[i];
//        printf("%d %d\r\n", adc_voltage_data_i[i], adc_current_data_i[i]);
    }

    adc_dc_hvol_avg = vol_sum / num;
    adc_dc_hcur_avg = cur_sum / num;
    // adc_dc_hcur_avg -=  1271;

    if( !adc_dc_hcur_offset ) {
        adc_dc_hcur_offset = (uint16_t)adc_dc_hcur_avg;
        printf("adc_dc_hcur_offset:%d\r\n", adc_dc_hcur_offset);
    }
    adc_dc_hcur_avg -= adc_dc_hcur_offset;
}
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
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
    for (i = 0; i < num; i++) {
        // adc_voltage_data[i] = (float)adc_data[i * 2];
        // adc_current_data[i] = (float)adc_data[i * 2 + 1];
        // vol_sum += adc_voltage_data_i[i];
        // cur_sum += adc_current_data_i[i];
        vol_sum += adc_data[i * 2];
        cur_sum += adc_data[i * 2 + 1];
//        printf("%d %d\r\n", adc_voltage_data_i[i], adc_current_data_i[i]);
    }

    adc_dc_hvol_avg = vol_sum / num;
    adc_dc_lcur_avg = cur_sum / num;
    // adc_dc_lcur_avg -= 1024;
    // adc_dc_lcur_avg -= 1886; //减去偏置电流

    if( !adc_dc_lcur_offset ) {
        adc_dc_lcur_offset = (uint16_t)adc_dc_lcur_avg;
        printf("adc_dc_lcur_offset:%d\r\n", adc_dc_lcur_offset);
    }
    adc_dc_lcur_avg -= adc_dc_lcur_offset;
}
#endif

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

