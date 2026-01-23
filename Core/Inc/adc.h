#ifndef __ADC_H
#define __ADC_H

#include "main.h"
#include "controller.h"
#include "define.h"


#define ADC_15MHZ_CONV_FREQ_1MHZ    0    // 15M / (2.5   + 12.5) = 1MHz
#define ADC_15MHZ_CONV_FREQ_714KHZ  1    // 15M / (8.5   + 12.5) = 714KHz
#define ADC_15MHZ_CONV_FREQ_556KHZ  2    // 15M / (14.5  + 12.5) = 556KHz
#define ADC_15MHZ_CONV_FREQ_357KHZ  3    // 15M / (29.5  + 12.5) = 357KHz
#define ADC_15MHZ_CONV_FREQ_273KHZ  4    // 15M / (42.5  + 12.5) = 273KHz
#define ADC_15MHZ_CONV_FREQ_217KHZ  5    // 15M / (56.5  + 12.5) = 217KHz
#define ADC_15MHZ_CONV_FREQ_176KHZ  6    // 15M / (72.5  + 12.5) = 176KHz
#define ADC_15MHZ_CONV_FREQ_59KHZ   7    // 15M / (240.5 + 12.5) = 59KHz
#define ADC_15MHZ_CONV_FREQ_938KHZ  8    // 15M / (3.5   + 12.5) = 938KHz
#define ADC_15MHZ_CONV_FREQ_882KHZ  9    // 15M / (4.5   + 12.5) = 882KHz
#define ADC_15MHZ_CONV_FREQ_833KHZ  10   // 15M / (5.5   + 12.5) = 833KHz
#define ADC_15MHZ_CONV_FREQ_789KHZ  11   // 15M / (6.5   + 12.5) = 789KHz
#define ADC_15MHZ_CONV_FREQ_750KHZ  12   // 15M / (7.5   + 12.5) = 750KHz

#if (HARDWARE_VERSION_CODE == HW_VER_1_0_INT)
#define ADC_CHANNEL_VHIN   ADC_Channel_9   // 高端直流电压
// #define ADC_CHANNEL_IHIN   LL_ADC_CHANNEL_11   // 高端直流电流
#define ADC_CHANNEL_IGND   ADC_Channel_5 // ADC_Channel_5 // ADC_Channel_6    // 低端电流
#define ADC_CHANNEL_VOUT   ADC_Channel_4 // ADC_Channel_4 // ADC_Channel_7    // 输出电压
#define ADC_CHANNEL_IOUT   ADC_Channel_5 // ADC_Channel_5 // ADC_Channel_6    // 输出电流
#elif (HARDWARE_VERSION_CODE == HW_VER_2_0_INT)
#define ADC_CHANNEL_VHIN   ADC_Channel_6   // 高端直流电压
// #define ADC_CHANNEL_IHIN   LL_ADC_CHANNEL_11   // 高端直流电流
#define ADC_CHANNEL_IGND   ADC_Channel_5 // 低端电流
#define ADC_CHANNEL_VOUT   ADC_Channel_4 // 输出电压
#define ADC_CHANNEL_IOUT   ADC_Channel_5 // 输出电流
#endif

#ifndef ADC_BUFFER_SIZE
#if Magic_Cool_Customer != Self_Test_100K
#define ADC_BUFFER_SIZE (256)  // ADC缓冲区大
#define ADC_CH_SIZE (ADC_BUFFER_SIZE>>1)  // ADC缓冲区大
#else
#define ADC_BUFFER_SIZE 60  // ADC缓冲区大
#define ADC_CH_SIZE 30  // ADC缓冲区大
#endif
#endif


extern uint32_t adc_freq;
extern uint16_t adc_data[ADC_BUFFER_SIZE];
extern float adc_voltage_data[ADC_CH_SIZE];
extern float adc_current_data[ADC_CH_SIZE];

extern float adc_vpp;  // 交流电压峰峰
extern float adc_ipp;  // 交流电流峰峰
extern float adc_vol_avg;  // 电压平均值
extern float adc_cur_avg;  // 电流平均值

extern float adc_dc_hvol_avg;  // 直流高压电压
extern float adc_dc_hcur_avg;  // 直流高压电流
extern float adc_dc_lcur_avg;  // 直流低压电流

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
extern uint16_t adc_dc_hcur_offset;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
extern uint16_t adc_dc_lcur_offset;
#endif


#ifdef MAGIC_COOL_VPP_RMS
extern float adc_vrms;  // 交流电压有效
extern float adc_irms;  // 交流电流有效
#endif

void ADC_Configure(void);

void adc_set_conv_freq(int freq_Level);
uint32_t adc_get_conv_freq(void);
void adc_voltage_current_get(int num);

void adc_output_conv(int num);
void adc_hvli_input_conv(int num);
void adc_hv_input_conv(int num);

float voltage_get_vpp_sum(int num);
float current_get_ipp_sum(int num);

// void dma2_ch0_irqhandler(void);
// void adc_irqhandler(void);

void adc_voltage_get_vpp(int num);
void adc_current_get_ipp(int num);

void adc_test(void);

#endif

