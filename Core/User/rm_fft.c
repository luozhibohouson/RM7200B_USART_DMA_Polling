  /* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

#include "main.h"
#include "rm_math.h"
#include "rm_fft.h"
#include "controller.h"

#include "stdio.h"
#include "stdbool.h"
#include "arm_math.h"  // 包含CMSIS DSP库
#include "adc.h"



#define FFT_SIZE 128  // FFT大小

float32_t fft_input[FFT_SIZE * 2];  // FFT输入缓冲区
//float32_t fft_output[FFT_SIZE];  // FFT输出缓冲区


// 处理ADC数据并进行FFT计算
float32_t ProcessADCData(float32_t* voltage_data, float32_t* current_data) 
{
    int print_flag = 0;
    float32_t max_Magnitude = 0.0;
//    printf("-------------------------------------------------------------\r\n");

    if (print_flag) printf("\r\n");

    // 处理电压信号的FFT
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_input[2*i] = voltage_data[i];     // 实部为采样值
        fft_input[2*i + 1] = 0.0f;              // 虚部初始化为0
        if (print_flag) printf("%.5f\r\n", fft_input[2*i]);
    }
    if (print_flag) printf("\r\n");

    // 执行FFT
    arm_cfft_instance_f32 S;
    arm_cfft_init_f32(&S, FFT_SIZE);
    arm_cfft_f32(&S, fft_input, 0, 1);
    if (print_flag) {
        for (int i = 0; i < FFT_SIZE; i++) {
            printf("%f\t\t\t%f\n", fft_input[i * 2], fft_input[i * 2 + 1]);
        }
        printf("------ \r\n");
    }

    // 在FFT后，fft_input数组的格式为：
    // [实部0, 虚部0, 实部1, 虚部1, ..., 实部(N-1), 虚部(N-1)]
    
    // 基频索引 (对于50Hz信号，取决于采样率)
    // 例如：采样率1kHz，FFT_SIZE=128时，基频索引为：128 * 50Hz / 1000Hz ≈ 6
    // idx = 64 * 27500 / 640000
    //const int fundamental_idx = 128 * freq / 640000;  // 需要根据实际采样率调整
    int fundamental_idx = 0;  // 需要根据实际采样率调整
    float32_t voltage_real = fft_input[2 * 0];
    float32_t voltage_imag = fft_input[2 * 0 + 1];
    float32_t voltage_magnitude = sqrtf(voltage_real * voltage_real +  voltage_imag * voltage_imag);
    max_Magnitude = voltage_magnitude;
    if (print_flag) printf("%d, %f\r\n", 0, voltage_magnitude);
    
    for (int i = 1; i < (FFT_SIZE / 2); i++) {
        voltage_real = fft_input[2 * i];
        voltage_imag = fft_input[2 * i + 1];
        voltage_magnitude = sqrtf(voltage_real * voltage_real +  voltage_imag * voltage_imag);
        if (max_Magnitude < voltage_magnitude) {
            max_Magnitude = voltage_magnitude;
            fundamental_idx = i;
        }
        if (print_flag) printf("[%d] %d, %f\r\n", fundamental_idx, i, voltage_magnitude);
    }
    
    if (print_flag) printf("\r\n------ \r\n");
    
    // 获取电压基频分量的幅值和相位
    voltage_real = fft_input[2 * fundamental_idx];
    voltage_imag = fft_input[2 * fundamental_idx + 1];
    voltage_magnitude = sqrtf(voltage_real * voltage_real +  voltage_imag * voltage_imag);
    float32_t voltage_phase = atan2f(voltage_imag, voltage_real);
    
    // 对电流信号执行相同的FFT处理
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_input[2*i] = current_data[i];     // 实部为采样值
        fft_input[2*i + 1] = 0.0f;              // 虚部初始化为0
        if (print_flag) printf("%.5f\r\n", fft_input[2*i]);
    }
    
    arm_cfft_f32(&S, fft_input, 0, 1);
    
    if (print_flag) {
        for (int i = 0; i < FFT_SIZE; i++) {
            printf("%f\t\t\t%f\n", fft_input[i * 2], fft_input[i * 2 + 1]);
        }
        printf("\r\n");
    }
    
    // 获取电流基频分量的幅值和相位
    float32_t current_real = fft_input[2 * fundamental_idx];
    float32_t current_imag = fft_input[2 * fundamental_idx + 1];
    float32_t current_magnitude = sqrtf(current_real * current_real + 
                                      current_imag * current_imag);
    float32_t current_phase = atan2f(current_imag, current_real);

    // 计算相位差
    float32_t phase_difference = voltage_phase - current_phase;
//    printf("phase_difference%.5f\r\n", phase_difference);
    
    if (phase_difference > PI) { // 大于180度
        phase_difference = phase_difference - PI * 2.0f;
    } 
    else if (phase_difference < (PI * (-1.0f))) {  // 小于-180度
        phase_difference = phase_difference + PI * 2.0f;
    }
    if (print_flag) printf("\r\n");
    
    // 输出结果
//    printf("Voltage Magnitude: %f\n", voltage_magnitude);
//    printf("Current Magnitude: %f\n", current_magnitude);
//    printf("Phase Difference: %f rad (%f deg)\n", 
//           phase_difference, 
//           phase_difference * 180.0f / PI);
    if (print_flag) printf("fft out:%d, %.5f %.5f, %f, %f\r\n", fundamental_idx,
        voltage_magnitude, current_magnitude, phase_difference, phase_difference * 180.0f / PI);
//        
//    printf("-------------------------------------------------------------\r\n");
    return phase_difference;
}
