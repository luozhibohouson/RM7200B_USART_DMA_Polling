  /* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
// #include "mcp4728.h"
#include "rm_math.h"
#include "controller.h"
// #include "air_flowmeter.h"
#include "magic_cool.h"

/*****************************************************************/
uint8_t key1_last = 1, key2_last = 1, key3_last = 1;  // KEY
uint32_t key_tick;



uint8_t autofreq_flag = 0;
uint16_t vol_temp = 0;
uint16_t vol_max1 = 0;
uint16_t vol_min0 = 0;
uint16_t vol_min1 = 0;




// void uart_cmd_process(void)
// {
//     ;
// }

// 按下
void key1_down_handle(void)
{
    // led_on();
}

// 抬起
void key1_up_handle(void)
{
    // led_off();
}

// 按下
void key2_down_handle(void)
{
    // led_on();
}

// 抬起
void key2_up_handle(void)
{
    // led_off();
}

// 按下
void key3_down_handle(void)
{
    // led_on();
}

// 抬起
void key3_up_handle(void)
{
    // led_off();
}

/*******************************************************************/

void key_scan(void)
{
    uint8_t key2;
    static uint8_t debounce_cnt2 = 0;
    static uint8_t key_status = 0;
    static uint32_t key2_current_time = 0;
    static uint8_t key_pressed_flag = 0; // 新增状态标志
    static uint8_t first_run = 1;

    const uint16_t key_long_time2 = 1000;  // 按键长按时间
    uint8_t change_flag = 0;

    if ((get_systick() - key_tick) <= 20) {  // 20ms扫描一次
        return;
    }
    key_tick = get_systick();

    key2 = GPIO_ReadInputDataBit(KEY_PIN_GPIO, KEY_PIN_PORT);

    if (first_run) {
        key2_last = key2;
        first_run = 0;
        return;
    }

    if (key2_last != key2) {
        debounce_cnt2++;
        if (debounce_cnt2 >= 3) {
            debounce_cnt2 = 0;
            key2_last = key2;

            if (key2 == 0) {
                // 按下：只记录状态，不立即触发
                key_pressed_flag = 1;
                key2_current_time = get_systick();
            } else {
                // 释放：根据按下状态判断
                if (key_pressed_flag == 1) {
                    key_status = 0x01; // 短按松手触发
                    change_flag = 1;
                } else {
                    key_status = 0;
                }
                key_pressed_flag = 0;
            }
        }
    } else {
        debounce_cnt2 = 0;

        if (key2 == 0) {
            // 持续按下检测长按
            if (key_pressed_flag == 1) {
                if ((get_systick() - key2_current_time) > key_long_time2) {
                    key2_current_time = get_systick();
                    key_status = 0x02; // 长按触发
                    key_pressed_flag = 2; // 标记为已长按
                    change_flag = 1;
                }
            }
        } else {
            // 释放后复位短按状态
            if (key_status == 0x01) {
                key_status = 0;
            }
        }
    }

    if (change_flag == 1)
        magic_cool_key_scan(1, key_status, 1);
}

/************************************************************************/
void set_power_enable(uint32_t enable)
{
    if (enable) {
        // LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_2);
        TIM_CtrlPWMOutputs(TIM13, ENABLE);
    } else {
        // LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2);
        TIM_CtrlPWMOutputs(TIM13, DISABLE);
    }
}

// void set_compare_dcoffset(int len)
// {
//     uint32_t temp = 0;
//     // CHD  电压过零比较输入负端
//     // 计算电压中值, ADC 12bit参考电压2.048V，mcp4728 12bit参考电压2.048V，ADC中值可直接设置到DAC
//     temp = adc_vol_avg * 3.3 / 2.048;;

//     // 设置电压DAC比较器偏置
//     mcp4728_set_dac(CHD, temp);

//     // CHC  电流过零比较输入负端
//     // 计算电流中值, ADC 12bit参考电压2.048V，mcp4728 12bit参考电压2.048V，ADC中值可直接设置到DAC
//     temp = adc_cur_avg * 3.3 / 2.048;
//     // 设置电流DAC比较器偏置
//     mcp4728_set_dac(CHC, temp);
// }


/*******************************************************************/
/*******************************************************************/


void rm_magic_config(void)
{
#if RM_MAGIC_CLEAR
    magic_clear_config(0);
#endif
#if RM_MAGIC_COOL
    magic_cool_config();
#endif
#if RM_MAGIC_ENG
    magic_eng_config();
#endif

    printf("POWERON rm_magic_config\r\n");
}

void rm_magic_run(void)
{
#if RM_MAGIC_CLEAR
    magic_clear_run(0);
#endif
#if RM_MAGIC_COOL
    magic_cool_run();
#endif
#if RM_MAGIC_ENG
    magic_eng_run();
#endif
}
