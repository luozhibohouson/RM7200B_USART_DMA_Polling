#ifndef __DEFINE_SELF_TEST_H
#define __DEFINE_SELF_TEST_H

#if 0
    #define     VOL_TARGET      50  // 流量目标
    #define     VOL_TARGET_90P  47
    #define     VOL_TARGET_80P  44
    #define     VOL_TARGET_70P  41
    #define     VOL_TARGET_60P  38
    #define     VOL_TARGET_50P  35
#else
    #define     VOL_TARGET      36 //40  // 流量目标
    #define     VOL_TARGET_90P  37 //37
    #define     VOL_TARGET_80P  34 //35
    #define     VOL_TARGET_70P  31 //33
    #define     VOL_TARGET_60P  28 //30
    #define     VOL_TARGET_50P  25 //27
#endif

// 泵频
#define PUMP_FREQ                  24900
#define FREQ_MIN                   (PUMP_FREQ - 100)
#define FREQ_MAX                   (PUMP_FREQ + 300)

// 硬件版本
// V1.1对比V1.0,增加了dc升压芯片的控制引脚,已做兼容
#define HW_V1_0                    "1"
#define HW_V1_1                    "2"
#define HARDWARE_VERSION           HW_V1_1

// 软件版本
#define APP_VERSION                "10"

// 驱动方式
#define PWM_DRIVER_METHOD          PWM_DIFFERENTIAL_DRIVE

// 供电选择
#define MCU_VDD                    MCU_VDD_3V0
#if MCU_VDD == MCU_VDD_3V3
  #define MCU_VDD_GAIN             3.3
  #define MCU_VDD_GAIN_10X         33
#if 0
  #define MCU_VDD_MAX_GAIN_10X     30
  #define MCU_VDD_MIN_GAIN_10X     0
#else // 未修改DC升压反馈电阻,最小只能到0.2V
  #define MCU_VDD_MAX_GAIN_10X     33
  #define MCU_VDD_MIN_GAIN_10X     2
#endif
#elif MCU_VDD == MCU_VDD_3V0
  #define MCU_VDD_GAIN             3.0
  // 硬件调整电阻后，DAC驱动电压最大只能到3V,最小是0V。原来是3.3V和0.2V
  #define MCU_VDD_GAIN_10X         30
  #define MCU_VDD_MAX_GAIN_10X     30
#if 0
  #define MCU_VDD_MIN_GAIN_10X     0
#else // 未修改DC升压反馈电阻,最小只能到0.2V
  #define MCU_VDD_MIN_GAIN_10X     2
#endif
#endif

// 电压增益与偏移
#if MCU_VDD == MCU_VDD_3V3
  #if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    #define MAGIC_COOL_VOLTAGE_GAIN  11.8
  #else
    #define MAGIC_COOL_VOLTAGE_GAIN  13.79
  #endif
#elif MCU_VDD == MCU_VDD_3V0
  #define MAGIC_COOL_VOLTAGE_GAIN  12.98
#endif

#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
  #define MAGIC_COOL_VOLTAGE_OFFSET 0
#else
  #define MAGIC_COOL_VOLTAGE_OFFSET 12.46
#endif

// DAC升压配置
#define PWM1_MIN_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MAX_GAIN_10X/MCU_VDD_GAIN_10X))
#define PWM1_MAX_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MIN_GAIN_10X/MCU_VDD_GAIN_10X))

// 功能开关
// 按键调整电压
#define KEY_VOL_CFG                 0

// 打印开关
#define ENABLE_PRINTF               1

// 串口通讯开关
#define ENABLE_USART                0

#endif

