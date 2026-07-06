#ifndef __DEFINE_H
#define __DEFINE_H

#include "tim.h"
#include <stdio.h>

// 驱动方式
#define PWM_SINGLE_END_DRIVE     0
#define PWM_DIFFERENTIAL_DRIVE   1

// 供电选择
#define MCU_VDD_3V3              0
#define MCU_VDD_3V0              1

// 硬件版本配置
#define HW_VER_1_0_INT           1
#define HW_VER_2_0_INT           2

#define HARDWARE_VERSION_CODE    HW_VER_1_0_INT

#if (HARDWARE_VERSION_CODE == HW_VER_1_0_INT)
  #define HW_V1_0                "1" // V1.0
  #define HARDWARE_VERSION_STR   HW_V1_0
#elif (HARDWARE_VERSION_CODE == HW_VER_2_0_INT)
  #define HW_V2_0                "2" // V2.0
  #define HARDWARE_VERSION_STR   HW_V2_0
#endif

// 保持兼容性，HARDWARE_VERSION 仍为字符串，用于串口通讯等
#define HARDWARE_VERSION         HARDWARE_VERSION_STR
#define APP_VERSION              "20" //V2.0





// 客户选择
#define Self_Test           0   //自测
#define Self_Test_100K      1   //100K气泵测试
#define CY_ChuanYi          2   //传音 -- 衡山项目
#define AK_Anker            3   //安克 -- 天山项目
#define RY_Honor            4   //荣耀
#define XM_Xiaomi           5   //小米
#define Self_Test_zhongrui  6   //中睿陶瓷测试
#define Self_Test_11x11     7   //11*11气泵测试
#define Self_Test_12x12     8   //12*12气泵测试
#define Glue_Test           9   //胶水测试
#define SongYang            10  //送样

#define Magic_Cool_Customer  Glue_Test

//根据 Magic_Cool_Customer 选择 具体客户配置
#include "define_self_test.h"
#include "define_self_test_100k.h"
#include "define_cy.h"
#include "define_anker.h"
#include "define_honor.h"
#include "define_xiaomi.h"
#include "define_self_test_zhongrui.h"
#include "define_self_test_11x11.h"
#include "define_self_test_12x12.h"
#include "define_glue_test.h"
#include "define_sy.h"

// printf 空操作屏蔽保持全局一致性
#if (!defined(ENABLE_PRINTF)) || (defined(ENABLE_USART))
#define printf(fmt, ...) ((void)0)
#endif

#endif
