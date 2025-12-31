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

#define HW_V1_0                  "1" // V1.0
#define HARDWARE_VERSION         HW_V1_0
#define APP_VERSION              "18" //V1.7

// 客户选择
#define Self_Test           0   //自测
#define Self_Test_100K      1   // 100K气泵测试
#define CY_ChuanYi          2   //传音 -- 衡山项目
#define AK_Anker            3   //安克 -- 天山项目
#define RY_Honor            4   //荣耀
#define XM_Xiaomi           5   //小米
#define Self_Test_zhongrui  6   //中睿陶瓷测试
#define Self_Test_11x11     7   //11*11气泵测试
#define Self_Test_12x12     8   //12*12气泵测试

#define Magic_Cool_Customer  Self_Test

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

// printf 空操作屏蔽保持全局一致性
#if (!defined(ENABLE_PRINTF)) || (defined(ENABLE_USART))
#define printf(fmt, ...) ((void)0)
#endif

#endif
