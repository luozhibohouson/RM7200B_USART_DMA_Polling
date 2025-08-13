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

// 客户选择
#define Self_Test       0   //自测
#define CY_ChuanYi      1   //传音
#define AK_Anker        2   //安克
#define RY_Honor        3   //荣耀

#define Magic_Cool_Customer  Self_Test

// 分发器：根据 Magic_Cool_Customer 选择具体客户配置
#if Magic_Cool_Customer == Self_Test
  #include "define_self_test.h"
#elif Magic_Cool_Customer == CY_ChuanYi
  #include "define_cy.h"
#elif Magic_Cool_Customer == AK_Anker
  #include "define_anker.h"
#elif Magic_Cool_Customer == RY_Honor
  #include "define_honor.h"
#else
  #error "Unknown Magic_Cool_Customer"
#endif

// printf 空操作屏蔽保持全局一致性
#if defined(ENABLE_PRINTF) && !(ENABLE_PRINTF)
#define printf(fmt, ...) ((void)0)
#endif

#endif
