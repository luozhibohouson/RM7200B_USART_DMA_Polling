/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#if (HARDWARE_VERSION_CODE == HW_VER_1_0_INT)
#define KEY_PIN_GPIO    GPIOA
#define KEY_PIN_PORT    GPIO_Pin_8

#define LED_PIN_GPIO    GPIOB
#define LED_PIN_PORT    GPIO_Pin_5

#define DCDC_PIN_GPIO    GPIOB
#define DCDC_PIN_PORT    GPIO_Pin_6

#elif (HARDWARE_VERSION_CODE == HW_VER_2_0_INT)
#define KEY_PIN_GPIO    GPIOB
#define KEY_PIN_PORT    GPIO_Pin_3

#define LED_PIN_GPIO    GPIOA
#define LED_PIN_PORT    GPIO_Pin_15
#endif
/* USER CODE END Private defines */

void GPIO_Configure(void);

/* USER CODE BEGIN Prototypes */
void key1_down_handle(void);
void key1_up_handle(void);
void key2_down_handle(void);
void key2_up_handle(void);
void key3_down_handle(void);
void key3_up_handle(void);

void key_scan(void);
void led_on(void);
void led_off(void);
void led_toggle(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

