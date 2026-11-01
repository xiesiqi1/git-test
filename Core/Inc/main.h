/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN3_Pin GPIO_PIN_9
#define IN3_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_11
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_12
#define IN2_GPIO_Port GPIOA
#define KEY_Pin GPIO_PIN_15
#define KEY_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_3
#define IN4_GPIO_Port GPIOB
/* ★ 2026-09-28 为 1.3 寸 OLED (SH1106 128x64) 新增的三根控制线。
 *   这三个名字来自 CubeMX 里给引脚起的 User Label(OLED_RES/OLED_CS/OLED_DC),
 *   所以重新生成代码时不会丢 —— 只要 Label 还在, 名字就还是这三个。
 *   SPI2 的两根数据线(PB13=SCK / PB15=MOSI)由 SPI 外设自己管, 不在这里。 */
#define OLED_RES_Pin GPIO_PIN_8
#define OLED_RES_GPIO_Port GPIOA
#define OLED_CS_Pin GPIO_PIN_12
#define OLED_CS_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_14
#define OLED_DC_GPIO_Port GPIOB
/* ★ 2026-09-29 新增: 声光提示 (2026 电赛 H 题要求"通过 B 点时有声光提示")。
 *   声 = 蜂鸣器 PB0 (有源模块, 低电平触发);  光 = PC13 板载 LED (低电平亮)。
 * ★ 这两个名字来自 CubeMX 里给引脚起的 User Label(BUZZER / LED), 所以重新
 *   生成代码时不会丢 —— 只要 Label 还在, 名字就还是这两个。
 * ⚠ PB0 复用为 TIM3_CH3。本工程 TIM3 只用 CH1/CH2(PB4/PB5) 做电机 PWM,
 *   CH3 是空闲的 —— 所以 PB0 才能当普通 GPIO 用。将来要改成"无源蜂鸣器"
 *   (需要方波)时才用得上 CH3, 那时这里要改成定时器通道而不是 GPIO。
 * ⚠ PC13 是 RTC_TAMPER 引脚, 输出能力弱(<=3 mA), 板载 LED 已串了较大限流
 *   电阻, 只适合点灯, 不要再挂别的负载。 */
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
