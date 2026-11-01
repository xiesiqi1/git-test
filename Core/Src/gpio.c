/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN3_Pin|IN1_Pin|IN2_Pin|OLED_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_CS_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /* ★ 2026-09-29: 声光提示的两个脚, 上电必须是"不响 / 不亮"。
   *   ★★ 电平方向【已实测确认】, 两个方向是【相反】的, 别照抄:
   *      · 蜂鸣器 PB0: 高电平响(BUZZER_ACTIVE_LEVEL = 1) -> 初置 RESET(低) = 不响
   *      · LED    PC13: 低电平亮(LED_ACTIVE_LEVEL    = 0) -> 初置 SET (高) = 不亮
   *   早先这里误以为蜂鸣器是低电平触发、写成了 SET, 结果上电瞬间会"响一下"
   *   (直到 Signal_Init() 把它拉低才安静)。现在按实测改对。
   *   ⚠ 这一段属于 CubeMX 生成区 —— 重新生成代码时初始电平会被改回 RESET,
   *   那时蜂鸣器又会在上电瞬间响一下。真正可靠的那道保险是 signal.c 的
   *   Signal_Init()(USER CODE 段, 不被覆盖), 它里面会再置一次。 */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 —— 改成板载 LED 输出 (2026-09-29)
     原来是"空闲脚配模拟输入"; 现在给声光提示当"光"。 */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* ⚠ 这里【不要】配 PD0、也不要开 __HAL_AFIO_REMAP_PD01_ENABLE()。
   * PD0-OSC_IN / PD1-OSC_OUT 是 8 MHz 晶振脚(HSE), 必须留给振荡器。
   * 2026-09-28: CubeMX 曾因为 .ioc 里 PD0-OSC_IN 被误删, 以为 HSE 没启用,
   * 于是把 PD0 当成"空闲脚"配成模拟输入、还打开了 PD01 重映射 —— 那会
   * 直接把 HSE 关掉(表现为上电没反应)。.ioc 已修回, 这里也一并删掉。 */

  /*Configure GPIO pins : PA2 PA10
     (PA8 已拨给 OLED_RES, 见下面输出组) */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 KEY_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10
                           PB11 PB8 PB9
     (PB0  已拨给 BUZZER, 见下面的输出组)
     (PB12/PB13/PB14/PB15 已拨给 OLED_CS / SPI2_SCK / OLED_DC / SPI2_MOSI) */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN3_Pin IN1_Pin IN2_Pin OLED_RES_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|IN1_Pin|IN2_Pin|OLED_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN4_Pin */
  GPIO_InitStruct.Pin = IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IN4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_CS_Pin OLED_DC_Pin
     (SPI2 的 PB13=SCK / PB15=MOSI 由 HAL_SPI_MspInit 配成复用推挽) */
  GPIO_InitStruct.Pin = OLED_CS_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ★ 2026-09-29: 蜂鸣器 PB0 (BUZZER_Pin)。
   * 速度用 LOW 就够 —— 它只按"声"的节奏翻转(最快也就几十 Hz), 不需要
   * 高速边沿。用 LOW 还能少一点开关噪声/干扰。 */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* ⚠ 这里同样不要 __HAL_AFIO_REMAP_PD01_ENABLE(): 那会把 PD0/PD1 从
   * 晶振脚改成 GPIO, HSE 就没了。上面注释已详述。 */

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
