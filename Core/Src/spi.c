/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
/*
 * 2026-09-28 为 1.3 寸 OLED (SH1106 128x64) 新增。
 *
 * ★ 为什么是 SPI2 而不是 SPI1:
 *     SPI1 默认引脚 PA5/PA6/PA7 在本工程全是循迹探头输入;
 *     SPI1 重映射版需要 PB3/PB4/PB5, 又和 IN4 + 两路 PWM 撞死。
 *     所以只能用 SPI2: SCK=PB13, MOSI=PB15。
 *
 * ★ 为什么 Direction 用 SPI_DIRECTION_2LINES (而不是 SPI_DIRECTION_1LINE):
 *     在 CubeMX 里给 SPI2 选的模式是 "Transmit Only Master"。
 *     查 CubeMX 自己的模式定义(SPI-*_Cube_Modes.xml)可以确认, 这个模式生成的
 *     就是 SPI_DIRECTION_2LINES —— 注释原文是
 *         "SPI is in 'Full-Duplex' Mode, but only 'Master Out' line is on pinout"
 *     也就是: SPI 内部仍按全双工跑(时序最标准), 但引脚上【只引出 SCK 和 MOSI】,
 *     MISO(PB14) 根本没配成复用功能, 于是被释放出来给 OLED 的 DC 脚用。
 *     (本工程一开始按 SPI_DIRECTION_1LINE 写, 后来改成 2LINES, 目的就是和
 *      CubeMX 重新生成出来的代码保持一致, 免得以后一对不上就找不到原因。)
 *
 * ★ 时钟: SPI2 挂在 APB1 上 (36 MHz), 预分频 8 -> 4.5 MHz。
 *     SH1106 在长排线上跑不了太快, 先给 4.5 MHz。
 *     ⚠ 若 OLED 出现花屏/条纹, 把 SPI_BAUDRATEPRESCALER_8 改成 _16 (2.25 MHz)。
 *
 * ★ CS 用软件管理 (SPI_NSS_SOFT): 硬件 NSS 引脚不用,
 *   片选由 GPIO PB12 手动拉低/拉高 (在 u8g2_hal.c 里做)。
 */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
