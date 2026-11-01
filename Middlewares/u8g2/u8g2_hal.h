#ifndef __U8G2_HAL_H
#define __U8G2_HAL_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "u8x8.h"

/* ★ 2026-09-28: 这里原来是 hspi1(SPI1)。改成 hspi2(SPI2) 的原因 ——
 *   SPI1 在 F103C8T6 上默认占 PA5/PA6/PA7, 这三只引脚本工程全给了循迹探头;
 *   SPI1 的重映射版又要 PB3/PB4/PB5, 那又和 IN4 + 两路 PWM 撞死了。
 *   所以 OLED 只能用 SPI2: SCK=PB13, MOSI=PB15。
 *   (SPI2 设为 Transmit Only Master, 所以 MISO(PB14) 空出来给 DC 用。) */
extern SPI_HandleTypeDef hspi2;

uint8_t u8x8_byte_hal_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_hal(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif
