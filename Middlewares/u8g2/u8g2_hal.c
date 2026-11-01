/* ============================================================================
 *  u8g2_hal.c —— 把 u8g2 需要做的三件事接到 STM32 上
 *      1) 发字节   -> 走硬件 SPI2 (hspi2)
 *      2) 拉引脚   -> CS / DC / RES 三根控制线 (宏名见 Core/Inc/main.h)
 *      3) 毫秒延时 -> HAL_Delay
 *
 *  2026-09-28 改动: 原来这里用的是 SPI1(PA5/PA6/PA7), 但那三只引脚本工程
 *  全给了循迹探头, 所以整体换到 SPI2。另外引脚宏名也从 CS_Pin/DC_Pin/RES_Pin
 *  改成 CubeMX 里给用户标签起的名字 OLED_CS_Pin / OLED_DC_Pin / OLED_RES_Pin
 *  —— 这样以后在 CubeMX 里重新生成代码, 标签还在, 名字不会丢。
 * ========================================================================== */

#include "u8g2_hal.h"

uint8_t u8x8_gpio_and_delay_hal(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;
    switch (msg)
    {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;

        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
            break;

        case U8X8_MSG_GPIO_CS:
            HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin,
                arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_GPIO_DC:
            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin,
                arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_GPIO_RESET:
            HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin,
                arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        default:
            break;
    }
    return 1;
}

uint8_t u8x8_byte_hal_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    switch (msg)
    {
        case U8X8_MSG_BYTE_SEND:
            /* 只发不读: SH1106 没有回读需求。SPI2 配的是
             * "Transmit Only Master" (= SPI_DIRECTION_2LINES 但只引 MOSI),
             * HAL_SPI_Transmit 在这两种配置下的时序完全一样。 */
            HAL_SPI_Transmit(&hspi2, (uint8_t *)arg_ptr, arg_int, HAL_MAX_DELAY);
            break;

        case U8X8_MSG_BYTE_INIT:
            break;

        case U8X8_MSG_BYTE_SET_DC:
            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin,
                arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
            break;

        default:
            return 0;
    }
    return 1;
}
