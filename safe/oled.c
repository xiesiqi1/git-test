#include "oled.h"
#include "oledfont.h"
#include "i2c.h"
#include "tim.h"
#include "main.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGE_COUNT];

/* ============================================================
 * I2C 底层通信
 * ============================================================ */

static HAL_StatusTypeDef OLED_WriteBuffer(uint8_t *buffer,
                                          uint16_t length,
                                          uint32_t timeout)
{
    if (buffer == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDRESS,
        buffer,
        length,
        timeout
    );
}

static HAL_StatusTypeDef OLED_WriteCommand(uint8_t command)
{
    uint8_t buffer[2];

    buffer[0] = 0x00U;
    buffer[1] = command;

    return OLED_WriteBuffer(
        buffer,
        2U,
        OLED_COMMAND_TIMEOUT_MS
    );
}

static HAL_StatusTypeDef OLED_WriteData(uint8_t data)
{
    uint8_t buffer[2];

    buffer[0] = 0x40U;
    buffer[1] = data;

    return OLED_WriteBuffer(
        buffer,
        2U,
        OLED_DATA_TIMEOUT_MS
    );
}

/* 兼容旧代码 */
void WriteCmd(unsigned char command)
{
    (void)OLED_WriteCommand((uint8_t)command);
}

void WriteDat(unsigned char data)
{
    (void)OLED_WriteData((uint8_t)data);
}

/* ============================================================
 * OLED 初始化
 * ============================================================ */

void OLED_Init(void)
{
    HAL_Delay(100U);

    WriteCmd(0xAEU);       /* Display OFF */

    WriteCmd(0xD5U);
    WriteCmd(0x80U);       /* Display clock */

    WriteCmd(0xA8U);
    WriteCmd(0x3FU);       /* Multiplex ratio 1/64 */

    WriteCmd(0xD3U);
    WriteCmd(0x00U);       /* Display offset */

    WriteCmd(0x40U);       /* Start line = 0 */

    WriteCmd(0x8DU);
    WriteCmd(0x14U);       /* Enable charge pump */

    WriteCmd(0x20U);
    WriteCmd(0x02U);       /* Page addressing mode */

    WriteCmd(0xA1U);       /* Segment remap */
    WriteCmd(0xC8U);       /* COM scan direction */

    WriteCmd(0xDAU);
    WriteCmd(0x12U);       /* COM pins */

    WriteCmd(0x81U);
    WriteCmd(0x7FU);       /* Contrast */

    WriteCmd(0xD9U);
    WriteCmd(0xF1U);       /* Pre-charge period */

    WriteCmd(0xDBU);
    WriteCmd(0x30U);       /* VCOMH */

    WriteCmd(0xA4U);       /* Display follows RAM */
    WriteCmd(0xA6U);       /* Normal display */

    WriteCmd(0xAFU);       /* Display ON */

    OLED_GRAM_FILL(0x00U);
    OLED_Refresh_Gram();
}

/* ============================================================
 * 设置 OLED 页和列地址
 * ============================================================ */

void OLED_SetPos(uint8_t x, uint8_t page)
{
    if ((x >= OLED_WIDTH) || (page >= OLED_PAGE_COUNT))
    {
        return;
    }

    /*
     * SSD1306 页地址
     */
    WriteCmd((uint8_t)(0xB0U | page));

    /*
     * 列低四位和高四位
     *
     * 注意：
     * 低地址不能加 0x01，否则会导致显示整体偏移一列。
     */
    WriteCmd((uint8_t)(x & 0x0FU));
    WriteCmd((uint8_t)(0x10U | ((x >> 4U) & 0x0FU)));
}

/* ============================================================
 * 刷新显存
 * ============================================================ */

static HAL_StatusTypeDef OLED_WritePage(uint8_t page)
{
    uint8_t buffer[OLED_WIDTH + 1U];
    uint8_t column;

    if (page >= OLED_PAGE_COUNT)
    {
        return HAL_ERROR;
    }

    OLED_SetPos(0U, page);

    /*
     * 第一个字节 0x40 表示后续为显示数据
     */
    buffer[0] = 0x40U;

    for (column = 0U; column < OLED_WIDTH; column++)
    {
        buffer[(uint16_t)column + 1U] =
            OLED_GRAM[column][page];
    }

    return OLED_WriteBuffer(
        buffer,
        (uint16_t)sizeof(buffer),
        OLED_DATA_TIMEOUT_MS
    );
}

void OLED_Refresh_Gram(void)
{
    uint8_t page;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        if (OLED_WritePage(page) != HAL_OK)
        {
            break;
        }
    }
}

void OLED_RequestRefresh(void)
{
    OLED_Refresh_Gram();
}

void OLED_Service(void)
{
    /*
     * 当前不使用后台刷新任务。
     */
}

/* ============================================================
 * OLED 开关
 * ============================================================ */

void OLED_ON(void)
{
    WriteCmd(0x8DU);
    WriteCmd(0x14U);
    WriteCmd(0xAFU);
}

void OLED_OFF(void)
{
    WriteCmd(0x8DU);
    WriteCmd(0x10U);
    WriteCmd(0xAEU);
}

/* ============================================================
 * 显存操作
 * ============================================================ */

void OLED_GRAM_FILL(uint8_t fill_Data)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        for (column = 0U; column < OLED_WIDTH; column++)
        {
            OLED_GRAM[column][page] = fill_Data;
        }
    }
}

void OLED_Fill(uint8_t fill_Data)
{
    OLED_GRAM_FILL(fill_Data);
    OLED_Refresh_Gram();
}

void OLED_CLS(void)
{
    OLED_Fill(0x00U);
}

/* ============================================================
 * 画点
 * ============================================================ */

void OLED_DrawPoint(uint8_t x,
                    uint8_t y,
                    uint8_t color)
{
    uint8_t page;
    uint8_t bit;
    uint8_t mask;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    page = y / 8U;
    bit = y % 8U;
    mask = (uint8_t)(1U << bit);

    if (color != 0U)
    {
        OLED_GRAM[x][page] |= mask;
    }
    else
    {
        OLED_GRAM[x][page] &= (uint8_t)(~mask);
    }
}

/* ============================================================
 * 显示字符
 *
 * size = 12：使用 asc2_1206
 * size = 16：使用 asc2_1608，8x16 字体
 * size = 24：使用 asc2_2412
 *
 * x、y 是像素坐标。
 * ============================================================ */

void OLED_ShowChar(uint8_t x,
                   uint8_t y,
                   uint8_t chr,
                   uint8_t size,
                   uint8_t mode)
{
    uint8_t temp;
    uint8_t byte_index;
    uint8_t bit_index;
    uint8_t start_y;
    uint8_t char_width;
    uint16_t char_size;
    uint8_t font_index;

    if ((chr < ' ') || (chr > '~'))
    {
        chr = '?';
    }

    if ((size != 12U) &&
        (size != 16U) &&
        (size != 24U))
    {
        return;
    }

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    char_width = (uint8_t)(size / 2U);

    if ((uint16_t)x + char_width > OLED_WIDTH)
    {
        return;
    }

    if ((uint16_t)y + size > OLED_HEIGHT)
    {
        return;
    }

    start_y = y;
    font_index = (uint8_t)(chr - ' ');

    /*
     * 12x6  = 12 字节
     * 16x8  = 16 字节
     * 24x12 = 36 字节
     */
    char_size = (uint16_t)(
        ((size / 8U) + ((size % 8U) != 0U ? 1U : 0U))
        * char_width
    );

    for (byte_index = 0U;
         byte_index < char_size;
         byte_index++)
    {
        if (size == 12U)
        {
            temp = asc2_1206[font_index][byte_index];
        }
        else if (size == 16U)
        {
            temp = asc2_1608[font_index][byte_index];
        }
        else
        {
            temp = asc2_2412[font_index][byte_index];
        }

        for (bit_index = 0U;
             bit_index < 8U;
             bit_index++)
        {
            if ((temp & 0x80U) != 0U)
            {
                OLED_DrawPoint(x, y, mode);
            }
            else
            {
                OLED_DrawPoint(
                    x,
                    y,
                    (uint8_t)!mode
                );
            }

            temp <<= 1U;
            y++;

            if ((uint8_t)(y - start_y) >= size)
            {
                y = start_y;
                x++;
                break;
            }
        }
    }
}

/* ============================================================
 * 显示字符串
 *
 * x、y 是像素坐标。
 *
 * 8x16 字体示例：
 *
 * OLED_ShowStr(0U, 0U,
 *              (const uint8_t *)"SPEED:120",
 *              16U,
 *              1U);
 * ============================================================ */

void OLED_ShowStr(uint8_t x,
                  uint8_t y,
                  const uint8_t *str,
                  uint8_t size,
                  uint8_t mode)
{
    uint8_t char_width;
    uint16_t next_x;
    uint16_t next_y;

    if (str == NULL)
    {
        return;
    }

    if ((size != 12U) &&
        (size != 16U) &&
        (size != 24U))
    {
        return;
    }

    char_width = (uint8_t)(size / 2U);

    while ((*str >= ' ') && (*str <= '~'))
    {
        next_x = (uint16_t)x + char_width;

        /*
         * 自动换行
         */
        if (next_x > OLED_WIDTH)
        {
            next_y = (uint16_t)y + size;

            if ((uint16_t)size + next_y > OLED_HEIGHT)
            {
                break;
            }

            x = 0U;
            y = (uint8_t)next_y;
        }

        if (((uint16_t)x + char_width > OLED_WIDTH) ||
            ((uint16_t)y + size > OLED_HEIGHT))
        {
            break;
        }

        OLED_ShowChar(
            x,
            y,
            *str,
            size,
            mode
        );

        x = (uint8_t)(x + char_width);
        str++;
    }
}

/* ============================================================
 * 高层字符串接口
 *
 * 行号：1~4
 * 列号：1~16
 *
 * 字体固定为 8x16。
 * 该函数只修改显存，不立即刷新。
 * ============================================================ */

void OLED_ShowString(uint8_t line,
                     uint8_t column,
                     const char *str)
{
    uint8_t x;
    uint8_t y;

    if (str == NULL)
    {
        return;
    }

    if ((line < 1U) ||
        (line > 4U) ||
        (column < 1U) ||
        (column > 16U))
    {
        return;
    }

    x = (uint8_t)((column - 1U) * 8U);
    y = (uint8_t)((line - 1U) * 16U);

    OLED_ShowStr(
        x,
        y,
        (const uint8_t *)str,
        16U,
        1U
    );
}

/* ============================================================
 * 数字显示
 * ============================================================ */

void OLED_ShowNum(uint8_t line,
                  uint8_t column,
                  uint32_t number,
                  uint8_t length)
{
    char buffer[16];

    if (length > 10U)
    {
        length = 10U;
    }

    (void)snprintf(
        buffer,
        sizeof(buffer),
        "%0*lu",
        length,
        (unsigned long)number
    );

    OLED_ShowString(
        line,
        column,
        buffer
    );
}

void OLED_ShowSignedNum(uint8_t line,
                        uint8_t column,
                        int32_t number,
                        uint8_t length)
{
    char buffer[16];

    if (length > 11U)
    {
        length = 11U;
    }

    (void)snprintf(
        buffer,
        sizeof(buffer),
        "%+*ld",
        length,
        (long)number
    );

    OLED_ShowString(
        line,
        column,
        buffer
    );
}

void OLED_ShowFloat(uint8_t line,
                    uint8_t column,
                    float number,
                    uint8_t decimal)
{
    char buffer[24];

    if (decimal > 3U)
    {
        decimal = 3U;
    }

    if (decimal == 0U)
    {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "%+.0f",
            number
        );
    }
    else if (decimal == 1U)
    {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "%+.1f",
            number
        );
    }
    else if (decimal == 2U)
    {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "%+.2f",
            number
        );
    }
    else
    {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "%+.3f",
            number
        );
    }

    OLED_ShowString(
        line,
        column,
        buffer
    );
}

/* ============================================================
 * 中文 16x16 显示
 *
 * 每个汉字占 32 字节：
 * 前 16 字节：上半部分
 * 后 16 字节：下半部分
 * ============================================================ */

void OLED_ShowCN(uint8_t x,
                 uint8_t page,
                 uint8_t index)
{
    uint16_t address;
    uint8_t column;

    if ((uint16_t)x + 16U > OLED_WIDTH)
    {
        return;
    }

    if (page >= OLED_PAGE_COUNT - 1U)
    {
        return;
    }

    address = (uint16_t)index * 32U;

    for (column = 0U; column < 16U; column++)
    {
        OLED_GRAM[x + column][page] =
            CN16x16[address + column];

        OLED_GRAM[x + column][page + 1U] =
            CN16x16[address + 16U + column];
    }
}

/* ============================================================
 * 位图显示
 *
 * x0、x1 使用像素列坐标。
 * page0、page1 使用页坐标。
 * ============================================================ */

void OLED_DrawBMP(uint8_t x0,
                  uint8_t page0,
                  uint8_t x1,
                  uint8_t page1,
                  const uint8_t *bmp)
{
    uint16_t index;
    uint8_t page;
    uint8_t column;

    if (bmp == NULL)
    {
        return;
    }

    if ((x0 >= OLED_WIDTH) ||
        (page0 >= OLED_PAGE_COUNT))
    {
        return;
    }

    if (x1 > OLED_WIDTH)
    {
        x1 = OLED_WIDTH;
    }

    if (page1 > OLED_PAGE_COUNT)
    {
        page1 = OLED_PAGE_COUNT;
    }

    if ((x1 <= x0) || (page1 <= page0))
    {
        return;
    }

    index = 0U;

    for (page = page0; page < page1; page++)
    {
        for (column = x0; column < x1; column++)
        {
            OLED_GRAM[column][page] = bmp[index];
            index++;
        }
    }
}

/* ============================================================
 * 画直线
 * ============================================================ */

void OLED_draw_line(uint8_t x0,
                    uint8_t y0,
                    float k,
                    uint8_t mode)
{
    uint16_t x;
    float y_float;
    int32_t y_value;

    if ((x0 >= OLED_WIDTH) ||
        (y0 >= OLED_HEIGHT))
    {
        return;
    }

    for (x = 0U; x < OLED_WIDTH; x++)
    {
        y_float =
            (float)y0 +
            k * ((float)x - (float)x0);

        if ((y_float < 0.0f) ||
            (y_float >= (float)OLED_HEIGHT))
        {
            continue;
        }

        y_value = (int32_t)(y_float + 0.5f);

        if ((y_value >= 0) &&
            (y_value < (int32_t)OLED_HEIGHT))
        {
            OLED_DrawPoint(
                (uint8_t)x,
                (uint8_t)y_value,
                mode
            );
        }
    }
}

