#ifndef OLED_H
#define OLED_H

#include "main.h"
#include <stdint.h>

#define OLED_WIDTH       128U
#define OLED_HEIGHT      64U
#define OLED_PAGE_COUNT  8U

#define OLED_ADDRESS             0x78U
#define OLED_COMMAND_TIMEOUT_MS  100U
#define OLED_DATA_TIMEOUT_MS     100U

extern I2C_HandleTypeDef hi2c1;

extern uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGE_COUNT];

/* 初始化和刷新 */
void OLED_Init(void);
void OLED_Refresh_Gram(void);
void OLED_RequestRefresh(void);
void OLED_Service(void);

/* OLED 开关 */
void OLED_ON(void);
void OLED_OFF(void);

/* 设置位置 */
void OLED_SetPos(uint8_t x, uint8_t page);

/* 显存操作 */
void OLED_GRAM_FILL(uint8_t fill_Data);
void OLED_Fill(uint8_t fill_Data);
void OLED_CLS(void);

/* 像素操作 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color);

/* 字符和字符串 */
void OLED_ShowChar(uint8_t x,
                   uint8_t y,
                   uint8_t chr,
                   uint8_t size,
                   uint8_t mode);

void OLED_ShowStr(uint8_t x,
                  uint8_t y,
                  const uint8_t *str,
                  uint8_t size,
                  uint8_t mode);

/* 高层接口，行号和列号从 1 开始 */
void OLED_ShowString(uint8_t line,
                     uint8_t column,
                     const char *str);

void OLED_ShowNum(uint8_t line,
                  uint8_t column,
                  uint32_t number,
                  uint8_t length);

void OLED_ShowSignedNum(uint8_t line,
                        uint8_t column,
                        int32_t number,
                        uint8_t length);

void OLED_ShowFloat(uint8_t line,
                    uint8_t column,
                    float number,
                    uint8_t decimal);

/* 中文 */
void OLED_ShowCN(uint8_t x,
                 uint8_t page,
                 uint8_t index);

/* 位图 */
void OLED_DrawBMP(uint8_t x0,
                  uint8_t page0,
                  uint8_t x1,
                  uint8_t page1,
                  const uint8_t *bmp);

/* 画线 */
void OLED_draw_line(uint8_t x0,
                    uint8_t y0,
                    float k,
                    uint8_t mode);


#endif

                    