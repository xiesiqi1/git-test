#include "usart1.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define USART2_TX_TIMEOUT 100U

static const uint8_t vofa_justfloat_tail[4] =
{
    0x00U, 0x00U, 0x80U, 0x7FU
};


static void USART2_Write(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)data,
                      length,
                      USART2_TX_TIMEOUT);
}

//发送一个字节
void USART2_SendString(const char *str)
{
    uint16_t length;

    if (str == NULL)
    {
        return;
    }

    length = (uint16_t)strlen(str);
    USART2_Write((const uint8_t *)str, length);
}

//发送普通字符串
void USART2_SendLine(const char *str)
{
    USART2_SendString(str);
    USART2_SendString("\r\n");
}

//发送字符串并自动添加换行
void USART2_SendByte(uint8_t data)
{
    USART2_Write(&data, 1U);
}

/* VOFA+ JustFloat：4 个 float 加 4 字节帧尾，共 20 字节 */
void USART2_SendVofa4(float ch1,
                      float ch2,
                      float ch3,
                      float ch4)
{
    uint8_t frame[20U];
    uint16_t offset = 0U;

    memcpy(&frame[offset], &ch1, sizeof(ch1));
    offset += sizeof(ch1);

    memcpy(&frame[offset], &ch2, sizeof(ch2));
    offset += sizeof(ch2);

    memcpy(&frame[offset], &ch3, sizeof(ch3));
    offset += sizeof(ch3);

    memcpy(&frame[offset], &ch4, sizeof(ch4));
    offset += sizeof(ch4);

    memcpy(&frame[offset],
           vofa_justfloat_tail,
           sizeof(vofa_justfloat_tail));

    USART2_Write(frame, sizeof(frame));
}


//printf通信
int __io_putchar(int ch)
{
    uint8_t data = (uint8_t)ch;

    HAL_UART_Transmit(&huart1,
                      &data,
                      1U,
                      100U);

    return ch;
}

int fputc(int ch, FILE *f)
{
    (void)f;
    return __io_putchar(ch);
}
