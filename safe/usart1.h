#ifndef __USART1_H
#define __USART1_H

#include "main.h"

void USART2_SendByte(uint8_t data);
void USART2_SendString(const char *str);
void USART2_SendLine(const char *str);

void USART2_SendVofa4(float ch1,
                      float ch2,
                      float ch3,
                      float ch4);
#endif	

                      