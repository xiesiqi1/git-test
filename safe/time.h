#ifndef __TIME_H
#define __TIME_H

#include "main.h"

void    Timer_Init(void);
void    Timer_Start(void);
void    Timer_Stop(void);
void    Timer_Reset(void);
uint32_t Timer_GetElapsed(void);
uint8_t Timer_IsRunning(void);

void Timer_GetString(char *buf, uint32_t ms);

#endif
