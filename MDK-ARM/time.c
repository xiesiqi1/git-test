#include "time.h"

static uint32_t start_tick   = 0;
static uint32_t elapsed_tick = 0;
static uint8_t  running      = 0;

void Timer_Init(void)
{
    start_tick   = 0;
    elapsed_tick = 0;
    running      = 0;
}

void Timer_Start(void)
{
    start_tick = HAL_GetTick();
    running    = 1;
}

void Timer_Stop(void)
{
    if (running)
    {
        elapsed_tick += HAL_GetTick() - start_tick;
        running = 0;
    }
}

void Timer_Reset(void)
{
    start_tick   = 0;
    elapsed_tick = 0;
    running      = 0;
}

uint32_t Timer_GetElapsed(void)
{
    if (running)
    {
        return elapsed_tick + (HAL_GetTick() - start_tick);
    }
    return elapsed_tick;
}

uint8_t Timer_IsRunning(void)
{
    return running;
}

void Timer_GetString(char *buf, uint32_t ms)
{
    uint32_t total_sec = ms / 1000;
    uint32_t milli     = ms % 1000;
    uint32_t min       = total_sec / 60;
    uint32_t sec       = total_sec % 60;

    buf[0] = '0' + (char)(min / 10);
    buf[1] = '0' + (char)(min % 10);
    buf[2] = ':';
    buf[3] = '0' + (char)(sec / 10);
    buf[4] = '0' + (char)(sec % 10);
    buf[5] = '.';
    buf[6] = '0' + (char)(milli / 100);
    buf[7] = '0' + (char)((milli / 10) % 10);
    buf[8] = '\0';
}