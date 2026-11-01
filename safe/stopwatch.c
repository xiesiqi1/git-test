/*
 * stopwatch.c —— 跑圈计时 (2026-09-29 新增)
 *
 * 背景: H 题要求"一圈 <= 20 s", 而本工程没有串口 —— 唯一能掐表的地方就是
 *       那块 OLED。计时必须在"车真的开始跑"和"车真的停"之间, 靠人手按秒表
 *       对不上一圈, 所以做成【自动跟随循迹开关】: 循迹启动 = 表开跑, 停车 =
 *       表停。双击/三击切极性会强制停车, 计时也跟着停, 不会留下一条假成绩。
 *
 * ★ 为什么不用一个硬件定时器: 只要 0.1 s 精度, HAL_GetTick()(1 ms) 足够;
 *   本机 TIM2/TIM3/TIM4 已经分别被编码器和 PWM 占用了。
 *
 * ★ 时间差一律用无符号减法: HAL_GetTick() 在 49 天时回绕, (now - t0) 在
 *   无符号语义下依然正确。
 */
#include "stopwatch.h"
#include "track.h"
#include "config.h"

#ifndef STOPWATCH_MAX_MS
#define STOPWATCH_MAX_MS   599999u
#endif

static uint32_t s_t0      = 0;   /* 本次起跑时刻(HAL_GetTick) */
static uint32_t s_now     = 0;   /* 本次已用时(ms), 停下后冻结 */
static uint32_t s_last    = 0;   /* 上一次跑完的成绩(ms) */
static uint8_t  s_running = 0;   /* 1 = 正在跑 */
static uint8_t  s_prev_en = 0;   /* 上一圈的循迹开关状态, 用来抓边沿 */

void Stopwatch_Init(void)
{
    s_t0      = HAL_GetTick();
    s_now     = 0u;
    s_last    = 0u;
    s_running = 0u;
    s_prev_en = (uint8_t)((Track_GetEnabled() != 0) ? 1u : 0u);
}

void Stopwatch_Task(void)
{
    uint8_t en = (uint8_t)((Track_GetEnabled() != 0) ? 1u : 0u);

    if (en != 0u && s_prev_en == 0u)
    {
        /* 0 -> 1: 启动。归零重跑, 上一次的成绩留在 s_last 里供对比。 */
        s_t0      = HAL_GetTick();
        s_now     = 0u;
        s_running = 1u;
    }
    else if (en == 0u && s_prev_en != 0u)
    {
        /* 1 -> 0: 停车。冻结当前读数, 并把它存成"上一次的成绩"。
         * ★ 启动前 Track_StartOk() 没通过时不会走到这里(那次根本没 0->1),
         *   所以"探头看不到线、车没动"不会污染成绩。 */
        s_running = 0u;
        s_last    = s_now;
    }

    s_prev_en = en;

    if (s_running != 0u)
    {
        s_now = (uint32_t)(HAL_GetTick() - s_t0);
        if (s_now > STOPWATCH_MAX_MS)
        {
            s_now = STOPWATCH_MAX_MS;      /* 到顶就不动了, 免得撑出屏外 */
        }
    }
}

uint8_t Stopwatch_IsRunning(void) { return s_running; }

/* 本次用时。运行中递增; 停下后保持冻结 —— 就是刚跑完的那一圈的成绩。 */
uint32_t Stopwatch_GetNowMs(void) { return s_now; }

/* 上一次跑完的成绩。 */
uint32_t Stopwatch_GetLastMs(void) { return s_last; }
