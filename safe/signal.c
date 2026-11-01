/* ============================================================================
 *  signal.c —— 声光提示: 蜂鸣器(PB0, 有源) + LED(PC13 板载)
 *  2026-09-29 新增。
 *
 *  ★ 为什么用"状态机 + HAL_GetTick"而不是 HAL_Delay:
 *    控制环是 5 ms 一拍的硬节拍, 提示音要响 400 ms —— 如果在这 400 ms 里
 *    死等, 循迹控制会整整丢 80 拍(车都冲出去了)。所以这里只记录
 *    "还要响多久 / 还要响几次", 由 Signal_Task() 每圈推进一点, 立刻返回。
 *
 *  ★ 两个引脚的电平方向由 config.h 的 BUZZER_ACTIVE_LEVEL / LED_ACTIVE_LEVEL
 *    决定。★ 本车【实测确认】: 蜂鸣器 = 1(高电平响)、LED = 0(低电平亮) ——
 *    两者方向【正好相反】, 所以下面 BzWrite/LedWrite 的条件也相反, 别照抄。
 *    接反了就改那两个宏(只能实测, 不能推理)。
 *
 *  ⚠ 上电初始电平很重要: 蜂鸣器是高电平响, 若 IO 一上电就是高, 会【响一下】。
 *    所以 Signal_Init() 一进来就把两个脚置成"不响/不亮", 而且不依赖
 *    CubeMX 生成的 gpio.c 初始电平(那部分重新生成代码时会被改回 RESET)。
 *
 *  ★ 事件用"只自增的计数器"跨上下文传递, 见 signal.h 的说明。
 * ========================================================================== */
#include "signal.h"
#include "config.h"
/* BUZZER_Pin/BUZZER_GPIO_Port 与 LED_Pin/LED_GPIO_Port 由 main.h 给出
 * (CubeMX 按 User Label 生成的名字 —— 见 Core/Inc/main.h)。 */

#ifndef BUZZER_ACTIVE_LEVEL
#define BUZZER_ACTIVE_LEVEL   0
#endif
#ifndef LED_ACTIVE_LEVEL
#define LED_ACTIVE_LEVEL      0
#endif
#ifndef SIGNAL_BEEP_MS
#define SIGNAL_BEEP_MS        60u
#endif
#ifndef SIGNAL_LONG_MS
#define SIGNAL_LONG_MS        400u
#endif
#ifndef SIGNAL_GAP_MS
#define SIGNAL_GAP_MS         120u
#endif

/* LED 闪烁的两个时长(只在本文件用, 不进 config.h, 免得参数太多) */
#define LED_BLINK_ON_MS       80u
#define LED_BLINK_GAP_MS      80u

/* ---- 跨上下文的请求计数: 只由中断自增, 只由主循环读取和记录 ---- */
static volatile uint8_t s_evt_b     = 0u;   /* 到过 B 点的累计次数 */
static volatile uint8_t s_evt_a     = 0u;   /* 到过 A 点的累计次数 */
static volatile uint8_t s_evt_clear = 0u;   /* 请求"灯灭+停响"的累计次数 */
static uint8_t s_done_b     = 0u;
static uint8_t s_done_a     = 0u;
static uint8_t s_done_clear = 0u;

/* ---- 蜂鸣器: phase 0 = 正在响, 1 = 两声之间, 2 = 空闲 ---- */
static uint8_t  s_bz_phase = 2u;
static uint8_t  s_bz_left  = 0u;    /* 还欠几声(含正在响的这声) */
static uint16_t s_bz_on    = 0u;
static uint16_t s_bz_gap   = 0u;
static uint32_t s_bz_next  = 0u;

/* ---- LED: mode 0 = 灭, 1 = 常亮, 2 = 闪 ---- */
static uint8_t  s_led_mode  = 0u;
static uint8_t  s_led_left  = 0u;
static uint8_t  s_led_phase = 0u;   /* 0 = 亮着, 1 = 灭着 */
static uint32_t s_led_next  = 0u;

/* ---------------------------------------------------------------------------
 *  引脚操作(电平方向由 config.h 的宏决定)
 * ------------------------------------------------------------------------- */
static void BzWrite(uint8_t on)
{
#if (BUZZER_ACTIVE_LEVEL)
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                      (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                      (on != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

static void LedWrite(uint8_t on)
{
#if (LED_ACTIVE_LEVEL)
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                      (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                      (on != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

static void BzStop(void)
{
    s_bz_phase = 2u;
    s_bz_left  = 0u;
    BzWrite(0u);
}

/* ---------------------------------------------------------------------------
 *  事件入口(中断安全: 只自增计数器)
 * ------------------------------------------------------------------------- */

void Signal_MarkB(void)
{
    s_evt_b++;                   /* 只自增 —— 主循环一定看得见 */
}

void Signal_MarkA(void)
{
    s_evt_a++;
}

void Signal_ClearLamp(void)
{
    s_evt_clear++;
}

/* ---------------------------------------------------------------------------
 *  直接控制(只在主循环上下文调用)
 * ------------------------------------------------------------------------- */

void Signal_BeepN(uint8_t n, uint16_t on_ms, uint16_t gap_ms)
{
    if (n == 0u)
    {
        return;                  /* 0 声 = 什么都不做, 不要打断正在响的 */
    }

    s_bz_on    = on_ms;
    s_bz_gap   = gap_ms;
    s_bz_left  = n;
    s_bz_phase = 0u;             /* 立刻开始响, 不等间隔 */
    BzWrite(1u);
    s_bz_next  = HAL_GetTick() + (uint32_t)on_ms;
}

void Signal_Beep(uint16_t on_ms)
{
    Signal_BeepN(1u, on_ms, 0u);
}

void Signal_LedSet(uint8_t on)
{
    s_led_mode  = (on != 0u) ? 1u : 0u;
    s_led_left  = 0u;
    LedWrite(on);
}

void Signal_LedBlink(uint8_t n)
{
    if (n == 0u)
    {
        return;
    }

    s_led_mode  = 2u;
    s_led_left  = n;
    s_led_phase = 0u;            /* 先亮 */
    LedWrite(1u);
    s_led_next  = HAL_GetTick() + LED_BLINK_ON_MS;
}

/* ---------------------------------------------------------------------------
 *  初始化
 * ------------------------------------------------------------------------- */

void Signal_Init(void)
{
    s_evt_b     = 0u;
    s_evt_a     = 0u;
    s_evt_clear = 0u;
    s_done_b     = 0u;
    s_done_a     = 0u;
    s_done_clear = 0u;

    s_led_mode  = 0u;
    s_led_left  = 0u;

    BzStop();                    /* 不响 */
    LedWrite(0u);                /* 不亮 */
}

/* ---------------------------------------------------------------------------
 *  每圈推进
 * ------------------------------------------------------------------------- */

/* 处理中断里记下的事件。
 * ★ 用"两个计数"而不是置标志位: 即使这一圈正好有事件进来, 也只是把计数
 *   又加了一次, 下一圈再处理 —— 一次都不会丢。 */
static void RequestTick(void)
{
    uint8_t eb = s_evt_b;
    uint8_t ea = s_evt_a;
    uint8_t ec = s_evt_clear;

    if (ec != s_done_clear)
    {
        s_done_clear = ec;
        BzStop();
        Signal_LedSet(0u);
    }

    if (eb != s_done_b)
    {
        s_done_b = eb;
        /* 通过 B 点: 短音 1 声 + 灯闪 1 下。
         * ★ 声和光【不一定同时被感知】—— 赛场环境噪声大, 裁判可能听不见
         *   蜂鸣器, 所以灯是必需的备用通道; 反过来从侧面看不见灯时靠声音。
         *   两个一起给, 才叫"声光提示"。 */
        Signal_BeepN(1u, (uint16_t)SIGNAL_BEEP_MS, 0u);
        Signal_LedBlink(1u);
    }

    if (ea != s_done_a)
    {
        s_done_a = ea;
        /* 回到 A 点(一圈终点): 长音 + 灯常亮。
         * ★ 用【长音】和【常亮】跟 B 点区分开 —— 不用数次数就能听/看出
         *   这是终点。 */
        Signal_Beep((uint16_t)SIGNAL_LONG_MS);
        Signal_LedSet(1u);
    }
}

/* 蜂鸣器时序 */
static void BuzzerTick(uint32_t now)
{
    if (s_bz_phase == 2u)
    {
        return;                  /* 空闲 */
    }

    if ((int32_t)(now - s_bz_next) < 0)
    {
        return;                  /* 当前这一段还没走完 */
    }

    if (s_bz_phase == 0u)
    {
        BzWrite(0u);             /* 这一段"响"结束了 */

        if (s_bz_left > 0u)
        {
            s_bz_left--;         /* 这一声结清 */
        }

        if (s_bz_left == 0u)
        {
            s_bz_phase = 2u;     /* 全部响完, 空闲 */
        }
        else
        {
            s_bz_phase = 1u;     /* 进入两声之间的静音段 */
            s_bz_next  = now + (uint32_t)s_bz_gap;
        }
    }
    else
    {
        BzWrite(1u);             /* 开始下一声 */
        s_bz_phase = 0u;
        s_bz_next  = now + (uint32_t)s_bz_on;
    }
}

/* LED 时序 */
static void LedTick(uint32_t now)
{
    if (s_led_mode != 2u)
    {
        return;                  /* 灭或常亮都不需要推进 */
    }

    if ((int32_t)(now - s_led_next) < 0)
    {
        return;
    }

    if (s_led_phase == 0u)
    {
        LedWrite(0u);            /* 亮 -> 灭 */
        if (s_led_left > 0u)
        {
            s_led_left--;
        }

        if (s_led_left == 0u)
        {
            s_led_mode = 0u;     /* 闪完了, 保持灭 */
        }
        else
        {
            s_led_phase = 1u;
            s_led_next  = now + LED_BLINK_GAP_MS;
        }
    }
    else
    {
        LedWrite(1u);            /* 灭 -> 亮 */
        s_led_phase = 0u;
        s_led_next  = now + LED_BLINK_ON_MS;
    }
}

void Signal_Task(void)
{
    RequestTick();
    BuzzerTick(HAL_GetTick());
    LedTick(HAL_GetTick());
}
