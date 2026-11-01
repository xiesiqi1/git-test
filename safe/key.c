/*
 * key.c —— PA15 按键: 软件消抖 + 单击 / 双击 / 三击 / 长按
 * 电路: PA15 <-> 按键 <-> GND (按下为低)
 *
 * 连按计数法: 每次"按下并松手"(且不是长按)算一次点击, 然后等 KEY_CLICK_MS:
 *   窗口内没有下一击 -> 按累计次数上报 KEY_SHORT / KEY_DOUBLE / KEY_TRIPLE
 *   累计到三击       -> 立即上报, 不用再等窗口
 * 长按: 按住达到 KEY_LONG_MS 立即上报 KEY_LONG, 并且这次按住不再计入点击
 *       (单击必须等松手才判定, 否则一按下就报单击, 长按永远轮不到)
 */
#include "key.h"

#define KEY_DEBOUNCE_MS   20u    /* 消抖时间 */
#define KEY_LONG_MS       800u   /* 按住多久算长按 */
#define KEY_CLICK_MS      350u   /* 两次点击的间隔上限; 超过就算"点击结束" */

static uint8_t  s_key_state = 1;      /* 1=松开, 0=按下(稳定态) */
static uint8_t  s_raw_last  = 1;      /* 上次读取到的原始电平 */
static uint32_t s_chg_tick  = 0;      /* 电平最后一次变化的时刻 */
static uint32_t s_press_tick = 0;     /* 本次按下的时刻 */
static uint32_t s_click_tick = 0;     /* 最近一次点击(松手)的时刻 */
static uint8_t  s_click_cnt = 0;      /* 累计点击次数(等待窗口结束) */
static uint8_t  s_long_done = 0;      /* 本次按住已经报过长按了 */
static uint8_t  s_event     = KEY_NONE;

/* 快速扫描, 建议在主循环中调用 */
uint8_t Key_Scan(void)
{
    uint8_t  raw = (uint8_t)HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    uint32_t now = HAL_GetTick();

    if (raw != s_raw_last)
    {
        s_raw_last = raw;
        s_chg_tick = now;
    }
    else if ((uint32_t)(now - s_chg_tick) >= KEY_DEBOUNCE_MS)
    {
        if (s_key_state != raw)              /* 电平稳定且与当前稳定态不同 */
        {
            s_key_state = raw;
            if (raw == 0)                    /* 按下: 记录起点, 准备判长按 */
            {
                s_press_tick = now;
                s_long_done  = 0;
            }
            else if (!s_long_done)           /* 松手且不是长按的松手 -> 算一次点击 */
            {
                s_click_cnt++;
                s_click_tick = now;
                if (s_click_cnt >= 3u)       /* 三击: 立即上报, 不用再等窗口 */
                {
                    s_click_cnt = 0;
                    s_event     = KEY_TRIPLE;
                }
            }
        }
    }

    /* 按住到阈值: 立刻报长按, 并清掉待定的点击(按住这么久显然不是连击) */
    if (s_key_state == 0 && !s_long_done
        && (uint32_t)(now - s_press_tick) >= KEY_LONG_MS)
    {
        s_long_done  = 1;
        s_click_cnt  = 0;
        s_event      = KEY_NONE;
        return KEY_LONG;
    }

    /* 等够窗口还没等到下一击 -> 按累计次数确认手势 */
    if (s_click_cnt > 0 && (uint32_t)(now - s_click_tick) >= KEY_CLICK_MS)
    {
        uint8_t n = s_click_cnt;
        s_click_cnt = 0;
        if      (n == 1u) s_event = KEY_SHORT;
        else if (n == 2u) s_event = KEY_DOUBLE;
        else              s_event = KEY_TRIPLE;
    }

    if (s_event != KEY_NONE)
    {
        uint8_t e = s_event;
        s_event   = KEY_NONE;
        return e;
    }
    return KEY_NONE;
}
