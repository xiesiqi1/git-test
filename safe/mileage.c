/* ============================================================================
 *  mileage.c —— 里程 / A·B 点判定 (2026-09-29 新增)
 *
 *  为什么用编码器当里程: 本工程【没有串口、没有陀螺仪、没有摄像头】,
 *  唯一能回答"我现在在赛道的哪一段"的传感器就是两个轮子的编码器。
 *
 *  ★ 里程定义 = (|左计数| + |右计数|) / 2
 *      · 取绝对值: 不怕某一侧编码器接线反相(否则一个正一个负, 平均恒为 0)。
 *      · 取两轮平均: 绕圈时内外轮走的路本来就不一样, 平均才代表
 *        【车体中心】走了多远 —— 而"到没到 A 点"问的正是车体中心。
 *
 *  ★ B 点 = 一圈的 24.4%(= 1.5 m / 6.142 m), 见 config.h 的说明。
 *
 *  ⚠ 本文件运行在【控制中断】里(Mileage_Task 由 CtrlLoop 调用), 所以
 *    触发提示只能调 signal.h 里那几个"中断安全"的接口(Signal_MarkB/A),
 *    不能直接去写 GPIO 或改 signal.c 的状态机。
 * ========================================================================== */
#include "mileage.h"
#include "encoder.h"
#include "track.h"
#include "signal.h"
#include "config.h"

#ifndef MILEAGE_LAP_COUNT
#define MILEAGE_LAP_COUNT    0
#endif
#ifndef MILEAGE_B_RATIO_PM
#define MILEAGE_B_RATIO_PM   244
#endif

static int32_t s_cnt      = 0;    /* 当前里程(编码器平均计数) */
static uint8_t s_mark     = 0u;   /* 0 = 未到 B, 1 = 已过 B, 2 = 已过 A */
static uint8_t s_prev_en  = 0u;   /* 上一拍的循迹开关, 用来抓启动边沿 */
static uint8_t s_was_run  = 0u;   /* 这一圈有没有真的跑起来过(→ 由 0 变 1) */

/* 取绝对值。写成这样是为了 INT32_MIN 也不会溢出(虽然计数不可能到那里)。 */
static int32_t Abs32(int32_t v)
{
    if (v >= 0)
    {
        return v;
    }
    return (int32_t)(-(v + 1)) + 1;
}

void Mileage_Init(void)
{
    s_cnt     = 0;
    s_mark    = 0u;
    s_was_run = 0u;
    s_prev_en = (uint8_t)((Track_GetEnabled() != 0) ? 1u : 0u);
}

void Mileage_Task(void)
{
    uint8_t en = (uint8_t)((Track_GetEnabled() != 0) ? 1u : 0u);
    int32_t lc;
    int32_t rc;
    int32_t lap;
    int32_t bp;

    /* ★★★ 2026-09-29 【必须放在第一行】: 推进编码器累计计数。
     *   硬件(TIM2/TIM4)只是在自由计数, 软件不主动去读、增量就不会进 total。
     *   本函数每 5ms 被 CtrlLoop 调一次, 是"整圈里最可靠的每拍调用点",
     *   所以累计就挂在这里。
     *   ⚠ 踩过的坑: 以前把累加塞在 Encoder_GetDelta() 里, 而删掉速度闭环后
     *     没人再调 GetDelta → ENC 恒为 0、里程恒为 0、A/B 点不响、
     *     停车里程门槛永远不成立。详见 encoder.h 开头。 */
    Encoder_Update();

    /* --- 启动边沿: 新的一圈开始 ---
     * 计数不用在这里清 —— 按启动时 main.c 已经 Encoder_Reset() 了。
     * 这里清的是"标记点"和上传提示灯。 */
    if (en != 0u && s_prev_en == 0u)
    {
        s_mark    = 0u;
        s_was_run = 1u;
        Signal_ClearLamp();            /* 上一圈到 A 点是常亮的, 这里灭掉 */
    }
    s_prev_en = en;

    /* --- 刷新里程 --- */
    lc    = Encoder_GetLeftCount();
    rc    = Encoder_GetRightCount();
    s_cnt = (Abs32(lc) + Abs32(rc)) / 2;

    /* --- 标记点判定: 只有"正在跑"且"已标定"才做 --- */
    if ((en == 0u) || (s_was_run == 0u))
    {
        return;
    }

    lap = (int32_t)MILEAGE_LAP_COUNT;
    if (lap <= 0)
    {
        return;                        /* 未标定: 不做任何判定 */
    }

    bp = (lap * (int32_t)MILEAGE_B_RATIO_PM) / 1000;

    /* B 点: 通过时给声光提示(短音 + 闪灯) */
    if ((s_mark == 0u) && (s_cnt >= bp))
    {
        s_mark = 1u;
        Signal_MarkB();
    }
    /* A 点: 一圈终点, 给一个不一样的提示(长音 + 常亮) */
    else if ((s_mark == 1u) && (s_cnt >= lap))
    {
        s_mark = 2u;
        Signal_MarkA();
    }
    else
    {
        /* 还没跨过下一个点 */
    }
}

int32_t Mileage_GetCount(void)
{
    return s_cnt;
}

int32_t Mileage_GetLapCount(void)
{
    return (int32_t)MILEAGE_LAP_COUNT;
}

int32_t Mileage_GetBPoint(void)
{
    int32_t lap = (int32_t)MILEAGE_LAP_COUNT;

    if (lap <= 0)
    {
        return 0;                      /* 未标定 */
    }
    return (lap * (int32_t)MILEAGE_B_RATIO_PM) / 1000;
}

uint8_t Mileage_GetMark(void)
{
    return s_mark;
}
