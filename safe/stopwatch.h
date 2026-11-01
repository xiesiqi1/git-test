#ifndef __STOPWATCH_H
#define __STOPWATCH_H

#include "main.h"

/* ============================================================================
 *  stopwatch.c —— 跑圈计时 (2026-09-29 新增)
 *
 *  H 题要求"一圈 <= 20 s"。本工程没有串口, 只能靠 OLED 掐表, 所以单独做一层。
 *
 *  ★ 它【不占用任何按键手势】, 而是自动跟随循迹开关:
 *      短按 PA15 启动 -> 计时归零, 开始跑
 *      短按 PA15 停止 -> 冻结, 那个数就是这一圈的成绩, 并成为下一圈的"上次"
 *      双击/三击切极性(会强制停车) -> 计时跟着停, 不会留下假成绩
 *    实现方式就是每圈调一次 Stopwatch_Task(), 它自己看 Track_GetEnabled()
 *    的 0->1 / 1->0 边沿, 所以 main.c 里只需要 Initialize + Task 两行。
 *
 *  ★ 时间基准是 HAL_GetTick()(1 ms), 不是硬件定时器 —— 计时精度只要求 0.1 s,
 *    而 SysTick 已经由控制环用着(CTRL_PERIOD_MS 分频), 不必再占一个定时器。
 * ==========================================================================*/

void Stopwatch_Init(void);      /* 上电调用一次: 清掉两个时间 */

/* ★ 必须在主循环里高速反复调用(它同时负责检测启停边沿和累加时间)。
 *   控制环跑在 SysTick 中断里, 优先级更高, 不受影响。 */
void Stopwatch_Task(void);

uint8_t  Stopwatch_IsRunning(void);

/* 本次用时(ms)。运行中不断递增; 停下后保持冻结值 —— 也就是刚跑完的成绩。 */
uint32_t Stopwatch_GetNowMs(void);

/* 上一次跑完的成绩(ms)。第一次启动前是 0。 */
uint32_t Stopwatch_GetLastMs(void);

#endif
