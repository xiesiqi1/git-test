#ifndef __SIGNAL_H
#define __SIGNAL_H

#include "main.h"

/* ============================================================================
 *  signal.h —— 声光提示 (蜂鸣器 PB0 + LED PC13)
 *  2026-09-29 新增。用于 2026 电赛 H 题要求的"通过 B 点时声光提示"。
 *
 *  为什么单独一个模块: 提示动作(响几声、灯怎么闪)是有【时序】的, 而控制环
 *  是 5 ms 一拍的硬节拍, 不能在里面延时。所以这里只做两件事:
 *      · 上层调 Signal_MarkB() / Signal_MarkA() —— 一次调用, 立刻返回;
 *      · 主循环反复调 Signal_Task() —— 由它按 HAL_GetTick() 推进时序。
 *
 *  ★★ 调用上下文(重要): 里程判定(mileage.c)跑在【控制中断】里, 而
 *     Signal_Task() 跑在主循环里, 两者会同时碰这套状态。所以:
 *        Signal_MarkA / Signal_MarkB / Signal_ClearLamp  —— 【中断安全】,
 *            内部只对一个计数器自增/置位, 真正干活留给 Signal_Task 做;
 *        Signal_Beep / Signal_BeepN / Signal_LedSet / Signal_LedBlink
 *            —— 【只能在主循环上下文调用】(别在中断里用)。
 *     ★ 为什么不用 volatile 标志位: "中断置位 -> 主循环清零"中间一旦被打断,
 *       那个位就会被主循环顺手清掉, 于是【B 点的提示音永远不响】。
 *       改成"只自增的计数器 + 主循环记住已处理到几" 就没有这个问题。
 *
 *  ⚠ Signal_Task() 必须在主循环里【经常】被调用(本工程每圈一次),
 *    否则响/闪的时机会被拉长。
 * ========================================================================== */

void Signal_Init(void);     /* 上电调用一次: 两个输出都置成"不响/不亮" */

void Signal_Task(void);     /* 主循环每圈调用: 处理请求 + 推进时序 */

/* ---- 下面 4 个只能在主循环上下文调用 ---- */
void Signal_BeepN(uint8_t n, uint16_t on_ms, uint16_t gap_ms);
void Signal_Beep(uint16_t on_ms);
void Signal_LedSet(uint8_t on);
void Signal_LedBlink(uint8_t n);

/* ---- 下面 3 个是【中断安全】的事件入口 ---- */
void Signal_MarkB(void);        /* 通过 B 点: 短音 1 声 + 灯闪 1 下 */
void Signal_MarkA(void);        /* 回到 A 点(一圈终点): 长音 + 灯常亮 */
void Signal_ClearLamp(void);    /* 灯灭 + 停响(每圈启动时复位用) */

#endif
