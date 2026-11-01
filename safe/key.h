#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#define KEY_Pin      GPIO_PIN_15   /* PA15 按键(按下=低电平) */
#define KEY_GPIO_Port GPIOA

/* Key_Scan() 返回值 */
#define KEY_NONE   0u   /* 无动作                                        */
#define KEY_SHORT  1u   /* 单击: 启动 / 停止 循迹                          */
#define KEY_DOUBLE 2u   /* 双击: 切换【传感器极性】(黑线对应低/高电平)       */
#define KEY_TRIPLE 3u   /* 三击: 切换【转向极性】(修正方向反过来)            */
#define KEY_LONG   4u   /* 长按(≥0.8s): 电机自检(左轮转1s + 右轮转1s)       */

/* 四种手势怎么分开的(连按计数法):
 *   单击/双击/三击 -> 每次"按下并松手"算一次点击, 然后等 350ms:
 *                     窗口内没有下一击就按累计次数上报; 累计到三击立即上报。
 *                     所以单击/双击会有约 0.35s 延迟, 三击是立刻响应的。
 *   长按           -> 按住达到 0.8s 立即触发, 这次按住不再计入点击。
 *
 * 为什么把"极性"和"转向"都做成按键可切换:
 *   本工程没有串口(PA9/PA2/PA3 被电机和循迹占用), 这两个开关搞反了
 *   只能上车试, 而改一次宏就要重新编译+下载+断电, 太慢。
 */
uint8_t Key_Scan(void);   /* 应在主循环里高速反复调用 */

#endif
