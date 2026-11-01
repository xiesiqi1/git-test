#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 编码器 / 电机硬件对应（与 TEST1.ioc、motor.c 保持一致）：
 * 编码器   : TIM2 CH1/CH2 (PA0/PA1) —— 左轮编码器
 *            TIM4 CH1/CH2 (PB6/PB7) —— 右轮编码器
 * 电机方向 : 左 = IN3(PA9)/IN4(PB3)   右 = IN1(PA11)/IN2(PA12)
 * 电机 PWM : 左 = TIM3_CH2(PB5)       右 = TIM3_CH1(PB4)
 * 若实际左右接反，到 track.c 里把左右映射对调即可
 *
 * ★★ 2026-09-29 【必读】累计计数必须靠 Encoder_Update() 推进
 *   TIM2/TIM4 只是两个自由计数的硬件计数器。软件要得到"一共走了多少"，
 *   必须【每拍】去读一次硬件计数器、把增量累加进 total。
 *   这一个动作由 Encoder_Update() 完成 —— 它【必须每个控制拍调用一次】，
 *   现在由 Mileage_Task() 的第一行负责（它在 CtrlLoop 里，每 5ms 必到）。
 *
 *   ⚠ 踩过的坑: 早先的写法是"GetDelta() 里顺手累加 total"，而删掉速度闭环
 *     之后再没有任何地方调用 GetDelta() → total 永远停在 0 → 屏上 ENC 几行
 *     全是 0、里程恒为 0、A/B 点提示不响、停车里程门槛永远不成立。
 *     【凡是"必须定期被执行"的动作，就要有一个明确的调用点，不能挂在
 *       "碰巧会被调用"的读接口上。】现在 Update 是写接口、Get 是纯读，职责分开。
 */

void Encoder_Init(void);            /* 启动 TIM2/TIM4 编码器模式 + 清零 */
void Encoder_Reset(void);           /* 计数清零（含上次读数基准） */

/* ★ 每个控制拍调用一次: 读硬件计数器 -> 算增量 -> 累加进累计计数。
 *   整个工程的累计里程/ENC 显示全部依赖它, 漏调 = 计数永远 0。 */
void Encoder_Update(void);

/* 最近一次 Encoder_Update() 算出的增量(有符号)。纯读, 不消费 ——
 * 同一拍里调几次都是同一个值。 */
int16_t Encoder_GetLeftDelta(void);
int16_t Encoder_GetRightDelta(void);

int32_t Encoder_GetLeftCount(void); /* 左轮累计计数（有符号，从启动时刻） */
int32_t Encoder_GetRightCount(void);/* 右轮累计计数（有符号，从启动时刻） */

/* 速度 = (delta / 控制周期) 折算为 计数/秒。基于最近一次 Update 的增量,
 * 所以刷新率就是控制频率(CTRL_PERIOD_MS)。 */
int32_t Encoder_GetLeftSpeed(void);
int32_t Encoder_GetRightSpeed(void);

#endif
