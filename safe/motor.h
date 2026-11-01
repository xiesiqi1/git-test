#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1
} MotorID;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} MotorDir;

/* 1 表示正在跑电机自检(Motor_SelfTest), 此时中断里的控制环必须让路,
 * 由 main.c 的 CtrlLoop() 检查。 */
extern volatile uint8_t g_motor_busy;

void Motor_Init(void);
void Motor_SetSpeed(MotorID motor, MotorDir dir, uint8_t speed); // speed: 0~100
void Motor_Left_Fwd(uint8_t speed);
void Motor_Right_Fwd(uint8_t speed);
void Motor_Left_Stop(void);
void Motor_Right_Stop(void);
void Motor_All_Stop(void);

/* 硬件自检(阻塞约 3 秒): 左轮前进1s -> 停 -> 右轮前进1s -> 停。
 * 用来不开上位机就确认: 电机接线对不对、左右有没有搞反、转向对不对。 */
void Motor_SelfTest(void);

/* 确认反馈(阻塞约 0.5 秒): 两个轮子一起短促抖两下。
 * 用于给"双击切传感器极性"这类看不见结果的操作一个明确回应。 */
void Motor_Ack(void);

/* 切换转向极性的反馈(阻塞约 0.7 秒): 左轮抖一下 -> 右轮抖一下。
 * 动作和 Motor_Ack 明显不同, 用来区分"刚才切的是哪个开关"。 */
void Motor_AckTurn(void);

#endif
