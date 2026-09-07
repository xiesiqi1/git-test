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

void Motor_Init(void);
void Motor_SetSpeed(MotorID motor, MotorDir dir, uint8_t speed); // speed: 0~100
void Motor_Left_Fwd(uint8_t speed);
void Motor_Left_Back(uint8_t speed);
void Motor_Right_Fwd(uint8_t speed);
void Motor_Right_Back(uint8_t speed);
void Motor_Left_Stop(void);
void Motor_Right_Stop(void);
void Motor_All_Stop(void);

#endif
