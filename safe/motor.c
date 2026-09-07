#include "motor.h"
#include "tim.h"


#define LEFT_IN1_PIN    IN3_Pin
#define LEFT_IN1_PORT   IN3_GPIO_Port
#define LEFT_IN2_PIN    IN4_Pin
#define LEFT_IN2_PORT   IN4_GPIO_Port


#define RIGHT_IN1_PIN   IN1_Pin
#define RIGHT_IN1_PORT  IN1_GPIO_Port
#define RIGHT_IN2_PIN   IN2_Pin
#define RIGHT_IN2_PORT  IN2_GPIO_Port


#define LEFT_PWM_CH     TIM_CHANNEL_2
#define RIGHT_PWM_CH    TIM_CHANNEL_1

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, LEFT_PWM_CH);
    HAL_TIM_PWM_Start(&htim3, RIGHT_PWM_CH);

    __HAL_TIM_SET_COMPARE(&htim3, LEFT_PWM_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim3, RIGHT_PWM_CH, 0);

    HAL_GPIO_WritePin(LEFT_IN1_PORT, LEFT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEFT_IN2_PORT, LEFT_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_IN1_PORT, RIGHT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_IN2_PORT, RIGHT_IN2_PIN, GPIO_PIN_RESET);
}

void Motor_SetSpeed(MotorID motor, MotorDir dir, uint8_t speed)
{
    if (speed > 100) speed = 100;

    GPIO_TypeDef *in1_port, *in2_port;
    uint16_t in1_pin, in2_pin;
    uint32_t pwm_ch;

    if (motor == MOTOR_LEFT) {
        in1_port = LEFT_IN1_PORT;
        in1_pin  = LEFT_IN1_PIN;
        in2_port = LEFT_IN2_PORT;
        in2_pin  = LEFT_IN2_PIN;
        pwm_ch   = LEFT_PWM_CH;
    } else {
        in1_port = RIGHT_IN1_PORT;
        in1_pin  = RIGHT_IN1_PIN;
        in2_port = RIGHT_IN2_PORT;
        in2_pin  = RIGHT_IN2_PIN;
        pwm_ch   = RIGHT_PWM_CH;
    }

   
    uint32_t arr_plus_one = (uint32_t)htim3.Init.Period + 1;  // 1000
    uint16_t pwm_val = (uint16_t)(((uint32_t)speed * arr_plus_one) / 100);

    if (pwm_val > htim3.Init.Period) {
        pwm_val = htim3.Init.Period;
    }

    switch (dir) {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, pwm_ch, pwm_val);
            break;
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, pwm_ch, pwm_val);
            break;
        default:
            HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, pwm_ch, 0);
            break;
    }
}

void Motor_Left_Fwd(uint8_t speed)    { Motor_SetSpeed(MOTOR_LEFT, MOTOR_FORWARD, speed); }
void Motor_Left_Back(uint8_t speed)   { Motor_SetSpeed(MOTOR_LEFT, MOTOR_BACKWARD, speed); }
void Motor_Right_Fwd(uint8_t speed)   { Motor_SetSpeed(MOTOR_RIGHT, MOTOR_FORWARD, speed); }
void Motor_Right_Back(uint8_t speed)  { Motor_SetSpeed(MOTOR_RIGHT, MOTOR_BACKWARD, speed); }
void Motor_Left_Stop(void)            { Motor_SetSpeed(MOTOR_LEFT, MOTOR_STOP, 0); }
void Motor_Right_Stop(void)           { Motor_SetSpeed(MOTOR_RIGHT, MOTOR_STOP, 0); }
void Motor_All_Stop(void)             { Motor_Left_Stop(); Motor_Right_Stop(); }
