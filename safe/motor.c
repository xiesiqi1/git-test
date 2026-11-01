#include "motor.h"
#include "tim.h"

//电机引脚配置，修改后记得改cubemax里面的配置
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

/* ============================================================
 *                 电 机 转 向 修 正 (救急用)
 * 实测某个轮子"前进"时实际在倒转 -> 不要改接线, 把对应数字改成 1 即可。
 * 判据: 长按 PA15 跑自检, 看轮子是不是朝车头方向转。
 * ============================================================ */
#define MOTOR_LEFT_REVERSE   0
#define MOTOR_RIGHT_REVERSE  0

volatile uint8_t g_motor_busy = 0;

//初始化电机驱动，停止状态
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

//电机速度，占空比为0-100%
void Motor_SetSpeed(MotorID motor, MotorDir dir, uint8_t speed)
{
    if (speed > 100) speed = 100;

    GPIO_TypeDef *in1_port, *in2_port;
    uint16_t in1_pin, in2_pin;
    uint32_t pwm_ch;
    uint8_t  reverse;

    //左右轮控制
    if (motor == MOTOR_LEFT) {
        in1_port = LEFT_IN1_PORT;
        in1_pin  = LEFT_IN1_PIN;
        in2_port = LEFT_IN2_PORT;
        in2_pin  = LEFT_IN2_PIN;
        pwm_ch   = LEFT_PWM_CH;
#if MOTOR_LEFT_REVERSE
        reverse = 1;
#else
        reverse = 0;
#endif
    } else {
        in1_port = RIGHT_IN1_PORT;
        in1_pin  = RIGHT_IN1_PIN;
        in2_port = RIGHT_IN2_PORT;
        in2_pin  = RIGHT_IN2_PIN;
        pwm_ch   = RIGHT_PWM_CH;
#if MOTOR_RIGHT_REVERSE
        reverse = 1;
#else
        reverse = 0;
#endif
    }

    /* 转向修正: 只翻转前进/后退, 不动停止 */
    if (reverse)
    {
        if (dir == MOTOR_FORWARD)       dir = MOTOR_BACKWARD;
        else if (dir == MOTOR_BACKWARD) dir = MOTOR_FORWARD;
    }

    
    //速度映射
    uint32_t arr_plus_one = (uint32_t)htim3.Init.Period + 1;  // 1000
    uint16_t pwm_val = (uint16_t)(((uint32_t)speed * arr_plus_one) / 100);

    if (pwm_val > htim3.Init.Period) {
        pwm_val = htim3.Init.Period;
    }
    
    //转动方向
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
void Motor_Right_Fwd(uint8_t speed)   { Motor_SetSpeed(MOTOR_RIGHT, MOTOR_FORWARD, speed); }
void Motor_Left_Stop(void)            { Motor_SetSpeed(MOTOR_LEFT, MOTOR_STOP, 0); }
void Motor_Right_Stop(void)           { Motor_SetSpeed(MOTOR_RIGHT, MOTOR_STOP, 0); }
void Motor_All_Stop(void)             { Motor_Left_Stop(); Motor_Right_Stop(); }

/* 硬件自检: 阻塞式, 期间 g_motor_busy=1 让 CtrlLoop 不要抢电机。
 * 用 25% 速度足够看清转向, 又不会让车窜出去(请把轮子架空再做)。 */
void Motor_SelfTest(void)
{
    g_motor_busy = 1;

    Motor_All_Stop();
    HAL_Delay(400);

    Motor_Left_Fwd(25);     /* ← 现在应该只有"左"边那个轮子在往前转 */
    HAL_Delay(1000);
    Motor_Left_Stop();
    HAL_Delay(400);

    Motor_Right_Fwd(25);    /* ← 现在应该只有"右"边那个轮子往前转 */
    HAL_Delay(1000);
    Motor_Right_Stop();

    Motor_All_Stop();
    g_motor_busy = 0;
}

/* 确认反馈: 两轮一起短促抖两下, 约 0.5 秒。
 * 幅度小、时间短, 即使车在地上也只会轻微挪一下。 */
void Motor_Ack(void)
{
    g_motor_busy = 1;

    Motor_All_Stop();
    HAL_Delay(80);

    Motor_Left_Fwd(20);  Motor_Right_Fwd(20);
    HAL_Delay(120);
    Motor_All_Stop();
    HAL_Delay(120);

    Motor_Left_Fwd(20);  Motor_Right_Fwd(20);
    HAL_Delay(120);
    Motor_All_Stop();

    g_motor_busy = 0;
}

/* 切换转向极性的确认反馈: 左轮抖一下 -> 右轮抖一下, 约 0.7 秒。
 * 与 Motor_Ack 的"两轮同时抖两下"明显不同, 所以光看动作/听声音就知道
 * 刚才切的是传感器极性还是转向极性。 */
void Motor_AckTurn(void)
{
    g_motor_busy = 1;

    Motor_All_Stop();
    HAL_Delay(100);

    Motor_Left_Fwd(20);
    HAL_Delay(200);
    Motor_All_Stop();
    HAL_Delay(150);

    Motor_Right_Fwd(20);
    HAL_Delay(200);
    Motor_All_Stop();

    g_motor_busy = 0;
}
