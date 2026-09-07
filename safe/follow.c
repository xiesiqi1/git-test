#include "follow.h"
#include "five_line_tracker.h"
#include "motor.h"

#include <stdint.h>

#define SENSOR_COUNT                 5U

/* 黑线检测参数 */
#define OUTER_LINE_THRESHOLD         0.30f
#define MIDDLE_SIGNAL_MIN            0.15f
#define MIDDLE_POSITION              8.0f

/* 最外侧连续检测两次后正式转弯 */
#define TURN_CONFIRM_COUNT           2U

/* 中间三路比例控制参数 */
#define STRAIGHT_DEAD_ZONE           0.50f
#define STRAIGHT_MAX_CORRECTION      12.0f

/* 第一次检测到最外侧时的预转弯差速 */
#define PRE_TURN_CORRECTION          10

/* 正式转弯时外侧轮 PWM */
#define HARD_TURN_OUTER_PWM          70

/* 电机正常运行的最低 PWM */
#define MOTOR_MIN_PWM                12

/*
 * 白底和黑线标定值。
 *
 * 代码假设 ADC 越大，黑线信号越强。
 */
static const uint16_t white_ref[SENSOR_COUNT] = {
    720U, 820U, 780U, 740U, 750U
};

static const uint16_t black_ref[SENSOR_COUNT] = {
    985U, 971U, 971U, 970U, 999U
};

/*
 * 传感器排列：
 *
 * 左侧                                           右侧
 * [4]          [3]          [2]          [1]          [0]
 * 最左         中左         中间         中右         最右
 */

/*
 * 最后检测到黑线的方向：
 * -1：左侧
 *  1：右侧
 */
static int8_t last_direction = 1;

/*
 * 等待第二次确认的方向：
 * -1：等待确认左转
 *  0：没有候选方向
 *  1：等待确认右转
 */
static int8_t candidate_side = 0;

/* 同一侧最外传感器连续检测次数 */
static uint8_t outer_detect_count = 0U;

static float abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float clamp_float(float value,
                         float min_value,
                         float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static int16_t clamp_speed(int16_t speed)
{
    if (speed < 0) {
        return 0;
    }

    if (speed > 100) {
        return 100;
    }

    return speed;
}

static void set_motors(int16_t left_speed,
                       int16_t right_speed)
{
    left_speed = clamp_speed(left_speed);
    right_speed = clamp_speed(right_speed);

    Motor_Left_Fwd((uint8_t)left_speed);
    Motor_Right_Fwd((uint8_t)right_speed);
}

/*
 * 将 ADC 值转换为 0.0~1.0 的黑线强度。
 *
 * 0.0：接近白底
 * 1.0：接近黑线
 */
static float black_strength(uint8_t channel,
                            uint16_t analog_value)
{
    float white_value;
    float black_value;
    float strength;

    white_value = (float)white_ref[channel];
    black_value = (float)black_ref[channel];

    if (black_value <= white_value) {
        return 0.0f;
    }

    if ((float)analog_value <= white_value) {
        return 0.0f;
    }

    if ((float)analog_value >= black_value) {
        return 1.0f;
    }

    strength =
        ((float)analog_value - white_value) /
        (black_value - white_value);

    return clamp_float(strength, 0.0f, 1.0f);
}

/*
 * 第一次检测到最外侧黑线时执行预转弯。
 *
 * side < 0：预左转
 * side > 0：预右转
 */
static void pre_turn(FollowConfig *config,
                     int8_t side)
{
    int16_t base_speed;
    int16_t correction;

    base_speed = (int16_t)config->speed;

    if (base_speed < MOTOR_MIN_PWM) {
        base_speed = MOTOR_MIN_PWM;
    }

    correction = PRE_TURN_CORRECTION;

    /*
     * 如果配置的最小纠偏量更大，就采用配置值。
     */
    if ((int16_t)config->min_correction > correction) {
        correction = (int16_t)config->min_correction;
    }

    if (side < 0) {
        /*
         * 预左转：
         * 左轮减速，右轮加速。
         */
        set_motors(base_speed - correction,
                   base_speed + correction);
    } else {
        /*
         * 预右转：
         * 左轮加速，右轮减速。
         */
        set_motors(base_speed + correction,
                   base_speed - correction);
    }

    last_direction = side;
}

/*
 * 同一最外侧连续检测两次后执行大力度转弯。
 *
 * 正式转弯时，内侧轮停止，只有外侧轮前进。
 */
static void hard_turn(int8_t side)
{
    if (side < 0) {
        /*
         * 强左转：
         * 左轮停止，只有右轮前进。
         */
        set_motors(0, HARD_TURN_OUTER_PWM);
        last_direction = -1;
    } else {
        /*
         * 强右转：
         * 右轮停止，只有左轮前进。
         */
        set_motors(HARD_TURN_OUTER_PWM, 0);
        last_direction = 1;
    }
}

/*
 * 使用中间三路进行比例循迹。
 *
 * strength[3]：中左
 * strength[2]：中间
 * strength[1]：中右
 *
 * 返回 1：中间三路检测到黑线，已经设置电机。
 * 返回 0：中间三路没有可靠检测到黑线。
 */
static uint8_t middle_control(
    FollowConfig *config,
    const float strength[SENSOR_COUNT])
{
    float total_signal;
    float error;
    float correction;
    float min_correction;
    int16_t base_speed;
    int16_t left_speed;
    int16_t right_speed;

    total_signal =
        strength[1] +
        strength[2] +
        strength[3];

    if (total_signal < MIDDLE_SIGNAL_MIN) {
        return 0U;
    }

    /*
     * 中右信号较强：error > 0
     * 中间信号较强：error 接近 0
     * 中左信号较强：error < 0
     */
    error =
        (strength[1] * MIDDLE_POSITION -
         strength[3] * MIDDLE_POSITION) /
        total_signal;

    /*
     * 接近中央时不纠偏，减少左右抖动。
     */
    if (abs_float(error) <= STRAIGHT_DEAD_ZONE) {
        error = 0.0f;
    }

    correction = config->Kp * error;
    min_correction = (float)config->min_correction;

    /*
     * 有偏差时至少给出 min_correction 的纠偏量。
     */
    if ((error > 0.0f) &&
        (correction < min_correction)) {
        correction = min_correction;
    } else if ((error < 0.0f) &&
               (correction > -min_correction)) {
        correction = -min_correction;
    }

    correction = clamp_float(
        correction,
        -STRAIGHT_MAX_CORRECTION,
        STRAIGHT_MAX_CORRECTION
    );

    base_speed = (int16_t)config->speed;

    if (base_speed < MOTOR_MIN_PWM) {
        base_speed = MOTOR_MIN_PWM;
    }

    /*
     * 黑线偏右：
     * 左轮加速，右轮减速，向右纠偏。
     *
     * 黑线偏左：
     * 左轮减速，右轮加速，向左纠偏。
     */
    left_speed =
        (int16_t)((float)base_speed + correction);

    right_speed =
        (int16_t)((float)base_speed - correction);

    set_motors(left_speed, right_speed);

    if (error >= 1.0f) {
        last_direction = 1;
    } else if (error <= -1.0f) {
        last_direction = -1;
    }

    return 1U;
}

/*
 * 完全丢线时，按最后检测到黑线的方向单轮找线。
 */
static void search_line(void)
{
    hard_turn(last_direction);
}

void Follow_Init(FiveLineTracker *tracker,
                 FollowConfig *config)
{
    (void)tracker;
    (void)config;

    Motor_Init();

    last_direction = 1;
    candidate_side = 0;
    outer_detect_count = 0U;
}

void Follow_Update(FiveLineTracker *tracker,
                   FollowConfig *config)
{
    uint16_t analog[SENSOR_COUNT];
    float strength[SENSOR_COUNT];
    uint8_t left_detected;
    uint8_t right_detected;
    int8_t detected_side;
    uint8_t i;

    if ((tracker == 0) || (config == 0)) {
        Motor_All_Stop();
        return;
    }

    FiveLineTracker_AnalogValues(tracker, analog);

    for (i = 0U; i < SENSOR_COUNT; i++) {
        strength[i] =
            black_strength(i, analog[i]);
    }

    /*
     * 通道4是最左侧，通道0是最右侧。
     */
    left_detected =
        (strength[4] >= OUTER_LINE_THRESHOLD) ?
        1U : 0U;

    right_detected =
        (strength[0] >= OUTER_LINE_THRESHOLD) ?
        1U : 0U;

    detected_side = 0;

    /*
     * 只有一侧最外传感器检测到黑线时，
     * 才将其作为转弯方向。
     */
    if (left_detected && !right_detected) {
        detected_side = -1;
    } else if (right_detected && !left_detected) {
        detected_side = 1;
    }

    if (detected_side != 0) {
        if (detected_side != candidate_side) {
            /*
             * 第一次检测到该侧：
             * 从 1 开始计数，并执行预转弯。
             *
             * 如果上一周期检测的是另一侧，
             * 当前侧也重新作为第一次检测。
             */
            candidate_side = detected_side;
            outer_detect_count = 1U;

            pre_turn(config, detected_side);
            return;
        }

        /*
         * 当前周期仍然检测到同一侧。
         */
        if (outer_detect_count < TURN_CONFIRM_COUNT) {
            outer_detect_count++;
        }

        if (outer_detect_count >= TURN_CONFIRM_COUNT) {
            /*
             * 同一最外侧连续检测两次：
             * 停止内侧轮，只有外侧轮前进。
             */
            hard_turn(candidate_side);
            return;
        }
    }

    /*
     * 没有单侧最外信号，或者左右最外侧同时触发：
     * 取消连续确认。
     */
    candidate_side = 0;
    outer_detect_count = 0U;

    /*
     * 中间三路正常比例循迹。
     */
    if (middle_control(config, strength)) {
        return;
    }

    /*
     * 五路都没有可靠检测到黑线，
     * 按最后方向单轮找线。
     */
    search_line();
}
