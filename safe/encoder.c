#include "encoder.h"
#include "tim.h"
#include "config.h"

/* 速度换算: 一拍内的计数增量 -> 计数/秒。系数随控制周期自动变化 */
#define ENC_SPEED_SCALE  ((int32_t)(1000u / CTRL_PERIOD_MS))

/* 每个编码器维护: 累计位置 + 上一次原始计数值 + 最近一次的增量。
 * ★ delta 存在结构体里(而不是"读的时候顺手算"): 这样 Update 是唯一的写点,
 *   Get 全是纯读 —— 见 encoder.h 开头那段说明(踩过的坑)。 */
typedef struct
{
    TIM_HandleTypeDef *htim;
    int32_t  total;      /* 累计位置（有符号） */
    uint16_t last_raw;   /* 上次读取的原始计数器 */
    int16_t  delta;      /* 最近一次 Update 算出的增量 */
} Encoder_t;

static Encoder_t s_left  = { &htim2, 0, 0, 0 };  /* TIM2 = 左轮 */
static Encoder_t s_right = { &htim4, 0, 0, 0 };  /* TIM4 = 右轮 */

//编码器初始化
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    Encoder_Reset();
}

//清零
void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    s_left.total    = 0;
    s_left.last_raw = 0;
    s_left.delta    = 0;
    s_right.total    = 0;
    s_right.last_raw = 0;
    s_right.delta    = 0;
}

/* 读一次硬件计数器并把增量累加进累计值。
 * 用 16bit 有符号差值: now - last_raw 自动处理 65535 回绕与反转,
 * 所以【只要两次读取之间增量不超过 ±32767】就不会算错。
 * 本工程每 5ms 调一次, 车速下 5ms 的计数远远小于这个上限, 安全。 */
static void Encoder_UpdateOne(Encoder_t *enc)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);

    enc->delta    = (int16_t)((uint16_t)(now - enc->last_raw));
    enc->last_raw = now;
    enc->total   += enc->delta;
}

/* ★★ 每个控制拍必须调用一次(现在由 Mileage_Task() 的第一行负责)。
 *   整个工程的 ENC 显示、里程、A/B 点提示、停车里程门槛全都建立在它之上;
 *   漏调 = 累计计数恒为 0。详见 encoder.h 开头。 */
void Encoder_Update(void)
{
    Encoder_UpdateOne(&s_left);
    Encoder_UpdateOne(&s_right);
}

int16_t Encoder_GetLeftDelta(void)  { return s_left.delta;  }
int16_t Encoder_GetRightDelta(void) { return s_right.delta; }

int32_t Encoder_GetLeftCount(void)  { return s_left.total;  }
int32_t Encoder_GetRightCount(void) { return s_right.total; }

/* 速度：delta / CTRL_PERIOD_MS * 1000 = 计数/秒。
 * 调用周期由 config.h 的 CTRL_PERIOD_MS 决定(它同时管着 SysTick 分频),
 * 所以改控制频率时测速会跟着同步, 不需要手动改这里的数字。 */
int32_t Encoder_GetLeftSpeed(void)  { return (int32_t)s_left.delta  * ENC_SPEED_SCALE; }
int32_t Encoder_GetRightSpeed(void) { return (int32_t)s_right.delta * ENC_SPEED_SCALE; }
