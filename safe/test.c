#include "test.h"
#include "oled.h"
#include "i2c.h"
#include "key.h"
#include "time.h"
#include "motor.h"
#include "five_line_tracker.h"
#include "follow.h"
#include "usart1.h"

#include <stdio.h>
#include <stdint.h>

static enum {
    IDLE,
    RUNNING,
    DONE
} state = IDLE;

static FiveLineTracker tracker;
static FollowConfig cfg;
static Tracker_Status st;

void Test_Init(void)
{
    OLED_Init();
    OLED_CLS();

    OLED_ShowString(1, 1, "Press KEY1 to");
    OLED_ShowString(2, 1, "start!");

    OLED_Refresh_Gram();

    tracker.hi2c = &hi2c2;
    tracker.addr = 0x50;

    st = FiveLineTracker_Init(&tracker);

    Motor_Init();

    /*
     * 循迹参数
     */
    cfg.speed             = 60U;
    cfg.turn_speed        = 45U;

    cfg.Kp                = 1.05f;
    cfg.Kd                = 0.09f;
    cfg.D_alpha           = 0.80f;

    cfg.curve_threshold   = 7U;
    cfg.min_correction    = 6U;

    Follow_Init(&tracker, &cfg);

    Key_Init();
    Timer_Init();

    state = IDLE;
}

void Test_Loop(void)
{
    static uint32_t follow_tick = 0U;
    static uint32_t oled_tick = 0U;

    KeyEvent key;
    uint32_t now;

    key = Key_Scan();
    now = HAL_GetTick();

    if (key == KEY_PRESS_1) {
        if (state == IDLE) {
            Timer_Reset();
            Timer_Start();

            state = RUNNING;

            follow_tick = now;
            oled_tick = now;

            OLED_CLS();
            OLED_Refresh_Gram();
        } else if (state == RUNNING) {
            Timer_Stop();
            Motor_All_Stop();

            state = DONE;
        }
    }

    if (state == RUNNING) {
        /*
         * 每 10 ms 更新一次循迹控制。
         */
        if ((now - follow_tick) >= 10U) {
            follow_tick = now;

            if (st == TRACKER_OK) {
                Follow_Update(
                    &tracker,
                    &cfg
                );
            }
        }

        /*
         * 每 100 ms 更新一次 OLED。
         */
        if ((now - oled_tick) >= 100U) {
            char time_buf[16];
            uint32_t elapsed_ms;

            oled_tick = now;

            elapsed_ms = Timer_GetElapsed();

            Timer_GetString(
                time_buf,
                elapsed_ms
            );

            OLED_CLS();

            OLED_ShowString(1, 1, "T:");
            OLED_ShowString(2, 1, time_buf);

            OLED_Refresh_Gram();
        }
    }

    if (state == DONE) {
        char time_buf[16];
        uint32_t elapsed_ms;

        elapsed_ms = Timer_GetElapsed();

        Timer_GetString(
            time_buf,
            elapsed_ms
        );

        OLED_CLS();

        OLED_ShowString(1, 1, "Done!");
        OLED_ShowString(2, 1, "T:");
        OLED_ShowString(2, 3, time_buf);

        OLED_Refresh_Gram();

        state = IDLE;
    }

    HAL_Delay(1);
}
