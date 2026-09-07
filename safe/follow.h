#ifndef FOLLOW_H
#define FOLLOW_H

#include "five_line_tracker.h"

#include <stdint.h>

typedef struct {
    uint8_t speed;
    uint8_t turn_speed;

    float Kp;
    float Kd;
    float D_alpha;

    uint8_t curve_threshold;
    uint8_t min_correction;
} FollowConfig;

void Follow_Init(FiveLineTracker *tracker,
                 FollowConfig *config);

void Follow_Update(FiveLineTracker *tracker,
                   FollowConfig *config);

#endif
