#ifndef __FIVE_LINE_TRACKER_H
#define __FIVE_LINE_TRACKER_H

#include "stm32f1xx_hal.h"

#define FIVE_LINE_TRACKER_DEFAULT_ADDR  0x50

typedef enum {
    TRACKER_OK = 0,
    TRACKER_ERROR_I2C_DATA_TOO_LONG = 1,
    TRACKER_ERROR_NACK_ADDRESS = 2,
    TRACKER_ERROR_NACK_DATA = 3,
    TRACKER_ERROR_I2C_OTHER = 4,
    TRACKER_ERROR_TIMEOUT = 5,
    TRACKER_ERROR_INVALID_PARAM = 6,
} Tracker_Status;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
} FiveLineTracker;

Tracker_Status FiveLineTracker_Init(FiveLineTracker *tracker);
uint8_t FiveLineTracker_DeviceID(FiveLineTracker *tracker);
uint8_t FiveLineTracker_FirmwareVersion(FiveLineTracker *tracker);
void FiveLineTracker_SetHighThresholds(FiveLineTracker *tracker, const uint16_t *thresholds);
void FiveLineTracker_SetLowThresholds(FiveLineTracker *tracker, const uint16_t *thresholds);
void FiveLineTracker_AnalogValues(FiveLineTracker *tracker, uint16_t *analog);
uint8_t FiveLineTracker_DigitalValues(FiveLineTracker *tracker);

#endif
