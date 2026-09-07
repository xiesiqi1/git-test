#include "five_line_tracker.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

#define REG_DEVICE_ID       0x00
#define REG_VERSION         0x01
#define REG_ANALOG_VALUES   0x10
#define REG_DIGITAL_VALUES  0x1A
#define REG_HIGH_THRESHOLDS 0x1C
#define REG_LOW_THRESHOLDS  0x26

#define TRACKER_LINE_NUM    5

/* 对应 Arduino ErrorCode 宏 */
#define ERR_OK                   0
#define ERR_I2C_DATA_TOO_LONG    1
#define ERR_I2C_NACK_ADDRESS     2
#define ERR_I2C_NACK_DATA        3
#define ERR_I2C_OTHER            4
#define ERR_I2C_TIMEOUT          5
#define ERR_INVALID_PARAMETER    6

/*
 * 对标 Arduino Initialize():
 *   wire_.beginTransmission(i2c_address_);
 *   return static_cast<ErrorCode>(wire_.endTransmission());
 *
 * Arduino Wire 用 7 位地址，HAL 用 8 位（左移 1 位）。
 * endTransmission() 返回 0 表示成功，HAL_I2C_IsDeviceReady 返回 HAL_OK 表示成功。
 */
Tracker_Status FiveLineTracker_Init(FiveLineTracker *tracker)
{
    if (HAL_I2C_IsDeviceReady(tracker->hi2c, (uint16_t)(tracker->addr << 1), 3, 100) == HAL_OK) {
        return TRACKER_OK;
    }
    return TRACKER_ERROR_NACK_ADDRESS;
}

/*
 * 对标 Arduino DeviceId():
 *   beginTransmission → write(REG) → endTransmission
 *   requestFrom(1) → read()
 *
 * HAL 用 Mem_Read 一步完成。
 */
uint8_t FiveLineTracker_DeviceID(FiveLineTracker *tracker)
{
    uint8_t id = 0;
    HAL_I2C_Mem_Read(tracker->hi2c, (uint16_t)(tracker->addr << 1),
                     REG_DEVICE_ID, I2C_MEMADD_SIZE_8BIT,
                     &id, 1, 100);
    return id;
}

uint8_t FiveLineTracker_FirmwareVersion(FiveLineTracker *tracker)
{
    uint8_t ver = 0;
    HAL_I2C_Mem_Read(tracker->hi2c, (uint16_t)(tracker->addr << 1),
                     REG_VERSION, I2C_MEMADD_SIZE_8BIT,
                     &ver, 1, 100);
    return ver;
}

/*
 * 对标 Arduino HighThresholds():
 *   beginTransmission → write(REG) → write(data, 10) → endTransmission
 *
 * HAL 用 Mem_Write 一步完成（含寄存器地址 + 数据）。
 */
void FiveLineTracker_SetHighThresholds(FiveLineTracker *tracker, const uint16_t *thresholds)
{
    HAL_I2C_Mem_Write(tracker->hi2c, (uint16_t)(tracker->addr << 1),
                      REG_HIGH_THRESHOLDS, I2C_MEMADD_SIZE_8BIT,
                      (uint8_t *)thresholds, TRACKER_LINE_NUM * 2, 100);
}

void FiveLineTracker_SetLowThresholds(FiveLineTracker *tracker, const uint16_t *thresholds)
{
    HAL_I2C_Mem_Write(tracker->hi2c, (uint16_t)(tracker->addr << 1),
                      REG_LOW_THRESHOLDS, I2C_MEMADD_SIZE_8BIT,
                      (uint8_t *)thresholds, TRACKER_LINE_NUM * 2, 100);
}

/*
 * 对标 Arduino AnalogValues():
 *   beginTransmission → write(REG) → endTransmission
 *   requestFrom(10) → readBytes(data, 10)
 *
 * HAL 用 Mem_Read 一步完成。
 */
void FiveLineTracker_AnalogValues(FiveLineTracker *tracker,
                                  uint16_t *analog)
{
    uint8_t data[TRACKER_LINE_NUM * 2U] = {0U};
    uint8_t i;

    if ((tracker == NULL) ||
        (tracker->hi2c == NULL) ||
        (analog == NULL)) {
        return;
    }

    if (HAL_I2C_Mem_Read(
            tracker->hi2c,
            (uint16_t)(tracker->addr << 1),
            REG_ANALOG_VALUES,
            I2C_MEMADD_SIZE_8BIT,
            data,
            sizeof(data),
            100U) != HAL_OK) {

        /*
         * 读取失败时不能使用未初始化数据。
         */
        for (i = 0U; i < TRACKER_LINE_NUM; i++) {
            analog[i] = 0U;
        }

        return;
    }

    for (i = 0U; i < TRACKER_LINE_NUM; i++) {
        /*
         * 设备发送顺序：
         * data[0] = 低字节
         * data[1] = 高字节
         */
        analog[i] =
            (uint16_t)data[2U * i]
            | ((uint16_t)data[2U * i + 1U] << 8);
    }
}

/*
 * 对标 Arduino DigitalValues():
 *   beginTransmission → write(REG) → endTransmission
 *   requestFrom(1) → read()
 */
uint8_t FiveLineTracker_DigitalValues(FiveLineTracker *tracker)
{
    uint8_t digital = 0;
    HAL_I2C_Mem_Read(tracker->hi2c, (uint16_t)(tracker->addr << 1),
                     REG_DIGITAL_VALUES, I2C_MEMADD_SIZE_8BIT,
                     &digital, 1, 100);
    return digital;
}
