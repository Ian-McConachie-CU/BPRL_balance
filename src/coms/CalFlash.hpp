#pragma once
#include <stdint.h>

/*
 * Persistent IMU calibration stored in STM32H743 Bank 2 sector 7
 * (0x081E0000, 128 KB), programmed directly via Bank 2 registers
 * because HAL_USE_EFL = FALSE.
 *
 * Layout: 128 bytes = 4 × 32-byte flash words.
 * Biases are in body frame (post-rotation-correction), subtracted from
 * IMU readings before the EKF predict step.
 */

struct CalibData {
    uint32_t magic;             // 0x42525943 ('BPRC')
    uint32_t version;           // 1
    float    gyro_bias[3][3];   // [imu][xyz]  rad/s
    float    accel_bias[3][3];  // [imu][xyz]  m/s²
    uint32_t crc32;             // CRC-32/ISO-HDLC over bytes [0 .. 80)
    uint8_t  _pad[44];          // pad to 128 bytes
};
static_assert(sizeof(CalibData) == 128, "CalibData must be 128 bytes");

// Read calibration from flash. Returns false and leaves |out| zeroed if
// no valid calibration is stored (bad magic or CRC mismatch).
bool cal_load(CalibData &out);

// Erase sector 7 then write |d| to flash.  Returns false on write error.
bool cal_save(const CalibData &d);

// Erase sector 7 (clears calibration).
void cal_clear();
