/**
 * @file imu_spim.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <system.h>

struct ImuSpiMasterRegisters {
    volatile uint16_t CONTROL;
    volatile int16_t TEMP_DATA;
    volatile int16_t ACCEL_DATA_X;
    volatile int16_t ACCEL_DATA_Y;
    volatile int16_t ACCEL_DATA_Z;
    volatile int16_t GYRO_DATA_X;
    volatile int16_t GYRO_DATA_Y;
    volatile int16_t GYRO_DATA_Z;
};

static inline void IMU_SPIM_SetPassthrough(uint32_t base, bool enabled) {
    __builtin_sthio(&((ImuSpiMasterRegisters*)base)->CONTROL, enabled ? 0x0001 : 0x0000);
}

static inline int IMU_SPIM_GetTempData(uint32_t base) {
    return __builtin_ldhio(&((ImuSpiMasterRegisters*)base)->TEMP_DATA);
}

static inline int IMU_SPIM_GetAccelDataX(uint32_t base) {
    return __builtin_ldhio(&((ImuSpiMasterRegisters*)base)->ACCEL_DATA_X);
}

static inline int IMU_SPIM_GetAccelDataY(uint32_t base) {
    return __builtin_ldhio(&((ImuSpiMasterRegisters*)base)->ACCEL_DATA_Y);
}

static inline int IMU_SPIM_GetAccelDataZ(uint32_t base) {
    return __builtin_ldhio(&((ImuSpiMasterRegisters*)base)->ACCEL_DATA_Z);
}

static inline int IMU_SPIM_GetGyroDataX(uint32_t base) {
    return __builtin_ldhio(&((ImuSpiMasterRegisters*)base)->GYRO_DATA_X);
}

static inline int IMU_SPIM_GetGyroDataY(uint32_t base) {
    return __builtin_ldhio(&((ImuSpiMasterRegisters*)base)->GYRO_DATA_Y);
}

static inline int IMU_SPIM_GetGyroDataZ(uint32_t base) {
    return __builtin_ldhio(&((ImuSpiMasterRegisters*)base)->GYRO_DATA_Z);
}
