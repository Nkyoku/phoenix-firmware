/**
 * @file imu.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <system.h>

struct ImuResult {
	int16_t temp_data;
	int16_t accel_data_x;
	int16_t accel_data_y;
	int16_t accel_data_z;
	int16_t gyro_data_x;
	int16_t gyro_data_y;
	int16_t gyro_data_z;
};

class Imu {
private:
	static constexpr uint32_t SPI_BASE = SPIM_0_BASE;
	static constexpr uint32_t SPI_SLAVE = 0;
    static constexpr uint32_t SPIM_BASE = IMU_SPIM_BASE;

public:
    // IMUを初期化する
	static bool initialize(void);
    
    // 測定データを読み出す
	static void readData(ImuResult *data);
    
    // 測定データが有効な値か取得する
    static bool isValid(void){
        return _valid;
    }
    
private:
	static void setBank(uint32_t bank);
	static uint8_t readRegister(uint32_t address);
	static void writeRegister(uint32_t address, uint8_t value);
	static void readRegisters(uint32_t address, uint32_t length, void *data);

	static uint8_t _bank;
    static bool _valid;
};
