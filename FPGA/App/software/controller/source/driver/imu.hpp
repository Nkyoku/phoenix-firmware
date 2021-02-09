#pragma once

#include <stdint.h>
#include <system.h>

struct ImuResult_t {
	int16_t TempData;
	int16_t AccelDataX;
	int16_t AccelDataY;
	int16_t AccelDataZ;
	int16_t GyroDataX;
	int16_t GyroDataY;
	int16_t GyroDataZ;
};

class Imu {
private:
	static constexpr uint32_t SPI_BASE = SPIM_0_BASE;
	static constexpr uint32_t SPI_SLAVE = 0;
    static constexpr uint32_t SPIM_BASE = IMU_SPIM_BASE;

public:
    // IMUを初期化する
	static bool Initialize(void);
    
    // 測定データを読み出す
	static void ReadData(ImuResult_t *data);
    
    // 測定データが有効な値か取得する
    static bool IsValid(void){
        return _Valid;
    }
    
private:
	static void SetBank(uint32_t bank);
	static uint8_t ReadRegister(uint32_t address);
	static void WriteRegister(uint32_t address, uint8_t value);
	static void ReadRegisters(uint32_t address, uint32_t length, void *data);

	static uint8_t _Bank;
    static bool _Valid;
};
