#pragma once

#include <stdint.h>
#include <memory>
#include "i2c.hpp"

class ADS1015
{
public:
    enum MUX_t
    {
        MUX_AIN1_to_AIN0 = 0x0,
        MUX_AIN3_to_AIN0 = 0x1,
        MUX_AIN3_to_AIN1 = 0x2,
        MUX_AIN3_to_AIN2 = 0x3,
        MUX_GND_to_AIN0 = 0x4,
        MUX_GND_to_AIN1 = 0x5,
        MUX_GND_to_AIN2 = 0x6,
        MUX_GND_to_AIN3 = 0x7
    };

    enum FSR_t
    {
        FSR_6144mV = 0x0,
        FSR_4096mV = 0x1,
        FSR_2048mV = 0x2,
        FSR_1024mV = 0x3,
        FSR_512mV = 0x4,
        FSR_256mV = 0x5
    };

    enum DR_t
    {
        DR_128SPS = 0x0,
        DR_250SPS = 0x1,
        DR_490SPS = 0x2,
        DR_920SPS = 0x3,
        DR_1600SPS = 0x4,
        DR_2400SPS = 0x5,
        DR_3300SPS = 0x6
    };

    ADS1015(std::shared_ptr<I2c> &i2c, uint8_t dev_addr)
        : _I2C(i2c), _Address(dev_addr) {}

    bool Initialize(void);
    bool StartConversion(MUX_t mux, FSR_t fsr, DR_t dr);
    bool IsConversionCompleted(bool *complete);
    bool GetConversionResult(int16_t *result);

private:
    enum REG_t
    {
        REG_CONVERSION_DATA = 0x00,
        REG_CONFIGURATION = 0x01,
    };

    ADS1015(const ADS1015 &);
    
    bool ReadRegister(REG_t reg, uint16_t *data);
    bool WriteRegister(REG_t reg, uint16_t data);

    std::shared_ptr<I2c> _I2C;
    uint8_t _Address = 0;
};
