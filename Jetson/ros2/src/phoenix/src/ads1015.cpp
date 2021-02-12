#include "ads1015.hpp"

bool ADS1015::Initialize()
{
    uint16_t data;
    return ReadRegister(REG_CONFIGURATION, &data);
}

bool ADS1015::StartConversion(MUX_t mux, FSR_t fsr, DR_t dr)
{
    uint16_t config = 0x8103 | (mux << 12) | (fsr << 9) | (dr << 5);
    return WriteRegister(REG_CONFIGURATION, config);
}

bool ADS1015::IsConversionCompleted(bool *complete)
{
    *complete = false;
    uint16_t data;
    if (ReadRegister(REG_CONFIGURATION, &data) == false)
    {
        return false;
    }
    if (data & 0x8000)
    {
        *complete = true;
    }
    return true;
}

bool ADS1015::GetConversionResult(int16_t *result)
{
    *result = 0;
    uint16_t data;
    if (ReadRegister(REG_CONVERSION_DATA, &data) == false)
    {
        return false;
    }
    *result = data;
    return true;
}

bool ADS1015::ReadRegister(REG_t reg, uint16_t *data)
{
    uint8_t buf[2];
    if (_I2C->Read(_Address, (uint8_t)reg, buf, sizeof(buf)) == false)
    {
        return false;
    }
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool ADS1015::WriteRegister(REG_t reg, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data;
    return _I2C->Write(_Address, (uint8_t)reg, buf, sizeof(buf));
}
