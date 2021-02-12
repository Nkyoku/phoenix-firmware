#pragma once

#include <stdint.h>
#include <mutex>
#include <string>

class I2c {
public:
    I2c() {}

    ~I2c() {
        Close();
    }

    bool Open(const std::string &device_name);

    void Close();

    bool IsOpened(void) const {
        return (_Fd != -1);
    }

    bool Read(uint8_t dev_addr, uint8_t reg_value, void *data_ptr, uint16_t data_length) {
        return Read(dev_addr, &reg_value, 1, data_ptr, data_length);
    }

    bool Read(uint8_t dev_addr, const void *reg_ptr, uint16_t reg_length, void *data_ptr, uint16_t data_length);

    bool Write(uint8_t dev_addr, uint8_t reg_value, const uint8_t *data_ptr, uint16_t data_length) {
        return Write(dev_addr, &reg_value, 1, data_ptr, data_length);
    }

    bool Write(uint8_t dev_addr, const void *reg_ptr, uint16_t reg_length, const void *data_ptr, uint16_t data_length);

private:
    I2c(const I2c &);

    std::mutex _Mutex;
    int _Fd = -1;
};
