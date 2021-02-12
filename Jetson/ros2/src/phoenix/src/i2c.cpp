#include "i2c.hpp"
#include <vector>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

bool I2c::Open(const std::string &device_name) {
    std::lock_guard<std::mutex> lock(_Mutex);
    if (_Fd != -1) {
        close(_Fd);
    }
    _Fd = open(device_name.c_str(), O_RDWR);
    return (_Fd != -1);
}

void I2c::Close(void) {
    std::lock_guard<std::mutex> lock(_Mutex);
    if (_Fd != -1) {
        close(_Fd);
        _Fd = -1;
    }
}

bool I2c::Read(uint8_t dev_addr, const void *reg_ptr, uint16_t reg_length, void *data_ptr, uint16_t data_length) {
    std::lock_guard<std::mutex> lock(_Mutex);
    if (IsOpened() == false) {
        return false;
    }
    struct i2c_msg messages[] = {
        {dev_addr, 0, reg_length, const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(reg_ptr))},
        {dev_addr, I2C_M_RD, data_length, reinterpret_cast<uint8_t *>(data_ptr)},
    };
    struct i2c_rdwr_ioctl_data ioctl_data = {messages, 2};
    if (ioctl(_Fd, I2C_RDWR, &ioctl_data) != 2) {
        fprintf(stderr, "I2c::Read failed to ioctl: %s\n", strerror(errno));
        return false;
    }
    return true;
}

bool I2c::Write(uint8_t dev_addr, const void *reg_ptr, uint16_t reg_length, const void *data_ptr, uint16_t data_length) {
    std::lock_guard<std::mutex> lock(_Mutex);
    if (UINT16_MAX < (reg_length + data_length)) {
        return false;
    }
    std::vector<uint8_t> buffer(reg_length + data_length);
    memcpy(buffer.data(), reg_ptr, reg_length);
    memcpy(buffer.data() + reg_length, data_ptr, data_length);
    struct i2c_msg message = {dev_addr, 0, static_cast<uint16_t>(buffer.size()), buffer.data()};
    struct i2c_rdwr_ioctl_data ioctl_data = {&message, 1};
    if (ioctl(_Fd, I2C_RDWR, &ioctl_data) != 1) {
        fprintf(stderr, "I2c::Write failed to ioctl: %s\n", strerror(errno));
        return false;
    }
    return true;
}
