#pragma once

#include <string>
#include <stdint.h>
#include <mutex>

/**
 * spidevのラッパークラス
 */
class Spi {
public:
    Spi(void) {}

    ~Spi() {
        Close();
    }

    bool Open(const std::string &device_name, int frequency);

    void Close();

    bool IsOpened(void) const {
        return (_Fd != -1);
    }

    bool SetMode(int mode);

    bool ReadWrite(const void *write_data, void *read_data, size_t length);

    bool Write(const void *write_data, size_t length) {
        return ReadWrite(write_data, nullptr, length);
    }

    bool ReadAfterWrite(const void *write_data, size_t write_length, void *read_data, size_t read_length);

private:
    Spi(const Spi &);

    std::mutex _Mutex;
    int _Fd = -1;
    int _Frequency = 0;
};
