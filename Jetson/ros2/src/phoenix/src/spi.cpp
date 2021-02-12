#include "spi.hpp"
#include <vector>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <limits>

bool Spi::Open(const std::string &device_name, int frequency) {
    if (frequency < 0) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_Mutex);
    if (_Fd != -1) {
        close(_Fd);
    }
    _Fd = open(device_name.c_str(), O_RDWR);
    if (_Fd == -1) {
        return false;
    }
    uint8_t lsb_first_u8 = 0, bits_per_word_u8 = 8;
    uint32_t frequency_u32 = frequency;
    ioctl(_Fd, SPI_IOC_WR_LSB_FIRST, &lsb_first_u8);
    ioctl(_Fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_u8);
    ioctl(_Fd, SPI_IOC_WR_MAX_SPEED_HZ, &frequency_u32);
    _Frequency = frequency;
    return (_Fd != -1);
}

void Spi::Close(void) {
    std::lock_guard<std::mutex> lock(_Mutex);
    if (_Fd != -1) {
        close(_Fd);
        _Fd = -1;
    }
}

bool Spi::SetMode(int mode) {
    static const int mode_table[4] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3};
    if ((mode < 0) || (3 < mode)) {
        return false;
    }
    uint8_t mode_u8 = mode_table[mode];
    return ioctl(_Fd, SPI_IOC_WR_MODE, &mode_u8) == 0;
}

bool Spi::ReadWrite(const void *write_data, void *read_data, size_t length) {
    if (std::numeric_limits<uint32_t>::max() < length) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_Mutex);
    spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.tx_buf = reinterpret_cast<uint64_t>(write_data);
    transfer.rx_buf = reinterpret_cast<uint64_t>(read_data);
    transfer.len = length;
    if (ioctl(_Fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        fprintf(stderr, "Spi::ReadWrite failed to ioctl: %s\n", strerror(errno));
        return false;
    }
    return true;
}

bool Spi::ReadAfterWrite(const void *write_data, size_t write_length, void *read_data, size_t read_length) {
    if ((std::numeric_limits<uint32_t>::max() < write_length) || (std::numeric_limits<uint32_t>::max() < read_length)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_Mutex);
    spi_ioc_transfer transfer[2];
    memset(&transfer, 0, sizeof(transfer));
    transfer[0].tx_buf = reinterpret_cast<uint64_t>(write_data);
    transfer[0].len = write_length;
    transfer[1].rx_buf = reinterpret_cast<uint64_t>(read_data);
    transfer[1].len = read_length;
    if (ioctl(_Fd, SPI_IOC_MESSAGE(2), transfer) < 0) {
        fprintf(stderr, "Spi::ReadAfterWrite failed to ioctl: %s\n", strerror(errno));
        return false;
    }
    return true;
}
