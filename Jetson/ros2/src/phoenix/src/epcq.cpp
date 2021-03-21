#include "epcq.hpp"
#include <string.h>
#include <chrono>

bool Epcq::ReadSilliconId(uint8_t *sillicon_id){
    uint8_t write_data[5] = {0xAB, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t read_data[5];
    if (_Spi->ReadWrite(write_data, read_data, sizeof(write_data))) {
        *sillicon_id = read_data[4];
        return true;
    }
    return false;
}

bool Epcq::ReadStatus(uint8_t *status) {
    uint8_t write_data[2] = {0x05, 0xFF};
    uint8_t read_data[2];
    if (_Spi->ReadWrite(write_data, read_data, sizeof(write_data))) {
        *status = read_data[1];
        return true;
    }
    return false;
}
#include <vector>
bool Epcq::Read(uint32_t address, void *data, size_t length) {
    /*uint8_t write_data[4] = {0x03};
    write_data[1] = (address >> 16) & 0xFF;
    write_data[2] = (address >> 8) & 0xFF;
    write_data[3] = address & 0xFF;
    if (_Spi->ReadAfterWrite(write_data, sizeof(write_data), data, length)) {
        return true;
    }*/
    std::vector<uint8_t> write_data(4 + length);
    std::vector<uint8_t> read_data(4 + length);
    write_data[0] = 0x03;
    write_data[1] = (address >> 16) & 0xFF;
    write_data[2] = (address >> 8) & 0xFF;
    write_data[3] = address & 0xFF;
    if (_Spi->ReadWrite(write_data.data(), read_data.data(), write_data.size())) {
        memcpy(data, read_data.data() + 4, length);
        return true;
    }
    return false;
}

bool Epcq::WritePage(uint32_t address, const void *data) {
    uint8_t write_data[4 + PAGE_SIZE] = {0x02};
    write_data[1] = (address >> 16) & 0xFF;
    write_data[2] = (address >> 8) & 0xFF;
    write_data[3] = address & 0xFF;
    memcpy(write_data + 4, data, PAGE_SIZE);
    if (SetWriteEnabled(true)) {
        if (_Spi->Write(write_data, sizeof(write_data))) {
            return WaitProgressBit(100);
        }
    }
    return false;
}

bool Epcq::EraseSector(uint32_t address) {
    uint8_t write_data[4] = {0xD8};
    write_data[1] = (address >> 16) & 0xFF;
    write_data[2] = (address >> 8) & 0xFF;
    write_data[3] = address & 0xFF;
    if (SetWriteEnabled(true)) {
        if (_Spi->Write(write_data, sizeof(write_data))) {
            return WaitProgressBit(2500);
        }
    }
    return false;
}

bool Epcq::SetWriteEnabled(bool enabled) {
    uint8_t write_data[1];
    write_data[0] = enabled ? 0x06 : 0x04;
    return _Spi->Write(write_data, sizeof(write_data));
}

bool Epcq::WaitProgressBit(int timeout_in_ms){
    auto timeout = std::chrono::milliseconds(timeout_in_ms);
    auto start = std::chrono::system_clock::now();
    uint8_t status = 0xFF;
    while (ReadStatus(&status)) {
        if (~status & STATUS_WIP) {
            return true;
        }
        if (timeout < (std::chrono::system_clock::now() - start)){
            break;
        }
    }
    return false;
}
