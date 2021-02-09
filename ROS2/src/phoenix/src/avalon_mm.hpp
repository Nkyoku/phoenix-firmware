#pragma once

#include "spi.hpp"
#include <memory>
#include <vector>

/**
 * SPI Slave to Avalon Master Bridgeを使用してメモリー操作する
 */
class AvalonMm {
public:
    /**
     * コンストラクタ
     * @param spi 通信に使用するSpi
     */
    AvalonMm(std::shared_ptr<Spi> &spi) : _Spi(spi) {}

    /**
     * メモリーにデータを書き込む
     * @param address 書き込むアドレス
     * @param data 書き込むデータへのポインタ
     * @param length 書き込むデータの長さ
     */
    bool Write(uint32_t address, const void *data, size_t length);

    /**
     * メモリーに32bitのデータを書き込む
     * @param address 書き込むアドレス
     * @param data 書き込むデータ
     */
    bool Write(uint32_t address, uint32_t data) {
        return Write(address, &data, 4);
    }

    /**
     * メモリーからデータを読み込む
     * @param address 読み込むアドレス
     * @param data 読み込んだデータの格納先へのポインタ
     * @param length 読み込むデータの長さ
     */
    bool Read(uint32_t address, void *data, size_t length);

    /**
     * メモリーから32bitのデータを読み込む
     * @param address 読み込むアドレス
     * @param data 読み込んだデータの格納先へのポインタ
     */
    bool Read(uint32_t address, uint32_t *data) {
        return Read(address, data, 4);
    }

private:
    static void InitializeHeader(int type, size_t length, uint32_t address, std::vector<uint8_t> &output);

    static void StreamBytesToPhysicalBytes(const std::vector<uint8_t> &stream_bytes, std::vector<uint8_t> &physical_bytes, size_t extend_idle = 0);

    static void PhysicalBytesToStreamBytes(const std::vector<uint8_t> &physical_bytes, std::vector<uint8_t> &stream_bytes);

    static bool StreamBytesToPayloadBytes(const std::vector<uint8_t> &stream_bytes, std::vector<uint8_t> &payload_bytes);

    static constexpr int CHANNEL_NUMBER = 0;

    std::shared_ptr<Spi> _Spi;
};
