#pragma once

#include "spi.hpp"
#include <memory>

/**
 * EPCQ互換のSPIフラッシュメモリーを操作する
 */
class Epcq {
public:
    /// ページサイズ
    static constexpr uint32_t PAGE_SIZE = 256;

    /// セクターサイズ
    static constexpr uint32_t SECTOR_SIZE = 65536;

    /**
     * コンストラクタ
     * @param spi 通信に使用するSpi
     */
    Epcq(std::shared_ptr<Spi> &spi) : _Spi(spi) {}

    /**
     * シリコンID(ABh)を読み込む
     * @param sillicon_id 読み込んだシリコンIDの格納先のポインタ
     * @return 操作が完了したらtrueを返す
     */
    bool ReadSilliconId(uint8_t *sillicon_id);

    /**
     * ステータスレジスタを読み込む
     * @param status 読み込んだステータスの格納先のポインタ
     * @return 操作が完了したらtrueを返す
     */
    bool ReadStatus(uint8_t *status);

    /**
     * メモリーからデータを読み込む
     * @param address 読み込むアドレス
     * @param data 読み込んだデータの格納先へのポインタ
     * @param length 読み込むデータの長さ
     * @return 操作が完了したらtrueを返す
     */
    bool Read(uint32_t address, void *data, size_t length);

    /**
     * メモリーにデータを書き込む
     * @param address 書き込むアドレス
     * @param data 書き込むデータへのポインタ
     * @return 操作が完了したらtrueを返す
     */
    bool WritePage(uint32_t address, const void *data);

    /**
     * セクターを消去する
     * @param address 消去するセクターアドレス
     * @return 操作が完了したらtrueを返す
     */
    bool EraseSector(uint32_t address);

private:
    /**
     * 書き込みを有効化する
     * @param enabled trueなら書き込みを有効化する、falseなら無効化する
     * @return 操作が完了したらtrueを返す
     */
    bool SetWriteEnabled(bool enabled);

    /**
     * ステータスレジスタのWIPビットが0になるのを待つ
     * @param タイムアウト時間[ms]
     * @return 時間内にWIPビットが0になったらtrueを返す
     */
    bool WaitProgressBit(int timeout_in_ms);

    /// ステータスレジスタのフラグ
    enum {
        STATUS_TB = 0x20,
        STATUS_BP2 = 0x10,
        STATUS_BP1 = 0x08,
        STATUS_BP0 = 0x04,
        STATUS_WEL = 0x02,
        STATUS_WIP = 0x01
    };

    std::shared_ptr<Spi> _Spi;
};
