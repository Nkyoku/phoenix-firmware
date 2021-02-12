#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>

/**
 * Avalon-ST Packets to Bytes Converterで変換されたバイトストリームをパケットに戻す
 */
class AvalonStBytesToPacketsConverter {
public:
    /**
     * パーサーの内部状態を初期化する。
     */
    void Reset(void) {
        _ChannelNumber = -1;
        _NextWillBeEscaped = 0;
        _NextWillBeChannel = false;
        _NextWillBeSof = false;
    }

    /**
     * 受信したデータをパースする。
     * パケット中のペイロードの1バイトごとにコールバック関数が呼ばれる。
     * @param data パースするデータへのポインタ
     * @param length パースするデータの長さ
     * @param callback ペイロードを1バイト処理するコールバック関数
     *                 callback(uint8_t data, int channel, bool sof, bool eof) -> bool
     */
    template <typename CallbackFunc>
    bool Parse(const void *data, size_t length, CallbackFunc callback) {
        const uint8_t *data_u8 = reinterpret_cast<const uint8_t *>(data);
        while (0 < length--) {
            uint8_t c = *data_u8++;
            if ((_NextWillBeEscaped != 0) || (c < 0x7A) || (0x7D < c)) {
                c ^= _NextWillBeEscaped;
                _NextWillBeEscaped = 0;
                if (_NextWillBeChannel) {
                    _NextWillBeChannel = false;
                    _ChannelNumber = c;
                } else {
                    if (callback(c, _ChannelNumber, _NextWillBeSof, _NextWillBeEof) == false) {
                        // コールバック関数がfalseを返したらパースを中断する
                        // 残りの入力データは破棄され、AvalonStBytesToPacketsConverterの内部状態は不定となる
                        return false;
                    }
                    _NextWillBeSof = false;
                    _NextWillBeEof = false;
                }
            } else if (c == 0x7A) {
                _NextWillBeSof = true;
            } else if (c == 0x7B) {
                _NextWillBeEof = true;
            } else if (c == 0x7C) {
                _NextWillBeChannel = true;
            } else if (c == 0x7D) {
                _NextWillBeEscaped = 0x20;
            }
        }
        return true;
    }

private:
    /// チャンネル番号
    int _ChannelNumber = -1;

    /// 次のバイトはエスケープ処理されている
    uint8_t _NextWillBeEscaped = 0;

    /// 次のバイトはチャンネル番号
    bool _NextWillBeChannel = false;

    /// 次のバイトはSOF
    bool _NextWillBeSof = false;

    /// 次のバイトはEOF
    bool _NextWillBeEof = false;
};

/**
 * Avalon-ST Packets to Bytes Converterでパケットをバイトストリームに変換する
 */
class AvalonStPacketsToBytesConverter {
public:
    /**
     * 変換を行う
     * @param payload ペイロード (長さ0のペイロードは長さ0のバイトストリームに変換される)
     * @param channel チャンネル番号 (0～255以外はチャンネル番号なし)
     * @param output 出力されたバイトストリーム。指定されたstd::vectorに追記される。
     */
    static void Convert(const std::vector<uint8_t> &payload, int channel, std::vector<uint8_t> &output);
};
