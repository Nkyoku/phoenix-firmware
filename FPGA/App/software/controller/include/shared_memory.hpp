#pragma once

#include <stdint.h>

#pragma pack(push, 4)

/**
 * 共有メモリーの構造を定義する構造体 (最大1024バイト)
 */
struct SharedMemory {
    /**
     * エラーフラグのビットマップ
     * この値を0xFFFFFFFFにセットするとエラーの解除が試みられる
     */
    uint32_t error_flags;

    /**
     * フォルトフラグのビットマップ
     */
    uint32_t fault_flags;

    /**
     * 先頭のチェックサム
     * head_checksumとtail_checksumが等しくそれらの値が正しいときにのみParametersは有効として扱われる
     * ROS2の命名規則に則りROS2で使うフィールド名は小文字になっている
     */
    uint32_t head_checksum;

    /**
     * JetsonからNios IIへ制御パラメータを伝達する構造体
     */
    struct Parameters {
        /**
         * フレーム番号。更新するたびに1ずつ増やす
         */
        uint32_t frame_number;

        /**
         * 車体左右の並進移動速度指令[m/s]
         */
        float speed_x;

        /**
         * 車体前後の並進移動速度指令[m/s]
         */
        float speed_y;

        /**
         * 回転速度指令[rad/s]
         */
        float speed_omega;

        /**
         * ドリブルパワー (-1.0 ～ 1.0)
         */
        float dribble_power;

        /**
         * 速度制御のゲイン
         */
        float speed_gain_p[4], speed_gain_i[4];

        /**
         * チェックサムを計算する
         * この関数はParametersが4の倍数バイトの大きさであることを前提にしている
         * @return チェックサム
         */
        uint32_t calculateChecksum(void) const {
            auto p = reinterpret_cast<const uint32_t*>(this);
            int count = sizeof(Parameters) / sizeof(uint32_t);
            uint32_t result = 0xA5A5A5A5;
            while (0 <= --count) {
                result += *p++;
            }
            return result;
        }
    } parameters;

    /**
     * 末尾のチェックサム
     * head_checksumとtail_checksumが等しいときにのみParametersは有効として扱われる
     */
    uint32_t tail_checksum;
};

#pragma pack(pop)
