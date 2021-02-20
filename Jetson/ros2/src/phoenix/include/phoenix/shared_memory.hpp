#pragma once

#include <stdint.h>

/**
 * 共有メモリーの構造を定義する構造体 (最大1024バイト)
 */
struct SharedMemory_t {
    /**
     * エラーフラグのビットマップ
     * この値を0xFFFFFFFFにセットするとエラーの解除が試みられる
     */
    uint32_t ErrorFlags;

    /**
     * フォルトフラグのビットマップ
     */
    uint32_t FaultFlags;

    /**
     * 先頭のチェックサム
     * HeadChecksumとTailChecksumが等しくそれらの値が正しいときにのみParametersは有効として扱われる
     * ROS2の命名規則に則りROS2で使うフィールド名は小文字になっている
     */
    uint32_t HeadChecksum;

    /**
     * JetsonからNios IIへ制御パラメータを伝達する構造体
     */
    struct Parameters_t {
        /**
         * フレーム番号。更新するたびに1ずつ増やす
         */
        uint32_t FrameNumber;

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
         * 速度制御の比例ゲイン
         */
        float speed_gain_p;

        /**
         * 速度制御の積分ゲイン
         */
        float speed_gain_i;

        /**
         * チェックサムを計算する
         * この関数はParameters_tが4の倍数バイトの大きさであることを前提にしている
         * @return チェックサム
         */
        uint32_t CalculateChecksum(void) const {
            auto p = reinterpret_cast<const uint32_t*>(this);
            int count = sizeof(Parameters_t) / sizeof(uint32_t);
            uint32_t result = 0xA5A5A5A5;
            while (0 <= --count) {
                result += *p++;
            }
            return result;
        }
    } Parameters;

    /**
     * 末尾のチェックサム
     * HeadChecksumとTailChecksumが等しいときにのみParametersは有効として扱われる
     */
    uint32_t TailChecksum;
};
