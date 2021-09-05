#pragma once

#include <stdint.h>
#include <system.h>
#include <altera_msgdma.h>
#include "data_holder.hpp"

/**
 * UARTでJetsonへ定期的にデータを送信する
 */
class StreamTransmitter {
public:
    /**
     * 初期化する
     */
    static void initialize(void) {
        alt_msgdma_dev *dev = alt_msgdma_open(MSGDMA_0_CSR_NAME);
        _device = dev;
    }

    /**
     * ステータスフラグを送信する
     */
    static void transmitStatus(void);

    /**
     * ADC2の測定値を送信する
     * @param adc2_data 送るデータ
     */
    static void transmitAdc2(const Adc2Data_t &adc2_data);

    /**
     * モーションデータを送信する
     * @param motion_data モーションデータ
     * @param control_data 制御データ
     * @param performance_counter パフォーマンスカウンタの値
     */
    static void transmitMotion(const MotionData_t &motion_data, const ControlData_t &control_data, int performance_counter);

private:
    /// mSGDMAのハンドル
    static alt_msgdma_dev *_device;
};
