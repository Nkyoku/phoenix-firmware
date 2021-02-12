#pragma once

#include <stdint.h>
#include <system.h>
#include <altera_msgdma.h>
#include <data_holder.hpp>

/**
 * UARTでJetsonへ定期的にデータを送信する
 */
class StreamTransmitter {
public:
    /**
     * 初期化する
     */
    static void Initialize(void) {
        alt_msgdma_dev *dev = alt_msgdma_open(MSGDMA_0_CSR_NAME);
        _Device = dev;
    }

    /**
     * ステータスフラグを送信する
     */
    static void TransmitStatus(void);

    /**
     * ADC2の測定値を送信する
     */
    static void TransmitAdc2(const Adc2Data_t &adc2_data);

    /**
     * モーションデータを送信する
     */
    static void TransmitMotion(const MotionData_t &motion_data, int performance_counter);

private:
    /// mSGDMAのハンドル
    static alt_msgdma_dev *_Device;
};
