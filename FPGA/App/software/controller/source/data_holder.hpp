#pragma once

#include <stdint.h>

/// 機体の動きに関するデータ
struct MotionData_t {
    struct Imu_t {
        float AccelX, AccelY, AccelZ;
        float GyroX, GyroY, GyroZ;
    } Imu;
    struct Wheel_t {
        float Velocity;
        float CurrentMeasD;
        float CurrentMeasQ;
        float CurrentRefQ;
    } Wheels[4];
};

/// ADC2で測定したデータ
struct Adc2Data_t {
    float Dc48vVoltage;
    float DribbleVoltage;
    float DribbleCurrent;
};

/**
 * ペリフェラルレジスタ等からのデータの読み出しと変換を一括で行う
 * 読んだ値を保存しアクセスできるようにする
 */
class DataHolder {
public:
    /**
     * 制御ループの先頭でレジスタを読む
     */
    static void FetchRegistersOnPreControlLoop(void);

    /**
     * ADC2の変換結果を読む
     */
    static void FetchAdc2Result(void);

    /**
     * MotionDataを取得する
     * @return　MotionDataへの参照
     */
    static const MotionData_t& GetMotionData(void){
        return _MotionData;
    }

    /**
     * Adc2Dataを取得する
     * @return Adc2Dataへの参照
     */
    static const Adc2Data_t& GetAdc2Data(void){
        return _Adc2Data;
    }

private:
    /// 機体の動きに関するデータ
    static MotionData_t _MotionData;

    /// ADC2で測定したデータ
    static Adc2Data_t _Adc2Data;
};
