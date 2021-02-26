#pragma once

#include <stdint.h>

/// ADC2で測定したデータ
struct Adc2Data_t {
    float Dc48vVoltage;
    float DribbleVoltage;
    float DribbleCurrent;
};

/// 機体の動きに関するデータ
struct MotionData_t {
    struct Imu_t {
        union {
            struct {
                float AccelX, AccelY, AccelZ;
            };
            float Accel[3];
        };
        union {
            struct {
                float GyroX, GyroY, GyroZ;
            };
            float Gyro[3];
        };
    } Imu;
    struct Wheel_t {
        float Velocity;
        float CurrentD;
        float CurrentQ;
    } Wheels[4];
};

/// 制御に関するデータ
struct ControlData_t {
    struct Wheel_t {
        float VelocityRef;
        float CurrentRef;
        float Energy;
    } Wheels[4];
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
     * Adc2Dataを取得する
     * @return Adc2Dataへの参照
     */
    static const Adc2Data_t& GetAdc2Data(void) {
        return _Adc2Data;
    }

    /**
     * MotionDataを取得する
     * @return　MotionDataへの参照
     */
    static const MotionData_t& GetMotionData(void) {
        return _MotionData;
    }

    /**
     * ControlDataを取得する
     * @return　ControlDataへの参照
     */
    static const ControlData_t& GetControlData(void) {
        return _ControlData;
    }

private:
    /// ADC2で測定したデータ
    static Adc2Data_t _Adc2Data;

    /// 機体の動きに関するデータ
    static MotionData_t _MotionData;

    /// 制御に関するデータ
    static ControlData_t _ControlData;
};
