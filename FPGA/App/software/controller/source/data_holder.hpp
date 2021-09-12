#pragma once

#include <stdint.h>
#include <Eigen/Dense>

/// ADC2で測定したデータ
struct Adc2Data_t {
    float dc48v_voltage;
    float dribble_voltage;
    float dribble_current;
};

/// 車体の動きに関するデータ
struct MotionData_t {
    Eigen::Vector3f accelerometer;
    Eigen::Vector3f gyroscope;
    Eigen::Vector3f gravity;
    Eigen::Vector4f wheel_velocity;
    Eigen::Vector4f wheel_current_d;
    Eigen::Vector4f wheel_current_q;
    Eigen::Vector3f body_acceleration;
    Eigen::Vector3f body_velocity;
};

/// 制御に関するデータ
struct ControlData_t {
    Eigen::Vector4f current_ref;
    Eigen::Vector4f body_ref_accel;
};

/**
 * ペリフェラルレジスタ等からのデータの読み出しと変換を一括で行う
 * 読んだ値を保存しアクセスできるようにする
 */
class DataHolder {
public:
    /**
     * 制御ループの前でレジスタを読む
     */
    static void fetchOnPreControlLoop(void);

    /**
     * 制御ループの後でレジスタを読む
     */
    static void fetchOnPostControlLoop(void);

    /**
     * ADC2の変換結果を読む
     */
    static void fetchAdc2Result(void);

    /**
     * Adc2Dataを取得する
     * @return Adc2Dataへの参照
     */
    static const Adc2Data_t& adc2Data(void) {
        return _adc2_data;
    }

    /**
     * MotionDataを取得する
     * @return　MotionDataへの参照
     */
    static const MotionData_t& motionData(void) {
        return _motion_data;
    }

    /**
     * ControlDataを取得する
     * @return　ControlDataへの参照
     */
    static const ControlData_t& controlData(void) {
        return _control_data;
    }

private:
    /// ADC2で測定したデータ
    static Adc2Data_t _adc2_data;

    /// 機体の動きに関するデータ
    static MotionData_t _motion_data;

    /// 制御に関するデータ
    static ControlData_t _control_data;
};
