#pragma once

#include <math.h>
#include <board.hpp>
#include <Eigen/Core>
#include "lpf.hpp"

/**
 * @brief 車体のトルクを推定する
 */
class TorqueObserver {
public:
    /**
     * @brief 内部状態をリセットする
     */
    void reset(void) {
        _lpf.reset();
        _last_omega_sum = 0.0f;
        _torque = 0.0f;
    }

    /**
     * @brief フィルタに新たな入力を与えて出力を更新する
     * @param wheel_odom 車輪の速度 [m/s]
     */
    void update(const Eigen::Vector4f &wheel_odom, const Eigen::Vector4f &current) {
        float omega_sum = wheel_odom.sum();
        float actual_wheel_torque_sum = (omega_sum - _last_omega_sum) * (WHEEL_INERTIA * IMU_OUTPUT_RATE / WHEEL_RADIUS);
        _last_omega_sum = omega_sum;
        float ideal_wheel_torque_sum = current.sum() * MOTOR_TORQUE_CONSTANT;
        float torque = (ideal_wheel_torque_sum - actual_wheel_torque_sum) * (sqrt(WHEEL_POS_R_2) / WHEEL_RADIUS);
        _torque = fabsf(_lpf(torque));
    }

    /**
     * @brief トルクを取得する
     * @return トルク [Nm]
     */
    float absTorque(void) const {
        return _torque;
    }

private:
    Lpf2ndOrder100 _lpf;
    float _last_omega_sum = 0.0f;
    float _torque = 0.0f;
};
