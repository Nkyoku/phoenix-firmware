#pragma once

#include <math.h>
#include <board.hpp>
#include <Eigen/Core>
#include <fpu.hpp>

/**
 * @brief 車輪の駆動力を推定する
 */
class DrivingForceObserver {
public:
    /**
     * @brief 内部状態をリセットする
     */
    void reset(void) {
        _last_wheel_velocity = 0.0f;
        _driving_force = 0.0f;
    }

    /**
     * @brief フィルタに新たな入力を与えて出力を更新する
     * @param wheel_velocity 車輪速度 [m/s]
     * @param current モーター電流 [A]
     * @param friction_current モーターの摩擦力に打ち勝つのに要する電流 [A]
     */
    void update(float wheel_velocity, float current, float friction_current) {
        // 車輪速度の変化から力を求める
        float force_from_velocity = (wheel_velocity - _last_wheel_velocity) * (WHEEL_INERTIA * IMU_OUTPUT_RATE / WHEEL_RADIUS / WHEEL_RADIUS);
        _last_wheel_velocity = wheel_velocity;

        // モーターに流れた電流から力を求める
        float deadband = fpu::max(fabsf(wheel_velocity) * (-10.0f * friction_current) + friction_current, 0.0f);
        if (deadband < current) {
            current -= deadband;
        }
        else if (current < -deadband) {
            current += deadband;
        }
        else {
            current = 0.0f;
        }
        float force_from_current = current * (MOTOR_TORQUE_CONSTANT / WHEEL_RADIUS);

        // 力の差が駆動力となる
        _driving_force = force_from_current - force_from_velocity;
    }

    /**
     * @brief 駆動力を取得する
     * @return 駆動力 [N]
     */
    float drivingForce(void) const {
        return _driving_force;
    }

private:
    float _last_wheel_velocity = 0.0f;
    float _driving_force = 0.0f;
};
