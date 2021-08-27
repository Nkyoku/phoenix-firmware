#pragma once

#include <math.h>
#include <fpu.hpp>
#include <board.hpp>
#include <Eigen/Core>

/**
 * @brief IMUの測定値とオドメトリから車体速度を推定する
 */
class VelocityFilter {
public:
    /**
     * @brief 内部状態をリセットする
     */
    void reset(void) {
        _peak_hold = 0.0f;
        _sigma_vx = 0.0f;
        _sigma_vy = 0.0f;
    }

    /**
     * @brief フィルタに新たな入力を与えて出力を更新する
     * @param accel 加速度センサーの測定値
     * @param gyro ジャイロスコープの測定値
     * @param odom_body 車輪速度(オドメトリ)を車体速度に変換したもの
     * @param omega_weight 角速度の重み(1ならIMU, 0ならオドメトリを信頼する)
     */
    void update(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, const Eigen::Vector4f& odom_body, float omega_weight) {
        using namespace Eigen;

        // スリップ分の車輪速度を検出する
        static constexpr float PEAK_HOLD_DECAY = 0.99f;
        static constexpr float SLIP_VELOCITY_MAX = 10.0f;
        static constexpr float SLIP_VELOCITY_BASE = 0.01f;
        static constexpr float SLIP_VELOCITY_SIGMA_2 = 0.01f;
        float slip_velocity = odom_body(3);
        float peak_slip_velocity = fpu::min(fpu::max(slip_velocity, _peak_hold * PEAK_HOLD_DECAY), SLIP_VELOCITY_MAX);
        _peak_hold = peak_slip_velocity;
        float sigma_vx_vy_2 = (peak_slip_velocity * peak_slip_velocity + SLIP_VELOCITY_BASE) * SLIP_VELOCITY_SIGMA_2;

        // 信念分布を更新する
        static constexpr float ACCELEROMETER_SIGMA_2 = 0.1f;
        float SIGMA_hat_11 = _sigma_vx + (ACCELEROMETER_SIGMA_2 / IMU_OUTPUT_RATE / IMU_OUTPUT_RATE);
        float SIGMA_hat_22 = _sigma_vy + (ACCELEROMETER_SIGMA_2 / IMU_OUTPUT_RATE / IMU_OUTPUT_RATE);

        // カルマンゲインを計算する
        float K_11 = SIGMA_hat_11 / (sigma_vx_vy_2 + SIGMA_hat_11);
        float K_22 = SIGMA_hat_22 / (sigma_vx_vy_2 + SIGMA_hat_22);

        // 観測値を元に信念分布の分散を更新する
        _sigma_vx = (1.0f - K_11) * SIGMA_hat_11;
        _sigma_vy = (1.0f - K_22) * SIGMA_hat_22;

        // 観測値を元に信念分布の平均を更新する
        float rotation = gyro.z() * (1.0f / IMU_OUTPUT_RATE);
        _body_velocity(2) = omega_weight * gyro.z() + (1.0f - omega_weight) * odom_body(2);
        float mu_1 = _body_velocity(0) + _body_velocity(1) * rotation; // 本来は三角関数が必要だがcos(x)=1, sin(x)=xと近似している
        float mu_hat_1 = mu_1 + accel.x() * (1.0f / IMU_OUTPUT_RATE);
        float mu_2 = _body_velocity(1) - _body_velocity(0) * rotation;
        float mu_hat_2 = mu_2 + accel.y() * (1.0f / IMU_OUTPUT_RATE);
        _body_velocity(0) = K_11 * (odom_body(0) - mu_hat_1) + mu_hat_1;
        _body_velocity(1) = K_22 * (odom_body(1) - mu_hat_2) + mu_hat_2;
    }

    /**
     * @brief 車体速度の推定値を取得する
     * @return 車体速度 X [m/s], Y [m/s], ω [rad/s]
     */
    const Eigen::Vector3f& bodyVelocity(void) const {
        return _body_velocity;
    }

private:
    float _peak_hold = 0.0f;
    float _sigma_vx = 0.0f;
    float _sigma_vy = 0.0f;
    Eigen::Vector3f _body_velocity;
};
