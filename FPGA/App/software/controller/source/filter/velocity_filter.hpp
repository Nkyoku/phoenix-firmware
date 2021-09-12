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
        using namespace Eigen;
        _sigma = Vector3f::Zero();
        _body_velocity = Vector3f::Zero();
        _last_gyro_z = 0.0f;
        _imu_error = Vector3f::Zero();
        _odom_error = 0.0f;
    }

    /**
     * @brief フィルタに新たな入力を与えて出力を更新する
     * @param accel 加速度センサーの測定値
     * @param gyro ジャイロスコープの測定値
     * @param velocity_by_wheels 車輪速度(オドメトリ)を車体速度に変換したもの
     * @param acceleration_by_wheels 車体加速度の推定値
     */
    void update(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, const Eigen::Vector4f& velocity_by_wheels,
                const Eigen::Vector3f& acceleration_by_wheels) {
        using namespace Eigen;

        // IMU角速度を微分しX,Y成分の加速度と一緒にまとめる
        Vector3f acceleration_by_imu{accel.x(), accel.y(), (gyro.z() - _last_gyro_z) * IMU_OUTPUT_RATE};
        _last_gyro_z = gyro.z();

        // IMUの誤差と分散を求める
        Vector3f imu_sigma2;
        for (int index = 0; index < 3; index++) {
            float error = fabsf(acceleration_by_wheels(index) - acceleration_by_imu(index));
            _imu_error(index) = fpu::min(fpu::max(error, 0.995f * _imu_error(index)), 10.0f);
            imu_sigma2(index) = (_imu_error(index) * _imu_error(index) + 0.0001f) * (1.0f / IMU_OUTPUT_RATE / IMU_OUTPUT_RATE);
        }

        // オドメトリの誤差と分散を求める
        _odom_error = fpu::min(fpu::max(fabsf(velocity_by_wheels(3)), 0.99f * _odom_error), 10.0f);
        Vector3f odom_sigma2 = (_odom_error * _odom_error) * Vector3f{
                                                                 powf(sqrt(WHEEL_POS_R_2) / WHEEL_POS_Y / 4, 2),
                                                                 powf(sqrt(WHEEL_POS_R_2) / WHEEL_POS_X / 4, 2),
                                                                 powf(1.0f / sqrt(WHEEL_POS_R_2) / 4, 2),
                                                             };

        // 以下のプログラムでは共分散を0としてカルマンフィルタを行列ではなく成分ごと独立して計算している
        // 実際にはX,Y成分間の共分散は0ではない

        // 信念分布を更新する
        Vector3f sigma_hat = _sigma + imu_sigma2;

        // カルマンゲインを計算する
        Vector3f k = sigma_hat.cwiseQuotient(sigma_hat + odom_sigma2);

        // 観測値を元に信念分布の分散を更新する
        _sigma = (Vector3f::Ones() - k).cwiseProduct(sigma_hat);

        // 観測値を元に信念分布の平均を更新する
        float rotation = gyro.z() * (1.0f / IMU_OUTPUT_RATE);
        Vector3f mu_hat = _body_velocity + acceleration_by_imu * (1.0f / IMU_OUTPUT_RATE);
        mu_hat.x() += rotation * _body_velocity.y(); // cos(x)=1, sin(x)=xと近似している
        mu_hat.y() -= rotation * _body_velocity.x();
        _body_velocity = k.cwiseProduct(velocity_by_wheels.block<3, 1>(0, 0) - mu_hat) + mu_hat;
    }

    /**
     * @brief 車体速度の推定値を取得する
     * @return 車体速度 X [m/s], Y [m/s], ω [rad/s]
     */
    const Eigen::Vector3f& bodyVelocity(void) const {
        return _body_velocity;
    }

private:
    /// 分散
    Eigen::Vector3f _sigma;

    /// 車体速度の推定値 dx/dt [m/s], dy/dt [m/s], ω [rad/s]
    Eigen::Vector3f _body_velocity;

    /// IMUの誤差推定値 dx^2/dt^2 [m/s^2], dy^2/dt^2 [m/s^2], dω/dt [rad/s^2]
    Eigen::Vector3f _imu_error;

    /// エンコーダの誤差推定値 [m/s]
    float _odom_error = 0.0f;

    /// 前回のIMUの角速度のZ成分 [rad/s]
    float _last_gyro_z = 0.0f;
};
