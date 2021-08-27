#pragma once

#include <math.h>
#include <fpu.hpp>
#include <Eigen/Core>

/**
 * @brief IMUの測定値から重力の影響を取り除くフィルタ
 */
class GravityFilter {
public:
    /**
     * @brief 内部状態をリセットする
     */
    void reset(void) {
        _initialized = false;
    }

    /**
     * @brief フィルタに新たな入力を与えて出力を更新する
     * @param accel 加速度センサーの測定値
     * @param gyro ジャイロスコープの測定値
     */
    void update(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro) {
        using namespace Eigen;

        static constexpr float GYRO_GAIN_P = 1.0;
        static constexpr float GYRO_GAIN_I = 0.001;
        static constexpr float GRAVITY_LOW_THRESHOLD = 0.0625f;
        static constexpr float GRAVITY_COMPENSATION = 0.001f;

        if (!_initialized) {
            // 重力加速度ベクトルを初期化する
            _initialized = true;
            _gravity = accel;
            _gyro_error_integ = Vector3f::Zero();
        }

        // 加速度ベクトルと重力加速度ベクトルの成す角度を求める
        Vector3f gyro_error;
        float gravity_scale = fpu::sqrt(_gravity.squaredNorm());
        if (GRAVITY_LOW_THRESHOLD < gravity_scale) {
            gyro_error = accel.cross(_gravity) / (gravity_scale * gravity_scale);
            _gyro_error_integ += gyro_error;
        }
        else {
            // 重力が異様に小さいときは加速度センサーによる角速度の補正を減らす
            gyro_error = Vector3f::Zero();
            _gyro_error_integ *= fpu::max(1.0f - 1.0f / IMU_OUTPUT_RATE, 0.0f);
        }

        // 角速度を加速度センサーから得た角度誤差で補正する
        Vector3f delta_omega = GYRO_GAIN_P * gyro_error + GYRO_GAIN_I * _gyro_error_integ;
        _compensated_gyro = gyro + delta_omega;

        // 重力ベクトルを回転する
        Matrix3f Rt = rotationMatrixTransposed(_compensated_gyro * (1.0f / IMU_OUTPUT_RATE));
        _gravity = Rt * _gravity;

        // 重力加速度ベクトルの大きさを徐々に加速度の大きさに近づける
        // 重力が小さいときは大きさではなくベクトルそのものを使って補正する
        float accel_scale = fpu::sqrt(accel.squaredNorm());
        if (GRAVITY_LOW_THRESHOLD < accel_scale) {
            _gravity *= ((1.0f - GRAVITY_COMPENSATION) + GRAVITY_COMPENSATION * accel_scale / gravity_scale);
        }
        else {
            _gravity = (1.0f - GRAVITY_COMPENSATION) * _gravity + GRAVITY_COMPENSATION * accel;
        }

        // 加速度ベクトルから重力の影響を除去する
        _compensated_accel = accel - _gravity;
    }

    /**
     * @brief 重力加速度ベクトル(の反力)を取得する
     * @return 重力加速度ベクトル(の反力) X, Y, Z [m/s^2]
     */
    const Eigen::Vector3f& gravity(void) const {
        return _gravity;
    }

    /**
     * @brief 重力を除去済みの加速度を取得する
     * @return 加速度 X, Y, Z [m/s^2]
     */
    const Eigen::Vector3f& acceleration(void) const {
        return _compensated_accel;
    }

    /**
     * @brief 重力で補正済みの角速度を取得する
     * @return 角速度 X, Y, Z [rad/s]
     */
    const Eigen::Vector3f& angularVelocity(void) const {
        return _compensated_gyro;
    }

private:
    static Eigen::Matrix3f rotationMatrixTransposed(const Eigen::Vector3f& gyro) {
        float z = gyro.z();
        float y = gyro.y();
        float x = gyro.x();
        return Eigen::Matrix3f{
            {1.0f, x * y + z, x * z - y},
            {-z, 1.0f - x * y * z, x + y * z},
            {y, -x, 1.0f},
        };
    }

    bool _initialized = false;
    Eigen::Vector3f _gravity;
    Eigen::Vector3f _compensated_accel;
    Eigen::Vector3f _compensated_gyro;
    Eigen::Vector3f _gyro_error_integ;
};
