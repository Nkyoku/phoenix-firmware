#pragma once

#include <Eigen/Core>

/**
 * @brief IMUの測定値とオドメトリから車体速度を推定する
 */
class VelocityFilter {
private:
    using Vector7f = Eigen::Matrix<float, 7, 1>;
    using Matrix7f = Eigen::Matrix<float, 7, 7>;

public:
    /**
     * @brief 内部状態をリセットする
     */
    void reset(void);

    /**
     * @brief フィルタに新たな入力を与えて出力を更新する
     * @param accel 加速度センサーの測定値
     * @param gyro ジャイロスコープの測定値
     * @param wheel_velocity 車輪速度
     * @param wheel_current モーター電流
     */
    void update(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, const Eigen::Vector4f& wheel_velocity, const Eigen::Vector4f& wheel_current);

    /**
     * @brief 車体速度の推定値を取得する
     * @return 車体速度 X [m/s], Y [m/s], ω [rad/s]
     */
    Eigen::Vector3f bodyVelocity(void) const {
        return {_mu(0), _mu(1), _mu(2)};
    }

    /**
     * @brief 摩擦係数の推定値を取得する
     * @return 摩擦係数 [Ns]
     */
    Eigen::Vector4f frictionCoefficients(void) const {
        return {_mu(3), _mu(4), _mu(5), _mu(6)};
    }

    /// 状態変数の最尤値
    Vector7f _mu;

    /// 共分散
    Matrix7f _sigma;

    /// 前回の更新時の車輪速度
    Eigen::Vector4f _last_wheel_velocity;

    /// 線形化された状態方程式
    Matrix7f G;

    /// 線形化された観測方程式
    Matrix7f H;

    /// 逆行列を求める際の途中計算結果
    Matrix7f L, invL;
};
