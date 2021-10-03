#pragma once

#include <Eigen/Core>

/**
 * @brief 加速度を制限する
 */
class AccelerationLimitter {
public:
    /**
     * @brief 内部状態をリセットする
     */
    void reset(void) {}

    /**
     * @brief フィルタに新たな入力を与えて出力を更新する
     * @param accel_in 目標加速度
     * @param current_limit 各車輪の電流制限値
     * @param accel_out 制限された加速度
     * @param current_out 各車輪の目標電流
     */
    int compute(const Eigen::Vector4f& accel_in, const Eigen::Vector4f& current_limit, Eigen::Vector4f& accel_out, Eigen::Vector4f& current_out);
};
