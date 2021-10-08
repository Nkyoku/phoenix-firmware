/**
 * @file wheel_controller.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Eigen/Core>
#include "filter/gravity_filter.hpp"
#include "filter/velocity_filter.hpp"
#include "filter/acceleration_limitter.hpp"

/**
 * 車輪制御を行う
 */
class WheelController {
public:
    /**
     * @brief 初期化を行う
     */
    static void initialize(void) {
        stopControl();
    }

    /**
     * @brief モーター制御を開始する
     */
    static void startControl(void);

    /**
     * @brief モーター制御を終了し、モーターを脱力する
     */
    static void stopControl(void);

    /**
     * @brief 指令値を更新する
     * @param new_parameters new_parameters 共有メモリーのParametersが更新されたときにtrueを指定する
     * @param sensor_only trueのときセンサーデータの更新のみ行う
     */
    static void update(bool new_parameters, bool sensor_only);

    /**
     * @brief 重力フィルタへアクセスする
     * @return 重力フィルタ
     */
    static const GravityFilter& gravityFilter(void) {
        return _gravity_filter;
    }

    /**
     * @brief 速度フィルタへアクセスする
     * @return 速度フィルタ
     */
    static const VelocityFilter& velocityFilter(void) {
        return _velocity_filter;
    }

    /**
     * @brief 車体加速度の推定値を取得する
     * @return 車体加速度 X, Y, Z [m/s^2]
     */
    static const Eigen::Vector3f& bodyAcceleration(void) {
        return _gravity_filter.acceleration();
    }

    /**
     * @brief 車体速度の推定値を取得する
     * @return 車体速度 X [m/s], Y [m/s], ω [rad/s]
     */
    static Eigen::Vector3f bodyVelocity(void) {
        return _velocity_filter.bodyVelocity();
    }

    /**
     * @brief 目標車体加速度を取得する
     * @return 目標車体加速度 X [m/s^2], Y [m/s^2], ω [rad/s^2], C [m/s^2]
     */
    static Eigen::Vector4f referenceAcceleration(void) {
        return _ref_body_accel;
    }

private:
    /**
     * 制御情報をクリアする
     */
    static void initializeState(void);

    /**
     * レジスタを初期化する
     */
    static void initializeRegisters(void);

    /// IMUの加速度から重力を分離するフィルタ
    static GravityFilter _gravity_filter;

    /// IMUとエンコーダから車体速度を求めるカルマンフィルタ
    static VelocityFilter _velocity_filter;

    /// 前回の制御ループでの速度誤差
    static Eigen::Vector4f _last_velocity_error;

    /// 車体加速度の指令値
    static Eigen::Vector4f _ref_body_accel;

    /// 電流制御の指令値 [A]
    static Eigen::Vector4f _ref_wheel_current;

    /// モーターの発生させた回生エネルギー (負の値をとる)
    static Eigen::Vector4f _regeneration_energy;
};
