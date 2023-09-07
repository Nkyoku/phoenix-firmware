/**
 * @file board.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

/// 円周率
static constexpr float PI = 3.1415926535f;

/// ADC1の出力値と電流との換算係数 [A/LSB]
static constexpr float ADC1_CURRENT_SCALE = 1.0f / 1977.5390625f;

/// IMUの加速度の換算係数 [m/s^2/LSB]
static constexpr float IMU_ACCELEROMETER_SCALE = 4.78515625e-3f;

/// IMUの角速度の換算係数 [rad/s/LSB]
static constexpr float IMU_GYROSCOPE_SCALE =  1.06526444e-3f;

/// IMUの割り込み周期 [Hz]
static constexpr float IMU_OUTPUT_RATE = 1000;

/// 車輪の実効的な半径 [m]
static constexpr float WHEEL_RADIUS = 0.0275f;

/// 車体中心を原点とした車輪のX座標[m]
static constexpr float WHEEL_POS_X = 0.0629f;

/// 車体中心を原点とした車輪のY座標[m]
static constexpr float WHEEL_POS_Y = 0.040848f;

/// 車体中心から車輪までの距離の二乗[m]
static constexpr float WHEEL_POS_R_2 = WHEEL_POS_X * WHEEL_POS_X + WHEEL_POS_Y * WHEEL_POS_Y;

/// モーターのトルク定数 [Nm/A]
static constexpr float MOTOR_TORQUE_CONSTANT = 0.131f;

/// モーターの巻線抵抗(相間) [Ω]
static constexpr float MOTOR_RESISTANCE = 6.89f;

/// モーターの定格出力 [W]
static constexpr float MOTOR_RATING_POWER = 70.0f;

/// 車輪が一回転したときのエンコーダのパルス数
static constexpr float ENCODER_PPR = 4096;

/// 車輪とモーターの慣性モーメント [kgm^2]
static constexpr float WHEEL_INERTIA = 3.6e-5f;

/// 車体の質量 [kg]
static constexpr float MACHINE_WEIGHT = 2.9f;

/// 車体の慣性モーメント [kgm^2]
static constexpr float MACHINE_INERTIA = 0.01f;
