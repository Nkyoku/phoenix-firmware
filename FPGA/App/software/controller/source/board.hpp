#pragma once

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

/// 車輪の実効的な円周 [m]
static constexpr float WHEEL_CIRCUMFERENCE = 0.173f;

/// 車体中心を原点とした車輪のX座標[m]
static constexpr float WHEEL_POS_X = 0.063f;

/// 車体中心を原点とした車輪のY座標[m]
static constexpr float WHEEL_POS_Y = 0.042f;

/// 車体中心から車輪までの距離の二乗[m]
static constexpr float WHEEL_POS_R_2 = WHEEL_POS_X * WHEEL_POS_X + WHEEL_POS_Y * WHEEL_POS_Y;

/// モーターの逆起電力定数(相間) [V/rps]
static constexpr float MOTOR_SPEED_CONSTANT = 1.0f / 72.7f * 60.0f;

/// モーターのトルク定数 [Nm/A]
static constexpr float MOTOR_TORQUE_CONSTANT = 0.131f;

/// モーターの巻線抵抗(相間) [Ω]
static constexpr float MOTOR_RESISTANCE = 6.89f;

/// 車輪が一回転したときのエンコーダのパルス数
static constexpr float ENCODER_PPR = 4096;

/// 車輪とモーターの慣性モーメント [kgm^2]
static constexpr float WHEEL_INERTIA = 3.6e-5f;

/// 機体の質量 [kg]
static constexpr float MACHINE_WEIGHT = 3.3f;

/// 機体の慣性モーメント [kgm^2]
static constexpr float MACHINE_INERTIA = 0.01f;
