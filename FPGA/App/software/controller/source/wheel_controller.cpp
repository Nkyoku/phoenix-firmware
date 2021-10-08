/**
 * @file wheel_controller.cpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#include "wheel_controller.hpp"
#include "centralized_monitor.hpp"
#include "shared_memory_manager.hpp"
#include "data_holder.hpp"
#include "board.hpp"
#include <peripheral/vector_controller.hpp>
#include <status_flags.hpp>
#include <fpu.hpp>
#include <system.h>
#include <math.h>

#define USE_SIMPLE_CONTROL 0

using namespace Eigen;

/// 並進速度指令値の最大値 [m/s]
static constexpr float MAX_TRANSLATION_REFERENCE = 20.0f;

/// 回転速度指令の最大値 [m/s]
static constexpr float MAX_OMEGA_REFERENCE = 20.0f;

/// 電流制限値の最小値 [A]
static constexpr float MIN_CURRENT_LIMIT_PER_MOTOR = 0.2f;

/// 電流制限値の最大値 [A]
static constexpr float MAX_CURRENT_LIMIT_PER_MOTOR = 3.0f;

/// スリップ抑制ゲイン [A/(m/s)]
static constexpr float ANTI_SLIP_GAIN = 0.25f;

/// 並進加速度の最大値 [m/s^2]
static constexpr float MAX_TRANSLATION_ACCELERATION = 10.0f;

/// 角加速度の最大値 [m/s^2]
static constexpr float MAX_ANGULAR_ACCELERATION = 100.0f;

/// 加速度指令値の減衰
static constexpr float REF_ACCEL_DECAY = 0.995f;

/// モータードライバのベース消費電力 [W]
static constexpr float BASE_POWER_CONSUMPTION_PER_MOTOR = 0.25f;

/// ブレーキを有効にする回生エネルギーの閾値
static constexpr float BRAKE_ENABLE_THRESHOLD = -0.01f;

/// ブレーキを無効にする回生エネルギーの閾値
static constexpr float BRAKE_DISABLE_THRESHOLD = -0.005f;

/// 過電流閾値[A]
static constexpr float OVER_CURRENT_THRESHOLD = 5.0f;

/**
 * @brief 車輪速度ベクトルを車体速度ベクトルに変換する
 * @param wheel_velocity 車輪速度ベクトル [m/s]
 * @return 車体速度ベクトル X [m/s], Y [m/s], ω [rad/s], C [m/s]
 */
static Eigen::Vector4f velocityVectorComposition(const Eigen::Vector4f &wheel_velocity) {
    Eigen::Vector4f body_velocity;
    body_velocity(0) = (wheel_velocity(1) - wheel_velocity(0) + wheel_velocity(2) - wheel_velocity(3)) * (sqrt(WHEEL_POS_R_2) / WHEEL_POS_Y / 4);
    body_velocity(1) = (wheel_velocity(0) + wheel_velocity(1) - wheel_velocity(2) - wheel_velocity(3)) * (sqrt(WHEEL_POS_R_2) / WHEEL_POS_X / 4);
    body_velocity(2) = (wheel_velocity(0) + wheel_velocity(1) + wheel_velocity(2) + wheel_velocity(3)) * (0.25f / sqrt(WHEEL_POS_R_2));
    body_velocity(3) = (wheel_velocity(0) - wheel_velocity(1) + wheel_velocity(2) - wheel_velocity(3));
    return body_velocity;
}

/**
 * @brief 車体速度ベクトルを車輪速度ベクトルに変換する
 * @param body_velocity 車体速度ベクトル X [m/s], Y [m/s], ω [rad/s], C [m/s]
 * @return 車輪速度ベクトル [m/s]
 */
static Eigen::Vector4f velocityVectorDecomposition(const Eigen::Vector3f &body_velocity) {
    Eigen::Vector4f wheel_velocity;
    float vx = body_velocity(0) * (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2));
    float vy = body_velocity(1) * (WHEEL_POS_X / sqrt(WHEEL_POS_R_2));
    float omega = body_velocity(2) * sqrt(WHEEL_POS_R_2);
    wheel_velocity(0) = omega - vx + vy;
    wheel_velocity(1) = omega + vx + vy;
    wheel_velocity(2) = omega + vx - vy;
    wheel_velocity(3) = omega - vx - vy;
    return wheel_velocity;
}

/**
 * @brief 車体速度ベクトルを車輪速度ベクトルに変換する
 * @param body_velocity 車体速度ベクトル X [m/s], Y [m/s], ω [rad/s], C [m/s]
 * @return 車輪速度ベクトル [m/s]
 */
static Eigen::Vector4f velocityVectorDecomposition(const Eigen::Vector4f &body_velocity) {
    Eigen::Vector4f wheel_velocity;
    float vx = body_velocity(0) * (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2));
    float vy = body_velocity(1) * (WHEEL_POS_X / sqrt(WHEEL_POS_R_2));
    float omega = body_velocity(2) * sqrt(WHEEL_POS_R_2);
    float cancel = body_velocity(3);
    wheel_velocity(0) = omega - vx + vy + cancel;
    wheel_velocity(1) = omega + vx + vy - cancel;
    wheel_velocity(2) = omega + vx - vy + cancel;
    wheel_velocity(3) = omega - vx - vy - cancel;
    return wheel_velocity;
}

void WheelController::startControl(void) {
    initializeRegisters();
    VectorController::clearFault();
    initializeState();
}

void WheelController::stopControl(void) {
    VectorController::setFault();
    initializeRegisters();
    initializeState();
}

void WheelController::initializeState(void) {
    _gravity_filter.reset();
    _velocity_filter.reset();
    _last_velocity_error.setZero();
    _ref_body_accel.setZero();
    _ref_wheel_current.setZero();
    _regeneration_energy.setZero();
}

void WheelController::initializeRegisters(void) {
    static constexpr int CURRENT_CONTROL_GAIN_P = 3500; // 電流制御の比例ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_I = 500;  // 電流制御の積分ゲイン
    VectorController::setGainP(CURRENT_CONTROL_GAIN_P);
    VectorController::setGainI(CURRENT_CONTROL_GAIN_I);
    VectorController::setCurrentReferenceQ(1, 0);
    VectorController::setCurrentReferenceQ(2, 0);
    VectorController::setCurrentReferenceQ(3, 0);
    VectorController::setCurrentReferenceQ(4, 0);
    VectorController::clearAllBrakeEnabled();
}

void WheelController::update(bool new_parameters, bool sensor_only) {
    // センサーデータを取得する
    auto &motion = DataHolder::motionData();

    // 車輪速度を取得し車体速度に換算する
    Vector4f wheel_velocity = motion.wheel_velocity;
    Vector4f body_velocity_by_wheels = velocityVectorComposition(wheel_velocity);

    // 速度指令値が異常でないことを確認する
    // 速度が速すぎるかNaNならspeed_ok==falseとなる
    auto &parameters = SharedMemoryManager::getParameters();
    bool speed_ok = true;
    speed_ok &= fabsf(parameters.speed_x) <= MAX_TRANSLATION_REFERENCE;
    speed_ok &= fabsf(parameters.speed_y) <= MAX_TRANSLATION_REFERENCE;
    speed_ok &= fabsf(parameters.speed_omega) <= MAX_OMEGA_REFERENCE;

    // 車体速度を推定する
    _gravity_filter.update(motion.accelerometer, motion.gyroscope);
    _velocity_filter.update(bodyAcceleration(), motion.gyroscope, wheel_velocity, motion.wheel_current_q);
    if (!isfinite(bodyVelocity()[0]) || !isfinite(bodyVelocity()[1]) || !isfinite(bodyVelocity()[2])) {
        CentralizedMonitor::setErrorFlags(ErrorArithmetic);
        return;
    }

    // 以下で制御を行う
    if (speed_ok && !sensor_only && !VectorController::isFault()) {
        bool brake_enabled[4];
        brake_enabled[0] = VectorController::isBrakeEnabled(1);
        brake_enabled[1] = VectorController::isBrakeEnabled(2);
        brake_enabled[2] = VectorController::isBrakeEnabled(3);
        brake_enabled[3] = VectorController::isBrakeEnabled(4);

        // 過電流を判定する
        auto norm = [](float x, float y) {
            return fpu::sqrt(x * x + y * y);
        };
        uint32_t error_flags = 0;
        if (!brake_enabled[0] && (OVER_CURRENT_THRESHOLD < norm(motion.wheel_current_d(0), motion.wheel_current_q(0)))) {
            error_flags |= ErrorCauseMotor1OverCurrent;
        }
        if (!brake_enabled[1] && (OVER_CURRENT_THRESHOLD < norm(motion.wheel_current_d(1), motion.wheel_current_q(1)))) {
            error_flags |= ErrorCauseMotor2OverCurrent;
        }
        if (!brake_enabled[2] && (OVER_CURRENT_THRESHOLD < norm(motion.wheel_current_d(2), motion.wheel_current_q(2)))) {
            error_flags |= ErrorCauseMotor3OverCurrent;
        }
        if (!brake_enabled[3] && (OVER_CURRENT_THRESHOLD < norm(motion.wheel_current_d(3), motion.wheel_current_q(3)))) {
            error_flags |= ErrorCauseMotor4OverCurrent;
        }
        if (error_flags != 0) {
            CentralizedMonitor::setErrorFlags(error_flags);
            return;
        }

        // 車体速度制御を行う
        Vector4f ref_body_velocity = {parameters.speed_x, parameters.speed_y, parameters.speed_omega, 0.0f};
#if USE_SIMPLE_CONTROL
        Vector4f ref_wheel_velocity = velocityVectorDecomposition(ref_body_velocity);
        for (int index = 0; index < 4; index++) {
            float error = ref_wheel_velocity(index) - wheel_velocity(index);
            static const float p_gain = 5.0f;
            static const float i_gain = 0.05f;
            float current = _ref_wheel_current(index) + p_gain * (error - _last_velocity_error(index)) + i_gain * error;
            _ref_wheel_current(index) = fpu::clamp(current, -MAX_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
            _last_velocity_error(index) = error;
        }
#else
        // 車体加速度の指令値を求める
        Vector4f body_velocity;
        body_velocity[0] = bodyVelocity()[0];
        body_velocity[1] = bodyVelocity()[1];
        body_velocity[2] = bodyVelocity()[2];
        body_velocity[3] = body_velocity_by_wheels[3];
        Vector4f ref_body_accel_unlimit;
        for (int index = 0; index < 4; index++) {
            float error = ref_body_velocity[index] - body_velocity[index];
            float p_gain = parameters.speed_gain_p[index];
            float i_gain = parameters.speed_gain_i[index];
            float accel = _ref_body_accel[index] + p_gain * (error - _last_velocity_error[index]) + i_gain * error;
            _last_velocity_error[index] = error;
            if (index != 2) {
                ref_body_accel_unlimit[index] = fpu::clamp(accel, -MAX_TRANSLATION_ACCELERATION, MAX_TRANSLATION_ACCELERATION);
            }
            else {
                ref_body_accel_unlimit[index] = fpu::clamp(accel, -MAX_ANGULAR_ACCELERATION, MAX_ANGULAR_ACCELERATION);
            }
        }

        // 各モーターへの電流の割り当てと制限を行う
        Vector4f current_limit, ref_current;
        Vector4f velocity_error = velocityVectorDecomposition(bodyVelocity()) - wheel_velocity;
        current_limit[0] = fpu::clamp(MAX_CURRENT_LIMIT_PER_MOTOR - fabsf(velocity_error[0]), MIN_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
        current_limit[1] = fpu::clamp(MAX_CURRENT_LIMIT_PER_MOTOR - fabsf(velocity_error[1]), MIN_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
        current_limit[2] = fpu::clamp(MAX_CURRENT_LIMIT_PER_MOTOR - fabsf(velocity_error[2]), MIN_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
        current_limit[3] = fpu::clamp(MAX_CURRENT_LIMIT_PER_MOTOR - fabsf(velocity_error[3]), MIN_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
        AccelerationLimitter limitter;
        if (!limitter.compute(ref_body_accel_unlimit, current_limit, _ref_body_accel, ref_current)) {
            CentralizedMonitor::setErrorFlags(ErrorArithmetic);
            return;
        }

        // 電流割り当ての結果、加速度が元の指令値より大きくなったときは次の制御ループに伝搬する加速度の値を制限する
        for (int index = 0; index < 4; index++) {
            float accel = fabsf(ref_body_accel_unlimit[index]);
            _ref_body_accel[index] = fpu::clamp(_ref_body_accel[index] * REF_ACCEL_DECAY, -accel, accel);
        }

        // 速度推定値から求めた車輪速度と実際の車輪速度の誤差に係数を掛けて電流指示値に加える
        _ref_wheel_current[0] = fpu::clamp(ref_current[0] + ANTI_SLIP_GAIN * velocity_error[0], -MAX_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
        _ref_wheel_current[1] = fpu::clamp(ref_current[1] + ANTI_SLIP_GAIN * velocity_error[1], -MAX_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
        _ref_wheel_current[2] = fpu::clamp(ref_current[2] + ANTI_SLIP_GAIN * velocity_error[2], -MAX_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
        _ref_wheel_current[3] = fpu::clamp(ref_current[3] + ANTI_SLIP_GAIN * velocity_error[3], -MAX_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
#endif

        // 回生エネルギーを計算し電気ブレーキを掛ける
        for (int index = 0; index < 4; index++) {
            static constexpr float KV = MOTOR_SPEED_CONSTANT / WHEEL_CIRCUMFERENCE;
            float current = _ref_wheel_current(index);
            float power = (KV * motion.wheel_velocity(index) + MOTOR_RESISTANCE * current) * current;
            float energy = _regeneration_energy[index];
            float energy_with_brake, energy_without_brake;
            energy_without_brake = energy + (power + BASE_POWER_CONSUMPTION_PER_MOTOR) * (1.0f / IMU_OUTPUT_RATE);
            energy_without_brake = fpu::min(energy_without_brake, 0.0f);
            energy_with_brake = energy + BASE_POWER_CONSUMPTION_PER_MOTOR * (1.0f / IMU_OUTPUT_RATE);
            energy_with_brake = fpu::min(energy_with_brake, 0.0f);
            if (BRAKE_DISABLE_THRESHOLD < energy_without_brake) {
                brake_enabled[index] = false;
            }
            else if (energy_without_brake < BRAKE_ENABLE_THRESHOLD) {
                brake_enabled[index] = true;
            }
            _regeneration_energy[index] = brake_enabled[index] ? energy_with_brake : energy_without_brake;
        }

        // 電流指令値を設定する
        static constexpr float RECIPROCAL_CURRENT_SCALE = 1.0f / ADC1_CURRENT_SCALE;
        if (brake_enabled[0])
            VectorController::setBrakeEnabled(1);
        if (brake_enabled[1])
            VectorController::setBrakeEnabled(2);
        if (brake_enabled[2])
            VectorController::setBrakeEnabled(3);
        if (brake_enabled[3])
            VectorController::setBrakeEnabled(4);
        VectorController::setCurrentReferenceQ(1, static_cast<int>(_ref_wheel_current(0) * RECIPROCAL_CURRENT_SCALE));
        VectorController::setCurrentReferenceQ(2, static_cast<int>(_ref_wheel_current(1) * RECIPROCAL_CURRENT_SCALE));
        VectorController::setCurrentReferenceQ(3, static_cast<int>(_ref_wheel_current(2) * RECIPROCAL_CURRENT_SCALE));
        VectorController::setCurrentReferenceQ(4, static_cast<int>(_ref_wheel_current(3) * RECIPROCAL_CURRENT_SCALE));
        if (!brake_enabled[0])
            VectorController::clearBrakeEnabled(1);
        if (!brake_enabled[1])
            VectorController::clearBrakeEnabled(2);
        if (!brake_enabled[2])
            VectorController::clearBrakeEnabled(3);
        if (!brake_enabled[3])
            VectorController::clearBrakeEnabled(4);
    }
    else if (new_parameters && !sensor_only) {
        // 新しい指令値を受信したので次のループから制御を開始する
        startControl();
    }
    else {
        VectorController::setFault();
        initializeRegisters();
        initializeState();
    }
}

GravityFilter WheelController::_gravity_filter;
VelocityFilter WheelController::_velocity_filter;
Eigen::Vector4f WheelController::_last_velocity_error;
Eigen::Vector4f WheelController::_ref_body_accel;
Eigen::Vector4f WheelController::_ref_wheel_current;
Eigen::Vector4f WheelController::_regeneration_energy;
