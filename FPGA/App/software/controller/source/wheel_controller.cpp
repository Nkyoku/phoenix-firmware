#include "wheel_controller.hpp"
#include <peripheral/vector_controller.hpp>
#include <status_flags.hpp>
#include <fpu.hpp>
#include "centralized_monitor.hpp"
#include "shared_memory_manager.hpp"

#define USE_SIMPLE_CONTROL 0

using namespace Eigen;

/// 並進速度指令値の最大値[m/s]
static constexpr float MAX_TRANSLATION_REFERENCE = 20.0f;

/// 回転速度指令の最大値[m/s]
static constexpr float MAX_OMEGA_REFERENCE = 20.0f;

/// 電流指令値の最大値[A]
static constexpr float MAX_CURRENT_LIMIT_PER_MOTOR = 4.0f;

/// 全てのモーターの電流指令値の絶対合計の最大値 [A]
static constexpr float TOTAL_CURRENT_LIMIT = 8.0f;

/// ある方向への電流制限値の最小値 [A]
/// 電流制限が掛けられたとしても最低でもこの数値の分の加速が可能となる
static constexpr float MINIMUM_ASSIGNED_CURRENT = 0.5f;

/// モータードライバのベース消費電力 [W]
static constexpr float BASE_POWER_CONSUMPTION_PER_MOTOR = 0.25f;

/// ブレーキを有効にする回生エネルギーの閾値
static constexpr float BRAKE_ENABLE_THRESHOLD = -0.01f;

/// ブレーキを無効にする回生エネルギーの閾値
static constexpr float BRAKE_DISABLE_THRESHOLD = -0.005f;

/// 過電流閾値[A]
static constexpr float OVER_CURRENT_THRESHOLD = 5.0f;

/// モーターの摩擦力に打ち勝つのに要する電流 [A]
static constexpr float FRICTION_CURRENT = 0.1f;

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

/**
 * @brief 車輪の駆動力ベクトルを車体の加速度ベクトルに変換する
 * @param wheel_force 車輪の駆動力ベクトル [N]
 * @return 車体の加速度ベクトル dx^2/dt^2 [m/s^2], dy^2/dt^2 [m/s^2], dω/dt [rad/s^2]
 */
static Eigen::Vector3f forceVectorComposition(const Eigen::Vector4f &wheel_force, float mass, float inertia) {
    Eigen::Vector3f body_acceleration;
    body_acceleration(0) = (wheel_force(1) - wheel_force(0) + wheel_force(2) - wheel_force(3)) * ((WHEEL_POS_Y / sqrt(WHEEL_POS_R_2)) / mass);
    body_acceleration(1) = (wheel_force(0) + wheel_force(1) - wheel_force(2) - wheel_force(3)) * ((WHEEL_POS_X / sqrt(WHEEL_POS_R_2)) / mass);
    body_acceleration(2) = (wheel_force(0) + wheel_force(1) + wheel_force(2) + wheel_force(3)) * (sqrt(WHEEL_POS_R_2) / inertia);
    return body_acceleration;
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
    _driving_force_observer[0].reset();
    _driving_force_observer[1].reset();
    _driving_force_observer[2].reset();
    _driving_force_observer[3].reset();
    _last_velocity_error = Vector4f::Zero();
    _ref_body_accel_unlimit = Vector4f::Zero();
    _ref_body_accel = Vector4f::Zero();
    _ref_wheel_current = Vector4f::Zero();
    _regeneration_energy = Vector4f::Zero();
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

    // 駆動力を推定する
    for (int index = 0; index < 4; index++) {
        _driving_force_observer[index].update(motion.wheel_velocity(index), motion.wheel_current_q(index), FRICTION_CURRENT);
    }
    Vector4f wheel_force = {
        _driving_force_observer[0].drivingForce(),
        _driving_force_observer[1].drivingForce(),
        _driving_force_observer[2].drivingForce(),
        _driving_force_observer[3].drivingForce(),
    };
    Vector3f body_acceleration_by_wheels = forceVectorComposition(wheel_force, MACHINE_WEIGHT, MACHINE_INERTIA);

    // 車輪速度を取得し車体速度に換算する
    Vector4f wheel_velocity = motion.wheel_velocity;
    Vector4f body_velocity_by_wheels = velocityVectorComposition(wheel_velocity);

    // 速度指令値が異常でないことを確認する
    // 速度が速すぎるかNaNならspeed_ok==falseとなる
    auto &parameters = SharedMemoryManager::getParameters();
    bool speed_ok = false;
    if (fabsf(parameters.speed_x) <= MAX_TRANSLATION_REFERENCE) {
        if (fabsf(parameters.speed_y) <= MAX_TRANSLATION_REFERENCE) {
            if (fabsf(parameters.speed_omega) <= MAX_OMEGA_REFERENCE) {
                speed_ok = true;
            }
        }
    }

    // 車体速度を推定する
    _gravity_filter.update(motion.accelerometer, motion.gyroscope);
    _velocity_filter.update(bodyAcceleration(), motion.gyroscope, body_velocity_by_wheels, body_acceleration_by_wheels);

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
            _ref_wheel_current(index) = fpu::clamp(current, -TOTAL_CURRENT_LIMIT / 4, TOTAL_CURRENT_LIMIT / 4);
            _last_velocity_error(index) = error;
        }
#else
        // 車体加速度の指令値を求める
        Vector4f body_velocity = {bodyVelocity()(0), bodyVelocity()(1), bodyVelocity()(2), body_velocity_by_wheels(3)};
        for (int index = 0; index < 4; index++) {
            float error = ref_body_velocity(index) - body_velocity(index);
            float p_gain = parameters.speed_gain_p[index];
            float i_gain = parameters.speed_gain_i[index];
            _ref_body_accel_unlimit(index) = _ref_body_accel(index) + p_gain * (error - _last_velocity_error(index)) + i_gain * error;
            _last_velocity_error(index) = error;
        }

        // 加速度を制限する
        // 角速度誤差が大きいほどX,Yの最大加速度を小さくする
        float max_accel = fpu::max(1.0f, fpu::min(10.0f, 10.0f - fabsf(ref_body_velocity(2) - motion.gyroscope(2)) * 10.0f));
        limitAcceleration(max_accel);
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

static const Vector4f LIMIT_ACCEL_K{
    WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT * MACHINE_WEIGHT * sqrt(WHEEL_POS_R_2) / WHEEL_POS_Y / 4,
    WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT *MACHINE_WEIGHT *sqrt(WHEEL_POS_R_2) / WHEEL_POS_X / 4,
    WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT *MACHINE_INERTIA / sqrt(WHEEL_POS_R_2) / 4,
    WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT,
};

static const Vector4f LIMIT_ACCEL_INV_K{
    1.0f / LIMIT_ACCEL_K(0),
    1.0f / LIMIT_ACCEL_K(1),
    1.0f / LIMIT_ACCEL_K(2),
    1.0f / LIMIT_ACCEL_K(3),
};

void WheelController::limitAcceleration(float max_accel) {
    _ref_wheel_current = Vector4f::Zero();
    _ref_body_accel = Vector4f::Zero();

    // C, ω, Y, Xの順に電流制限を行う
    for (int index = 4; 0 <= --index;) {
        float current_limit = (TOTAL_CURRENT_LIMIT - _ref_wheel_current.cwiseAbs().sum() - MINIMUM_ASSIGNED_CURRENT * index) * 0.25f;

        float accel = _ref_body_accel_unlimit(index);
        if (index <= 1) {
            accel = fpu::max(-max_accel, fpu::min(accel, max_accel));
        }

        float abs_accel = fpu::min(fabsf(LIMIT_ACCEL_K(index) * accel), current_limit) * LIMIT_ACCEL_INV_K(index);
        _ref_body_accel(index) = (0.0f <= accel) ? abs_accel : -abs_accel;
        Vector4f current = LIMIT_ACCEL_K.cwiseProduct(_ref_body_accel);
        _ref_wheel_current(0) = current(2) - current(0) + current(1) + current(3);
        _ref_wheel_current(1) = current(2) + current(0) + current(1) - current(3);
        _ref_wheel_current(2) = current(2) + current(0) - current(1) + current(3);
        _ref_wheel_current(3) = current(2) - current(0) - current(1) - current(3);
    }

    // 電流を個別に制限する
    for (int index = 0; index < 4; index++) {
        _ref_wheel_current(index) = fpu::clamp(_ref_wheel_current(index), -MAX_CURRENT_LIMIT_PER_MOTOR, MAX_CURRENT_LIMIT_PER_MOTOR);
    }
}

GravityFilter WheelController::_gravity_filter;
VelocityFilter WheelController::_velocity_filter;
DrivingForceObserver WheelController::_driving_force_observer[4];
Eigen::Vector4f WheelController::_last_velocity_error;
Eigen::Vector4f WheelController::_ref_body_accel_unlimit;
Eigen::Vector4f WheelController::_ref_body_accel;
Eigen::Vector4f WheelController::_ref_wheel_current;
Eigen::Vector4f WheelController::_regeneration_energy;
