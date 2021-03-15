#include "wheel_controller.hpp"
#include <peripheral/vector_controller.hpp>
#include "shared_memory_manager.hpp"

/// すべての車輪がグリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_0000[12] = {
    -0.25f, 0.25f, 0.25f,
    0.25f, 0.25f, 0.25f,
    0.25f, -0.25f, 0.25f,
    -0.25f, -0.25f, 0.25f
};

/// 車輪1がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_0001[12] = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.5f, 0.5f,
    0.5f, -0.5f, 0.0f,
    -0.5f, 0.0f, 0.5f
};

/// 車輪2がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_0010[12] = {
    0.0f, 0.5f, 0.5f,
    0.0f, 0.0f, 0.0f,
    0.5f, 0.0f, 0.5f,
    -0.5f, -0.5f, 0.0f
};

/// 車輪3がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_0100[12] = {
    -0.5f, 0.5f, 0.0f,
    0.5f, 0.0f, 0.5f,
    0.0f, 0.0f, 0.0f,
    0.0f, -0.5f, 0.5f
};

/// 車輪4がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_1000[12] = {
    -0.5f, 0.0f, 0.5f,
    0.5f, 0.5f, 0.0f,
    0.0f, -0.5f, 0.5f,
    0.0f, 0.0f, 0.0f
};

/// 車輪1,2がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_0011[12] = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.5f, 0.0f, 0.5f,
    -0.5f, 0.0f, 0.5f
};

/// 車輪2,3がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_0110[12] = {
    0.0f, 0.5f, 0.5f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, -0.5f, 0.5f
};

/// 車輪3,4がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_1100[12] = {
    -0.5f, 0.0f, 0.5f,
    0.5f, 0.0f, 0.5f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f
};

/// 車輪1,4がスリップしているときに使用される力分解行列
static constexpr float FORCE_MATRIX_1001[12] = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.5f, 0.5f,
    0.0f, -0.5f, 0.5f,
    0.0f, 0.0f, 0.0f
};

/// 車輪のスリップ状態と力分解行列の対応表
static const float * const FORCE_MATRIX_TABLE[16] = {
    FORCE_MATRIX_0000,
    FORCE_MATRIX_0001,
    FORCE_MATRIX_0010,
    FORCE_MATRIX_0011,
    FORCE_MATRIX_0100,
    FORCE_MATRIX_0000,
    FORCE_MATRIX_0110,
    FORCE_MATRIX_0000,
    FORCE_MATRIX_1000,
    FORCE_MATRIX_1001,
    FORCE_MATRIX_0000,
    FORCE_MATRIX_0000,
    FORCE_MATRIX_1100,
    FORCE_MATRIX_0000,
    FORCE_MATRIX_0000,
    FORCE_MATRIX_0000
};

void WheelController::StartControl(void) {
    static constexpr int CURRENT_CONTROL_GAIN_P = 3500; // 電流制御の比例ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_I = 500; // 電流制御の積分ゲイン
    VectorController::SetGainP(CURRENT_CONTROL_GAIN_P);
    VectorController::SetGainI(CURRENT_CONTROL_GAIN_I);
    VectorController::ClearFault();
    VectorController::SetCurrentReferenceQ(1, 0);
    VectorController::SetCurrentReferenceQ(2, 0);
    VectorController::SetCurrentReferenceQ(3, 0);
    VectorController::SetCurrentReferenceQ(4, 0);
    memset(_SpeedReference, 0, sizeof(_SpeedReference));
    memset(_LastSpeedError, 0, sizeof(_LastSpeedError));
    memset(_CurrentReference, 0, sizeof(_CurrentReference));
    memset(_CurrentReferenceOfWheel, 0, sizeof(_CurrentReferenceOfWheel));
    memset(_CurrentReferenceOfMachine, 0, sizeof(_CurrentReferenceOfMachine));
    memset(_RegenerationEnergy, 0, sizeof(_RegenerationEnergy));
    memset(_LastCompensationForce, 0, sizeof(_LastCompensationForce));
    _SlipFlags = 0;
}

void WheelController::StopControl(void) {
    VectorController::SetFault();
    VectorController::ClearAllBrakeEnabled();
    VectorController::SetCurrentReferenceQ(1, 0);
    VectorController::SetCurrentReferenceQ(2, 0);
    VectorController::SetCurrentReferenceQ(3, 0);
    VectorController::SetCurrentReferenceQ(4, 0);
    VectorController::SetGainP(0);
    VectorController::SetGainI(0);
}

void WheelController::Update(bool new_parameters, bool sensor_only) {
    // センサーデータを取得する
    auto &motion = DataHolder::GetMotionData();

    // 車輪速度をLPFでフィルタリングする
    // 機体速度の推定とスリップ検知に使用する
    WheelVelocity_t filtered_wheel_velocity;
    for (int index = 0; index < 4; index++) {
        filtered_wheel_velocity.V[index] = _LpfWheelVelocity[index](motion.Wheels[index].Velocity);
    }

    // 機体速度を推定する
    // 結果は_MachineVelocityに格納される
    EstimateMachineVelocity(filtered_wheel_velocity, motion.Imu);

    // スリップを検知する
    // 結果は_SlipIndicatorに格納される
    DetectSlip(filtered_wheel_velocity, _CurrentReference);

    // _SlipIndicatorの大きい上位2つの車輪を挙げる
    // 結果はslipped_wheelsに車輪1が1ビット目、車輪2が2ビット目・・・というように格納される
    {
        static constexpr float TRIP_THRESHOLD = 1.0f; // スリップ判定をONにする値
        static constexpr float RELEASE_THRESHOLD = 0.8f; // スリップ判定をOFFにする値
        static bool last_state[4];
        int slipped_wheels = 0;
        float biggest_one = fmaxf(fmaxf(_SlipIndicator[0], _SlipIndicator[1]), fmaxf(_SlipIndicator[2], _SlipIndicator[3]));
        slipped_wheels |= static_cast<int>(biggest_one == _SlipIndicator[0]);
        slipped_wheels |= static_cast<int>(biggest_one == _SlipIndicator[1]) << 1;
        slipped_wheels |= static_cast<int>(biggest_one == _SlipIndicator[2]) << 2;
        slipped_wheels |= static_cast<int>(biggest_one == _SlipIndicator[3]) << 3;
        float next_one = biggest_one;
        switch (slipped_wheels) {
        case 0x1:
            next_one = fmaxf(_SlipIndicator[1], fmaxf(_SlipIndicator[2], _SlipIndicator[3]));
            break;
        case 0x2:
            next_one = fmaxf(_SlipIndicator[0], fmaxf(_SlipIndicator[2], _SlipIndicator[3]));
            break;
        case 0x4:
            next_one = fmaxf(fmaxf(_SlipIndicator[0], _SlipIndicator[1]), _SlipIndicator[3]);
            break;
        case 0x8:
            next_one = fmaxf(fmaxf(_SlipIndicator[0], _SlipIndicator[1]), _SlipIndicator[2]);
            break;
        }
        for (int index = 0; index < 4; index++) {
            last_state[index] = (last_state[index] ? RELEASE_THRESHOLD : TRIP_THRESHOLD) < _SlipIndicator[index];
            if (last_state[index])
                slipped_wheels |= static_cast<int>(next_one == _SlipIndicator[index]) << index;
            else
                slipped_wheels &= ~(1 << index);
        }
        _SlipFlags = slipped_wheels;
    }

    // フォルト状態だったらモーター制御をしない
    if (VectorController::IsFault()) {
        if (new_parameters == true) {
            // 新しい指令値を受信したら次のループから制御を開始する
            StartControl();

            // 制御の内部情報を初期化する
            _MachineVelocityReference.Vx = 0.0f;
            _MachineVelocityReference.Vy = 0.0f;
            _MachineVelocityReference.Omega = 0.0f;
        }
        return;
    }

    // 指令値を取得する
    auto &parameters = SharedMemory::GetParameters();

    // 速度指令値が異常でないことを確認する
    // 速度が速すぎるかNaNならspeed_ok==falseとなる
    bool speed_ok = false;
    if (fabsf(parameters.speed_x) <= MAX_TRANSLATION_REFERENCE) {
        if (fabsf(parameters.speed_y) <= MAX_TRANSLATION_REFERENCE) {
            if (fabsf(parameters.speed_omega) <= MAX_OMEGA_REFERENCE) {
                speed_ok = true;
            }
        }
    }

    bool brake_enabled[4];
    brake_enabled[0] = VectorController::IsBrakeEnabled(1);
    brake_enabled[1] = VectorController::IsBrakeEnabled(2);
    brake_enabled[2] = VectorController::IsBrakeEnabled(3);
    brake_enabled[3] = VectorController::IsBrakeEnabled(4);

    if (speed_ok && !sensor_only) {
        // 指令値と指令値の変化を制限する
        {
            // 指令値を実現可能な速度に制限する
            MachineVelocity_t input = {parameters.speed_x, parameters.speed_y, parameters.speed_omega};
            float max_velocity = MaximumWheelVelocity(input);
            float voltage_limit = WHEEL_VELOCITY_LIMIT / fmaxf(max_velocity, WHEEL_VELOCITY_LIMIT);
            float total_current = SteadyCurrent(input);
            float current_limit = STEADY_CURRENT_LIMIT / fmaxf(total_current, STEADY_CURRENT_LIMIT);
            float limit = fminf(voltage_limit, sqrtf(current_limit));
            input.Vx *= limit;
            input.Vy *= limit;
            input.Omega *= limit;

            // 指令値の変化を制限する
            static constexpr float DELTA_VX = ACCELERATION_LIMIT_VX / IMU_OUTPUT_RATE;
            static constexpr float DELTA_VY = ACCELERATION_LIMIT_VY / IMU_OUTPUT_RATE;
            static constexpr float DELTA_OMEGA = ACCELERATION_LIMIT_OMEGA / IMU_OUTPUT_RATE;
            float dvx = fminf(fmaxf(input.Vx - _MachineVelocityReference.Vx, -DELTA_VX), DELTA_VX);
            float dvy = fminf(fmaxf(input.Vy - _MachineVelocityReference.Vy, -DELTA_VY), DELTA_VY);
            float omega = fminf(fmaxf(input.Omega, _MachineVelocityReference.Omega - DELTA_OMEGA), _MachineVelocityReference.Omega + DELTA_OMEGA);
            _MachineVelocityReference.Omega = omega;
            float dtheta = omega * (1.0f / IMU_OUTPUT_RATE);
            float vx = _MachineVelocityReference.Vx + _MachineVelocityReference.Vy * dtheta; // 本来は三角関数が必要だがcos(x)=1, sin(x)=xと近似している
            float vy = _MachineVelocityReference.Vy - _MachineVelocityReference.Vx * dtheta;
            _MachineVelocityReference.Vx = vx + dvx;
            _MachineVelocityReference.Vy = vy + dvy;
        }

        // 車輪速度の指令値を計算する
        WheelVelocity_t wheel_ref = VelocityVectorDecomposition(_MachineVelocityReference);
        _SpeedReference[0] = wheel_ref.V[0];
        _SpeedReference[1] = wheel_ref.V[1];
        _SpeedReference[2] = wheel_ref.V[2];
        _SpeedReference[3] = wheel_ref.V[3];

        // 機体速度と指令値の誤差から得られた車輪速度の補正量を計算する
        float compensation_gain_p = fmaxf(0.0f, parameters.compensation_gain_p);
        float compensation_gain_i = fmaxf(0.0f, parameters.compensation_gain_i);
        float current_usage_for_compensation;
        {
            static constexpr float VX_COMPENSATION_LIMIT = 0.2f; // X方向のずれを補正する最大値 [m/s]
            static constexpr float VY_COMPENSATION_LIMIT = 0.2f; // Y方向のずれを補正する最大値 [m/s]
            static constexpr float OMEGA_COMPENSATION_LIMIT = 1.0f; // 回転速度のずれを補正する最大値 [rad/s]
            static constexpr float TOTAL_CURRENT_LIMIT = 6.0f;
            static constexpr float CURRENT_LIMIT_PER_MOTOR = 3.0f;
            static constexpr float DECAY_MIN = 1.0f - 10.0f / IMU_OUTPUT_RATE; // 0.99f at 1kHz
            static constexpr int ignore_vx_table = 0b1110111011100000;
            static constexpr int ignore_vy_table = 0b1111110010101000;
            bool ignore_vx = (ignore_vx_table >> _SlipFlags) & 0x1;
            bool ignore_vy = (ignore_vy_table >> _SlipFlags) & 0x1;
            float decay = fminf(fabsf(_MachineVelocityReference.Vx) + fabsf(_MachineVelocityReference.Vy) + fabsf(_MachineVelocityReference.Omega) + DECAY_MIN, 1.0f); // 指令速度が0に近いときに電流を減衰させる
            float error_vx = ignore_vx ? 0.0f : (_MachineVelocityReference.Vx - _MachineVelocity.Vx);
            float error_vy = ignore_vy ? 0.0f : (_MachineVelocityReference.Vy - _MachineVelocity.Vy);
            float error_omega = _MachineVelocityReference.Omega - _MachineVelocity.Omega;
            MachineVelocity_t error;
            error.Vx = fminf(fmaxf(error_vx, -VX_COMPENSATION_LIMIT), VX_COMPENSATION_LIMIT);
            error.Vy = fminf(fmaxf(error_vy, -VY_COMPENSATION_LIMIT), VY_COMPENSATION_LIMIT);
            error.Omega = fminf(fmaxf(error_omega, -OMEGA_COMPENSATION_LIMIT), OMEGA_COMPENSATION_LIMIT);
            const float* force_matrix = FORCE_MATRIX_TABLE[_SlipFlags];
            float total_current = 0.0f;
            float max_current = 0.0f;
            for (int index = 0; index < 4; index++) {
                float force = force_matrix[3 * index] * error.Vx + force_matrix[3 * index + 1] * error.Vy + force_matrix[3 * index + 2] * error.Omega;
                float current = _CurrentReferenceOfMachine[index] * decay;
                if (_SlipFlags & (1 << index)){
                    current *= 0.9f;
                }
                current += compensation_gain_i * force + compensation_gain_p * (force - _LastCompensationForce[index]);
                _LastCompensationForce[index] = force;
                _CurrentReferenceOfMachine[index] = current;
                total_current += fabsf(current);
                max_current = fmaxf(max_current, fabsf(current));
            }
            float weight1 = fmaxf(total_current * (1.0f / TOTAL_CURRENT_LIMIT), 1.0f);
            float weight2 = max_current * (1.0f / CURRENT_LIMIT_PER_MOTOR);
            float weight = 1.0f / fmaxf(weight1, weight2);
            for (int index = 0; index < 4; index++) {
                _CurrentReferenceOfMachine[index] *= weight;
            }
            current_usage_for_compensation = total_current * weight;
        }

        // 電流制限値を計算する
        // 個々のモーターが加速に使用する電力を (TOTAL_CURRENT_LIMIT - current_usage_for_compensation) * DC48V_VOLTAGE / 4 [W]に制限する
        // 電流制限値は_CurrentLimitに格納される
        {
            float temp = (TOTAL_CURRENT_LIMIT - current_usage_for_compensation) * MOTOR_RESISTANCE * DC48V_VOLTAGE;
            for (int index = 0; index < 4; index++) {
                float emf_voltage = wheel_ref.V[index] * (MOTOR_SPEED_CONSTANT / WHEEL_CIRCUMFERENCE);
                float current = sqrtf(emf_voltage * emf_voltage + temp) - fabsf(emf_voltage) / (2.0f * MOTOR_RESISTANCE);
                _CurrentLimit[index] = fminf(fmaxf(current, MIN_CURRENT_LIMIT_PER_MOTOR), MAX_CURRENT_LIMIT_PER_MOTOR);
            }
        }

        // 各モーターの速度制御を行う
        float speed_gain_p = fmaxf(0.0f, parameters.speed_gain_p);
        float speed_gain_i = fmaxf(0.0f, parameters.speed_gain_i);
        for (int index = 0; index < 4; index++) {
            static constexpr float DECAY = 1.0f - 10.0f / IMU_OUTPUT_RATE; // 0.99f at 1kHz

            // 速度のPI制御を行う
            float speed_ref = wheel_ref.V[index];
            float speed_meas = motion.Wheels[index].Velocity;
            float speed_error_from_wheel = speed_ref - speed_meas;
            float weight = fminf(fabsf(speed_meas) + fabsf(speed_ref) + DECAY, 1.0f); // 指令速度と実際の速度がともに0に近いときに電流を減衰させる
            float value_i = speed_gain_i * speed_error_from_wheel;
            float value_p = speed_gain_p * (speed_error_from_wheel - _LastSpeedError[index]);
            _LastSpeedError[index] = speed_error_from_wheel;
            _CurrentReferenceOfWheel[index] = _CurrentReferenceOfWheel[index] * weight + value_p + value_i;
        }

        // 打ち消す方向に働く電流を徐々に減衰させる
        {
            static constexpr float CANCELLED_CURRENT_DECAY = 10.0f / IMU_OUTPUT_RATE; // 0.01f at 1kHz
            float cancel_current = (_CurrentReferenceOfWheel[0] - _CurrentReferenceOfWheel[1] + _CurrentReferenceOfWheel[2] - _CurrentReferenceOfWheel[3]) * CANCELLED_CURRENT_DECAY;
            _CurrentReferenceOfWheel[0] -= cancel_current;
            _CurrentReferenceOfWheel[1] += cancel_current;
            _CurrentReferenceOfWheel[2] -= cancel_current;
            _CurrentReferenceOfWheel[3] += cancel_current;
        }

        // 回生エネルギーを計算する
        // 同時に電流制限も掛ける
        for (int index = 0; index < 4; index++) {
            static constexpr float KV = MOTOR_SPEED_CONSTANT / WHEEL_CIRCUMFERENCE;
            float speed_meas = motion.Wheels[index].Velocity;
            float current_ref_of_wheel = fminf(fmaxf(_CurrentReferenceOfWheel[index], -_CurrentLimit[index]), _CurrentLimit[index]);
            _CurrentReferenceOfWheel[index] = current_ref_of_wheel;
            float current_ref_of_machine = _CurrentReferenceOfMachine[index];
            float current_ref = current_ref_of_wheel + current_ref_of_machine;
            _CurrentReference[index] = current_ref;
            float power = (KV * speed_meas + MOTOR_RESISTANCE * current_ref) * current_ref;
            float energy = _RegenerationEnergy[index];
            float energy_with_brake, energy_without_brake;
            energy_without_brake = energy + (power + BASE_POWER_CONSUMPTION_PER_MOTOR) * (1.0f / IMU_OUTPUT_RATE);
            energy_without_brake = fminf(energy_without_brake, 0.0f);
            energy_with_brake = energy + BASE_POWER_CONSUMPTION_PER_MOTOR * (1.0f / IMU_OUTPUT_RATE);
            energy_with_brake = fminf(energy_with_brake, 0.0f);
            if (BRAKE_DISABLE_THRESHOLD < energy_without_brake) {
                brake_enabled[index] = false;
            }
            else if (energy_without_brake < BRAKE_ENABLE_THRESHOLD) {
                brake_enabled[index] = true;
            }
            _RegenerationEnergy[index] = brake_enabled[index] ? energy_with_brake : energy_without_brake;
        }
    }
    else {
        for (int index = 0; index < 4; index++) {
            _CurrentReference[index] = 0.0f;
            _CurrentReferenceOfWheel[index] = 0.0f;
            _CurrentReferenceOfMachine[index] = 0.0f;
            _LastSpeedError[index] = 0.0f;
            _LastCompensationForce[index] = 0.0f;
            _RegenerationEnergy[index] = 0.0f;
            brake_enabled[index] = false;
        }
    }

    // 電流指令値を設定する
    static constexpr float RECIPROCAL_CURRENT_SCALE = 1.0f / ADC1_CURRENT_SCALE;
    if (brake_enabled[0])
        VectorController::SetBrakeEnabled(1);
    if (brake_enabled[1])
        VectorController::SetBrakeEnabled(2);
    if (brake_enabled[2])
        VectorController::SetBrakeEnabled(3);
    if (brake_enabled[3])
        VectorController::SetBrakeEnabled(4);
    VectorController::SetCurrentReferenceQ(1, static_cast<int>(_CurrentReference[0] * RECIPROCAL_CURRENT_SCALE));
    VectorController::SetCurrentReferenceQ(2, static_cast<int>(_CurrentReference[1] * RECIPROCAL_CURRENT_SCALE));
    VectorController::SetCurrentReferenceQ(3, static_cast<int>(_CurrentReference[2] * RECIPROCAL_CURRENT_SCALE));
    VectorController::SetCurrentReferenceQ(4, static_cast<int>(_CurrentReference[3] * RECIPROCAL_CURRENT_SCALE));
    if (!brake_enabled[0])
        VectorController::ClearBrakeEnabled(1);
    if (!brake_enabled[1])
        VectorController::ClearBrakeEnabled(2);
    if (!brake_enabled[2])
        VectorController::ClearBrakeEnabled(3);
    if (!brake_enabled[3])
        VectorController::ClearBrakeEnabled(4);
}

float WheelController::SteadyCurrent(const MachineVelocity_t &machine) {
    float ix = machine.Vy * machine.Omega * (MACHINE_WEIGHT * sqrt(WHEEL_POS_R_2) / WHEEL_POS_Y / 2 * WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT);
    float iy = machine.Vx * machine.Omega * (MACHINE_WEIGHT * sqrt(WHEEL_POS_R_2) / WHEEL_POS_X / 2 * WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT);
    return fabsf(ix + iy) + fabsf(ix - iy);
}

float WheelController::MaximumWheelVelocity(const MachineVelocity_t &machine) {
    WheelVelocity_t wheel = VelocityVectorDecomposition(machine);
    return fmaxf(fmaxf(fabsf(wheel.V1), fabsf(wheel.V2)), fmaxf(fabsf(wheel.V3), fabsf(wheel.V4)));
}

void WheelController::EstimateMachineVelocity(const WheelVelocity_t &wheel, const MotionData_t::Imu_t &imu) {
    static float slip_velocity_peak_hold = 0.0f;
    static float machine_velocity_sigma_vx = 0.0f;
    static float machine_velocity_sigma_vy = 0.0f;

    // スリップ分の車輪速度を検出する
    static constexpr float PEAK_HOLD_DECAY = 0.99f;
    static constexpr float SLIP_VELOCITY_MAX = 10.0f;
    static constexpr float SLIP_VELOCITY_BASE = 0.01f;
    static constexpr float SLIP_VELOCITY_SIGMA_2 = 0.01f;
    float omega_from_wheels = (wheel.V1 + wheel.V2 + wheel.V3 + wheel.V4) * (0.25f / sqrt(WHEEL_POS_R_2));
    float cancelled_velocity = fabsf(wheel.V1 - wheel.V2 + wheel.V3 - wheel.V4);
    float slip_velocity = fmaxf(fabsf(omega_from_wheels - imu.GyroZ), cancelled_velocity);
    float peak_slip_velocity = fminf(fmaxf(slip_velocity, slip_velocity_peak_hold * PEAK_HOLD_DECAY), SLIP_VELOCITY_MAX);
    slip_velocity_peak_hold = peak_slip_velocity;
    float sigma_vx_vy_2 = (peak_slip_velocity * peak_slip_velocity + SLIP_VELOCITY_BASE) * SLIP_VELOCITY_SIGMA_2;

    // 信念分布を更新する
    static constexpr float ACCELEROMETER_SIGMA_2 = 0.1f;
    float SIGMA_hat_11 = machine_velocity_sigma_vx + (ACCELEROMETER_SIGMA_2 / IMU_OUTPUT_RATE / IMU_OUTPUT_RATE);
    float SIGMA_hat_22 = machine_velocity_sigma_vy + (ACCELEROMETER_SIGMA_2 / IMU_OUTPUT_RATE / IMU_OUTPUT_RATE);

    // カルマンゲインを計算する
    float K_11 = SIGMA_hat_11 / (sigma_vx_vy_2 + SIGMA_hat_11);
    float K_22 = SIGMA_hat_22 / (sigma_vx_vy_2 + SIGMA_hat_22);

    // 観測値を元に信念分布の分散を更新する
    machine_velocity_sigma_vx = (1.0f - K_11) * SIGMA_hat_11;
    machine_velocity_sigma_vy = (1.0f - K_22) * SIGMA_hat_22;

    // 観測値を元に信念分布の平均を更新する
    float rotation = imu.GyroZ * (1.0f / IMU_OUTPUT_RATE);
    _MachineVelocity.Omega = imu.GyroZ;
    float mu_1 = _MachineVelocity.Vx + _MachineVelocity.Vy * rotation; // 本来は三角関数が必要だがcos(x)=1, sin(x)=xと近似している
    float mu_hat_1 = mu_1 + imu.AccelX * (1.0f / IMU_OUTPUT_RATE);
    float velocity_by_motor_x = (wheel.V2 - wheel.V1 + wheel.V3 - wheel.V4) * (sqrt(WHEEL_POS_R_2) / WHEEL_POS_Y / 4);
    float mu_2 = _MachineVelocity.Vy - _MachineVelocity.Vx * rotation;
    float mu_hat_2 = mu_2 + imu.AccelY * (1.0f / IMU_OUTPUT_RATE);
    float velocity_by_motor_y = (wheel.V1 + wheel.V2 - wheel.V3 - wheel.V4) * (sqrt(WHEEL_POS_R_2) / WHEEL_POS_X / 4);
    _MachineVelocity.Vx = K_11 * (velocity_by_motor_x - mu_hat_1) + mu_hat_1;
    _MachineVelocity.Vy = K_22 * (velocity_by_motor_y - mu_hat_2) + mu_hat_2;
}

void WheelController::DetectSlip(const WheelVelocity_t &wheel, const float *reference_current) {
    static constexpr float DECAY = 1.0f - 10.0f / IMU_OUTPUT_RATE; // 0.99 at 1kHz
    static constexpr float BASE_CURRENT = 1.0f;
    WheelVelocity_t wheel_from_imu = VelocityVectorDecomposition(_MachineVelocity);
    for (int index = 0; index < 4; index++) {
        float den = fmaxf(fabsf(reference_current[index]), BASE_CURRENT);
        _SlipIndicator[index] = fmaxf(fabsf(wheel.V[index] - wheel_from_imu.V[index]) / den, _SlipIndicator[index] * DECAY);
    }
}

int WheelController::_Timeout = 0;
MachineVelocity_t WheelController::_MachineVelocityReference;
float WheelController::_SpeedReference[4];
float WheelController::_LastSpeedError[4];
float WheelController::_CurrentReference[4];
float WheelController::_CurrentReferenceOfWheel[4];
float WheelController::_CurrentReferenceOfMachine[4];
float WheelController::_RegenerationEnergy[4];
float WheelController::_CurrentLimit[4];
float WheelController::_LastCompensationForce[4];
float WheelController::_SlipIndicator[4];
uint32_t WheelController::_SlipFlags;
MachineVelocity_t WheelController::_MachineVelocity;
Lpf2ndOrder50 WheelController::_LpfWheelVelocity[4];
