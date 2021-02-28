#include <math.h> // math.h内のfmaxf,fminfを後でカスタム命令版に置き換えるため最初にincludeする
#include "wheel_controller.hpp"
#include <board.hpp>
#include <peripheral/vector_controller.hpp>
#include "shared_memory_manager.hpp"
#include "data_holder.hpp"

void WheelController::StartControl(void) {
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
    memset(_RegenerationEnergy, 0, sizeof(_RegenerationEnergy));
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

void WheelController::Update(bool new_parameters, bool brake_enabled) {
    // センサーデータを取得する
    auto &motion = DataHolder::GetMotionData();

    // 車輪速度をフィルタリングする
    float wheel_velocity[4];
    for (int index = 0; index < 4; index++) {
        wheel_velocity[index] = _LpfWheelVelocity[index](motion.Wheels[index].Velocity);
    }

    // カルマンフィルタにより加速度センサーと車輪速度から機体速度を求める
    {
        // 電流をベクトル合成してモーターが機体に与える力を計算する
        // モーターが車輪を回す力は小さいので無視する
        float accel_by_motor_x, accel_by_motor_y;
        {
            float i1 = motion.Wheels[0].CurrentQ;
            float i2 = motion.Wheels[1].CurrentQ;
            float i3 = motion.Wheels[2].CurrentQ;
            float i4 = motion.Wheels[3].CurrentQ;
            accel_by_motor_x = (i2 - i1 + i3 - i4) * MOTOR_TORQUE_CONSTANT / WHEEL_RADIUS * (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2)) / MACHINE_WEIGHT;
            accel_by_motor_y = (i1 + i2 - i3 - i4) * MOTOR_TORQUE_CONSTANT / WHEEL_RADIUS * (WHEEL_POS_X / sqrt(WHEEL_POS_R_2)) / MACHINE_WEIGHT;
        }

        // スリップとスタックを検知する
        float slip_stuck_x = fabsf(_LpfSlipStuck[0](accel_by_motor_x - motion.Imu.AccelX));
        float slip_stuck_y = fabsf(_LpfSlipStuck[1](accel_by_motor_y - motion.Imu.AccelY));

        // 車輪速度をベクトル合成して機体速度に変換する
        float velocity_by_motor_x = (wheel_velocity[1] - wheel_velocity[0] + wheel_velocity[2] - wheel_velocity[3]) * (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2)) / 4;
        float velocity_by_motor_y = (wheel_velocity[0] + wheel_velocity[1] - wheel_velocity[2] - wheel_velocity[3]) * (WHEEL_POS_X / sqrt(WHEEL_POS_R_2)) / 4;

        // 車輪速度から求めた機体速度の不確かさを求める
        // スリップ・スタックの度合が小さく車輪速度から求めた機体速度の絶対値が小さいほど不確かさが小さい
        static constexpr float WHEEL_VELOCITY_SIGMA_2 = 0.01f;
        float sigma_vx = velocity_by_motor_x * velocity_by_motor_x * slip_stuck_x * WHEEL_VELOCITY_SIGMA_2;
        float sigma_vy = velocity_by_motor_y * velocity_by_motor_y * slip_stuck_y * WHEEL_VELOCITY_SIGMA_2;

        // 信念分布を更新する
        static constexpr float ACCELEROMETER_SIGMA_2 = 0.1f;
        float SIGMA_hat_11 = _MachineVelocitySigma[0] + ACCELEROMETER_SIGMA_2 * (1.0f / IMU_OUTPUT_RATE / IMU_OUTPUT_RATE);
        float SIGMA_hat_22 = _MachineVelocitySigma[1] + ACCELEROMETER_SIGMA_2 * (1.0f / IMU_OUTPUT_RATE / IMU_OUTPUT_RATE);

        // カルマンゲインを計算する
        float K_11 = SIGMA_hat_11 / (sigma_vx + SIGMA_hat_11);
        float K_22 = SIGMA_hat_22 / (sigma_vy + SIGMA_hat_22);

        // 観測値を元に信念分布の分散を更新する
        _MachineVelocitySigma[0] = (1.0f - K_11) * SIGMA_hat_11;
        _MachineVelocitySigma[1] = (1.0f - K_22) * SIGMA_hat_22;

        // 観測値を元に信念分布の平均を更新する
        float mu_hat_1 = _MachineVelocity[0] + motion.Imu.AccelX * (1.0f / IMU_OUTPUT_RATE);
        _MachineVelocity[0] = K_11 * (velocity_by_motor_x - mu_hat_1) + mu_hat_1;
        float mu_hat_2 = _MachineVelocity[1] + motion.Imu.AccelY * (1.0f / IMU_OUTPUT_RATE);
        _MachineVelocity[1] = K_22 * (velocity_by_motor_y - mu_hat_2) + mu_hat_2;
        _MachineVelocity[2] = motion.Imu.GyroZ;
    }

    if (VectorController::IsFault() == false) {
        // 指令値を取得する
        auto &parameters = SharedMemory::GetParameters();
        float speed_gain_p = fmaxf(0.0f, parameters.speed_gain_p);
        float speed_gain_i = fmaxf(0.0f, parameters.speed_gain_i);
        float speed_x_ref = parameters.speed_x;
        float speed_y_ref = parameters.speed_y;
        float speed_r_ref = parameters.speed_omega * sqrt(WHEEL_POS_R_2);

        // 車輪のスリップを検知して電流制限値を求める
        {
            // 機体速度からあるべき車輪速度を求める
            float abs_velocity_diff[4];
            float vx = _MachineVelocity[0] * (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2));
            float vy = _MachineVelocity[1] * (WHEEL_POS_X / sqrt(WHEEL_POS_R_2));
            float vr = _MachineVelocity[2] * sqrt(WHEEL_POS_R_2);
            abs_velocity_diff[0] = fabsf(vy - vx + vr - wheel_velocity[0]);
            abs_velocity_diff[1] = fabsf(vx + vy + vr - wheel_velocity[1]);
            abs_velocity_diff[2] = fabsf(vx - vy + vr - wheel_velocity[2]);
            abs_velocity_diff[3] = fabsf(vr - vx - vy - wheel_velocity[3]);

            // あるべき車輪速度とのずれが大きい車輪はスリップしていると見なして電流制限値を減らす
            static constexpr float CURRENT_AVERAGING = 0.01f;
            static constexpr float GAIN_I = -0.01;
            float sum = 0.0f;
            for(int index = 0; index < 4; index++){
                float current_limit = fmaxf(_CurrentLimit[index] + GAIN_I * abs_velocity_diff[index] + CURRENT_AVERAGING, MIN_CURRENT_LIMIT_PER_MOTOR);
                _CurrentLimit[index] = current_limit;
                sum += current_limit;
            }
            float limit_factor = TOTAL_CURRENT_LIMIT / sum;
            for(int index = 0; index < 4; index++){
                _CurrentLimit[index] =  fminf(_CurrentLimit[index] * limit_factor, MAX_CURRENT_LIMIT_PER_MOTOR);
            }
        }

        // ベクトル分解して各車輪の速度指令値を求める
        float vx = speed_x_ref * (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2));
        float vy = speed_y_ref * (WHEEL_POS_X / sqrt(WHEEL_POS_R_2));
        float vr = speed_r_ref;
        _SpeedReference[0] = -vx + vy + vr;
        _SpeedReference[1] = +vx + vy + vr;
        _SpeedReference[2] = +vx - vy + vr;
        _SpeedReference[3] = -vx - vy + vr;

        // 速度指令値が異常でないことを確認する
        // 速度が速すぎるかNaNならspeed_ok==falseとなる
        bool speed_ok = false;
        if (fabsf(_SpeedReference[0]) <= MAX_SPEED_REFERENCE) {
            if (fabsf(_SpeedReference[1]) <= MAX_SPEED_REFERENCE) {
                if (fabsf(_SpeedReference[2]) <= MAX_SPEED_REFERENCE) {
                    if (fabsf(_SpeedReference[3]) <= MAX_SPEED_REFERENCE) {
                        speed_ok = true;
                    }
                }
            }
        }
        bool brake_enabled[4];
        brake_enabled[0] = VectorController::IsBrakeEnabled(1);
        brake_enabled[1] = VectorController::IsBrakeEnabled(2);
        brake_enabled[2] = VectorController::IsBrakeEnabled(3);
        brake_enabled[3] = VectorController::IsBrakeEnabled(4);
        if (speed_ok) {
            // 各モーターの速度制御を行う
            for (int index = 0; index < 4; index++) {
                // 速度のPI制御を行う(微分PI)
                float speed_ref = _SpeedReference[index];
                float speed_meas = motion.Wheels[index].Velocity;
                float error = speed_ref - speed_meas;
                float value_i = speed_gain_i * error;
                float value_p = speed_gain_p * (error - _LastSpeedError[index]);
                _LastSpeedError[index] = error;
                float current_ref = _CurrentReference[index];
                current_ref += value_p + value_i;

                // 指令速度、現在速度の両方が0に近いとき、電流指令値を0に減衰させる
                float v = fabsf(speed_meas) + fabsf(speed_ref) + 1.0f;
                float current_decay = 1.0f - 0.01f / (v * v);
                current_ref *= current_decay;

                // 電流制限をかける
                current_ref = fmaxf(-_CurrentLimit[index], fminf(current_ref, _CurrentLimit[index]));
                _CurrentReference[index] = current_ref;
            }

            // 回生エネルギーを計算する
            for (int index = 0; index < 4; index++) {
                static constexpr float KV = MOTOR_SPEED_CONSTANT / WHEEL_CIRCUMFERENCE;
                float speed_meas = motion.Wheels[index].Velocity;
                float current_ref = _CurrentReference[index];
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
                _LastSpeedError[index] = 0.0f;
                _RegenerationEnergy[index] = 0.0f;
                brake_enabled[index] = false;
            }
        }

        // 電流指令値を設定する
        static constexpr float RECIPROCAL_CURRENT_SCALE = 1.0f / ADC1_CURRENT_SCALE;
        VectorController::SetCurrentReferenceQ(1, static_cast<int>(_CurrentReference[0] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(2, static_cast<int>(_CurrentReference[1] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(3, static_cast<int>(_CurrentReference[2] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(4, static_cast<int>(_CurrentReference[3] * RECIPROCAL_CURRENT_SCALE));
        brake_enabled[0] ? VectorController::SetBrakeEnabled(1) : VectorController::ClearBrakeEnabled(1);
        brake_enabled[1] ? VectorController::SetBrakeEnabled(2) : VectorController::ClearBrakeEnabled(2);
        brake_enabled[2] ? VectorController::SetBrakeEnabled(3) : VectorController::ClearBrakeEnabled(3);
        brake_enabled[3] ? VectorController::SetBrakeEnabled(4) : VectorController::ClearBrakeEnabled(4);
    }
    else if (new_parameters == true) {
        StartControl();
    }
}

float WheelController::_SpeedReference[4];
float WheelController::_LastSpeedError[4];
float WheelController::_CurrentReference[4];
float WheelController::_RegenerationEnergy[4];
float WheelController::_CurrentLimit[4];
float WheelController::_MachineVelocity[3];
float WheelController::_MachineVelocitySigma[2];
Lpf2ndOrder50 WheelController::_LpfWheelVelocity[4];
Lpf2ndOrder200 WheelController::_LpfSlipStuck[2];
