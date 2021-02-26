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
    auto &motion_data = DataHolder::GetMotionData();

    // 加速度センサーをフィルタリングする
    // バタワース特性4次IIRフィルタ, カットオフ20Hz
    float accel[3];
    {
        static float accel_z1[3][2];
        static float accel_z2[3][2];
        for (int axis = 0; axis < 3; axis++) {
            // セクション1
            static constexpr float S1 = 0.00376220298169872977f;
            static constexpr float A21 = -1.89341560102250028f;
            static constexpr float A31 = 0.908464412949295252f;
            static constexpr float B21 = 2.0f;
            float temp1 = S1 * motion_data.Imu.Accel[axis] - A21 * accel_z1[axis][0] - A31 * accel_z1[axis][1];
            float sect1 = temp1 + B21 * accel_z1[axis][0] + accel_z1[axis][1];
            accel_z1[axis][1] = accel_z1[axis][0];
            accel_z1[axis][0] = temp1;

            // セクション2
            static constexpr float S2 = 0.00353349592337796892f;
            static constexpr float A22 = -1.77831348813943535f;
            static constexpr float A32 = 0.792447471832947059f;
            static constexpr float B22 = 2.0f;
            float temp2 = S2 * sect1 - A22 * accel_z2[axis][0] - A32 * accel_z2[axis][1];
            accel[axis] = temp2 + B22 * accel_z2[axis][0] + accel_z2[axis][1];
            accel_z2[axis][1] = accel_z2[axis][0];
            accel_z2[axis][0] = temp2;
        }
    }

    if (VectorController::IsFault() == false) {
        // 指令値を取得する
        auto &parameters = SharedMemory::GetParameters();
        float speed_gain_p = fmaxf(0.0f, parameters.speed_gain_p);
        float speed_gain_i = fmaxf(0.0f, parameters.speed_gain_i);
        float speed_x_ref = parameters.speed_x;
        float speed_y_ref = parameters.speed_y;
        float speed_r_ref = parameters.speed_omega * sqrt(WHEEL_POS_R2);

        // ベクトル合成して現在の車体の速度を求める
        /*float v1_meas = motion_data.Wheels[0].Velocity;
         float v2_meas = motion_data.Wheels[1].Velocity;
         float v3_meas = motion_data.Wheels[2].Velocity;
         float v4_meas = motion_data.Wheels[3].Velocity;
         float speed_x_meas = (-v1_meas + v2_meas + v3_meas - v4_meas) * (WHEEL_POS_Y / sqrt(WHEEL_POS_R2));
         float speed_y_meas = (+v1_meas + v2_meas - v3_meas - v4_meas) * (WHEEL_POS_X / sqrt(WHEEL_POS_R2));
         float speed_r_meas = (+v1_meas + v2_meas + v3_meas + v4_meas);*/

        // 車輪の荷重を計算し、荷重の掛かっているモーターにより多くの電流を割り当てる
        float current_limit[4];
        {
            float ax = accel[0] * (MACHINE_WEIGHT * CENTER_OF_GRAVITY_HEIGHT / (WHEEL_POS_X * 4));
            float ay = accel[1] * (MACHINE_WEIGHT * CENTER_OF_GRAVITY_HEIGHT / (WHEEL_POS_Y * 4));
            float az = accel[2] * MACHINE_WEIGHT;
            float weight[4];
            weight[0] = fmaxf(0.0f, az - ax - ay);
            weight[1] = fmaxf(0.0f, az - ax + ay);
            weight[2] = fmaxf(0.0f, az + ax + ay);
            weight[3] = fmaxf(0.0f, az + ax - ay);
            float total_weight = fabsf(weight[0]) + fabsf(weight[1]) + fabsf(weight[2]) + fabsf(weight[3]);
            float limit_coeffient = TOTAL_CURRENT_LIMIT / fmaxf(total_weight, 1e-3f);
            static constexpr float I_LIMIT_MAX = CURRENT_LIMIT_PER_MOTOR;
            static constexpr float I_LIMIT_MIN = TOTAL_CURRENT_LIMIT / 2 - CURRENT_LIMIT_PER_MOTOR;
            for (int index = 0; index < 4; index++) {
                current_limit[index] = fmaxf(I_LIMIT_MIN, fminf(weight[index] * limit_coeffient, I_LIMIT_MAX));
                _CurrentLimit[index] = current_limit[index];
            }
        }

        // ベクトル分解して各車輪の速度指令値を求める
        float vx = speed_x_ref * (WHEEL_POS_Y / sqrt(WHEEL_POS_R2));
        float vy = speed_y_ref * (WHEEL_POS_X / sqrt(WHEEL_POS_R2));
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
                float speed_meas = motion_data.Wheels[index].Velocity;
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
                current_ref = fmaxf(-current_limit[index], fminf(current_ref, current_limit[index]));
                _CurrentReference[index] = current_ref;
            }

            // 運動に寄与しない電流を打ち消す
            /*float current_decay = (_CurrentReference[0] - _CurrentReference[1] + _CurrentReference[2] - _CurrentReference[3]) * 0.01f;
             _CurrentReference[0] -= current_decay;
             _CurrentReference[1] += current_decay;
             _CurrentReference[2] -= current_decay;
             _CurrentReference[3] += current_decay;*/

            // 回生エネルギーを計算する
            for (int index = 0; index < 4; index++) {
                static constexpr float KV = MOTOR_SPEED_CONSTANT / WHEEL_CIRCUMFERENCE;
                float speed_meas = motion_data.Wheels[index].Velocity;
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
