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
    if (VectorController::IsFault() == false) {
        // センサーデータと指令値を取得する
        auto &motion_data = DataHolder::GetMotionData();
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
        if (speed_ok) {
            // 各モーターの速度制御を行う
            for (int index = 0; index < 4; index++) {
                float speed_ref = _SpeedReference[index];
                float speed_meas = motion_data.Wheels[index].Velocity;

                bool is_brake_enabled;
                if (VectorController::IsBrakeEnabled(1 + index)) {
                    is_brake_enabled = true;
                    if (0.0f <= speed_meas) {
                        if ((speed_meas - speed_ref) <= BRAKE_DISABLE_THRESHOLD) {
                            is_brake_enabled = false;
                        }
                    }
                    else {
                        if ((speed_ref - speed_meas) <= BRAKE_DISABLE_THRESHOLD) {
                            is_brake_enabled = false;
                        }
                    }
                    if (!is_brake_enabled) {
                        VectorController::ClearBrakeEnabled(1 + index);
                    }
                }
                else {
                    is_brake_enabled = false;
                    if (BRAKE_ENABLE_THRESHOLD <= fabsf(speed_meas)){
                        if (0.0f <= speed_meas) {
                            if (BRAKE_ENABLE_THRESHOLD <= (speed_meas - speed_ref)) {
                                is_brake_enabled = true;
                            }
                        }
                        else {
                            if (BRAKE_ENABLE_THRESHOLD <= (speed_ref - speed_meas)) {
                                is_brake_enabled = true;
                            }
                        }
                    }
                    if (is_brake_enabled) {
                        VectorController::SetBrakeEnabled(1 + index);
                    }
                }

                float error = speed_ref - speed_meas;
                float value_i = speed_gain_i * error;
                float value_p = speed_gain_p * (error - _LastSpeedError[index]);
                _LastSpeedError[index] = error;
                if (is_brake_enabled) {
                    _CurrentReference[index] = 0.0f;
                }
                else {
                    float current_ref = _CurrentReference[index];
                    current_ref += value_p + value_i;
                    current_ref = fmaxf(-CURRENT_LIMIT, fminf(current_ref, CURRENT_LIMIT));
                    _CurrentReference[index] = current_ref;
                }
            }
        }
        else {
            for (int index = 0; index < 4; index++) {
                _CurrentReference[index] = 0.0f;
                _LastSpeedError[index] = 0.0f;
            }
        }

        // 電流指令値を設定する
        static constexpr float RECIPROCAL_CURRENT_SCALE = 1.0f / ADC1_CURRENT_SCALE;
        VectorController::SetCurrentReferenceQ(1, static_cast<int>(_CurrentReference[0] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(2, static_cast<int>(_CurrentReference[1] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(3, static_cast<int>(_CurrentReference[2] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(4, static_cast<int>(_CurrentReference[3] * RECIPROCAL_CURRENT_SCALE));
    }
    else if (new_parameters == true) {
        StartControl();
    }
}

float WheelController::_SpeedReference[4];
float WheelController::_LastSpeedError[4];
float WheelController::_CurrentReference[4];
