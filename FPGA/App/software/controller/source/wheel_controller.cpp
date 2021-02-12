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
    memset(_LastSpeedError, 0, sizeof(_LastSpeedError));
    memset(_LastCurrentReference, 0, sizeof(_LastCurrentReference));
}

void WheelController::StopControl(void) {
    VectorController::SetFault();
    VectorController::SetCurrentReferenceQ(1, 0);
    VectorController::SetCurrentReferenceQ(2, 0);
    VectorController::SetCurrentReferenceQ(3, 0);
    VectorController::SetCurrentReferenceQ(4, 0);
    VectorController::SetGainP(0);
    VectorController::SetGainI(0);
}

void WheelController::Update(bool new_parameters) {
    if (VectorController::IsFault() == false) {
        auto &motion_data = DataHolder::GetMotionData();
        auto &parameters = SharedMemory::GetParameters();
        float speed_gain_p = fmaxf(0.0f, parameters.speed_gain_p);
        float speed_gain_i = fmaxf(0.0f, parameters.speed_gain_i);
        for (int index = 0; index < 4; index++) {
            float speed_ref = parameters.wheel_speed[index];
            if (fabsf(speed_ref) <= MaxSpeedReference) {
                float speed_meas = motion_data.Wheels[index].Velocity;
                float error = speed_ref - speed_meas;
                float value_i = speed_gain_i * error;
                float value_p = speed_gain_p * (error - _LastSpeedError[index]);
                _LastSpeedError[index] = error;

                float current_ref = _LastCurrentReference[index];
                current_ref += value_p + value_i;
                current_ref = fmaxf(-MaxCurrentReference, fminf(current_ref, MaxCurrentReference));
                _LastCurrentReference[index] = current_ref;
            }
            else {
                _LastCurrentReference[index] = 0.0f;
                _LastSpeedError[index] = 0.0f;
            }
        }
        static constexpr float RECIPROCAL_CURRENT_SCALE = 1.0f / ADC1_CURRENT_SCALE;
        VectorController::SetCurrentReferenceQ(1, static_cast<int>(_LastCurrentReference[0] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(2, static_cast<int>(_LastCurrentReference[1] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(3, static_cast<int>(_LastCurrentReference[2] * RECIPROCAL_CURRENT_SCALE));
        VectorController::SetCurrentReferenceQ(4, static_cast<int>(_LastCurrentReference[3] * RECIPROCAL_CURRENT_SCALE));
    }
    else if (new_parameters == true) {
        StartControl();
    }
}

float WheelController::_LastSpeedError[4];
float WheelController::_LastCurrentReference[4];
