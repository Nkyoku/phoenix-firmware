#include <math.h> // math.h内のfmaxf,fminfを後でカスタム命令版に置き換えるため最初にincludeする
#include "dribble_controller.hpp"
#include "board.hpp"
#include <driver/adc2.hpp>
#include <peripheral/motor_controller.hpp>
#include "centralized_monitor.hpp"
#include "shared_memory_manager.hpp"

void DribbleController::StartControl(void) {
    MotorController::SetPower(0);
    MotorController::ClearFault();
}

void DribbleController::StopControl(void) {
    MotorController::SetFault();
    MotorController::ClearBrakeEnabled();
    MotorController::SetPower(0);
}

void DribbleController::Update(bool new_parameters, bool brake_enabled) {
    if (!MotorController::IsFault()) {
        float ref_power;
        if (!brake_enabled) {
            MotorController::ClearBrakeEnabled();
            if (fabsf(SharedMemory::GetParameters().dribble_power) <= 1.0f) {
                ref_power = SharedMemory::GetParameters().dribble_power;
            }
            else {
                ref_power = 0.0f; // 異常な値あるいはNaN
            }
        }
        else {
            MotorController::SetBrakeEnabled();
            ref_power = 0.0f;
        }
        float prev_power = MotorController::GetPower() * (1.0f / MotorController::FULL_SCALE_OF_POWER);
        static constexpr float ACCELERATION_RAMP_RATE_LIMIT_PER_PERIOD = ACCELERATION_RAMP_RATE_LIMIT / IMU_OUTPUT_RATE / 48.0f;
        static constexpr float DECELERATION_RAMP_RATE_LIMIT_PER_PERIOD = DECELERATION_RAMP_RATE_LIMIT / IMU_OUTPUT_RATE / 48.0f;
        static constexpr float MAX_POWER = static_cast<float>(MotorController::MAXIMUM_POWER) / MotorController::FULL_SCALE_OF_POWER;
        float upper_limit, lower_limit;
        if (0.0f <= prev_power) {
            upper_limit = fminf(prev_power + ACCELERATION_RAMP_RATE_LIMIT_PER_PERIOD, MAX_POWER);
            lower_limit = fmaxf(prev_power - DECELERATION_RAMP_RATE_LIMIT_PER_PERIOD, -MAX_POWER);
        }
        else {
            upper_limit = fminf(prev_power + DECELERATION_RAMP_RATE_LIMIT_PER_PERIOD, MAX_POWER);
            lower_limit = fmaxf(prev_power - ACCELERATION_RAMP_RATE_LIMIT_PER_PERIOD, -MAX_POWER);
        }
        ref_power = fminf(fmaxf(lower_limit, ref_power), upper_limit);
        float power = ref_power * MotorController::FULL_SCALE_OF_POWER;
        MotorController::SetPower(static_cast<int>(power));
    }
    else if (new_parameters == true) {
        StartControl();
    }
}
