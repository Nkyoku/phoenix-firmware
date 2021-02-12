#include <math.h> // math.h内のfmaxf,fminfを後でカスタム命令版に置き換えるため最初にincludeする
#include "dribble_controller.hpp"
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
    MotorController::SetPower(0);
}

void DribbleController::Update(bool new_parameters) {
    float dribble_power = SharedMemory::GetParameters().dribble_power;
    if (MotorController::IsFault() == false) {
        if (fabsf(dribble_power) <= 1.0f) {
            int previous_power = MotorController::GetPower();
            float upper_limit = fminf(previous_power + SLEWRATE_LIMIT, MotorController::MAXIMUM_POWER);
            float lower_limit = fmaxf(previous_power - SLEWRATE_LIMIT, -MotorController::MAXIMUM_POWER);
            float power = fmaxf(lower_limit, fminf(dribble_power * MotorController::FULL_SCALE_OF_POWER, upper_limit));
            MotorController::SetPower(static_cast<int>(power));
        }
        else {
            StopControl();
        }
    }
    else if (new_parameters == true) {
        StartControl();
    }
}
