/**
 * @file dribble_controller.cpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#include "dribble_controller.hpp"
#include "board.hpp"
#include "data_holder.hpp"
#include "centralized_monitor.hpp"
#include "shared_memory_manager.hpp"
#include <peripheral/motor_controller.hpp>
#include <status_flags.hpp>
#include <fpu.hpp>

/// 加速時の電圧ランプレート制限 [V/s]
static constexpr float ACCELERATION_RAMP_RATE_LIMIT = 500.0f;

/// 減速時の電圧ランプレート制限 [V/s]
static constexpr float DECELERATION_RAMP_RATE_LIMIT = 100.0f;

/// 過電流閾値 [A]
static constexpr float OVER_CURRENT_THRESHOLD = 1.0f;

void DribbleController::startControl(void) {
    MotorController::setPower(0);
    MotorController::clearFault();
}

void DribbleController::stopControl(void) {
    MotorController::setFault();
    MotorController::clearBrakeEnabled();
    MotorController::setPower(0);
}

void DribbleController::update(bool new_parameters, bool brake_enabled) {
    if (!MotorController::isFault()) {
        // 過電流を判定する
        if (OVER_CURRENT_THRESHOLD < DataHolder::adc2Data().dribble_current) {
            CentralizedMonitor::setErrorFlags(ErrorCauseMotor5OverCurrent);
            return;
        }

        // 指令値を取得する
        float ref_power;
        if (!brake_enabled) {
            MotorController::clearBrakeEnabled();
            if (fabsf(SharedMemoryManager::getParameters().dribble_power) <= 1.0f) {
                ref_power = SharedMemoryManager::getParameters().dribble_power;
            }
            else {
                ref_power = 0.0f; // 異常な値あるいはNaN
            }
        }
        else {
            MotorController::setBrakeEnabled();
            ref_power = 0.0f;
        }

        // 回転速度が急激に変化しないように変化率を制限する
        float prev_power = MotorController::getPower() * (1.0f / MotorController::FULL_SCALE_OF_POWER);
        static constexpr float ACCELERATION_RAMP_RATE_LIMIT_PER_PERIOD = ACCELERATION_RAMP_RATE_LIMIT / IMU_OUTPUT_RATE / 48.0f;
        static constexpr float DECELERATION_RAMP_RATE_LIMIT_PER_PERIOD = DECELERATION_RAMP_RATE_LIMIT / IMU_OUTPUT_RATE / 48.0f;
        static constexpr float MAX_POWER = static_cast<float>(MotorController::MAXIMUM_POWER) / MotorController::FULL_SCALE_OF_POWER;
        float upper_limit, lower_limit;
        if (0.0f <= prev_power) {
            upper_limit = fpu::min(prev_power + ACCELERATION_RAMP_RATE_LIMIT_PER_PERIOD, MAX_POWER);
            lower_limit = fpu::max(prev_power - DECELERATION_RAMP_RATE_LIMIT_PER_PERIOD, -MAX_POWER);
        }
        else {
            upper_limit = fpu::min(prev_power + DECELERATION_RAMP_RATE_LIMIT_PER_PERIOD, MAX_POWER);
            lower_limit = fpu::max(prev_power - ACCELERATION_RAMP_RATE_LIMIT_PER_PERIOD, -MAX_POWER);
        }
        ref_power = fpu::clamp(ref_power, lower_limit, upper_limit);
        float power = ref_power * MotorController::FULL_SCALE_OF_POWER;
        MotorController::setPower(static_cast<int>(power));
    }
    else if (new_parameters == true) {
        startControl();
    }
}
