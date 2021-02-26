#include "gamepad_thread.hpp"
#include <Windows.h>
#include <Xinput.h>
#include <string.h>
#undef min
#undef max
#include <algorithm>

#pragma comment(lib, "xinput.lib")

GamepadThread::GamepadThread(QObject *parent)
    : QThread(parent) {
}

GamepadThread::~GamepadThread() {
}

std::shared_ptr<GamepadThread::InputState_t> GamepadThread::inputState(int deviceId) {
    if ((0 <= deviceId) && (deviceId <= MAX_DEVICE_COUNT)) {
        return _InputStates[deviceId];
    }
    return std::shared_ptr<GamepadThread::InputState_t>();
}

void GamepadThread::vibrate(int deviceId, float power) {
    if ((0 <= deviceId) && (deviceId <= MAX_DEVICE_COUNT)) {
        _VibrationPowers[deviceId] = static_cast<int>(std::min(std::max(0.0f, power), 1.0f) * 65535);
    }
}

void GamepadThread::run(void) {
    while (!isInterruptionRequested()) {
        for (int index = 0; index < MAX_DEVICE_COUNT; index++) {
            XINPUT_STATE state;
            memset(&state, 0, sizeof(state));
            if (XInputGetState(index, &state) == ERROR_SUCCESS) {
                if (!_IsDeviceConnected[index] || (_LastPacketNumbers[index] != state.dwPacketNumber)) {
                    _LastPacketNumbers[index] = state.dwPacketNumber;
                    if (_IsDeviceConnected[index] == false) {
                        _VibrationPowers[index] = -1;
                    }
                    // 入力を読み取って格納する
                    auto input_state = std::make_shared<InputState_t>();
                    input_state->buttons = state.Gamepad.wButtons;
                    input_state->leftTrigger = state.Gamepad.bLeftTrigger / 255.0f;
                    input_state->rightTrigger = state.Gamepad.bRightTrigger / 255.0f;
                    input_state->leftStickX = state.Gamepad.sThumbLX / 32768.0f;
                    input_state->leftStickY = state.Gamepad.sThumbLY / 32768.0f;
                    input_state->rightStickX = state.Gamepad.sThumbRX / 32768.0f;
                    input_state->rightStickY = state.Gamepad.sThumbRY / 32768.0f;
                    applyTriggerDeadZone(input_state->leftTrigger, (float)XINPUT_GAMEPAD_TRIGGER_THRESHOLD / 255.0f);
                    applyTriggerDeadZone(input_state->rightTrigger, (float)XINPUT_GAMEPAD_TRIGGER_THRESHOLD / 255.0f);
                    applyStickDeadZone(input_state->leftStickX, input_state->leftStickY, (float)XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE / 32768.0f);
                    applyStickDeadZone(input_state->rightStickX, input_state->rightStickY, (float)XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE / 32768.0f);
                    _InputStates[index] = input_state;
                }
                if (_IsDeviceConnected[index] == false) {
                    _IsDeviceConnected[index] = true;
                    emit gamepadConnected(index);
                }

                // バイブレーションの値が変更されていれば反映する
                XINPUT_VIBRATION vibration_state;
                int vibration_power = _VibrationPowers[index].exchange(-1);
                if (0 <= vibration_power) {
                    vibration_state.wLeftMotorSpeed = vibration_power;
                    vibration_state.wRightMotorSpeed = vibration_power;
                    XInputSetState(index, &vibration_state);
                }
            } else {
                if (_IsDeviceConnected[index] == true) {
                    _InputStates[index].reset();
                    _IsDeviceConnected[index] = false;
                    emit gamepadDisconnected(index);
                }
            }
        }
        msleep(10);
    }
}

void GamepadThread::applyTriggerDeadZone(float &value, float dead_zone) {
    if (value < dead_zone) {
        value = 0.0f;
    } else {
        value = (value - dead_zone) / (1.0f - dead_zone);
    }
}

void GamepadThread::applyStickDeadZone(float &x_value, float &y_value, float dead_zone) {
    float mag = sqrtf(x_value * x_value + y_value * y_value);
    if (mag < dead_zone) {
        x_value = 0.0f;
        y_value = 0.0f;
    } else {
        float norm_mag = (std::min(mag, 1.0f) - dead_zone) / (1.0f - dead_zone);
        x_value = x_value * norm_mag / mag;
        y_value = y_value * norm_mag / mag;
    }
}
