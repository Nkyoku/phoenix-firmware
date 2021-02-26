#pragma once

#include <QtCore/QThread>
#include <array>
#include <atomic>
#include <memory>
#include <stdint.h>

class GamepadThread : public QThread {
    Q_OBJECT

public:
    struct InputState_t {
        uint16_t buttons;
        float leftTrigger;
        float rightTrigger;
        float leftStickX;
        float leftStickY;
        float rightStickX;
        float rightStickY;
    };

    GamepadThread(QObject *parent = nullptr);

    virtual ~GamepadThread();

    std::shared_ptr<InputState_t> inputState(int deviceId);

    Q_SLOT void vibrate(int deviceId, float power);

    Q_SIGNAL void gamepadConnected(int deviceId);

    Q_SIGNAL void gamepadDisconnected(int deviceId);

private:
    void run(void) override;

    static void applyTriggerDeadZone(float &value, float dead_zone);

    static void applyStickDeadZone(float &x_value, float &y_value, float dead_zone);

    static constexpr int MAX_DEVICE_COUNT = 4;

    std::array<bool, MAX_DEVICE_COUNT> _IsDeviceConnected = {};

    std::array<uint32_t, MAX_DEVICE_COUNT> _LastPacketNumbers = {};

    std::array<std::shared_ptr<InputState_t>, MAX_DEVICE_COUNT> _InputStates;

    std::array<std::atomic<int>, MAX_DEVICE_COUNT> _VibrationPowers = {};
};
