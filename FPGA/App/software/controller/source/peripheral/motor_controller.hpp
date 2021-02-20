#pragma once

#include <stdint.h>
#include <system.h>

class MotorControllerStatus {
    friend class MotorController;

public:
    uint16_t Status;

    int AnyFault(void) {
        return ~Status & 0x7;
    }

    int OverTemperatureFault(void) {
        return (~Status & 0x4) >> 2;
    }

    int OverTemperatureFaultN(void) {
        return (Status & 0x4) >> 2;
    }

    int OverCurrentFault(void) {
        return (~Status & 0x2) >> 1;
    }

    int OverCurrentFaultN(void) {
        return (Status & 0x2) >> 1;
    }

    int HallSensorFault(void) {
        return ~Status & 0x1;
    }

    int HallSensorFaultN(void) {
        return Status & 0x1;
    }

private:
    MotorControllerStatus(int status) {
        Status = status;
    }
};

class MotorController {
private:
    static constexpr uint32_t BASE = MOTOR_CONTROLLER_5_BASE;

    struct Register_t {
        volatile uint16_t STATUS;
        volatile uint16_t INTFLAG;
        volatile uint16_t FAULT;
        volatile int16_t POWER;
    };

public:
    /// デューティ比が1となるときのPOWERレジスタの値
    static constexpr int FULL_SCALE_OF_POWER = 3000;

    /// POWERレジスタに設定可能な最大値の絶対値
    static constexpr int MAXIMUM_POWER = 2985;

    static MotorControllerStatus GetStatus(void) {
        return MotorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->STATUS));
    }

    static MotorControllerStatus GetInterruptFlag(void) {
        return MotorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->INTFLAG));
    }

    static void SetFault(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x1);
    }

    static void ClearFault(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x2);
    }

    static void ResetFault(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x3);
    }

    static bool IsFault(void) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->FAULT) & 0x1;
    }

    static void SetBrakeEnabled(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x4);
    }

    static void ClearBrakeEnabled(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x8);
    }

    static bool IsBrakeEnabled(void) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->FAULT) & 0x4;
    }

    static int GetPower(void) {
        return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->POWER);
    }

    static void SetPower(int value) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->POWER, value);
    }
};
