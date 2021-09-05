#pragma once

#include <stdint.h>
#include <system.h>

struct MotorControllerStatus {
    friend class MotorController;

public:
    uint16_t status;

    int anyFault(void) {
        return ~status & 0x7;
    }

    int overTemperatureFault(void) {
        return (~status & 0x4) >> 2;
    }

    int overTemperatureFaultN(void) {
        return (status & 0x4) >> 2;
    }

    int overCurrentFault(void) {
        return (~status & 0x2) >> 1;
    }

    int overCurrentFaultN(void) {
        return (status & 0x2) >> 1;
    }

    int hallSensorFault(void) {
        return ~status & 0x1;
    }

    int hallSensorFaultN(void) {
        return status & 0x1;
    }

private:
    MotorControllerStatus(int status_) {
        status = status_;
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

    static MotorControllerStatus getStatus(void) {
        return MotorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->STATUS));
    }

    static MotorControllerStatus getInterruptFlag(void) {
        return MotorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->INTFLAG));
    }

    static void setFault(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x1);
    }

    static void clearFault(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x2);
    }

    static void resetFault(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x3);
    }

    static bool isFault(void) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->FAULT) & 0x1;
    }

    static void setBrakeEnabled(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x4);
    }

    static void clearBrakeEnabled(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x8);
    }

    static bool isBrakeEnabled(void) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->FAULT) & 0x4;
    }

    static int getPower(void) {
        return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->POWER);
    }

    static void setPower(int value) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->POWER, value);
    }
};
