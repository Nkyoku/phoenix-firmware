/**
 * @file vector_controller.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <system.h>

class VectorController;

struct VectorControllerStatus {
    friend class VectorController;

public:
    uint16_t status;

    int anyFault(void) {
        return ~status & 0xFFF0;
    }

    int overTemperatureFault(void) {
        // モーター1の障害フラグは0bit目、モーター2の障害フラグは1bit目といった順に格納して返す
        // 他の種類の障害のビット列も同様
        return (~status >> 12) & 0xF;
    }

    int overTemperatureFaultN(void) {
        return (status >> 12) & 0xF;
    }

    int overCurrentFault(void) {
        return (~status >> 8) & 0xF;
    }

    int overCurrentFaultN(void) {
        return (status >> 8) & 0xF;
    }

    int hallSensorFault(void) {
        return (~status >> 4) & 0xF;
    }

    int hallSensorFaultN(void) {
        return (status >> 4) & 0xF;
    }

    int EncoderFault(void) {
        return ~status & 0xF;
    }

    int EncoderFaultN(void) {
        return status & 0xF;
    }

private:
    VectorControllerStatus(int status) {
        status = status;
    }
};

class VectorController {
private:
    static constexpr uint32_t BASE = VECTOR_CONTROLLER_MASTER_0_BASE;

    struct Register_t {
        volatile uint16_t STATUS;
        volatile uint16_t INTFLAG;
        volatile uint16_t FAULT;
        volatile uint16_t POSITION;
        volatile int16_t ENCODER1;
        volatile int16_t ENCODER2;
        volatile int16_t ENCODER3;
        volatile int16_t ENCODER4;
        volatile int16_t IMEASD1;
        volatile int16_t IMEASQ1;
        volatile int16_t IMEASD2;
        volatile int16_t IMEASQ2;
        volatile int16_t IMEASD3;
        volatile int16_t IMEASQ3;
        volatile int16_t IMEASD4;
        volatile int16_t IMEASQ4;
        volatile int16_t IREFD1;
        volatile int16_t IREFQ1;
        volatile int16_t IREFD2;
        volatile int16_t IREFQ2;
        volatile int16_t IREFD3;
        volatile int16_t IREFQ3;
        volatile int16_t IREFD4;
        volatile int16_t IREFQ4;
        volatile uint16_t KP;
        volatile uint16_t KI;
    };

public:
    // 固定小数点数で表されるKP, KIの小数点の位置
    static constexpr int GainScale = 14;

    static VectorControllerStatus getStatus(void) {
        return VectorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->STATUS));
    }

    static VectorControllerStatus getInterruptFlag(void) {
        return VectorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->INTFLAG));
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

    static void setBrakeEnabled(int number) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x1 << (2 * number));
    }

    static void clearBrakeEnabled(int number) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x2 << (2 * number));
    }

    static bool isBrakeEnabled(int number) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->FAULT) & (0x1 << (2 * number));
    }

    static void setAllBrakeEnabled(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x154);
    }

    static void clearAllBrakeEnabled(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x2A8);
    }

    static int getPositionStatus(void) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->POSITION);
    }

    static int getEncoderValue(int number) {
        switch (number) {
        case 1:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER1);
        case 2:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER2);
        case 3:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER3);
        case 4:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER4);
        default:
            return 0;
        }
    }

    static int getCurrentMeasurementD(int number) {
        switch (number) {
        case 1:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD1);
        case 2:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD2);
        case 3:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD3);
        case 4:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD4);
        default:
            return 0;
        }
    }

    static int getCurrentMeasurementQ(int number) {
        switch (number) {
        case 1:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ1);
        case 2:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ2);
        case 3:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ3);
        case 4:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ4);
        default:
            return 0;
        }
    }

    static int getCurrentReferenceD(int number) {
        switch (number) {
        case 1:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD1);
        case 2:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD2);
        case 3:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD3);
        case 4:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD4);
        default:
            return 0;
        }
    }

    static int getCurrentReferenceQ(int number) {
        switch (number) {
        case 1:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ1);
        case 2:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ2);
        case 3:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ3);
        case 4:
            return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ4);
        default:
            return 0;
        }
    }

    static void setCurrentReferenceD(int number, int value) {
        switch (number) {
        case 1:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD1, value);
            break;
        case 2:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD2, value);
            break;
        case 3:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD3, value);
            break;
        case 4:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD4, value);
            break;
        default:
            break;
        }
    }

    static void setCurrentReferenceQ(int number, int value) {
        switch (number) {
        case 1:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ1, value);
            break;
        case 2:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ2, value);
            break;
        case 3:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ3, value);
            break;
        case 4:
            __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ4, value);
            break;
        default:
            break;
        }
    }

    static int getGainP(void) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->KP);
    }

    static void setGainP(int value) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->KP, value);
    }

    static int getGainI(void) {
        return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->KI);
    }

    static void setGainI(int value) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->KI, value);
    }
};
