/**
 * @file load_switch.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <altera_avalon_pio_regs.h>
#include <stdint.h>
#include <system.h>
#include "pin_name.hpp"

class LoadSwitch;

class LoadSwitchFault {
    friend class LoadSwitch;

public:
    uint32_t getBits(void) {
        return _bits & (Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault | Pio1Motor1SwitchFault | Pio1ModuleSleep | Pio1FpgaStop);
    }

    // いずれかのロードスイッチのfault出力がアサートされているときtrueを返す
    // MOD_SLEEP, FPGA_STOPは含まない
    bool anyFault(void) {
        return (_bits & (Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault | Pio1Motor1SwitchFault)) ? true : false;
    }

    bool motor1Fault(void) {
        return (_bits & Pio1Motor1SwitchFault) ? true : false;
    }

    bool motor2Fault(void) {
        return (_bits & Pio1Motor2SwitchFault) ? true : false;
    }

    bool motor3Fault(void) {
        return (_bits & Pio1Motor3SwitchFault) ? true : false;
    }

    bool motor4Fault(void) {
        return (_bits & Pio1Motor4SwitchFault) ? true : false;
    }

    bool motor5Fault(void) {
        return (_bits & Pio1Motor5SwitchFault) ? true : false;
    }

    bool moduleSleep(void) {
        return (_bits & Pio1ModuleSleep) ? true : false;  // モーターと関係ないがついでにこのクラスで取得する
    }

    bool fpgaStop(void) {
        return (_bits & Pio1FpgaStop) ? true : false;  // モーターと関係ないがついでにこのクラスで取得する
    }

private:
    LoadSwitchFault(uint32_t bits) :
        _bits(bits) {
    }

    uint32_t _bits;
};

// ホールセンサーやエンコーダに電源を供給するロードスイッチを制御するクラス
class LoadSwitch {
public:
    static void setMotor1Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor1SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor1SwitchEnable);
        }
    }

    static void setMotor2Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor2SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor2SwitchEnable);
        }
    }

    static void setMotor3Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor3SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor3SwitchEnable);
        }
    }

    static void setMotor4Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor4SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor4SwitchEnable);
        }
    }

    static void setMotor5Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable);
        }
    }

    static void setAllOff(void) {
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable | Pio2Motor4SwitchEnable | Pio2Motor3SwitchEnable | Pio2Motor2SwitchEnable | Pio2Motor1SwitchEnable);
    }

    static void setAllOn(void) {
        IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable | Pio2Motor4SwitchEnable | Pio2Motor3SwitchEnable | Pio2Motor2SwitchEnable | Pio2Motor1SwitchEnable);
    }

    static LoadSwitchFault getFault(bool ignore_masked_bits) {
        if (ignore_masked_bits == false) {
            return LoadSwitchFault(IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE));
        }
        else {
            return LoadSwitchFault(IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & IORD_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE));
        }
    }
};
