#pragma once

#include <altera_avalon_pio_regs.h>
#include <stdint.h>
#include <system.h>
#include "pin_name.hpp"

class LoadSwitch;

class LoadSwitchFault {
    friend class LoadSwitch;

public:
    uint32_t Bits(void) {
        return _Bits & (Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault | Pio1Motor1SwitchFault | Pio1ModuleSleep | Pio1FpgaStop);
    }

    // いずれかのロードスイッチのfault出力がアサートされているときtrueを返す
    // MOD_SLEEP, FPGA_STOPは含まない
    bool AnyFault(void) {
        return (_Bits & (Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault | Pio1Motor1SwitchFault)) ? true : false;
    }

    bool Motor1Fault(void) {
        return (_Bits & Pio1Motor1SwitchFault) ? true : false;
    }

    bool Motor2Fault(void) {
        return (_Bits & Pio1Motor2SwitchFault) ? true : false;
    }

    bool Motor3Fault(void) {
        return (_Bits & Pio1Motor3SwitchFault) ? true : false;
    }

    bool Motor4Fault(void) {
        return (_Bits & Pio1Motor4SwitchFault) ? true : false;
    }

    bool Motor5Fault(void) {
        return (_Bits & Pio1Motor5SwitchFault) ? true : false;
    }

    bool ModuleSleep(void) {
        return (_Bits & Pio1ModuleSleep) ? true : false;  // モーターと関係ないがついでにこのクラスで取得する
    }

    bool FpgaStop(void) {
        return (_Bits & Pio1FpgaStop) ? true : false;  // モーターと関係ないがついでにこのクラスで取得する
    }

private:
    LoadSwitchFault(uint32_t bits) :
        _Bits(bits) {
    }

    uint32_t _Bits;
};

// ホールセンサーやエンコーダに電源を供給するロードスイッチを制御するクラス
class LoadSwitch {
public:
    static void SetMotor1Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor1SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor1SwitchEnable);
        }
    }

    static void SetMotor2Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor2SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor2SwitchEnable);
        }
    }

    static void SetMotor3Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor3SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor3SwitchEnable);
        }
    }

    static void SetMotor4Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor4SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor4SwitchEnable);
        }
    }

    static void SetMotor5Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable);
        }
    }

    static void SetAllOff(void) {
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable | Pio2Motor4SwitchEnable | Pio2Motor3SwitchEnable | Pio2Motor2SwitchEnable | Pio2Motor1SwitchEnable);
    }

    static void SetAllOn(void) {
        IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5SwitchEnable | Pio2Motor4SwitchEnable | Pio2Motor3SwitchEnable | Pio2Motor2SwitchEnable | Pio2Motor1SwitchEnable);
    }

    static LoadSwitchFault GetFault(bool ignore_masked_bits) {
        if (ignore_masked_bits == false) {
            return LoadSwitchFault(IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE));
        }
        else {
            return LoadSwitchFault(IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & IORD_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE));
        }
    }
};
