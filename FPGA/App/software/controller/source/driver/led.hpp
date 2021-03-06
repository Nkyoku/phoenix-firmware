#pragma once

#include <altera_avalon_pio_regs.h>
#include <system.h>
#include "pin_name.hpp"

// モーターの状態を示すLEDを制御するクラス
class Led {
public:
    static void SetMotor1Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor1Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor1Led);
        }
    }

    static void SetMotor2Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor2Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor2Led);
        }
    }

    static void SetMotor3Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor3Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor3Led);
        }
    }

    static void SetMotor4Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor4Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor4Led);
        }
    }

    static void SetMotor5Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5Led);
        }
    }

    static void SetAllOff(void) {
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5Led | Pio2Motor4Led | Pio2Motor3Led | Pio2Motor2Led | Pio2Motor1Led);
    }

    static void SetAllOn(void) {
        IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5Led | Pio2Motor4Led | Pio2Motor3Led | Pio2Motor2Led | Pio2Motor1Led);
    }
};
