/**
 * @file led.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <altera_avalon_pio_regs.h>
#include <system.h>
#include "pin_name.hpp"

// モーターの状態を示すLEDを制御するクラス
class Led {
public:
    static void setMotor1Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor1Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor1Led);
        }
    }

    static void setMotor2Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor2Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor2Led);
        }
    }

    static void setMotor3Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor3Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor3Led);
        }
    }

    static void setMotor4Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor4Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor4Led);
        }
    }

    static void setMotor5Enabled(bool enabled) {
        if (enabled == false) {
            IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5Led);
        }
        else {
            IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5Led);
        }
    }

    static void setAllOff(void) {
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_2_BASE, Pio2Motor5Led | Pio2Motor4Led | Pio2Motor3Led | Pio2Motor2Led | Pio2Motor1Led);
    }

    static void setAllOn(void) {
        IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_2_BASE, Pio2Motor5Led | Pio2Motor4Led | Pio2Motor3Led | Pio2Motor2Led | Pio2Motor1Led);
    }
};
