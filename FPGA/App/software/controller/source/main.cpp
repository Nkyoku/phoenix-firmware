#include <stdio.h>
#include <peripheral/motor_controller.hpp>
#include <peripheral/vector_controller.hpp>
#include <driver/adc2.hpp>
#include <driver/imu.hpp>
#include <driver/led.hpp>
#include <driver/load_switch.hpp>
#include <driver/critical_section.hpp>
#include "shared_memory_manager.hpp"
#include "centralized_monitor.hpp"
#include "wheel_controller.hpp"
#include "dribble_controller.hpp"
#include "stream_transmitter.hpp"

// Memo : IRQ and priorities
// msgdma_0    IRQ0, RIL=1, RRS=1
// timer_0     IRQ1, RIL=3, RRS=3
// pio_0       IRQ2, RIL=2, RRS=2
// pio_1       IRQ3, RIL=3, RRS=3
// jtag_uart_0 IRQ4, RIL=1, RRS=1
// i2cm_0      IRQ5, RIL=1, RRS=1
// spim_0      IRQ6, RIL=1, RRS=1
// vcm_0       IRQ7, RIL=3, RRS=3
// mc_5        IRQ8, RIL=3, RRS=3

static inline void initialize_peripheral(void) {
    Imu::Initialize();
    Adc2::Initialize();
    SharedMemory::Initialize();
    StreamTransmitter::Initialize();
    CentralizedMonitor::Initialize();
    WheelController::Initialize();
    DribbleController::Initialize();
}

static inline void start_peripheral(void) {
    Adc2::Start();
    CentralizedMonitor::Start();
}

int main(void) {
    // ペリフェラルとハードウェアの初期化を行う
    {
        CriticalSection cs;
        initialize_peripheral();
        start_peripheral();
    }

    while (true) {
    }

    return 0;
}
