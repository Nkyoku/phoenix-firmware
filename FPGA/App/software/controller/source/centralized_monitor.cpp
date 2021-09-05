#include "centralized_monitor.hpp"
#include <driver/critical_section.hpp>
#include <driver/load_switch.hpp>
#include <driver/adc2.hpp>
#include <driver/led.hpp>
#include <peripheral/motor_controller.hpp>
#include <peripheral/vector_controller.hpp>
#include <sys/unistd.h>
#include <system.h>
#include <altera_avalon_pio_regs.h>
#include <altera_avalon_timer_regs.h>
#include <altera_avalon_performance_counter.h>
#include <sys/alt_irq.h>
#include <status_flags.hpp>
#include "wheel_controller.hpp"
#include "dribble_controller.hpp"
#include "shared_memory_manager.hpp"
#include "stream_transmitter.hpp"
#include "data_holder.hpp"

#define DEBUG_PRINTF 0
#if DEBUG_PRINTF
#include <stdio.h>
#include <sys/alt_stdio.h>
#endif

/// ADC2のタイムアウトカウンタの初期値
static constexpr int ADC2_TIMEOUT_THRESHOLD = 50;

/// 指令値が更新されなくなってから自動停止するまでの時間
static constexpr int PARAMETER_TIMEOUT = 500;

/// DC48Vの下限電圧[mV]
static constexpr float DC48V_UNDER_VOLTAGE_THRESHOLD = 40.0f;

/// DC48Vの上限電圧[mV]
static constexpr float DC48V_OVER_VOLTAGE_THRESHOLD = 52.5f;

void CentralizedMonitor::initialize(void) {
    // 割り込みハンドラを設定する
    alt_ic_isr_register(TIMER_0_IRQ_INTERRUPT_CONTROLLER_ID, TIMER_0_IRQ, timerHandler, nullptr, nullptr);
    alt_ic_isr_register(PIO_0_IRQ_INTERRUPT_CONTROLLER_ID, PIO_0_IRQ, pio0Handler, nullptr, nullptr);
    alt_ic_isr_register(PIO_1_IRQ_INTERRUPT_CONTROLLER_ID, PIO_1_IRQ, pio1Handler, nullptr, nullptr);
    alt_ic_isr_register(VECTOR_CONTROLLER_MASTER_0_IRQ_INTERRUPT_CONTROLLER_ID, VECTOR_CONTROLLER_MASTER_0_IRQ, vectorControllerHandler, nullptr, nullptr);
    alt_ic_isr_register(MOTOR_CONTROLLER_5_IRQ_INTERRUPT_CONTROLLER_ID, MOTOR_CONTROLLER_5_IRQ, motorControllerHandler, nullptr, nullptr);

    // モーター関連のセンサーの電源を投入する
    LoadSwitch::setAllOn();
    usleep(1000);

    // モーター関連の割り込みフラグをリセットする
    resetMotorInterruptFlags();
}

void CentralizedMonitor::start(void) {
    // pio_0の割り込みを有効にする
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_0_BASE, Pio0Pulse1kHz);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PIO_0_BASE, 0);

    // pio_1のフォルト関連の割り込みを有効にする
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE, Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault |
                                                    Pio1Motor1SwitchFault | Pio1ModuleSleep | Pio1FpgaStop);

    // timer_0を開始する
    IOWR_ALTERA_AVALON_TIMER_CONTROL(TIMER_0_BASE, ALTERA_AVALON_TIMER_CONTROL_ITO_MSK | ALTERA_AVALON_TIMER_CONTROL_CONT_MSK | ALTERA_AVALON_TIMER_CONTROL_START_MSK);
    IOWR_ALTERA_AVALON_TIMER_STATUS(TIMER_0_BASE, 0);
    IOWR_ALTERA_AVALON_TIMER_PERIOD_0(TIMER_0_BASE, 0);
}

void CentralizedMonitor::adc2Callback(void) {
    // ADC2のタイムアウトカウンタを初期化する
    _adc2_timeout = ADC2_TIMEOUT_THRESHOLD;

    // 低電圧、過電圧を判定しエラーフラグに反映する
    DataHolder::fetchAdc2Result();
    auto &adc2_data = DataHolder::adc2Data();
    if (adc2_data.dc48v_voltage < DC48V_UNDER_VOLTAGE_THRESHOLD) {
        setErrorFlags(ErrorCauseDc48vUnderVoltage);
    }
    else if (DC48V_OVER_VOLTAGE_THRESHOLD < adc2_data.dc48v_voltage) {
        setErrorFlags(ErrorCauseDc48vOverVoltage);
    }

    // 測定値を送信する
    StreamTransmitter::transmitAdc2(adc2_data);
}

void CentralizedMonitor::clearErrorFlags(void) {
    CriticalSection cs;

    uint32_t new_error_flags = _error_flags;

    // pio_1のフォルト関連の割り込みを再び有効化する
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE, Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault | Pio1Motor1SwitchFault | Pio1ModuleSleep | Pio1FpgaStop);

    // pio_1に関するエラーフラグの解除を試みる
    uint32_t pio_1_data = IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE);
    if (~pio_1_data & Pio1ModuleSleep) {
        new_error_flags &= ~ErrorCauseModuleSleep;
    }
    if (~pio_1_data & Pio1FpgaStop) {
        new_error_flags &= ~ErrorCauseFpgaStop;
        resetMotorInterruptFlags();
    }

    // ADC2に関するエラーフラグの解除を試みる
    int dc48v_voltage = Adc2::getDc48v();
    if (DC48V_UNDER_VOLTAGE_THRESHOLD <= dc48v_voltage) {
        new_error_flags &= ~ErrorCauseDc48vUnderVoltage;
    }
    if (dc48v_voltage <= DC48V_OVER_VOLTAGE_THRESHOLD) {
        new_error_flags &= ~ErrorCauseDc48vOverVoltage;
    }

    // モーターのエラーフラグを解除を試みる
    auto vcm_status = VectorController::getStatus();
    auto mc5_status = MotorController::getStatus();
    uint32_t hall_fault_n = vcm_status.hallSensorFaultN() | (mc5_status.hallSensorFaultN() << 4);
    new_error_flags &= ~(hall_fault_n * ErrorCauseMotor1HallSensor);

    // 軽度の過電流エラーを解除する
    new_error_flags &= ~(ErrorCauseMotor5OverCurrent | ErrorCauseMotor4OverCurrent | ErrorCauseMotor3OverCurrent | ErrorCauseMotor2OverCurrent | ErrorCauseMotor1OverCurrent);

    // 新しいエラーフラグを格納する
    _error_flags = new_error_flags;
    SharedMemoryManager::writeErrorFlags(new_error_flags);
}

void CentralizedMonitor::setErrorFlags(uint32_t error_flags) {
#if DEBUG_PRINTF
    uint32_t previous = _error_flags;
#endif
    uint32_t new_error_flags;
    {
        CriticalSection cs;
        new_error_flags = _error_flags | error_flags;
        _error_flags = new_error_flags;
        SharedMemoryManager::writeErrorFlags(new_error_flags);
    }
#if DEBUG_PRINTF
    if (new_error_flags != previous) {
        printf("Error=%08X\n", (unsigned int)new_error_flags);
    }
#endif
    if (new_error_flags != 0) {
        WheelController::stopControl();
        DribbleController::stopControl();
    }
}

void CentralizedMonitor::setFaultFlags(uint32_t fault_flags) {
#if DEBUG_PRINTF
    uint32_t previous = _fault_flags;
#endif
    uint32_t new_fault_flags;
    {
        CriticalSection cs;
        new_fault_flags = _fault_flags | fault_flags;
        _fault_flags = new_fault_flags;
        SharedMemoryManager::writeFaultFlags(new_fault_flags);
    }
#if DEBUG_PRINTF
    if (new_fault_flags != previous) {
        printf("Fault=%08X\n", (unsigned int)new_fault_flags);
    }
#endif
    if (new_fault_flags != 0) {
        WheelController::stopControl();
        DribbleController::stopControl();
    }
}

void CentralizedMonitor::doPeriodicCommonWork(void) {
    // パフォーマンスカウンタの測定を開始する
    static int performance_counter = 0;
    PERF_RESET(reinterpret_cast<void *>(PERFORMANCE_COUNTER_0_BASE));
    PERF_START_MEASURING(reinterpret_cast<void *>(PERFORMANCE_COUNTER_0_BASE));
    PERF_BEGIN(reinterpret_cast<void *>(PERFORMANCE_COUNTER_0_BASE), 1);

    // センサーデータを読み出す
    DataHolder::fetchOnPreControlLoop();

    // ADC2のタイムアウトカウンタを減算しすでに0だったらフォルトを発生する
    int adc2_timeout = _adc2_timeout;
    if (0 <= --adc2_timeout) {
        _adc2_timeout = adc2_timeout;
    }
    else {
        setFaultFlags(FaultCauseAdc2Timeout);
    }

    if (isAnyProblemOccured() == false) {
        // Jetsonから書き込まれた制御パラメータを確認する
        bool new_parameters = SharedMemoryManager::updateParameters();
        if (new_parameters) {
            _parameter_timeout = PARAMETER_TIMEOUT;
        }
        else if (0 < _parameter_timeout) {
            _parameter_timeout--;
        }
        bool stop_motors = _parameter_timeout <= 0;

        // 車輪モーターの指令値を更新する
        WheelController::update(new_parameters, stop_motors);

        // ドリブルモーターの指令値を更新する
        DribbleController::update(new_parameters, stop_motors);

#if DEBUG_PRINTF
        if (new_parameters) {
            alt_putstr("New param\n");
        }
#endif
    }
    else {
        // パラメータをクリアする
        SharedMemoryManager::clearParameters();

        // 車輪モーターのセンサーデータ等を更新する
        WheelController::update(false, true);

        // Jetsonからエラーフラグのクリアが指示されていればクリアを試みる
        if (SharedMemoryManager::isRequestedClearingErrorFlags() == true) {
#if DEBUG_PRINTF
            alt_putstr("Flag cleared\n");
#endif
            clearErrorFlags();
        }

        _parameter_timeout = 0;
    }

    // 制御データを読み出してJetsonへデータを送信する
    DataHolder::fetchOnPostControlLoop();
    StreamTransmitter::transmitMotion(DataHolder::motionData(), DataHolder::controlData(), performance_counter);

    // Lチカ
    // 全般的な異常がある -> 全てのLEDを点滅
    // モーターに異常がある -> 該当するLEDを点滅
    // モーター制御をしていない -> 異常が無ければ全てのLEDを消灯
    // モーター制御をしている -> LEDを点灯
    static int cnt = 0;
    uint32_t error_flags = _error_flags;
    uint32_t fault_flags = _fault_flags;
    bool general_fault =
        (error_flags & (ErrorCauseDc48vUnderVoltage | ErrorCauseDc48vOverVoltage)) || (fault_flags & (FaultCauseAdc2Timeout | FaultCauseImuTimeout));
    if (++cnt == 50) {
        if (general_fault) {
            Led::setAllOff();
        }
        else if (!VectorController::isFault() && !MotorController::isFault()) {
            Led::setAllOn();
        }
        else {
            Led::setAllOff();
        }
    }
    else if (100 <= cnt) {
        cnt = 0;
        if (general_fault) {
            Led::setAllOn();
        }
        else {
            // 各モーターに異常が発生していれば該当するモーターのLEDを点灯する
            bool no_fault = !VectorController::isFault() && !MotorController::isFault();
            Led::setMotor1Enabled(no_fault || (error_flags & (ErrorCauseMotor1OverCurrent | ErrorCauseMotor1HallSensor)) ||
                                  (fault_flags & (FaultCauseMotor1OverTemperature | FaultCauseMotor1OverCurrent | FaultCauseMotor1LoadSwitch)));
            Led::setMotor2Enabled(no_fault || (error_flags & (ErrorCauseMotor2OverCurrent | ErrorCauseMotor2HallSensor)) ||
                                  (fault_flags & (FaultCauseMotor2OverTemperature | FaultCauseMotor2OverCurrent | FaultCauseMotor2LoadSwitch)));
            Led::setMotor3Enabled(no_fault || (error_flags & (ErrorCauseMotor3OverCurrent | ErrorCauseMotor3HallSensor)) ||
                                  (fault_flags & (FaultCauseMotor3OverTemperature | FaultCauseMotor3OverCurrent | FaultCauseMotor3LoadSwitch)));
            Led::setMotor4Enabled(no_fault || (error_flags & (ErrorCauseMotor4OverCurrent | ErrorCauseMotor4HallSensor)) ||
                                  (fault_flags & (FaultCauseMotor4OverTemperature | FaultCauseMotor4OverCurrent | FaultCauseMotor4LoadSwitch)));
            Led::setMotor5Enabled(no_fault || (error_flags & (ErrorCauseMotor5OverCurrent | ErrorCauseMotor5HallSensor)) ||
                                  (fault_flags & (FaultCauseMotor5OverTemperature | FaultCauseMotor5OverCurrent | FaultCauseMotor5LoadSwitch)));
        }
    }

    // パフォーマンスカウンタのセクション1の測定を終了する
    // 測定値は次の処理の始めに送信される
    PERF_END(reinterpret_cast<void *>(PERFORMANCE_COUNTER_0_BASE), 1);
    PERF_STOP_MEASURING(reinterpret_cast<void *>(PERFORMANCE_COUNTER_0_BASE));
    uint64_t counter_64 = perf_get_section_time(reinterpret_cast<void *>(PERFORMANCE_COUNTER_0_BASE), 1);
    performance_counter = (counter_64 & 0xFFFFFFFFFFFF0000ULL) ? 65535 : static_cast<int>(counter_64);

    // ステータスフラグを送信する
    StreamTransmitter::transmitStatus();
}

void CentralizedMonitor::timerHandler(void *context) {
    // TOフラグをクリアする
    IOWR_ALTERA_AVALON_TIMER_STATUS(TIMER_0_BASE, 0);

    // フォルトフラグを更新する
    setFaultFlags(FaultCauseImuTimeout);

    // pio_0の割り込みを無効化する
    // 以降、pio0Handler()は呼ばれない
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_0_BASE, 0);

    // 定期的な処理を行う
    doPeriodicCommonWork();
}

void CentralizedMonitor::pio0Handler(void *context) {
    // pio_0のエッジ検知フラグをクリア
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PIO_0_BASE, 0);

    // timer_0をカウンタをリセットする
    // 固定周期タイマーのPERIOD_nへの書き込みはカウンタを初期値へリセットするだけでなくカウントを停止させるのですぐに再起動させる
    IOWR_ALTERA_AVALON_TIMER_PERIOD_0(TIMER_0_BASE, 0);
    IOWR_ALTERA_AVALON_TIMER_CONTROL(TIMER_0_BASE,
                                     ALTERA_AVALON_TIMER_CONTROL_ITO_MSK | ALTERA_AVALON_TIMER_CONTROL_CONT_MSK | ALTERA_AVALON_TIMER_CONTROL_START_MSK);

    // 定期的な処理を行う
    doPeriodicCommonWork();
}

void CentralizedMonitor::pio1Handler(void *context) {
    // pio_1から割り込み要因のI/Oビットを取得し以降のその要因の割り込みを禁止する
    uint32_t irq_masks = IORD_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE);
    uint32_t irq_flags = IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & irq_masks;
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE, irq_masks & ~irq_flags);

    // エラーフラグを更新する
    if (irq_flags & Pio1ModuleSleep) {
        setErrorFlags(ErrorCauseModuleSleep);
    }
    if (irq_flags & Pio1FpgaStop) {
        setErrorFlags(ErrorCauseFpgaStop);
    }

    // フォルトフラグを更新する
    if (irq_flags & Pio1Motor1SwitchFault) {
        setFaultFlags(FaultCauseMotor1LoadSwitch);
        LoadSwitch::setMotor1Enabled(false);
    }
    if (irq_flags & Pio1Motor2SwitchFault) {
        setFaultFlags(FaultCauseMotor2LoadSwitch);
        LoadSwitch::setMotor2Enabled(false);
    }
    if (irq_flags & Pio1Motor3SwitchFault) {
        setFaultFlags(FaultCauseMotor3LoadSwitch);
        LoadSwitch::setMotor3Enabled(false);
    }
    if (irq_flags & Pio1Motor4SwitchFault) {
        setFaultFlags(FaultCauseMotor4LoadSwitch);
        LoadSwitch::setMotor4Enabled(false);
    }
    if (irq_flags & Pio1Motor5SwitchFault) {
        setFaultFlags(FaultCauseMotor5LoadSwitch);
        LoadSwitch::setMotor5Enabled(false);
    }
}

void CentralizedMonitor::vectorControllerHandler(void *context) {
    auto int_flags = VectorController::getInterruptFlag();
#if DEBUG_PRINTF
    DEBUG_PRINTF("VC:INT=%04X,STA=%04X\n", int_flags.status, VectorController::getStatus().status);
#endif
    int hall_fault = int_flags.hallSensorFault();
    setErrorFlags(hall_fault * ErrorCauseMotor1HallSensor);
    if (~IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & Pio1FpgaStop) {
        // DRV8312のOTW, FAULTは12V電源が喪失するとアサートされてしまうのでFPGA_STOPがデアサートされている間のみ反応する
        int driver_otw = int_flags.overTemperatureFault();
        int driver_fault = int_flags.overCurrentFault();
        setFaultFlags((driver_otw * FaultCauseMotor1OverTemperature) | (driver_fault * FaultCauseMotor1OverCurrent));
    }
}

void CentralizedMonitor::motorControllerHandler(void *context) {
    auto int_flags = MotorController::getInterruptFlag();
#if DEBUG_PRINTF
    DEBUG_PRINTF("MC:INT=%04X,STA=%04X\n", int_flags.status, MotorController::getStatus().status);
#endif
    int hall_fault = int_flags.hallSensorFault();
    setErrorFlags(hall_fault * ErrorCauseMotor5HallSensor);
    if (~IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & Pio1FpgaStop) {
        // DRV8312のOTW, FAULTは12V電源が喪失するとアサートされてしまうのでFPGA_STOPがデアサートされている間のみ反応する
        int driver_otw = int_flags.overTemperatureFault();
        int driver_fault = int_flags.overCurrentFault();
        setFaultFlags((driver_otw * FaultCauseMotor5OverTemperature) | (driver_fault * FaultCauseMotor5OverCurrent));
    }
}

void CentralizedMonitor::resetMotorInterruptFlags(void) {
    // getInterruptFlag()で割り込みフラグがクリアされ、現在もフォルト状態が発生しているならresetFault()で割り込みフラグが再びセットされる
    (void)MotorController::getInterruptFlag();
    MotorController::resetFault();
    (void)VectorController::getInterruptFlag();
    VectorController::resetFault();
}

volatile uint32_t CentralizedMonitor::_error_flags = 0;
volatile uint32_t CentralizedMonitor::_fault_flags = 0;
int CentralizedMonitor::_adc2_timeout = ADC2_TIMEOUT_THRESHOLD;
int CentralizedMonitor::_parameter_timeout = 0;
