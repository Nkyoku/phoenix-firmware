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

void CentralizedMonitor::Initialize(void) {
    // 割り込みハンドラを設定する
    alt_ic_isr_register(TIMER_0_IRQ_INTERRUPT_CONTROLLER_ID, TIMER_0_IRQ, TimerHandler, nullptr, nullptr);
    alt_ic_isr_register(PIO_0_IRQ_INTERRUPT_CONTROLLER_ID, PIO_0_IRQ, Pio0Handler, nullptr, nullptr);
    alt_ic_isr_register(PIO_1_IRQ_INTERRUPT_CONTROLLER_ID, PIO_1_IRQ, Pio1Handler, nullptr, nullptr);
    alt_ic_isr_register(VECTOR_CONTROLLER_MASTER_0_IRQ_INTERRUPT_CONTROLLER_ID, VECTOR_CONTROLLER_MASTER_0_IRQ, VectorControllerHandler, nullptr, nullptr);
    alt_ic_isr_register(MOTOR_CONTROLLER_5_IRQ_INTERRUPT_CONTROLLER_ID, MOTOR_CONTROLLER_5_IRQ, MotorControllerHandler, nullptr, nullptr);

    // モーター関連のセンサーの電源を投入する
    LoadSwitch::SetAllOn();
    usleep(1000);

    // モーター関連の割り込みフラグをリセットする
    ResetMotorInterruptFlags();
}

void CentralizedMonitor::Start(void) {
    // pio_0の割り込みを有効にする
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_0_BASE, Pio0Pulse1kHz);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PIO_0_BASE, 0);

    // pio_1のフォルト関連の割り込みを有効にする
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE, Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault | Pio1Motor1SwitchFault | Pio1ModuleSleep | Pio1FpgaStop);

    // timer_0を開始する
    IOWR_ALTERA_AVALON_TIMER_CONTROL(TIMER_0_BASE, ALTERA_AVALON_TIMER_CONTROL_ITO_MSK | ALTERA_AVALON_TIMER_CONTROL_CONT_MSK | ALTERA_AVALON_TIMER_CONTROL_START_MSK);
    IOWR_ALTERA_AVALON_TIMER_STATUS(TIMER_0_BASE, 0);
    IOWR_ALTERA_AVALON_TIMER_PERIOD_0(TIMER_0_BASE, 0);
}

void CentralizedMonitor::Adc2Callback(void) {
    // ADC2のタイムアウトカウンタを初期化する
    _Adc2Timeout = ADC2_TIMEOUT_THRESHOLD;

    // 低電圧、過電圧を判定しエラーフラグに反映する
    DataHolder::FetchAdc2Result();
    auto &adc2_data = DataHolder::GetAdc2Data();
    if (adc2_data.Dc48vVoltage < DC48V_UNDER_VOLTAGE_THRESHOLD) {
        //SetErrorFlags(ErrorCauseDc48vUnderVoltage);
    }
    else if (DC48V_OVER_VOLTAGE_THRESHOLD < adc2_data.Dc48vVoltage) {
        SetErrorFlags(ErrorCauseDc48vOverVoltage);
    }

    // 測定値を送信する
    StreamTransmitter::TransmitAdc2(adc2_data);
}

void CentralizedMonitor::ClearErrorFlags(void) {
    CriticalSection cs;

    uint32_t new_error_flags = _ErrorFlags;

    // pio_1のフォルト関連の割り込みを再び有効化する
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE, Pio1Motor5SwitchFault | Pio1Motor4SwitchFault | Pio1Motor3SwitchFault | Pio1Motor2SwitchFault | Pio1Motor1SwitchFault | Pio1ModuleSleep | Pio1FpgaStop);

    // pio_1に関するエラーフラグの解除を試みる
    uint32_t pio_1_data = IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE);
    if (~pio_1_data & Pio1ModuleSleep) {
        new_error_flags &= ~ErrorCauseModuleSleep;
    }
    if (~pio_1_data & Pio1FpgaStop) {
        new_error_flags &= ~ErrorCauseFpgaStop;
        ResetMotorInterruptFlags();
    }

    // ADC2に関するエラーフラグの解除を試みる
    int dc48v_voltage = Adc2::GetDc48v();
    if (DC48V_UNDER_VOLTAGE_THRESHOLD <= dc48v_voltage) {
        new_error_flags &= ~ErrorCauseDc48vUnderVoltage;
    }
    if (dc48v_voltage <= DC48V_OVER_VOLTAGE_THRESHOLD) {
        new_error_flags &= ~ErrorCauseDc48vOverVoltage;
    }

    // モーターのエラーフラグを解除を試みる
    auto vcm_status = VectorController::GetStatus();
    auto mc5_status = MotorController::GetStatus();
    uint32_t hall_fault_n = vcm_status.HallSensorFaultN() | (mc5_status.HallSensorFaultN() << 4);
    new_error_flags &= ~(hall_fault_n * ErrorCauseMotor1HallSensor);

    // 軽度の過電流エラーを解除する
    new_error_flags &= ~(ErrorCauseMotor5OverCurrent | ErrorCauseMotor4OverCurrent | ErrorCauseMotor3OverCurrent | ErrorCauseMotor2OverCurrent | ErrorCauseMotor1OverCurrent);

    // 新しいエラーフラグを格納する
    _ErrorFlags = new_error_flags;
    SharedMemory::WriteErrorFlags(new_error_flags);
}

void CentralizedMonitor::SetErrorFlags(uint32_t error_flags) {
#if DEBUG_PRINTF
    uint32_t previous = _ErrorFlags;
#endif
    uint32_t new_error_flags;
    {
        CriticalSection cs;
        new_error_flags = _ErrorFlags | error_flags;
        _ErrorFlags = new_error_flags;
        SharedMemory::WriteErrorFlags(new_error_flags);
    }
#if DEBUG_PRINTF
    if (new_error_flags != previous) {
        printf("Error=%08X\n", (unsigned int)new_error_flags);
    }
#endif
    if (new_error_flags != 0) {
        WheelController::StopControl();
        DribbleController::StopControl();
    }
}

void CentralizedMonitor::SetFaultFlags(uint32_t fault_flags) {
#if DEBUG_PRINTF
    uint32_t previous = _FaultFlags;
#endif
    uint32_t new_fault_flags;
    {
        CriticalSection cs;
        new_fault_flags = _FaultFlags | fault_flags;
        _FaultFlags = new_fault_flags;
        SharedMemory::WriteFaultFlags(new_fault_flags);
    }
#if DEBUG_PRINTF
    if (new_fault_flags != previous) {
        printf("Fault=%08X\n", (unsigned int)new_fault_flags);
    }
#endif
    if (new_fault_flags != 0) {
        WheelController::StopControl();
        DribbleController::StopControl();
    }
}

void CentralizedMonitor::DoPeriodicCommonWork(void) {
    // パフォーマンスカウンタの測定を開始する
    static int performance_counter = 0;
    PERF_RESET(reinterpret_cast<void*>(PERFORMANCE_COUNTER_0_BASE));
    PERF_START_MEASURING(reinterpret_cast<void*>(PERFORMANCE_COUNTER_0_BASE));
    PERF_BEGIN(reinterpret_cast<void*>(PERFORMANCE_COUNTER_0_BASE), 1);

    // IMU、モータードライバ、制御データなどを読みJetsonへ送信する
    DataHolder::FetchRegistersOnPreControlLoop();
    StreamTransmitter::TransmitMotion(DataHolder::GetMotionData(), DataHolder::GetControlData(), performance_counter);

    // ADC2のタイムアウトカウンタを減算しすでに0だったらフォルトを発生する
    int adc2_timeout = _Adc2Timeout;
    if (0 <= --adc2_timeout) {
        _Adc2Timeout = adc2_timeout;
    }
    else {
        SetFaultFlags(FaultCauseAdc2Timeout);
    }

    if (IsAnyProblemOccured() == false) {
        // Jetsonから書き込まれた制御パラメータを確認する
        bool new_parameters = SharedMemory::UpdateParameters();
        if (new_parameters) {
            _ParameterTimeout = PARAMETER_TIMEOUT;
        }
        else if (0 < _ParameterTimeout) {
            _ParameterTimeout--;
        }
        bool stop_motors = _ParameterTimeout <= 0;

        // 車輪モーターの指令値を更新する
        WheelController::Update(new_parameters, stop_motors);

        // ドリブルモーターの指令値を更新する
        DribbleController::Update(new_parameters, stop_motors);

#if DEBUG_PRINTF
        if (new_parameters) {
            alt_putstr("New param\n");
        }
#endif
    }
    else {
        // パラメータをクリアする
        SharedMemory::ClearParameters();

        // 車輪モーターのセンサーデータ等を更新する
        WheelController::Update(false, true);

        // Jetsonからエラーフラグのクリアが指示されていればクリアを試みる
        if (SharedMemory::IsRequestedClearingErrorFlags() == true) {
#if DEBUG_PRINTF
            alt_putstr("Flag cleared\n");
#endif
            ClearErrorFlags();
        }

        _ParameterTimeout = 0;
    }

    // Lチカ
    // 全般的な異常がある -> 全てのLEDを点滅
    // モーターに異常がある -> 該当するLEDを点滅
    // モーター制御をしていない -> 異常が無ければ全てのLEDを消灯
    // モーター制御をしている -> LEDを点灯
    static int cnt = 0;
    uint32_t error_flags = _ErrorFlags;
    uint32_t fault_flags = _FaultFlags;
    bool general_fault = (error_flags & (ErrorCauseDc48vUnderVoltage | ErrorCauseDc48vOverVoltage)) || (fault_flags & (FaultCauseAdc2Timeout | FaultCauseImuTimeout));
    if (++cnt == 50) {
        if (general_fault) {
            Led::SetAllOff();
        }
        else if (!VectorController::IsFault() && !MotorController::IsFault()) {
            Led::SetAllOn();
        }
        else {
            Led::SetAllOff();
        }
    }
    else if (100 <= cnt) {
        cnt = 0;
        if (general_fault) {
            Led::SetAllOn();
        }
        else {
            // 各モーターに異常が発生していれば該当するモーターのLEDを点灯する
            bool no_fault = !VectorController::IsFault() && !MotorController::IsFault();
            Led::SetMotor1Enabled(no_fault || (error_flags & (ErrorCauseMotor1OverCurrent | ErrorCauseMotor1HallSensor))
                || (fault_flags & (FaultCauseMotor1OverTemperature | FaultCauseMotor1OverCurrent | FaultCauseMotor1LoadSwitch)));
            Led::SetMotor2Enabled(no_fault || (error_flags & (ErrorCauseMotor2OverCurrent | ErrorCauseMotor2HallSensor))
                || (fault_flags & (FaultCauseMotor2OverTemperature | FaultCauseMotor2OverCurrent | FaultCauseMotor2LoadSwitch)));
            Led::SetMotor3Enabled(no_fault || (error_flags & (ErrorCauseMotor3OverCurrent | ErrorCauseMotor3HallSensor))
                || (fault_flags & (FaultCauseMotor3OverTemperature | FaultCauseMotor3OverCurrent | FaultCauseMotor3LoadSwitch)));
            Led::SetMotor4Enabled(no_fault || (error_flags & (ErrorCauseMotor4OverCurrent | ErrorCauseMotor4HallSensor))
                || (fault_flags & (FaultCauseMotor4OverTemperature | FaultCauseMotor4OverCurrent | FaultCauseMotor4LoadSwitch)));
            Led::SetMotor5Enabled(no_fault || (error_flags & (ErrorCauseMotor5OverCurrent | ErrorCauseMotor5HallSensor))
                || (fault_flags & (FaultCauseMotor5OverTemperature | FaultCauseMotor5OverCurrent | FaultCauseMotor5LoadSwitch)));
        }
    }

    // パフォーマンスカウンタのセクション1の測定を終了する
    // 測定値は次の処理の始めに送信される
    PERF_END(reinterpret_cast<void*>(PERFORMANCE_COUNTER_0_BASE), 1);
    PERF_STOP_MEASURING(reinterpret_cast<void*>(PERFORMANCE_COUNTER_0_BASE));
    uint64_t counter_64 = perf_get_section_time(reinterpret_cast<void*>(PERFORMANCE_COUNTER_0_BASE), 1);
    performance_counter = (counter_64 & 0xFFFFFFFFFFFF0000ULL) ? 65535 : static_cast<int>(counter_64);

    // ステータスフラグを送信する
    StreamTransmitter::TransmitStatus();
}

void CentralizedMonitor::TimerHandler(void *context) {
    // TOフラグをクリアする
    IOWR_ALTERA_AVALON_TIMER_STATUS(TIMER_0_BASE, 0);

    // フォルトフラグを更新する
    SetFaultFlags(FaultCauseImuTimeout);

    // pio_0の割り込みを無効化する
    // 以降、Pio0Handler()は呼ばれない
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_0_BASE, 0);

    // 定期的な処理を行う
    DoPeriodicCommonWork();
}

void CentralizedMonitor::Pio0Handler(void *context) {
    // pio_0のエッジ検知フラグをクリア
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PIO_0_BASE, 0);

    // timer_0をカウンタをリセットする　(固定周期タイマーのPERIOD_nへの書き込みはカウンタを初期値へリセットしカウントを停止させる)
    IOWR_ALTERA_AVALON_TIMER_PERIOD_0(TIMER_0_BASE, 0);
    IOWR_ALTERA_AVALON_TIMER_CONTROL(TIMER_0_BASE, ALTERA_AVALON_TIMER_CONTROL_ITO_MSK | ALTERA_AVALON_TIMER_CONTROL_CONT_MSK | ALTERA_AVALON_TIMER_CONTROL_START_MSK);

    // 定期的な処理を行う
    DoPeriodicCommonWork();
}

void CentralizedMonitor::Pio1Handler(void *context) {
    // pio_1から割り込み要因のI/Oビットを取得し以降のその要因の割り込みを禁止する
    uint32_t irq_masks = IORD_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE);
    uint32_t irq_flags = IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & irq_masks;
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PIO_1_BASE, irq_masks & ~irq_flags);

    // エラーフラグを更新する
    if (irq_flags & Pio1ModuleSleep) {
        SetErrorFlags(ErrorCauseModuleSleep);
    }
    if (irq_flags & Pio1FpgaStop) {
        SetErrorFlags(ErrorCauseFpgaStop);
    }

    // フォルトフラグを更新する
    if (irq_flags & Pio1Motor1SwitchFault) {
        SetFaultFlags(FaultCauseMotor1LoadSwitch);
        LoadSwitch::SetMotor1Enabled(false);
    }
    if (irq_flags & Pio1Motor2SwitchFault) {
        SetFaultFlags(FaultCauseMotor2LoadSwitch);
        LoadSwitch::SetMotor2Enabled(false);
    }
    if (irq_flags & Pio1Motor3SwitchFault) {
        SetFaultFlags(FaultCauseMotor3LoadSwitch);
        LoadSwitch::SetMotor3Enabled(false);
    }
    if (irq_flags & Pio1Motor4SwitchFault) {
        SetFaultFlags(FaultCauseMotor4LoadSwitch);
        LoadSwitch::SetMotor4Enabled(false);
    }
    if (irq_flags & Pio1Motor5SwitchFault) {
        SetFaultFlags(FaultCauseMotor5LoadSwitch);
        LoadSwitch::SetMotor5Enabled(false);
    }
}

void CentralizedMonitor::VectorControllerHandler(void *context) {
    auto int_flags = VectorController::GetInterruptFlag();
#if DEBUG_PRINTF
    DEBUG_PRINTF("VC:INT=%04X,STA=%04X\n", int_flags.Status, VectorController::GetStatus().Status);
#endif
    int hall_fault = int_flags.HallSensorFault();
    SetErrorFlags(hall_fault * ErrorCauseMotor1HallSensor);
    if (~IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & Pio1FpgaStop) {
        // DRV8312のOTW, FAULTは12V電源が喪失するとアサートされてしまうのでFPGA_STOPがデアサートされている間のみ反応する
        int driver_otw = int_flags.OverTemperatureFault();
        int driver_fault = int_flags.OverCurrentFault();
        SetFaultFlags((driver_otw * FaultCauseMotor1OverTemperature) | (driver_fault * FaultCauseMotor1OverCurrent));
    }
}

void CentralizedMonitor::MotorControllerHandler(void *context) {
    auto int_flags = MotorController::GetInterruptFlag();
#if DEBUG_PRINTF
    DEBUG_PRINTF("MC:INT=%04X,STA=%04X\n", int_flags.Status, MotorController::GetStatus().Status);
#endif
    int hall_fault = int_flags.HallSensorFault();
    SetErrorFlags(hall_fault * ErrorCauseMotor5HallSensor);
    if (~IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE) & Pio1FpgaStop) {
        // DRV8312のOTW, FAULTは12V電源が喪失するとアサートされてしまうのでFPGA_STOPがデアサートされている間のみ反応する
        int driver_otw = int_flags.OverTemperatureFault();
        int driver_fault = int_flags.OverCurrentFault();
        SetFaultFlags((driver_otw * FaultCauseMotor5OverTemperature) | (driver_fault * FaultCauseMotor5OverCurrent));
    }
}

void CentralizedMonitor::ResetMotorInterruptFlags(void) {
    // GetInterruptFlag()で割り込みフラグがクリアされ、現在もフォルト状態が発生しているならResetFault()で割り込みフラグが再びセットされる
    (void)MotorController::GetInterruptFlag();
    MotorController::ResetFault();
    (void)VectorController::GetInterruptFlag();
    VectorController::ResetFault();
}

volatile uint32_t CentralizedMonitor::_ErrorFlags = 0;
volatile uint32_t CentralizedMonitor::_FaultFlags = 0;
int CentralizedMonitor::_Adc2Timeout = ADC2_TIMEOUT_THRESHOLD;
int CentralizedMonitor::_ParameterTimeout = 0;
