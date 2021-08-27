#pragma once

#include <stdint.h>

/**
 * FPGAの周辺機能の異常を監視・記録し、異常発生時に安全にロボットを停止させる
 */
class CentralizedMonitor {
public:
    /**
     * 初期化する
     */
    static void Initialize(void);

    /**
     * 開始する
     */
    static void Start(void);

    /**
     * エラーフラグを取得する
     * @return エラーフラグのビットマップ
     */
    static uint32_t GetErrorFlags(void) {
        return _ErrorFlags;
    }

    /**
     * フォルトフラグを取得する
     * @return フォルトフラグのビットマップ
     */
    static uint32_t GetFaultFlags(void) {
        return _FaultFlags;
    }

    /**
     * 何らかの問題が発生しているか取得する
     * @return 問題が起きている場合にtrueを返す
     */
    static bool IsAnyProblemOccured(void){
        return (_ErrorFlags != 0) || (_FaultFlags != 0);
    }

    /**
     * ADC2の測定完了時にAdc2::Handler()から呼ばれるコールバック
     */
    static void Adc2Callback(void);

    /**
     * エラーフラグのクリアを試みる
     */
    static void ClearErrorFlags(void);

    /**
     * エラーフラグをセットする
     * @param error_flags セットするエラーフラグのビットマップ
     */
    static void SetErrorFlags(uint32_t error_flags);

    /**
     * フォルトフラグをセットする
     * @param fault_flags セットするフォルトフラグのビットマップ
     */
    static void SetFaultFlags(uint32_t fault_flags);

private:
    /**
     * Pio0Handler()あるいはTimerHandler()の共通処理を行う
     */
    static void DoPeriodicCommonWork(void);

    /**
     * timer_0の割り込みハンドラ
     * IMUからの割り込み信号が2ms以内に来なかったときに停止処理を行う
     * @param context
     */
    static void TimerHandler(void *context);

    /**
     * pio_0の割り込みハンドラ
     * IMUからデータを受信した後にモーター制御や通信を行う
     * @param context
     */
    static void Pio0Handler(void *context);

    /**
     * pio_1の割り込みハンドラ
     * 異常信号を検知して停止処理を行う
     * メモ:レベル割り込みなので他の割り込みの処理中に短いパルスが入ったときに取りこぼすおそれがある
     * @param context
     */
    static void Pio1Handler(void *context);

    /**
     * vector_controller_master_0の割り込みハンドラ
     * @param context
     */
    static void VectorControllerHandler(void *context);

    /**
     * motor_controller_5の割り込みハンドラ
     * @param context
     */
    static void MotorControllerHandler(void *context);

    /**
     * モーター関連の割り込みフラグをリセットする
     */
    static void ResetMotorInterruptFlags(void);

    /// エラーフラグのビットマップ
    static volatile uint32_t _ErrorFlags;

    /// フォルトフラグのビットマップ
    static volatile uint32_t _FaultFlags;

    /// ADC2のタイムアウトカウンタ
    static int _Adc2Timeout;

    /// 指令値のタイムアウトカウンタ
    static int _ParameterTimeout;

    /// ADC2のタイムアウトカウンタの初期値
    static constexpr int ADC2_TIMEOUT_THRESHOLD = 50;

    /// 指令値が更新されなくなってから自動停止するまでの時間
    static constexpr int PARAMETER_TIMEOUT = 500;

    /// DC48Vの下限電圧[mV]
    static constexpr float DC48V_UNDER_VOLTAGE_THRESHOLD = 40.0f;

    /// DC48Vの上限電圧[mV]
    static constexpr float DC48V_OVER_VOLTAGE_THRESHOLD = 52.5f;
};
