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
    static void initialize(void);

    /**
     * 開始する
     */
    static void start(void);

    /**
     * エラーフラグを取得する
     * @return エラーフラグのビットマップ
     */
    static uint32_t getErrorFlags(void) {
        return _error_flags;
    }

    /**
     * フォルトフラグを取得する
     * @return フォルトフラグのビットマップ
     */
    static uint32_t getFaultFlags(void) {
        return _fault_flags;
    }

    /**
     * 何らかの問題が発生しているか取得する
     * @return 問題が起きている場合にtrueを返す
     */
    static bool isAnyProblemOccured(void) {
        return (_error_flags != 0) || (_fault_flags != 0);
    }

    /**
     * ADC2の測定完了時にAdc2::handler()から呼ばれるコールバック
     */
    static void adc2Callback(void);

    /**
     * エラーフラグのクリアを試みる
     */
    static void clearErrorFlags(void);

    /**
     * エラーフラグをセットする
     * @param error_flags セットするエラーフラグのビットマップ
     */
    static void setErrorFlags(uint32_t error_flags);

    /**
     * フォルトフラグをセットする
     * @param fault_flags セットするフォルトフラグのビットマップ
     */
    static void setFaultFlags(uint32_t fault_flags);

private:
    /**
     * pio0Handler()あるいはtimerHandler()の共通処理を行う
     */
    static void doPeriodicCommonWork(void);

    /**
     * timer_0の割り込みハンドラ
     * IMUからの割り込み信号が2ms以内に来なかったときに停止処理を行う
     * @param context
     */
    static void timerHandler(void *context);

    /**
     * pio_0の割り込みハンドラ
     * IMUからデータを受信した後にモーター制御や通信を行う
     * @param context
     */
    static void pio0Handler(void *context);

    /**
     * pio_1の割り込みハンドラ
     * 異常信号を検知して停止処理を行う
     * メモ:レベル割り込みなので他の割り込みの処理中に短いパルスが入ったときに取りこぼすおそれがある
     * @param context
     */
    static void pio1Handler(void *context);

    /**
     * vector_controller_master_0の割り込みハンドラ
     * @param context
     */
    static void vectorControllerHandler(void *context);

    /**
     * motor_controller_5の割り込みハンドラ
     * @param context
     */
    static void motorControllerHandler(void *context);

    /**
     * モーター関連の割り込みフラグをリセットする
     */
    static void resetMotorInterruptFlags(void);

    /// エラーフラグのビットマップ
    static volatile uint32_t _error_flags;

    /// フォルトフラグのビットマップ
    static volatile uint32_t _fault_flags;

    /// ADC2のタイムアウトカウンタ
    static int _adc2_timeout;

    /// 指令値のタイムアウトカウンタ
    static int _parameter_timeout;
};
