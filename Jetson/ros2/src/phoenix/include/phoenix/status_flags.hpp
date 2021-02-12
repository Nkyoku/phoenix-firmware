#pragma once

/**
 * エラーフラグのビット
 * エラーは致命的でないがロボットの動作に支障をきたす問題のことである
 * Jetsonからの要求で解除できる
 */
enum ErrorCause_t {
    /// Jetsonがスリープ状態である
    ErrorCauseModuleSleep = 1u << 0,

    /// GreenPAKから停止指示が発されている
    ErrorCauseFpgaStop = 1u << 1,

    /// DC/DCコンバータの出力電圧低下
    ErrorCauseDc48vUnderVoltage = 1u << 2,

    /// DC/DCコンバータの出力電圧超過
    ErrorCauseDc48vOverVoltage = 1u << 3,

    /// 軽度の過電流
    ErrorCauseMotor1OverCurrent = 1u << 4,
    ErrorCauseMotor2OverCurrent = 1u << 5,
    ErrorCauseMotor3OverCurrent = 1u << 6,
    ErrorCauseMotor4OverCurrent = 1u << 7,
    ErrorCauseMotor5OverCurrent = 1u << 8,

    /// ホールセンサー異常、断線
    ErrorCauseMotor1HallSensor = 1u << 9,
    ErrorCauseMotor2HallSensor = 1u << 10,
    ErrorCauseMotor3HallSensor = 1u << 11,
    ErrorCauseMotor4HallSensor = 1u << 12,
    ErrorCauseMotor5HallSensor = 1u << 13,
};

/**
 * @brief フォルトフラグのビット
 * フォルトは致命的でロボットの動作を続けられない種類の問題のことである
 * FPGAをリセットするまで解除できない
 */
enum FaultCause_t {
    /// ADC2からデータが取得できない
    FaultCauseAdc2Timeout = 1u << 0,

    /// IMUからデータが取得できない
    FaultCauseImuTimeout = 1u << 1,

    /// モータードライバの過熱シャットダウン
    FaultCauseMotor1OverTemperature = 1u << 2,
    FaultCauseMotor2OverTemperature = 1u << 3,
    FaultCauseMotor3OverTemperature = 1u << 4,
    FaultCauseMotor4OverTemperature = 1u << 5,
    FaultCauseMotor5OverTemperature = 1u << 6,

    /// モータードライバの過電流シャットダウン
    FaultCauseMotor1OverCurrent = 1u << 7,
    FaultCauseMotor2OverCurrent = 1u << 8,
    FaultCauseMotor3OverCurrent = 1u << 9,
    FaultCauseMotor4OverCurrent = 1u << 10,
    FaultCauseMotor5OverCurrent = 1u << 11,

    /// ホールセンサー、エンコーダのロードスイッチの過電流
    FaultCauseMotor1LoadSwitch = 1u << 12,
    FaultCauseMotor2LoadSwitch = 1u << 13,
    FaultCauseMotor3LoadSwitch = 1u << 14,
    FaultCauseMotor4LoadSwitch = 1u << 15,
    FaultCauseMotor5LoadSwitch = 1u << 16,
};
