#pragma once

/**
 * 車輪制御を行う
 */
class WheelController {
public:
    /**
     * 初期化を行う
     */
    static void Initialize(void) {
        StopControl();
    }

    /**
     * モーター制御を開始する
     */
    static void StartControl(void);

    /**
     * モーター制御を終了し、モーターを脱力する
     */
    static void StopControl(void);

    /**
     * 指令値を更新する
     * @param new_parameters new_parameters 共有メモリーのParametersが更新されたときにtrueを指定する
     * @param brake_enabled ショートブレーキを使用するときに指定する
     */
    static void Update(bool new_parameters, bool brake_enabled);

    /**
     * 速度指令値を取得する
     * @return 速度指令値の配列へのポインタ (インデックスは0~3のみ有効)
     */
    static const float* GetWheelVelocityReference(void) {
        return _SpeedReference;
    }

    /**
     * 回生エネルギーを取得する
     * @return 回生エネルギーの配列へのポインタ (インデックスは0~3のみ有効)
     */
    static const float* GetWheelRenegerationEnergy(void) {
        return _CurrentLimit;
    }

private:
    /// 速度制御の指令値
    static float _SpeedReference[4];

    /// 速度制御の前回の誤差
    static float _LastSpeedError[4];

    /// 電流制御の指令値
    static float _CurrentReference[4];

    /// モーターの発生させた回生エネルギー (負の値をとる)
    static float _RegenerationEnergy[4];
    static float _CurrentLimit[4];

    /// 電流制御の比例ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_P = 3500;

    /// 電流制御の積分ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_I = 500;

    /// 車体中心を原点とした車輪のX座標[m]
    static constexpr float WHEEL_POS_X = 0.063f;

    /// 車体中心を原点とした車輪のY座標[m]
    static constexpr float WHEEL_POS_Y = 0.042f;

    /// 車体中心から車輪までの距離[m]
    static constexpr float WHEEL_POS_R2 = WHEEL_POS_X * WHEEL_POS_X + WHEEL_POS_Y * WHEEL_POS_Y;

    /// 速度指令値の最大値[m/s]
    static constexpr float MAX_SPEED_REFERENCE = 20.0f;

    /// 電流指令値の最大値[A]
    static constexpr float CURRENT_LIMIT_PER_MOTOR = 3.75f;

    /// 全てのモーターの電流指令値の絶対合計の最大値[A]
    static constexpr float TOTAL_CURRENT_LIMIT = 8.0f;

    /// モーターの逆起電力定数(相間) [V/rps]
    static constexpr float MOTOR_SPEED_CONSTANT = 1.0f / 72.7f * 60.0f;

    /// モーターの巻線抵抗(相間) [Ω]
    static constexpr float MOTOR_RESISTANCE = 6.89f;

    /// モータードライバのベース消費電力 [W]
    static constexpr float BASE_POWER_CONSUMPTION_PER_MOTOR = 0.25f;

    /// ブレーキを有効にする回生エネルギーの閾値
    static constexpr float BRAKE_ENABLE_THRESHOLD = -0.025f;

    /// ブレーキを無効にする回生エネルギーの閾値
    static constexpr float BRAKE_DISABLE_THRESHOLD = -0.0125f;
};
