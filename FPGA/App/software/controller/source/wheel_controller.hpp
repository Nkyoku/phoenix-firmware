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

private:
    /// 速度制御の指令値
    static float _SpeedReference[4];

    /// 速度制御の前回の誤差
    static float _LastSpeedError[4];

    /// 電流制御の指令値
    static float _CurrentReference[4];

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
    static constexpr float CURRENT_LIMIT = 1.0f;

    /// ブレーキを有効化する速度差の閾値[m/s]
    static constexpr float BRAKE_ENABLE_THRESHOLD = 0.5f;

    /// ブレーキを無効化する速度差の閾値[m/s]
    static constexpr float BRAKE_DISABLE_THRESHOLD = 0.25f;
};
