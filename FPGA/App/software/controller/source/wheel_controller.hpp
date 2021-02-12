#pragma once

/**
 * 車輪制御を行う
 */
class WheelController {
public:
    /**
     * 初期化を行う
     */
    static void Initialize(void){
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
     */
    static void Update(bool new_parameters);

private:
    /// 速度制御の前回の誤差
    static float _LastSpeedError[4];

    /// 電流制御の前回の指令値
    static float _LastCurrentReference[4];

    /// 電流制御の比例ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_P = 3500;

    /// 電流制御の積分ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_I = 500;


    static constexpr float MaxSpeedReference = 100.0f;

    static constexpr float MaxCurrentReference = 0.5f;




};
