#pragma once

#include <stdint.h>

/**
 * ドリブル制御を行う
 */
class DribbleController {
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
     * ADC2がドリブル電流の測定を完了した際にAdc2::Handler()から呼ばれる
     * @param dribble_current Adc2::GetDribbleCurrent()で取得できる値と同じ
     */
    static void Adc2UpdateCurrent(int dribble_current);

    /**
     * 指令値を更新する
     * @param new_parameters 共有メモリーのParametersが更新されたときにtrueを指定する
     */
    static void Update(bool new_parameters);

private:
    /// ドリブルモーターのパワー設定値からPOWERレジスタへの換算係数
    static constexpr int POWER_SCALE = 3000;

    /// ドリブルモーターのPOWERレジスタの最大値
    static constexpr int MAXIMUM_POWER = 2985;

    /// POWERレジスタの値の変化量の制限
    static constexpr int SLEWRATE_LIMIT = 54;

    /// 電流制限値[mA]
    static constexpr int CURRENT_LIMIT = 716;

    /// 過電流閾値[mA]
    static constexpr int OVER_CURRENT_THRESHOLD = CURRENT_LIMIT * 2;
};
