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
     * 指令値を更新する
     * @param new_parameters 共有メモリーのParametersが更新されたときにtrueを指定する
     */
    static void Update(bool new_parameters);

private:
    /// POWERレジスタの値の変化量の制限
    static constexpr int SLEWRATE_LIMIT = 54;

    /// 電流制限値[mA]
    static constexpr int CURRENT_LIMIT = 716;

    /// 過電流閾値[mA]
    static constexpr int OVER_CURRENT_THRESHOLD = CURRENT_LIMIT * 2;
};
