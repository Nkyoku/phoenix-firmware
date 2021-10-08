/**
 * @file dribble_controller.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

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
    static void initialize(void){
        stopControl();
    }

    /**
     * モーター制御を開始する
     */
    static void startControl(void);

    /**
     * モーター制御を終了し、モーターを脱力する
     */
    static void stopControl(void);

    /**
     * 指令値を更新する
     * @param new_parameters 共有メモリーのParametersが更新されたときにtrueを指定する
     * @param brake_enabled ショートブレーキを使用するときに指定する
     */
    static void update(bool new_parameters, bool brake_enabled);
};
