#pragma once

#include <system.h>
#include <cstdint>

#undef sqrtf
#undef lroundf
#undef fmaxf
#undef fminf

/// 半精度浮動小数点数をuint16_tに格納するために型宣言する
using __fp16 = std::uint16_t;

namespace fpu {

static inline float max(float a, float b) {
    return __builtin_custom_fnff(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FMAXS_N, a, b);
}

static inline float min(float a, float b) {
    return __builtin_custom_fnff(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FMINS_N, a, b);
}

static inline int round(float a) {
    return __builtin_custom_inf(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_ROUND_N, a);
}

static inline float sqrt(float a) {
    return __builtin_custom_fnf(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FSQRTS_N, a);
}

static inline float clamp(float value, float min_value, float max_value) {
    return max(min_value, min(max_value, value));
}

/**
 * @brief 単精度浮動小数点数を半精度浮動小数点数に変換する。
 * カスタム命令により高速に変換できる。
 * @param a 単精度浮動小数点数
 * @return 半精度浮動小数点数 (上位16bitは0)
 */
static inline int to_fp16(float a) {
    return __builtin_custom_inf(ALT_CI_FLOAT32TO16_0_N, a);
}

}
