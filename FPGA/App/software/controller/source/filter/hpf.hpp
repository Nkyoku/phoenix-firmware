/**
 * @file hpf.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

/// 微分器
class Differentiator {
public:
    float operator()(float x) {
        float y = x - z[0];
        z[0] = x;
        return y;
    }

    void reset(void) {
        z[0] = 0.0f;
    }

private:
    float z[1];
};



/// サンプルレートの5分の1のカットオフ周波数を持つ1次IIR HPF
class Hpf1stOrder5 {
public:
    float operator()(float x) {
        static constexpr float S1 = 0.579192220162268234f;
        static constexpr float A21 = -0.158384440324536385f;
        float t = S1 * x - A21 * z[0];
        float y = t - z[0];
        z[0] = t;
        return y;
    }

    void reset(void) {
        z[0] = 0.0f;
    }

private:
    float z[1];
};
