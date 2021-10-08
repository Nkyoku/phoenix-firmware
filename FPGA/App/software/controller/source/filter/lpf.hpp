/**
 * @file lpf.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

/// サンプルレートの50分の1のカットオフ周波数を持つ2次IIR LPF
class Lpf2ndOrder50 {
public:
    float operator ()(float x) {
        static constexpr float S1 = 0.00362168151492864212f;
        static constexpr float A21 = -1.82269492519630827f;
        static constexpr float A31 = 0.837181651256022619f;
        static constexpr float B21 = 2.0f;
        float t = S1 * x - A21 * z[0] - A31 * z[1];
        float y = t + B21 * z[0] + z[1];
        z[1] = z[0];
        z[0] = t;
        return y;
    }

    void reset(void) {
        z[0] = 0.0f;
        z[1] = 0.0f;
    }

private:
    float z[2];
};

/// サンプルレートの100分の1のカットオフ周波数を持つ2次IIR LPF
class Lpf2ndOrder100 {
public:
    float operator ()(float x) {
        static constexpr float S1 = 0.000944691843840150748f;
        static constexpr float A21 = -1.9111970674260732f;
        static constexpr float A31 = 0.914975834801433741f;
        static constexpr float B21 = 2.0f;
        float t = S1 * x - A21 * z[0] - A31 * z[1];
        float y = t + B21 * z[0] + z[1];
        z[1] = z[0];
        z[0] = t;
        return y;
    }

    void reset(void) {
        z[0] = 0.0f;
        z[1] = 0.0f;
    }

private:
    float z[2];
};

/// サンプルレートの200分の1のカットオフ周波数を持つ2次IIR LPF
class Lpf2ndOrder200 {
public:
    float operator ()(float x) {
        static constexpr float S1 = 0.000241359049041980727f;
        static constexpr float A21 = -1.95557824031503547f;
        static constexpr float A31 = 0.956543676511203422f;
        static constexpr float B21 = 2.0f;
        float t = S1 * x - A21 * z[0] - A31 * z[1];
        float y = t + B21 * z[0] + z[1];
        z[1] = z[0];
        z[0] = t;
        return y;
    }

    void reset(void) {
        z[0] = 0.0f;
        z[1] = 0.0f;
    }

private:
    float z[2];
};
