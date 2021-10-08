#include "acceleration_limitter.hpp"
#include "const_matrix.hpp"
#include "fpu.hpp"
#include "board.hpp"
#include <math.h>

// 目的関数の重み
static constexpr float WEIGHT_X = 1.0f;
static constexpr float WEIGHT_Y = 1.0f;
static constexpr float WEIGHT_W = 1.0f;
static constexpr float WEIGHT_C = 4.0f;

using namespace Eigen;
using namespace ctmath;

#define SOLVE4()                                                              \
    constexpr auto T = Q.inv().round(ROUND_BIT);                              \
    y[0] = T(0, 0) * p[0] + T(0, 1) * p[1] + T(0, 2) * p[2] + T(0, 3) * p[3]; \
    y[1] = T(1, 0) * p[0] + T(1, 1) * p[1] + T(1, 2) * p[2] + T(1, 3) * p[3]; \
    y[2] = T(2, 0) * p[0] + T(2, 1) * p[1] + T(2, 2) * p[2] + T(2, 3) * p[3]; \
    y[3] = T(3, 0) * p[0] + T(3, 1) * p[1] + T(3, 2) * p[2] + T(3, 3) * p[3]

#define SOLVE5(i0)                                                                              \
    constexpr auto A = D.extractRow(i0);                                                        \
    constexpr auto T = mergeBlocks(Q, A.t(), A).inv().round(ROUND_BIT);                         \
    float b[1];                                                                                 \
    b[0] = (active_set_signs & (1 << i0)) ? -current_limit(i0) : current_limit(i0);             \
    y[0] = T(0, 0) * p[0] + T(0, 1) * p[1] + T(0, 2) * p[2] + T(0, 3) * p[3] + T(0, 4) * b[0];  \
    y[1] = T(1, 0) * p[0] + T(1, 1) * p[1] + T(1, 2) * p[2] + T(1, 3) * p[3] + T(1, 4) * b[0];  \
    y[2] = T(2, 0) * p[0] + T(2, 1) * p[1] + T(2, 2) * p[2] + T(2, 3) * p[3] + T(2, 4) * b[0];  \
    y[3] = T(3, 0) * p[0] + T(3, 1) * p[1] + T(3, 2) * p[2] + T(3, 3) * p[3] + T(3, 4) * b[0];  \
    l[i0] = T(4, 0) * p[0] + T(4, 1) * p[1] + T(4, 2) * p[2] + T(4, 3) * p[3] + T(4, 4) * b[0]; \
    l[i0] = (active_set_signs & (1 << i0)) ? -l[i0] : l[i0]

#define SOLVE6(i0, i1)                                                                                           \
    constexpr auto A = D.extractRow(i0, i1);                                                                     \
    constexpr auto T = mergeBlocks(Q, A.t(), A).inv().round(ROUND_BIT);                                          \
    float b[2];                                                                                                  \
    b[0] = (active_set_signs & (1 << i0)) ? -current_limit(i0) : current_limit(i0);                              \
    b[1] = (active_set_signs & (1 << i1)) ? -current_limit(i1) : current_limit(i1);                              \
    y[0] = T(0, 0) * p[0] + T(0, 1) * p[1] + T(0, 2) * p[2] + T(0, 3) * p[3] + T(0, 4) * b[0] + T(0, 5) * b[1];  \
    y[1] = T(1, 0) * p[0] + T(1, 1) * p[1] + T(1, 2) * p[2] + T(1, 3) * p[3] + T(1, 4) * b[0] + T(1, 5) * b[1];  \
    y[2] = T(2, 0) * p[0] + T(2, 1) * p[1] + T(2, 2) * p[2] + T(2, 3) * p[3] + T(2, 4) * b[0] + T(2, 5) * b[1];  \
    y[3] = T(3, 0) * p[0] + T(3, 1) * p[1] + T(3, 2) * p[2] + T(3, 3) * p[3] + T(3, 4) * b[0] + T(3, 5) * b[1];  \
    l[i0] = T(4, 0) * p[0] + T(4, 1) * p[1] + T(4, 2) * p[2] + T(4, 3) * p[3] + T(4, 4) * b[0] + T(4, 5) * b[1]; \
    l[i1] = T(5, 0) * p[0] + T(5, 1) * p[1] + T(5, 2) * p[2] + T(5, 3) * p[3] + T(5, 4) * b[0] + T(5, 5) * b[1]; \
    l[i0] = (active_set_signs & (1 << i0)) ? -l[i0] : l[i0];                                                     \
    l[i1] = (active_set_signs & (1 << i1)) ? -l[i1] : l[i1]

#define SOLVE7(i0, i1, i2)                                                                                                        \
    constexpr auto A = D.extractRow(i0, i1, i2);                                                                                  \
    constexpr auto T = mergeBlocks(Q, A.t(), A).inv().round(ROUND_BIT);                                                           \
    float b[3];                                                                                                                   \
    b[0] = (active_set_signs & (1 << i0)) ? -current_limit(i0) : current_limit(i0);                                               \
    b[1] = (active_set_signs & (1 << i1)) ? -current_limit(i1) : current_limit(i1);                                               \
    b[2] = (active_set_signs & (1 << i2)) ? -current_limit(i2) : current_limit(i2);                                               \
    y[0] = T(0, 0) * p[0] + T(0, 1) * p[1] + T(0, 2) * p[2] + T(0, 3) * p[3] + T(0, 4) * b[0] + T(0, 5) * b[1] + T(0, 6) * b[2];  \
    y[1] = T(1, 0) * p[0] + T(1, 1) * p[1] + T(1, 2) * p[2] + T(1, 3) * p[3] + T(1, 4) * b[0] + T(1, 5) * b[1] + T(1, 6) * b[2];  \
    y[2] = T(2, 0) * p[0] + T(2, 1) * p[1] + T(2, 2) * p[2] + T(2, 3) * p[3] + T(2, 4) * b[0] + T(2, 5) * b[1] + T(2, 6) * b[2];  \
    y[3] = T(3, 0) * p[0] + T(3, 1) * p[1] + T(3, 2) * p[2] + T(3, 3) * p[3] + T(3, 4) * b[0] + T(3, 5) * b[1] + T(3, 6) * b[2];  \
    l[i0] = T(4, 0) * p[0] + T(4, 1) * p[1] + T(4, 2) * p[2] + T(4, 3) * p[3] + T(4, 4) * b[0] + T(4, 5) * b[1] + T(4, 6) * b[2]; \
    l[i1] = T(5, 0) * p[0] + T(5, 1) * p[1] + T(5, 2) * p[2] + T(5, 3) * p[3] + T(5, 4) * b[0] + T(5, 5) * b[1] + T(5, 6) * b[2]; \
    l[i2] = T(6, 0) * p[0] + T(6, 1) * p[1] + T(6, 2) * p[2] + T(6, 3) * p[3] + T(6, 4) * b[0] + T(6, 5) * b[1] + T(6, 6) * b[2]; \
    l[i0] = (active_set_signs & (1 << i0)) ? -l[i0] : l[i0];                                                                      \
    l[i1] = (active_set_signs & (1 << i1)) ? -l[i1] : l[i1];                                                                      \
    l[i2] = (active_set_signs & (1 << i2)) ? -l[i2] : l[i2]

#define SOLVE8()                                                                                                                                  \
    constexpr auto T = mergeBlocks(Q, D.t(), D).inv().round(ROUND_BIT);                                                                           \
    float b[4];                                                                                                                                   \
    b[0] = (active_set_signs & 0x1) ? -current_limit(0) : current_limit(0);                                                                       \
    b[1] = (active_set_signs & 0x2) ? -current_limit(1) : current_limit(1);                                                                       \
    b[2] = (active_set_signs & 0x4) ? -current_limit(2) : current_limit(2);                                                                       \
    b[3] = (active_set_signs & 0x8) ? -current_limit(3) : current_limit(3);                                                                       \
    y[0] = T(0, 0) * p[0] + T(0, 1) * p[1] + T(0, 2) * p[2] + T(0, 3) * p[3] + T(0, 4) * b[0] + T(0, 5) * b[1] + T(0, 6) * b[2] + T(0, 7) * b[3]; \
    y[1] = T(1, 0) * p[0] + T(1, 1) * p[1] + T(1, 2) * p[2] + T(1, 3) * p[3] + T(1, 4) * b[0] + T(1, 5) * b[1] + T(1, 6) * b[2] + T(1, 7) * b[3]; \
    y[2] = T(2, 0) * p[0] + T(2, 1) * p[1] + T(2, 2) * p[2] + T(2, 3) * p[3] + T(2, 4) * b[0] + T(2, 5) * b[1] + T(2, 6) * b[2] + T(2, 7) * b[3]; \
    y[3] = T(3, 0) * p[0] + T(3, 1) * p[1] + T(3, 2) * p[2] + T(3, 3) * p[3] + T(3, 4) * b[0] + T(3, 5) * b[1] + T(3, 6) * b[2] + T(3, 7) * b[3]; \
    l[0] = T(4, 0) * p[0] + T(4, 1) * p[1] + T(4, 2) * p[2] + T(4, 3) * p[3] + T(4, 4) * b[0] + T(4, 5) * b[1] + T(4, 6) * b[2] + T(4, 7) * b[3]; \
    l[1] = T(5, 0) * p[0] + T(5, 1) * p[1] + T(5, 2) * p[2] + T(5, 3) * p[3] + T(5, 4) * b[0] + T(5, 5) * b[1] + T(5, 6) * b[2] + T(5, 7) * b[3]; \
    l[2] = T(6, 0) * p[0] + T(6, 1) * p[1] + T(6, 2) * p[2] + T(6, 3) * p[3] + T(6, 4) * b[0] + T(6, 5) * b[1] + T(6, 6) * b[2] + T(6, 7) * b[3]; \
    l[3] = T(7, 0) * p[0] + T(7, 1) * p[1] + T(7, 2) * p[2] + T(7, 3) * p[3] + T(7, 4) * b[0] + T(7, 5) * b[1] + T(7, 6) * b[2] + T(7, 7) * b[3]; \
    l[0] = (active_set_signs & 0x1) ? -l[0] : l[0];                                                                                               \
    l[1] = (active_set_signs & 0x2) ? -l[1] : l[1];                                                                                               \
    l[2] = (active_set_signs & 0x4) ? -l[2] : l[2];                                                                                               \
    l[3] = (active_set_signs & 0x8) ? -l[3] : l[3]

static constexpr inline float sqr(float x) {
    return x * x;
}

bool AccelerationLimitter::compute(const Vector4f& accel_in, const Vector4f& current_limit, Vector4f& accel_out, Vector4f& current_out) {
    const float WHEEL_POS_R = sqrt(WHEEL_POS_R_2);
    const float KX = WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT * MACHINE_WEIGHT * WHEEL_POS_R / WHEEL_POS_Y / 4;
    const float KY = WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT * MACHINE_WEIGHT * WHEEL_POS_R / WHEEL_POS_X / 4;
    const float KW = WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT * MACHINE_INERTIA / 4 / WHEEL_POS_R;
    const float KC = WHEEL_RADIUS / MOTOR_TORQUE_CONSTANT * MACHINE_WEIGHT / 4;

    const float p[4] = {
        KX * WEIGHT_X * accel_in(0),
        KY * WEIGHT_Y * accel_in(1),
        KW * WEIGHT_W * accel_in(2),
        KC * WEIGHT_C * accel_in(3),
    };
    constexpr ConstMatrix4f Q = {WEIGHT_X, 0, 0, 0, 0, WEIGHT_Y, 0, 0, 0, 0, WEIGHT_W, 0, 0, 0, 0, WEIGHT_C};
    constexpr ConstMatrix4f D = {-1, 1, 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1};
    constexpr int ROUND_BIT = 10;

    float x[4] = {0, 0, 0, 0};
    int active_set_bits = 0;
    int active_set_signs = 0;

    // 反復は最大5回で終了する
    int iteration;
    for (iteration = 0; iteration < 5; iteration++) {
        // 現在の制約下でラグランジュの未定乗数法を解く
        float y[4], l[4];
        l[0] = std::numeric_limits<float>::infinity();
        l[1] = std::numeric_limits<float>::infinity();
        l[2] = std::numeric_limits<float>::infinity();
        l[3] = std::numeric_limits<float>::infinity();
        switch (active_set_bits) {
        case 0x0:
            {
                SOLVE4();
                break;
            }

        case 0x1:
            {
                SOLVE5(0);
                break;
            }

        case 0x2:
            {
                SOLVE5(1);
                break;
            }

        case 0x3:
            {
                SOLVE6(0, 1);
                break;
            }

        case 0x4:
            {
                SOLVE5(2);
                break;
            }

        case 0x5:
            {
                SOLVE6(0, 2);
                break;
            }

        case 0x6:
            {
                SOLVE6(1, 2);
                break;
            }

        case 0x7:
            {
                SOLVE7(0, 1, 2);
                break;
            }

        case 0x8:
            {
                SOLVE5(3);
                break;
            }

        case 0x9:
            {
                SOLVE6(0, 3);
                break;
            }

        case 0xA:
            {
                SOLVE6(1, 3);
                break;
            }

        case 0xB:
            {
                SOLVE7(0, 1, 3);
                break;
            }

        case 0xC:
            {
                SOLVE6(2, 3);
                break;
            }

        case 0xD:
            {
                SOLVE7(0, 2, 3);
                break;
            }

        case 0xE:
            {
                SOLVE7(1, 2, 3);
                break;
            }

        case 0xF:
            {
                SOLVE8();
                break;
            }

        default:
            active_set_bits = 0;
            y[0] = 0.0f;
            y[1] = 0.0f;
            y[2] = 0.0f;
            y[3] = 0.0f;
            break;
        }

        // printf("y=[%f; %f; %f; %f], lambda=[%f; %f; %f; %f]\n", y[0], y[1], y[2], y[3], l[0], l[1], l[2], l[3]);

        float current[4];
        current[0] = D(0, 0) * y[0] + D(0, 1) * y[1] + D(0, 2) * y[2] + D(0, 3) * y[3];
        current[1] = D(1, 0) * y[0] + D(1, 1) * y[1] + D(1, 2) * y[2] + D(1, 3) * y[3];
        current[2] = D(2, 0) * y[0] + D(2, 1) * y[1] + D(2, 2) * y[2] + D(2, 3) * y[3];
        current[3] = D(3, 0) * y[0] + D(3, 1) * y[1] + D(3, 2) * y[2] + D(3, 3) * y[3];

        // yが実行可能か調べる
        float over_current = fpu::max(fpu::max(fabs(current[0]) - current_limit[0], fabs(current[1]) - current_limit[1]),
                                      fpu::max(fabs(current[2]) - current_limit[2], fabs(current[3]) - current_limit[3]));
        float norm2 = (sqr(y[0] - x[0]) + sqr(y[1] - x[1]) + sqr(y[2] - x[2]) + sqr(y[3] - x[3]));
        if ((over_current < 1e-6f) || (norm2 < 1e-12f)) {
            // yは実行可能領域内にある
            x[0] = y[0];
            x[1] = y[1];
            x[2] = y[2];
            x[3] = y[3];

            // 収束判定
            if (0.0f <= fpu::min(fpu::min(l[0], l[1]), fpu::min(l[2], l[3]))) {
                // 収束した
                accel_out[0] = x[0] * (1.0f / KX);
                accel_out[1] = x[1] * (1.0f / KY);
                accel_out[2] = x[2] * (1.0f / KW);
                accel_out[3] = x[3] * (1.0f / KC);
                current_out[0] = fpu::clamp(current[0], -current_limit[0], current_limit[0]);
                current_out[1] = fpu::clamp(current[1], -current_limit[1], current_limit[1]);
                current_out[2] = fpu::clamp(current[2], -current_limit[2], current_limit[2]);
                current_out[3] = fpu::clamp(current[3], -current_limit[3], current_limit[3]);
                // printf("Finished\n");
                return true;
            }
            else {
                // 収束しなかった
                // 不要な制約をアクティブセットから削除する
                int index = 0;
                float min_lambda = l[0];
                if (l[1] < min_lambda) {
                    min_lambda = l[1];
                    index = 1;
                }
                if (l[2] < min_lambda) {
                    min_lambda = l[2];
                    index = 2;
                }
                if (l[3] < min_lambda) {
                    min_lambda = l[3];
                    index = 3;
                }
                active_set_bits &= ~(1 << index);
                // printf("Delete set %d, lambda=%f\n", index, min_lambda);
            }
        }
        else {
            // yは実行可能領域外にある
            // xからyにいたる経路で始めに当たる制約条件を見つけてアクティブセットに追加する
            int min_lower_bound_index = -1, min_upper_bound_index = -1;
            float min_lower_bound_t = std::numeric_limits<float>::infinity();
            float min_upper_bound_t = std::numeric_limits<float>::infinity();
            const float d[4] = {
                y[0] - x[0],
                y[1] - x[1],
                y[2] - x[2],
                y[3] - x[3],
            };
            for (int index = 0; index < 4; index++) {
                if (active_set_bits & (1 << index)) {
                    continue;
                }
                float ax, ad;
                switch (index) {
                case 0:
                    {
                        constexpr auto A = D.extractRow(0);
                        ax = A(0, 0) * x[0] + A(0, 1) * x[1] + A(0, 2) * x[2] + A(0, 3) * x[3];
                        ad = A(0, 0) * d[0] + A(0, 1) * d[1] + A(0, 2) * d[2] + A(0, 3) * d[3];
                        break;
                    }
                case 1:
                    {
                        constexpr auto A = D.extractRow(1);
                        ax = A(0, 0) * x[0] + A(0, 1) * x[1] + A(0, 2) * x[2] + A(0, 3) * x[3];
                        ad = A(0, 0) * d[0] + A(0, 1) * d[1] + A(0, 2) * d[2] + A(0, 3) * d[3];
                        break;
                    }
                case 2:
                    {
                        constexpr auto A = D.extractRow(2);
                        ax = A(0, 0) * x[0] + A(0, 1) * x[1] + A(0, 2) * x[2] + A(0, 3) * x[3];
                        ad = A(0, 0) * d[0] + A(0, 1) * d[1] + A(0, 2) * d[2] + A(0, 3) * d[3];
                        break;
                    }
                case 3:
                    {
                        constexpr auto A = D.extractRow(3);
                        ax = A(0, 0) * x[0] + A(0, 1) * x[1] + A(0, 2) * x[2] + A(0, 3) * x[3];
                        ad = A(0, 0) * d[0] + A(0, 1) * d[1] + A(0, 2) * d[2] + A(0, 3) * d[3];
                        break;
                    }
                }
                float reci_ad = 1.0f / ad;
                float lb_t = (-current_limit[index] - ax) * reci_ad;
                float ub_t = (current_limit[index] - ax) * reci_ad;
                if ((0.0f <= lb_t) && (lb_t < min_lower_bound_t)) {
                    min_lower_bound_t = lb_t;
                    min_lower_bound_index = index;
                }
                if ((0.0f <= ub_t) && (ub_t < min_upper_bound_t)) {
                    min_upper_bound_t = ub_t;
                    min_upper_bound_index = index;
                }
            }
            if (min_lower_bound_t < min_upper_bound_t) {
                x[0] += min_lower_bound_t * d[0];
                x[1] += min_lower_bound_t * d[1];
                x[2] += min_lower_bound_t * d[2];
                x[3] += min_lower_bound_t * d[3];
                active_set_bits |= 1 << min_lower_bound_index;
                active_set_signs |= 1 << min_lower_bound_index;
                // printf("New set -%d, t=%f\n", min_lower_bound_index, min_lower_bound_t);
            }
            else {
                x[0] += min_upper_bound_t * d[0];
                x[1] += min_upper_bound_t * d[1];
                x[2] += min_upper_bound_t * d[2];
                x[3] += min_upper_bound_t * d[3];
                active_set_bits |= 1 << min_upper_bound_index;
                active_set_signs &= ~(1 << min_upper_bound_index);
                // printf("New set %d, t=%f\n", min_upper_bound_index, min_upper_bound_t);
            }
        }
    }

    return false;
}
