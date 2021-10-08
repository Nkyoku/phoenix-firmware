/**
 * @file velocity_filter.cpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#include "velocity_filter.hpp"
#include "board.hpp"
#include "fpu.hpp"
#include <math.h>

#ifdef _MSC_VER
#define MUL(dst, src1, src2) dst = src1 * src2
#define ACC(dst, src)        dst += src
#define SAC(dst, src)        dst -= src
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#define MUL(dst, src1, src2) __asm__ __volatile__("custom 252,%0,%1,%2" : "=r"(dst) : "r"(src1), "r"(src2) : "memory")
#define ACC(dst, src)        __asm__ __volatile__("custom 253,%0,%1,%2" : "=r"(dst) : "r"(dst), "r"(src) : "memory")
#define SAC(dst, src)        __asm__ __volatile__("custom 254,%0,%1,%2" : "=r"(dst) : "r"(dst), "r"(src) : "memory")
#endif

using namespace Eigen;

/// 車輪速度の標準偏差 [m/s]
static constexpr float SIGMA_VELOCITY = 0.002f;

/// 電流測定値の標準偏差 [A]
static constexpr float SIGMA_CURRENT = 0.01f;

/// 加速度センサーの標準偏差 [m/s^2]
static constexpr float SIGMA_ACCELEROMETER = 0.01f;

/// ジャイロセンサーの標準偏差 [rad/s]
static constexpr float SIGMA_GYROSCOPE = 0.01f;

/// 摩擦係数の変化割合
static constexpr float SIGMA_KF = 100.0f;

/// 摩擦係数の最小値
static constexpr float MIN_KF = 1.0f;

/// 摩擦係数の最大値
static constexpr float MAX_KF = 1000.0f;

/**
 * @brief wheel_velocity - velocity を計算する
 */
static inline Vector4f wheelVelocityError(const Vector4f& wheel_velocity, float vx, float vy, float omega) {
    Vector4f error;
    vx *= (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2));
    vy *= (WHEEL_POS_X / sqrt(WHEEL_POS_R_2));
    omega *= sqrt(WHEEL_POS_R_2);
    error(0) = wheel_velocity(0) - (omega - vx + vy);
    error(1) = wheel_velocity(1) - (omega + vx + vy);
    error(2) = wheel_velocity(2) - (omega + vx - vy);
    error(3) = wheel_velocity(3) - (omega - vx - vy);
    return error;
}

/**
 * @brief C = A * B を計算する
 */
static void matmul(const Matrix<float, 7, 7>& A, const Matrix<float, 7, 7>& B, Matrix<float, 7, 7>& C) {
    for (size_t col = 0; col < 7; col++) {
        float b0 = B(0, col);
        float b1 = B(1, col);
        float b2 = B(2, col);
        float b3 = B(3, col);
        float b4 = B(4, col);
        float b5 = B(5, col);
        float b6 = B(6, col);
        for (size_t row = 0; row < 7; row++) {
            float s0, s1, s2, s3, s4, s5, s6;
            float a0, a1, a2, a3, a4, a5, a6;
            a0 = A(row, 0);
            a1 = A(row, 1);
            MUL(s0, a0, b0);
            MUL(s1, a1, b1);
            a2 = A(row, 2);
            ACC(s0, s1);
            MUL(s2, a2, b2);
            a3 = A(row, 3);
            ACC(s0, s2);
            MUL(s3, a3, b3);
            a4 = A(row, 4);
            ACC(s0, s3);
            MUL(s4, a4, b4);
            a5 = A(row, 5);
            ACC(s0, s4);
            MUL(s5, a5, b5);
            a6 = A(row, 6);
            ACC(s0, s5);
            MUL(s6, a6, b6);
            ACC(s0, s6);
            C(row, col) = s0;
        }
    }
}

/**
 * @brief C = A * B^T を計算する
 */
static void matmult(const Matrix<float, 7, 7>& A, const Matrix<float, 7, 7>& B, Matrix<float, 7, 7>& C) {
    for (size_t col = 0; col < 7; col++) {
        float b0 = B(col, 0);
        float b1 = B(col, 1);
        float b2 = B(col, 2);
        float b3 = B(col, 3);
        float b4 = B(col, 4);
        float b5 = B(col, 5);
        float b6 = B(col, 6);
        for (size_t row = 0; row < 7; row++) {
            float s0, s1, s2, s3, s4, s5, s6;
            float a0, a1, a2, a3, a4, a5, a6;
            a0 = A(row, 0);
            a1 = A(row, 1);
            MUL(s0, a0, b0);
            MUL(s1, a1, b1);
            a2 = A(row, 2);
            ACC(s0, s1);
            MUL(s2, a2, b2);
            a3 = A(row, 3);
            ACC(s0, s2);
            MUL(s3, a3, b3);
            a4 = A(row, 4);
            ACC(s0, s3);
            MUL(s4, a4, b4);
            a5 = A(row, 5);
            ACC(s0, s4);
            MUL(s5, a5, b5);
            a6 = A(row, 6);
            ACC(s0, s5);
            MUL(s6, a6, b6);
            ACC(s0, s6);
            C(row, col) = s0;
        }
    }
}

/**
 * @brief C = A * B の下三角行列のみ計算して対称行列を作成する
 */
static void matmuls(const Matrix<float, 7, 7>& A, const Matrix<float, 7, 7>& B, Matrix<float, 7, 7>& C) {
    for (size_t col = 0; col < 7; col++) {
        float b0 = B(0, col);
        float b1 = B(1, col);
        float b2 = B(2, col);
        float b3 = B(3, col);
        float b4 = B(4, col);
        float b5 = B(5, col);
        float b6 = B(6, col);
        for (size_t row = col; row < 7; row++) {
            float s0, s1, s2, s3, s4, s5, s6;
            float a0, a1, a2, a3, a4, a5, a6;
            a0 = A(row, 0);
            a1 = A(row, 1);
            MUL(s0, a0, b0);
            MUL(s1, a1, b1);
            a2 = A(row, 2);
            ACC(s0, s1);
            MUL(s2, a2, b2);
            a3 = A(row, 3);
            ACC(s0, s2);
            MUL(s3, a3, b3);
            a4 = A(row, 4);
            ACC(s0, s3);
            MUL(s4, a4, b4);
            a5 = A(row, 5);
            ACC(s0, s4);
            MUL(s5, a5, b5);
            a6 = A(row, 6);
            ACC(s0, s5);
            MUL(s6, a6, b6);
            ACC(s0, s6);
            C(row, col) = s0;
            C(col, row) = s0;
        }
    }
}

/**
 * @brief A * B * A^T を計算する
 */
static inline void matmulmult(const Matrix<float, 7, 7>& A, const Matrix<float, 7, 7>& B, Matrix<float, 7, 7>& A_B_AT) {
    Matrix<float, 7, 7> B_AT;
    matmult(B, A, B_AT);
    matmuls(A, B_AT, A_B_AT);
}

/**
 * @brief A * B * A^T を計算する
 */
static inline void matmulmult(const Matrix<float, 7, 7>& A, const Matrix<float, 7, 7>& B, Matrix<float, 7, 7>& A_B_AT, Matrix<float, 7, 7>& B_AT) {
    matmult(B, A, B_AT);
    matmuls(A, B_AT, A_B_AT);
}

/**
 * @brief 対称行列の逆行列を計算する。符号を反転する
 */
static inline void invmuls(const Matrix<float, 7, 7>& A, Matrix<float, 7, 7>& L, Matrix<float, 7, 7>& invL, Matrix<float, 7, 7>& invA) {
    // コレスキー分解する
    for (size_t row = 0; row < 7; row++) {
        for (size_t col = 0;;) {
            float sum = A(row, col);
            for (size_t i = 0; i < col; i++) {
                sum -= L(row, i) * L(col, i);
            }
            if (row != col) {
                L(row, col) = sum * invL(col, col);
                col++;
            }
            else {
                L(row, col) = fpu::sqrt(sum);
                invL(row, col) = 1.0f / L(row, col);
                break;
            }
        }
    }

    // 下三角行列の逆行列を計算する
    for (size_t col = 0; col < 6; col++) {
        for (size_t row = col + 1; row < 7; row++) {
            float sum = 0.0f;
            for (size_t i = 0; i < row; i++) {
                sum -= L(row, i) * invL(i, col);
            }
            invL(row, col) = sum * invL(row, row);
        }
    }

    // A^-1 = -L^-T * L^-1
    for (size_t col = 0; col < 7; col++) {
        float b0 = -invL(0, col);
        float b1 = invL(1, col);
        float b2 = invL(2, col);
        float b3 = invL(3, col);
        float b4 = invL(4, col);
        float b5 = invL(5, col);
        float b6 = invL(6, col);
        for (size_t row = col; row < 7; row++) {
            float s0, s1, s2, s3, s4, s5, s6;
            float a0, a1, a2, a3, a4, a5, a6;
            a0 = invL(0, row);
            a1 = invL(1, row);
            MUL(s0, a0, b0);
            MUL(s1, a1, b1);
            a2 = invL(2, row);
            SAC(s0, s1);
            MUL(s2, a2, b2);
            a3 = invL(3, row);
            SAC(s0, s2);
            MUL(s3, a3, b3);
            a4 = invL(4, row);
            SAC(s0, s3);
            MUL(s4, a4, b4);
            a5 = invL(5, row);
            SAC(s0, s4);
            MUL(s5, a5, b5);
            a6 = invL(6, row);
            SAC(s0, s5);
            MUL(s6, a6, b6);
            SAC(s0, s6);
            invA(row, col) = s0;
            invA(col, row) = s0;
        }
    }
}

/**
 * @brief d = A * b + c を計算する
 */
static inline void matmuladdvec(const Matrix<float, 7, 7>& A, const Matrix<float, 7, 1>& b, const Matrix<float, 7, 1>& c, Matrix<float, 7, 1>& d) {
    float d0 = c(0);
    float d1 = c(1);
    float d2 = c(2);
    float d3 = c(3);
    float d4 = c(4);
    float d5 = c(5);
    float d6 = c(6);
    for (size_t col = 0; col < 7; col++) {
        float s0, s1, s2, s3, s4, s5, s6;
        float a0, a1, a2, a3, a4, a5, a6;
        float m = b(col);
        a0 = A(0, col);
        a1 = A(1, col);
        MUL(s0, a0, m);
        a2 = A(2, col);
        ACC(d0, s0);
        MUL(s1, a1, m);
        a3 = A(3, col);
        ACC(d1, s1);
        MUL(s2, a2, m);
        a4 = A(4, col);
        ACC(d2, s2);
        MUL(s3, a3, m);
        a5 = A(5, col);
        ACC(d3, s3);
        MUL(s4, a4, m);
        a6 = A(6, col);
        ACC(d4, s4);
        MUL(s5, a5, m);
        MUL(s6, a6, m);
        ACC(d5, s5);
        ACC(d6, s6);
    }
    d(0) = d0;
    d(1) = d1;
    d(2) = d2;
    d(3) = d3;
    d(4) = d4;
    d(5) = d5;
    d(6) = d6;
}

void VelocityFilter::reset(void) {
    memset(this, 0, sizeof(*this));
    G(3, 3) = 1.0f;
    G(4, 4) = 1.0f;
    G(5, 5) = 1.0f;
    G(6, 6) = 1.0f;
    H(6, 2) = 1.0f;
}

void VelocityFilter::update(const Vector3f& accel, const Vector3f& gyro, const Vector4f& wheel_velocity, const Vector4f& wheel_current) {
    // 定数の定義
    constexpr float DELTA_TIME = 1.0f / IMU_OUTPUT_RATE;
    const float WHEEL_POS_R = sqrt(WHEEL_POS_R_2);
    const float COS_PHI = WHEEL_POS_X / WHEEL_POS_R;
    const float SIN_PHI = WHEEL_POS_Y / WHEEL_POS_R;

    // 車輪速度を微分する
    Vector4f domega;
    domega(0) = (IMU_OUTPUT_RATE / WHEEL_RADIUS) * (wheel_velocity(0) - _last_wheel_velocity(0));
    domega(1) = (IMU_OUTPUT_RATE / WHEEL_RADIUS) * (wheel_velocity(1) - _last_wheel_velocity(1));
    domega(2) = (IMU_OUTPUT_RATE / WHEEL_RADIUS) * (wheel_velocity(2) - _last_wheel_velocity(2));
    domega(3) = (IMU_OUTPUT_RATE / WHEEL_RADIUS) * (wheel_velocity(3) - _last_wheel_velocity(3));
    _last_wheel_velocity(0) = wheel_velocity(0);
    _last_wheel_velocity(1) = wheel_velocity(1);
    _last_wheel_velocity(2) = wheel_velocity(2);
    _last_wheel_velocity(3) = wheel_velocity(3);

    // 事前状態推定値を求める
    float vx = _mu(0);
    float vy = _mu(1);
    float Omega = _mu(2);
    Vector4f kf, force;
    kf(0) = _mu(3);
    kf(1) = _mu(4);
    kf(2) = _mu(5);
    kf(3) = _mu(6);
    Vector4f romega_minus_v = wheelVelocityError(wheel_velocity, vx, vy, Omega);
    force(0) = kf(0) * romega_minus_v(0);
    force(1) = kf(1) * romega_minus_v(1);
    force(2) = kf(2) * romega_minus_v(2);
    force(3) = kf(3) * romega_minus_v(3);
    Vector7f mu_hat;
    mu_hat(0) = _mu(0) + (SIN_PHI / MACHINE_WEIGHT * DELTA_TIME) * (force(1) - force(0) + force(2) - force(3)) + Omega * vy * DELTA_TIME;
    mu_hat(1) = _mu(1) + (COS_PHI / MACHINE_WEIGHT * DELTA_TIME) * (force(0) + force(1) - force(2) - force(3)) - Omega * vx * DELTA_TIME;
    mu_hat(2) = _mu(2) + (WHEEL_POS_R / MACHINE_INERTIA * DELTA_TIME) * (force(0) + force(1) + force(2) + force(3));
    mu_hat(3) = _mu(3);
    mu_hat(4) = _mu(4);
    mu_hat(5) = _mu(5);
    mu_hat(6) = _mu(6);

    // 状態方程式を線形化する
    float kf_sum = kf(0) + kf(1) + kf(2) + kf(3);
    G(0, 0) = 1.0f - (DELTA_TIME * SIN_PHI / MACHINE_WEIGHT * SIN_PHI) * kf_sum;
    G(0, 1) = (DELTA_TIME * SIN_PHI / MACHINE_WEIGHT * COS_PHI) * (kf(0) - kf(1) + kf(2) - kf(3)) + DELTA_TIME * Omega;
    G(0, 2) = (DELTA_TIME * SIN_PHI / MACHINE_WEIGHT * WHEEL_POS_R) * (kf(0) - kf(1) - kf(2) + kf(3)) + DELTA_TIME * vy;
    G(0, 3) = (-DELTA_TIME * SIN_PHI / MACHINE_WEIGHT) * romega_minus_v(0);
    G(0, 4) = (DELTA_TIME * SIN_PHI / MACHINE_WEIGHT) * romega_minus_v(1);
    G(0, 5) = (DELTA_TIME * SIN_PHI / MACHINE_WEIGHT) * romega_minus_v(2);
    G(0, 6) = (-DELTA_TIME * SIN_PHI / MACHINE_WEIGHT) * romega_minus_v(3);
    G(1, 0) = (DELTA_TIME * COS_PHI / MACHINE_WEIGHT * SIN_PHI) * (kf(0) - kf(1) + kf(2) - kf(3)) - DELTA_TIME * Omega;
    G(1, 1) = 1.0f - (DELTA_TIME * COS_PHI / MACHINE_WEIGHT * COS_PHI) * kf_sum;
    G(1, 2) = (DELTA_TIME * COS_PHI / MACHINE_WEIGHT) * WHEEL_POS_R * (kf(2) - kf(0) - kf(1) + kf(3)) - DELTA_TIME * vx;
    G(1, 3) = (DELTA_TIME * COS_PHI / MACHINE_WEIGHT) * romega_minus_v(0);
    G(1, 4) = (DELTA_TIME * COS_PHI / MACHINE_WEIGHT) * romega_minus_v(1);
    G(1, 5) = (-DELTA_TIME * COS_PHI / MACHINE_WEIGHT) * romega_minus_v(2);
    G(1, 6) = (-DELTA_TIME * COS_PHI / MACHINE_WEIGHT) * romega_minus_v(3);
    G(2, 0) = (DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA * SIN_PHI) * (kf(0) - kf(1) - kf(2) + kf(3));
    G(2, 1) = (DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA * COS_PHI) * (kf(2) - kf(0) - kf(1) + kf(3));
    G(2, 2) = 1.0f - (DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA * WHEEL_POS_R) * kf_sum;
    G(2, 3) = (DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA) * romega_minus_v(0);
    G(2, 4) = (DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA) * romega_minus_v(1);
    G(2, 5) = (DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA) * romega_minus_v(2);
    G(2, 6) = (DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA) * romega_minus_v(3);

    // 事前誤差を計算する
    Matrix7f S_hat;
    matmulmult(G, _sigma, S_hat);
    float kf_sum_2 = kf_sum * kf_sum;
    S_hat(0, 0) += powf((DELTA_TIME * SIN_PHI / MACHINE_WEIGHT * WHEEL_RADIUS * SIGMA_VELOCITY), 2) * kf_sum_2;
    S_hat(1, 1) += powf((DELTA_TIME * COS_PHI / MACHINE_WEIGHT * WHEEL_RADIUS * SIGMA_VELOCITY), 2) * kf_sum_2;
    S_hat(2, 2) += powf((DELTA_TIME * WHEEL_POS_R / MACHINE_INERTIA * WHEEL_RADIUS * SIGMA_VELOCITY), 2) * kf_sum_2;
    S_hat(3, 3) += powf(DELTA_TIME * SIGMA_KF, 2);
    S_hat(4, 4) += powf(DELTA_TIME * SIGMA_KF, 2);
    S_hat(5, 5) += powf(DELTA_TIME * SIGMA_KF, 2);
    S_hat(6, 6) += powf(DELTA_TIME * SIGMA_KF, 2);

    // 観測予測値を計算する
    float vx_hat = mu_hat(0);
    float vy_hat = mu_hat(1);
    float Omega_hat = mu_hat(2);
    Vector4f kf_hat, force_hat;
    kf_hat(0) = mu_hat(3);
    kf_hat(1) = mu_hat(4);
    kf_hat(2) = mu_hat(5);
    kf_hat(3) = mu_hat(6);
    Vector4f romega_minus_v_hat = wheelVelocityError(wheel_velocity, vx_hat, vy_hat, Omega_hat);
    force_hat(0) = kf_hat(0) * romega_minus_v_hat(0);
    force_hat(1) = kf_hat(1) * romega_minus_v_hat(1);
    force_hat(2) = kf_hat(2) * romega_minus_v_hat(2);
    force_hat(3) = kf_hat(3) * romega_minus_v_hat(3);
    Vector7f h_minus_z; // 観測値と予測値との差
    h_minus_z(0) = (MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA) * wheel_current(0) - (WHEEL_RADIUS / WHEEL_INERTIA) * force_hat(0) - domega(0);
    h_minus_z(1) = (MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA) * wheel_current(1) - (WHEEL_RADIUS / WHEEL_INERTIA) * force_hat(1) - domega(1);
    h_minus_z(2) = (MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA) * wheel_current(2) - (WHEEL_RADIUS / WHEEL_INERTIA) * force_hat(2) - domega(2);
    h_minus_z(3) = (MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA) * wheel_current(3) - (WHEEL_RADIUS / WHEEL_INERTIA) * force_hat(3) - domega(3);
    h_minus_z(4) = (SIN_PHI / MACHINE_WEIGHT) * (force_hat(1) - force_hat(0) + force_hat(2) - force_hat(3)) - accel.x();
    h_minus_z(5) = (COS_PHI / MACHINE_WEIGHT) * (force_hat(0) + force_hat(1) - force_hat(2) - force_hat(3)) - accel.y();
    h_minus_z(6) = Omega_hat - gyro.z();

    // 観測方程式を線形化する
    float kf_hat_sum = kf_hat(0) + kf_hat(1) + kf_hat(2) + kf_hat(3);
    H(0, 0) = (-WHEEL_RADIUS / WHEEL_INERTIA * SIN_PHI) * kf_hat(0);
    H(1, 0) = (WHEEL_RADIUS / WHEEL_INERTIA * SIN_PHI) * kf_hat(1);
    H(2, 0) = (WHEEL_RADIUS / WHEEL_INERTIA * SIN_PHI) * kf_hat(2);
    H(3, 0) = (-WHEEL_RADIUS / WHEEL_INERTIA * SIN_PHI) * kf_hat(3);
    H(0, 1) = (WHEEL_RADIUS / WHEEL_INERTIA * COS_PHI) * kf_hat(0);
    H(1, 1) = (WHEEL_RADIUS / WHEEL_INERTIA * COS_PHI) * kf_hat(1);
    H(2, 1) = (-WHEEL_RADIUS / WHEEL_INERTIA * COS_PHI) * kf_hat(2);
    H(3, 1) = (-WHEEL_RADIUS / WHEEL_INERTIA * COS_PHI) * kf_hat(3);
    H(0, 2) = (WHEEL_RADIUS / WHEEL_INERTIA * WHEEL_POS_R) * kf_hat(0);
    H(1, 2) = (WHEEL_RADIUS / WHEEL_INERTIA * WHEEL_POS_R) * kf_hat(1);
    H(2, 2) = (WHEEL_RADIUS / WHEEL_INERTIA * WHEEL_POS_R) * kf_hat(2);
    H(3, 2) = (WHEEL_RADIUS / WHEEL_INERTIA * WHEEL_POS_R) * kf_hat(3);
    H(0, 3) = (-WHEEL_RADIUS / WHEEL_INERTIA) * romega_minus_v_hat(0);
    H(1, 4) = (-WHEEL_RADIUS / WHEEL_INERTIA) * romega_minus_v_hat(1);
    H(2, 5) = (-WHEEL_RADIUS / WHEEL_INERTIA) * romega_minus_v_hat(2);
    H(3, 6) = (-WHEEL_RADIUS / WHEEL_INERTIA) * romega_minus_v_hat(3);
    H(4, 0) = (-SIN_PHI / MACHINE_WEIGHT * SIN_PHI) * kf_hat_sum;
    H(4, 1) = (SIN_PHI / MACHINE_WEIGHT * COS_PHI) * (kf_hat(0) - kf_hat(1) + kf_hat(2) - kf_hat(3));
    H(4, 2) = (SIN_PHI / MACHINE_WEIGHT * WHEEL_POS_R) * (kf_hat(0) - kf_hat(1) - kf_hat(2) + kf_hat(3));
    H(4, 3) = (-SIN_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(0);
    H(4, 4) = (SIN_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(1);
    H(4, 5) = (SIN_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(2);
    H(4, 6) = (-SIN_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(3);
    H(5, 0) = (COS_PHI / MACHINE_WEIGHT * SIN_PHI) * (kf_hat(0) - kf_hat(1) + kf_hat(2) - kf_hat(3));
    H(5, 1) = (-COS_PHI / MACHINE_WEIGHT * COS_PHI) * kf_hat_sum;
    H(5, 2) = (COS_PHI / MACHINE_WEIGHT * WHEEL_POS_R) * (kf_hat(2) - kf_hat(0) - kf_hat(1) + kf_hat(3));
    H(5, 3) = (COS_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(0);
    H(5, 4) = (COS_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(1);
    H(5, 5) = (-COS_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(2);
    H(5, 6) = (-COS_PHI / MACHINE_WEIGHT) * romega_minus_v_hat(3);

    // 事前誤差を更新する
    Matrix7f S_hat_HT, H_S_hat_HT;
    matmulmult(H, S_hat, H_S_hat_HT, S_hat_HT);
    H_S_hat_HT(0, 0) += powf(MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA * SIGMA_CURRENT, 2) + powf(SIGMA_VELOCITY / DELTA_TIME, 2);
    H_S_hat_HT(1, 1) += powf(MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA * SIGMA_CURRENT, 2) + powf(SIGMA_VELOCITY / DELTA_TIME, 2);
    H_S_hat_HT(2, 2) += powf(MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA * SIGMA_CURRENT, 2) + powf(SIGMA_VELOCITY / DELTA_TIME, 2);
    H_S_hat_HT(3, 3) += powf(MOTOR_TORQUE_CONSTANT / WHEEL_INERTIA * SIGMA_CURRENT, 2) + powf(SIGMA_VELOCITY / DELTA_TIME, 2);
    H_S_hat_HT(4, 4) += powf(SIGMA_ACCELEROMETER, 2);
    H_S_hat_HT(5, 5) += powf(SIGMA_ACCELEROMETER, 2);
    H_S_hat_HT(6, 6) += powf(SIGMA_GYROSCOPE, 2);
    Matrix7f inv_H_S_hat_HT;
    invmuls(H_S_hat_HT, L, invL, inv_H_S_hat_HT);

    // カルマンゲインを計算する
    Matrix7f K, K_H;
    matmul(S_hat_HT, inv_H_S_hat_HT, K);
    matmul(K, H, K_H);
    K_H(0, 0) += 1.0f;
    K_H(1, 1) += 1.0f;
    K_H(2, 2) += 1.0f;
    K_H(3, 3) += 1.0f;
    K_H(4, 4) += 1.0f;
    K_H(5, 5) += 1.0f;
    K_H(6, 6) += 1.0f;
    matmul(K_H, S_hat, _sigma);

    // 状態変数を更新する
    matmuladdvec(K, h_minus_z, mu_hat, _mu); //_mu = mu_hat + K * h_minus_z;
    _mu(3) = fpu::clamp(_mu(3), MIN_KF, MAX_KF);
    _mu(4) = fpu::clamp(_mu(4), MIN_KF, MAX_KF);
    _mu(5) = fpu::clamp(_mu(5), MIN_KF, MAX_KF);
    _mu(6) = fpu::clamp(_mu(6), MIN_KF, MAX_KF);
}

#ifndef _MSC_VER
#pragma GCC diagnostic pop
#endif
