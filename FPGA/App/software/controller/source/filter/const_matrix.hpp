#pragma once

#include <cstddef>
#include <limits>
#include <initializer_list>

/**
 * @brief Compile Time Math
 */
namespace ctmath {

template<typename T, size_t ROWS, size_t COLS>
class ConstMatrix {
public:
    constexpr ConstMatrix(void) : elem() {
        for (size_t row = 0; row < ROWS; row++) {
            for (size_t col = 0; col < COLS; col++) {
                elem[row][col] = 0;
            }
        }
    }

    constexpr ConstMatrix(std::initializer_list<T> init) : elem() {
        for (size_t i = 0; i < (ROWS * COLS); i++) {
            elem[i / COLS][i % COLS] = (i < init.size()) ? init.begin()[i] : 0;
        }
    }

    constexpr ConstMatrix(const ConstMatrix<T, ROWS, COLS>& src) : elem() {
        for (size_t row = 0; row < ROWS; row++) {
            for (size_t col = 0; col < COLS; col++) {
                elem[row][col] = src.elem[row][col];
            }
        }
    }

    ConstMatrix<T, ROWS, COLS>& operator=(const ConstMatrix<T, ROWS, COLS>& src) {
        for (size_t row = 0; row < ROWS; row++) {
            for (size_t col = 0; col < COLS; col++) {
                elem[row][col] = src.elem[row][col];
            }
        }
        return *this;
    }

    constexpr ConstMatrix<T, ROWS, COLS> operator+(void) const {
        return *this;
    }

    constexpr ConstMatrix<T, ROWS, COLS> operator-(void) const {
        ConstMatrix<T, ROWS, COLS> result;
        for (size_t row = 0; row < ROWS; row++) {
            for (size_t col = 0; col < COLS; col++) {
                result.elem[row][col] = -elem[row][col];
            }
        }
        return result;
    }

    constexpr ConstMatrix<T, COLS, ROWS> t(void) const {
        ConstMatrix<T, COLS, ROWS> result;
        for (size_t row = 0; row < ROWS; row++) {
            for (size_t col = 0; col < COLS; col++) {
                result.elem[col][row] = elem[row][col];
            }
        }
        return result;
    }

    constexpr T operator()(size_t row, size_t col) const {
        return elem[row][col];
    }

    constexpr size_t width(void) const {
        return COLS;
    }

    constexpr size_t height(void) const {
        return ROWS;
    }

    constexpr ConstMatrix<T, 1, COLS> extractRow(size_t i0) const {
        ConstMatrix<T, 1, COLS> result;
        for (size_t col = 0; col < COLS; col++) {
            result.elem[0][col] = elem[i0][col];
        }
        return result;
    }

    constexpr ConstMatrix<T, 2, COLS> extractRow(size_t i0, size_t i1) const {
        ConstMatrix<T, 2, COLS> result;
        for (size_t col = 0; col < COLS; col++) {
            result.elem[0][col] = elem[i0][col];
            result.elem[1][col] = elem[i1][col];
        }
        return result;
    }

    constexpr ConstMatrix<T, 3, COLS> extractRow(size_t i0, size_t i1, size_t i2) const {
        ConstMatrix<T, 3, COLS> result;
        for (size_t col = 0; col < COLS; col++) {
            result.elem[0][col] = elem[i0][col];
            result.elem[1][col] = elem[i1][col];
            result.elem[2][col] = elem[i2][col];
        }
        return result;
    }

    /**
     * @brief LU分解で逆行列を求める
     */
    constexpr ConstMatrix<T, ROWS, COLS> inv(void) const {
        static_assert(COLS == ROWS, "COLS must euqal ROWS");

        // LU分解する
        ConstMatrix<T, ROWS, COLS> L, U(*this), Q;
        for (size_t i = 0; i < ROWS; i++) {
            Q.elem[i][i] = 1;
        }
        for (size_t i = 0; i < (ROWS - 1); i++) {
            // ピボット選択
            T hold_val = 0;
            size_t hold_index = 0;
            for (size_t k = i; k < ROWS; k++) {
                T val = U.elem[k][i];
                if ((val < 0) && (hold_val < -val)) {
                    hold_val = -val;
                    hold_index = k;
                }
                else if (hold_val < val) {
                    hold_val = val;
                    hold_index = k;
                }
            }

            // 行の入れ替えを行う
            if (hold_index != i) {
                for (size_t k = 0; k < ROWS; k++) {
                    float temp_u = U.elem[i][k];
                    U.elem[i][k] = U.elem[hold_index][k];
                    U.elem[hold_index][k] = temp_u;
                    float temp_l = L.elem[i][k];
                    L.elem[i][k] = L.elem[hold_index][k];
                    L.elem[hold_index][k] = temp_l;
                    float temp_q = Q.elem[i][k];
                    Q.elem[i][k] = Q.elem[hold_index][k];
                    Q.elem[hold_index][k] = temp_q;
                }
            }

            // 上三角行列Lと単位下三角行列Uの構成
            for (size_t k = 0; k < i; k++) {
                L.elem[k][i] = 0;
            }
            L.elem[i][i] = 1;

            for (size_t j = i + 1; j < ROWS; j++) {
                T ratio = U.elem[j][i] / U.elem[i][i];
                L.elem[j][i] = ratio;
                for (size_t m = 0; m < COLS; m++) {
                    U.elem[j][m] -= U.elem[i][m] * ratio;
                }
            }
        }
        L.elem[ROWS - 1][COLS - 1] = 1;

        // 下三角行列の逆行列を計算する
        ConstMatrix<T, ROWS, COLS> invL;
        for (size_t col = 0; col < COLS; col++) {
            invL.elem[col][col] = 1;
            for (size_t row = col + 1; row < ROWS; row++) {
                T sum = 0;
                for (size_t i = 0; i < row; i++) {
                    sum -= L.elem[row][i] * invL.elem[i][col];
                }
                invL.elem[row][col] = sum;
            }
        }

        // 上三角行列の逆行列を計算する
        ConstMatrix<T, ROWS, COLS> invU;
        for (size_t col = COLS; 0 < col--;) {
            invU.elem[col][col] = 1 / U.elem[col][col];
            for (size_t row = col; 0 < row--;) {
                T sum = 0;
                for (size_t i = ROWS; row < i--;) {
                    sum -= U.elem[row][i] * invU.elem[i][col];
                }
                invU.elem[row][col] = sum / U.elem[row][row];
            }
        }

        return invU * invL * Q;
    }

    /**
     * @brief 指定したビット位置で丸めを行う
     * @param bit ビット位置
     */
    constexpr ConstMatrix<T, ROWS, COLS> round(int bit = 0) const {
        ConstMatrix<T, COLS, ROWS> result;
        for (size_t row = 0; row < ROWS; row++) {
            for (size_t col = 0; col < COLS; col++) {
                T value = elem[row][col];
                T abs_value = (value < 0) ? -value : value;
                double geta = static_cast<double>(abs_value) * (1U << bit) + 0.5;
                if (geta <= std::numeric_limits<unsigned int>::max()) {
                    unsigned int geta_int = static_cast<unsigned int>(geta);
                    if (geta_int != 0) {
                        T sign = value / abs_value;
                        result.elem[col][row] = sign * static_cast<T>(static_cast<double>(geta_int) / (1U << bit));
                    }
                    else {
                        result.elem[col][row] = 0;
                    }
                }
                else {
                    T sign = value / abs_value;
                    result.elem[col][row] = sign * std::numeric_limits<T>::infinity();
                }
            }
        }
        return result;
    }

    static constexpr ConstMatrix<T, ROWS, COLS> zeros(void) {
        return ConstMatrix<T, ROWS, COLS>();
    }

    T elem[ROWS][COLS];
};

template<typename T, size_t ROWS, size_t COLS>
static constexpr ConstMatrix<T, ROWS, COLS> operator+(const ConstMatrix<T, ROWS, COLS>& a, const ConstMatrix<T, ROWS, COLS>& b) {
    ConstMatrix<T, ROWS, COLS> result;
    for (size_t row = 0; row < ROWS; row++) {
        for (size_t col = 0; col < COLS; col++) {
            result.elem[row][col] = a.elem[row][col] + b.elem[row][col];
        }
    }
    return result;
}

template<typename T, size_t ROWS, size_t COLS>
static constexpr ConstMatrix<T, ROWS, COLS> operator-(const ConstMatrix<T, ROWS, COLS>& a, const ConstMatrix<T, ROWS, COLS>& b) {
    ConstMatrix<T, ROWS, COLS> result;
    for (size_t row = 0; row < ROWS; row++) {
        for (size_t col = 0; col < COLS; col++) {
            result.elem[row][col] = a.elem[row][col] - b.elem[row][col];
        }
    }
    return result;
}

template<typename T, size_t ROWS, size_t COLS, size_t N>
static constexpr ConstMatrix<T, ROWS, COLS> operator*(const ConstMatrix<T, ROWS, N>& a, const ConstMatrix<T, N, COLS>& b) {
    ConstMatrix<T, ROWS, COLS> result;
    for (size_t row = 0; row < ROWS; row++) {
        for (size_t col = 0; col < COLS; col++) {
            T sum = 0;
            for (size_t i = 0; i < N; i++) {
                sum += a.elem[row][i] * b.elem[i][col];
            }
            result.elem[row][col] = sum;
        }
    }
    return result;
}

template<typename T, size_t ROWS1, size_t COLS1, size_t ROWS2, size_t COLS2>
static constexpr ConstMatrix<T, ROWS1 + ROWS2, COLS1 + COLS2> mergeBlocks(const ConstMatrix<T, ROWS1, COLS1>& a, const ConstMatrix<T, ROWS1, COLS2>& b,
                                                                          const ConstMatrix<T, ROWS2, COLS1>& c,
                                                                          const ConstMatrix<T, ROWS2, COLS2>& d = ConstMatrix<T, ROWS2, COLS2>::zeros()) {
    constexpr size_t ROWS = ROWS1 + ROWS2;
    constexpr size_t COLS = COLS1 + COLS2;
    ConstMatrix<T, ROWS, COLS> result;
    for (size_t row = 0; row < ROWS1; row++) {
        for (size_t col = 0; col < COLS1; col++) {
            result.elem[row][col] = a.elem[row][col];
        }
        for (size_t col = 0; col < COLS2; col++) {
            result.elem[row][col + COLS1] = b.elem[row][col];
        }
    }
    for (size_t row = 0; row < ROWS2; row++) {
        for (size_t col = 0; col < COLS1; col++) {
            result.elem[row + ROWS1][col] = c.elem[row][col];
        }
        for (size_t col = 0; col < COLS2; col++) {
            result.elem[row + ROWS1][col + COLS1] = d.elem[row][col];
        }
    }
    return result;
}

using ConstMatrix1f = ConstMatrix<float, 1, 1>;
using ConstMatrix2f = ConstMatrix<float, 2, 2>;
using ConstMatrix3f = ConstMatrix<float, 3, 3>;
using ConstMatrix4f = ConstMatrix<float, 4, 4>;

using ConstVector1f = ConstMatrix<float, 1, 1>;
using ConstVector2f = ConstMatrix<float, 2, 1>;
using ConstVector3f = ConstMatrix<float, 3, 1>;
using ConstVector4f = ConstMatrix<float, 4, 1>;

} // namespace ctmath
