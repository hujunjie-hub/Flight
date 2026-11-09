/*
 * KF-GINS_QUAD fixed-dimension math layer.
 *
 * Derived work of KF-GINS (https://github.com/i2Nav-WHU/KF-GINS,
 * commit 8291a93e49de513fe9d21f819500d39082ded611, GPL-3.0).
 * See ../../MODIFICATIONS.md for the controlled modifications.
 *
 * Design constraints (spec 05 §5.4 / §6.2, spec 07 §6):
 *   - compile-time fixed dimensions, no dynamic resize;
 *   - no heap, no exceptions, no RTTI, no iostream;
 *   - double precision internal baseline;
 *   - all large buffers are owned by the caller's static workspace, never
 *     allocated on a thread stack by this header.
 *
 * This header intentionally implements only the small subset of linear
 * algebra that the trimmed core needs.  It is not a general purpose
 * replacement for Eigen.
 */

#ifndef KFGINS_QUAD_MATH_HPP
#define KFGINS_QUAD_MATH_HPP

#include <cmath>
#include <cstddef>

namespace kfg {

constexpr int kRank21 = 21;  // upstream core error-state dimension
constexpr int kRank22 = 22;  // QUAD extension: 21 core + baro bias
constexpr int kNoise18 = 18; // upstream process-noise dimension
constexpr int kNoise19 = 19; // QUAD extension: 18 core + baro bias walk

/*
 * Row-major fixed-size matrix.  Storage is a plain array so that the object
 * can live in static storage and can never allocate.
 *
 * Aliasing rule: the output of every function below must not alias any input
 * unless the function explicitly documents in-place operation.
 */
template <int R, int C>
struct Mat {
    double d[R * C];

    double &operator()(int r, int c) { return d[r * C + c]; }
    const double &operator()(int r, int c) const { return d[r * C + c]; }

    double &operator[](int i) { return d[i]; }
    const double &operator[](int i) const { return d[i]; }

    static constexpr int rows() { return R; }
    static constexpr int cols() { return C; }
};

using Vec3 = Mat<3, 1>;
using Vec22 = Mat<22, 1>;
using Mat3 = Mat<3, 3>;
using Mat22 = Mat<22, 22>;
using Mat3x22 = Mat<3, 22>;
using Mat22x3 = Mat<22, 3>;

template <int R, int C>
inline void setZero(Mat<R, C> &a) {
    for (int i = 0; i < R * C; ++i) {
        a.d[i] = 0.0;
    }
}

template <int N>
inline void setEye(Mat<N, N> &a) {
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            a(r, c) = (r == c) ? 1.0 : 0.0;
        }
    }
}

template <int R, int C>
inline void copy(const Mat<R, C> &src, Mat<R, C> &dst) {
    for (int i = 0; i < R * C; ++i) {
        dst.d[i] = src.d[i];
    }
}

/* out = A * B ; out must not alias A or B. */
template <int R, int K, int C>
inline void matmul(const Mat<R, K> &a, const Mat<K, C> &b, Mat<R, C> &out) {
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            double sum = 0.0;
            for (int k = 0; k < K; ++k) {
                sum += a(r, k) * b(k, c);
            }
            out(r, c) = sum;
        }
    }
}

/* out = A * B^T ; out must not alias A or B. */
template <int R, int K, int C>
inline void matmulTransposed(const Mat<R, K> &a, const Mat<C, K> &b, Mat<R, C> &out) {
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            double sum = 0.0;
            const int arow = r * K;
            const int brow = c * K;
            for (int k = 0; k < K; ++k) {
                sum += a.d[arow + k] * b.d[brow + k];
            }
            out(r, c) = sum;
        }
    }
}

/* out = A^T * B ; out must not alias A or B. */
template <int R, int K, int C>
inline void matmulLeftTransposed(const Mat<K, R> &a, const Mat<K, C> &b, Mat<R, C> &out) {
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            double sum = 0.0;
            for (int k = 0; k < K; ++k) {
                sum += a(k, r) * b(k, c);
            }
            out(r, c) = sum;
        }
    }
}

template <int N>
inline void symmetrize(Mat<N, N> &a) {
    for (int r = 0; r < N; ++r) {
        for (int c = r + 1; c < N; ++c) {
            const double v = 0.5 * (a(r, c) + a(c, r));
            a(r, c) = v;
            a(c, r) = v;
        }
    }
}

template <int N>
inline bool allFinite(const Mat<N, N> &a) {
    for (int i = 0; i < N * N; ++i) {
        if (!std::isfinite(a.d[i])) {
            return false;
        }
    }
    return true;
}

template <int N>
inline bool allFiniteVec(const Mat<N, 1> &a) {
    for (int i = 0; i < N; ++i) {
        if (!std::isfinite(a.d[i])) {
            return false;
        }
    }
    return true;
}

/*
 * Cholesky factorisation A = L * L^T for small symmetric positive definite
 * matrices (N <= 3 in the current observation models).  Returns false when A
 * is not SPD; the caller must then reject the update instead of inverting.
 */
template <int N>
inline bool cholesky(const Mat<N, N> &a, Mat<N, N> &l) {
    setZero(l);
    for (int c = 0; c < N; ++c) {
        double diag = a(c, c);
        for (int k = 0; k < c; ++k) {
            diag -= l(c, k) * l(c, k);
        }
        if (!(diag > 0.0) || !std::isfinite(diag)) {
            return false;
        }
        const double root = std::sqrt(diag);
        l(c, c) = root;
        for (int r = c + 1; r < N; ++r) {
            double sum = a(r, c);
            for (int k = 0; k < c; ++k) {
                sum -= l(r, k) * l(c, k);
            }
            l(r, c) = sum / root;
        }
    }
    return true;
}

/* Solves L * L^T * x = b given the Cholesky factor L. */
template <int N>
inline void choleskySolve(const Mat<N, N> &l, const Mat<N, 1> &b, Mat<N, 1> &x) {
    Mat<N, 1> y;
    for (int r = 0; r < N; ++r) {
        double sum = b(r, 0);
        for (int k = 0; k < r; ++k) {
            sum -= l(r, k) * y(k, 0);
        }
        y(r, 0) = sum / l(r, r);
    }
    for (int r = N - 1; r >= 0; --r) {
        double sum = y(r, 0);
        for (int k = r + 1; k < N; ++k) {
            sum -= l(k, r) * x(k, 0);
        }
        x(r, 0) = sum / l(r, r);
    }
}

/* Solves A * x = b for SPD A via Cholesky.  Returns false if A is not SPD. */
template <int N>
inline bool spdSolve(const Mat<N, N> &a, const Mat<N, 1> &b, Mat<N, 1> &x) {
    Mat<N, N> l;
    if (!cholesky(a, l)) {
        return false;
    }
    choleskySolve(l, b, x);
    return true;
}

/* True when the symmetric matrix is positive definite (Cholesky succeeds). */
template <int N>
inline bool isPositiveDefinite(const Mat<N, N> &a) {
    Mat<N, N> l;
    return cholesky(a, l);
}

inline Mat3 skew(const Vec3 &v) {
    Mat3 m;
    m(0, 0) = 0.0;
    m(0, 1) = -v(2, 0);
    m(0, 2) = v(1, 0);
    m(1, 0) = v(2, 0);
    m(1, 1) = 0.0;
    m(1, 2) = -v(0, 0);
    m(2, 0) = -v(1, 0);
    m(2, 1) = v(0, 0);
    m(2, 2) = 0.0;
    return m;
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
    Vec3 r;
    r(0, 0) = a(1, 0) * b(2, 0) - a(2, 0) * b(1, 0);
    r(1, 0) = a(2, 0) * b(0, 0) - a(0, 0) * b(2, 0);
    r(2, 0) = a(0, 0) * b(1, 0) - a(1, 0) * b(0, 0);
    return r;
}

inline double dot(const Vec3 &a, const Vec3 &b) {
    return a(0, 0) * b(0, 0) + a(1, 0) * b(1, 0) + a(2, 0) * b(2, 0);
}

inline double norm(const Vec3 &a) { return std::sqrt(dot(a, a)); }

template <int N>
inline double dotN(const Mat<N, 1> &a, const Mat<N, 1> &b) {
    double sum = 0.0;
    for (int i = 0; i < N; ++i) {
        sum += a(i, 0) * b(i, 0);
    }
    return sum;
}

/* out = m * v */
inline Vec3 matvec(const Mat3 &m, const Vec3 &v) {
    Vec3 r;
    for (int i = 0; i < 3; ++i) {
        r(i, 0) = m(i, 0) * v(0, 0) + m(i, 1) * v(1, 0) + m(i, 2) * v(2, 0);
    }
    return r;
}

/* out = M^T * v */
inline Vec3 matvecTransposed(const Mat3 &m, const Vec3 &v) {
    Vec3 r;
    for (int i = 0; i < 3; ++i) {
        r(i, 0) = m(0, i) * v(0, 0) + m(1, i) * v(1, 0) + m(2, i) * v(2, 0);
    }
    return r;
}

inline Mat3 matmul3(const Mat3 &a, const Mat3 &b) {
    Mat3 out;
    matmul<3, 3, 3>(a, b, out);
    return out;
}

inline Mat3 transpose3(const Mat3 &a) {
    Mat3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out(r, c) = a(c, r);
        }
    }
    return out;
}

/*
 * Small-value operators.  They are only used with 3x1 / 3x3 / 1x22 operands;
 * the 22x22 paths always use the explicit output-parameter functions above so
 * that no large temporary is created on the stack.
 */
template <int R, int C>
inline Mat<R, C> operator+(const Mat<R, C> &a, const Mat<R, C> &b) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) {
        out.d[i] = a.d[i] + b.d[i];
    }
    return out;
}

template <int R, int C>
inline Mat<R, C> operator-(const Mat<R, C> &a, const Mat<R, C> &b) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) {
        out.d[i] = a.d[i] - b.d[i];
    }
    return out;
}

template <int R, int C>
inline Mat<R, C> operator*(const Mat<R, C> &a, double s) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) {
        out.d[i] = a.d[i] * s;
    }
    return out;
}

template <int R, int C>
inline Mat<R, C> operator*(double s, const Mat<R, C> &a) {
    return a * s;
}

template <int R, int C>
inline Mat<R, C> operator/(const Mat<R, C> &a, double s) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) {
        out.d[i] = a.d[i] / s;
    }
    return out;
}

} // namespace kfg

#endif // KFGINS_QUAD_MATH_HPP
