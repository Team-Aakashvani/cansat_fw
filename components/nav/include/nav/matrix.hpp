/**
 * @file matrix.hpp
 * @brief Fixed-size matrix and vector templates for embedded navigation.
 *
 * Provides Mat<R,C> and Vec<N> as stack-allocated, cache-friendly structures.
 * All operations are statically-sized and free of dynamic allocation.
 * Layout is row-major, identical to the default Eigen RowMajor layout —
 * migration to Eigen replaces Mat<R,C> with Eigen::Matrix<double,R,C> and
 * Vec<N> with Eigen::Matrix<double,N,1> with no algorithmic changes.
 *
 * Numerical operations provided:
 *   - Matrix multiply, add, subtract, scalar multiply
 *   - Transpose
 *   - Outer product
 *   - Trace, diagonal
 *   - Symmetrisation (M ← ½(M + Mᵀ))
 *   - PSD jitter (M ← M + ε·tr(M)/n · I)
 *   - Cholesky LLᵀ decomposition (for m≤15; used by EKF)
 *   - Forward/backward triangular solve
 *   - Identity, zero constructors
 *   - Diagonal matrix from vector
 *
 * MISRA-inspired constraints:
 *   - No heap allocation anywhere.
 *   - No undefined behaviour from out-of-range access (assert-guarded).
 *   - All operations are O(R×C) or better with compile-time bounds.
 *
 * @note All arithmetic uses `double` for the navigation stack.
 *       Downgrade to `float` on targets where double is prohibitive.
 */
#pragma once

#include <array>
#include <cmath>
#include <cstring>
#include <cassert>

namespace nav {

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
template<int R, int C>
struct Mat;

template<int N>
using Vec = Mat<N, 1>;

// ---------------------------------------------------------------------------
// Mat<R, C> — row-major fixed-size matrix
// ---------------------------------------------------------------------------
template<int R, int C>
struct Mat {
    static_assert(R > 0 && C > 0, "Matrix dimensions must be positive");

    double d[R][C];

    // ---- Construction ----------------------------------------------------

    /// Uninitialized (fastest; zero explicitly with Mat::zero())
    Mat() = default;

    /// Fill with a single scalar
    static Mat<R,C> filled(double v) noexcept {
        Mat<R,C> m;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                m.d[i][j] = v;
        return m;
    }

    static Mat<R,C> zero() noexcept { return filled(0.0); }

    /// Identity (square only — enforced at call site via static_assert)
    static Mat<R,C> eye() noexcept {
        static_assert(R == C, "eye() requires square matrix");
        Mat<R,C> m = zero();
        for (int i = 0; i < R; ++i) m.d[i][i] = 1.0;
        return m;
    }

    /// Diagonal matrix from a Vec<R>
    static Mat<R,C> diag(const Vec<R>& v) noexcept {
        static_assert(R == C, "diag() requires square matrix");
        Mat<R,C> m = zero();
        for (int i = 0; i < R; ++i) m.d[i][i] = v.d[i][0];
        return m;
    }

    // ---- Element access --------------------------------------------------
    inline double& operator()(int i, int j) noexcept {
        assert(i >= 0 && i < R && j >= 0 && j < C);
        return d[i][j];
    }
    inline double operator()(int i, int j) const noexcept {
        assert(i >= 0 && i < R && j >= 0 && j < C);
        return d[i][j];
    }

    // Convenience for column vectors
    inline double& operator()(int i) noexcept {
        static_assert(C == 1, "Scalar index only valid for column vectors");
        return d[i][0];
    }
    inline double operator()(int i) const noexcept {
        static_assert(C == 1, "Scalar index only valid for column vectors");
        return d[i][0];
    }

    // ---- Basic arithmetic ------------------------------------------------

    Mat<R,C> operator+(const Mat<R,C>& rhs) const noexcept {
        Mat<R,C> out;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                out.d[i][j] = d[i][j] + rhs.d[i][j];
        return out;
    }
    Mat<R,C>& operator+=(const Mat<R,C>& rhs) noexcept {
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                d[i][j] += rhs.d[i][j];
        return *this;
    }
    Mat<R,C> operator-(const Mat<R,C>& rhs) const noexcept {
        Mat<R,C> out;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                out.d[i][j] = d[i][j] - rhs.d[i][j];
        return out;
    }
    Mat<R,C>& operator-=(const Mat<R,C>& rhs) noexcept {
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                d[i][j] -= rhs.d[i][j];
        return *this;
    }
    Mat<R,C> operator*(double s) const noexcept {
        Mat<R,C> out;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                out.d[i][j] = d[i][j] * s;
        return out;
    }
    Mat<R,C>& operator*=(double s) noexcept {
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                d[i][j] *= s;
        return *this;
    }
    Mat<R,C> operator-() const noexcept {
        Mat<R,C> out;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                out.d[i][j] = -d[i][j];
        return out;
    }

    // ---- Matrix multiply -------------------------------------------------
    template<int K>
    Mat<R,K> operator*(const Mat<C,K>& rhs) const noexcept {
        Mat<R,K> out = Mat<R,K>::zero();
        for (int i = 0; i < R; ++i)
            for (int k = 0; k < C; ++k) {
                if (d[i][k] == 0.0) continue;
                for (int j = 0; j < K; ++j)
                    out.d[i][j] += d[i][k] * rhs.d[k][j];
            }
        return out;
    }

    // ---- Transpose -------------------------------------------------------
    Mat<C,R> T() const noexcept {
        Mat<C,R> out;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                out.d[j][i] = d[i][j];
        return out;
    }

    // ---- Trace -----------------------------------------------------------
    double trace() const noexcept {
        static_assert(R == C, "trace() requires square matrix");
        double s = 0.0;
        for (int i = 0; i < R; ++i) s += d[i][i];
        return s;
    }

    // ---- Diagonal vector -------------------------------------------------
    Vec<R> diagonal() const noexcept {
        static_assert(R == C, "diagonal() requires square matrix");
        Vec<R> v;
        for (int i = 0; i < R; ++i) v.d[i][0] = d[i][i];
        return v;
    }

    // ---- Dot product (for column vectors) --------------------------------
    double dot(const Vec<R>& rhs) const noexcept {
        static_assert(C == 1, "dot() only valid for column vectors");
        double s = 0.0;
        for (int i = 0; i < R; ++i) s += d[i][0] * rhs.d[i][0];
        return s;
    }

    // ---- L2 norm (for column vectors) ------------------------------------
    double norm() const noexcept {
        static_assert(C == 1, "norm() only valid for column vectors");
        double s = 0.0;
        for (int i = 0; i < R; ++i) s += d[i][0] * d[i][0];
        return std::sqrt(s);
    }

    // ---- Outer product: (R×1) ⊗ (1×C) → (R×C) ---------------------------
    template<int C2>
    Mat<R,C2> outer(const Mat<C2,1>& rhs) const noexcept {
        static_assert(C == 1, "outer() requires left operand to be a column vector");
        Mat<R,C2> out;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C2; ++j)
                out.d[i][j] = d[i][0] * rhs.d[j][0];
        return out;
    }

    // ---- Symmetrisation (in-place) ---------------------------------------
    void symmetrise() noexcept {
        static_assert(R == C, "symmetrise() requires square matrix");
        for (int i = 0; i < R; ++i)
            for (int j = i + 1; j < C; ++j) {
                double avg = 0.5 * (d[i][j] + d[j][i]);
                d[i][j] = avg;
                d[j][i] = avg;
            }
    }

    // ---- PSD jitter (in-place) -------------------------------------------
    /// Adds ε · max(trace()/n, 1) · I to guarantee positive-definiteness.
    void jitter_psd(double eps = 1.0e-9) noexcept {
        static_assert(R == C, "jitter_psd() requires square matrix");
        double scale = trace() / R;
        if (scale < 1.0) scale = 1.0;
        for (int i = 0; i < R; ++i) d[i][i] += eps * scale;
    }

    // ---- Diagonal ceiling ------------------------------------------------
    void clamp_diagonal_max(double max_val) noexcept {
        static_assert(R == C, "clamp_diagonal_max() requires square matrix");
        for (int i = 0; i < R; ++i)
            if (d[i][i] > max_val) d[i][i] = max_val;
    }

    // ---- Check diagonal positivity ---------------------------------------
    bool has_negative_diagonal() const noexcept {
        static_assert(R == C, "requires square matrix");
        for (int i = 0; i < R; ++i)
            if (d[i][i] < 0.0) return true;
        return false;
    }
};

// ---------------------------------------------------------------------------
// Non-member operators
// ---------------------------------------------------------------------------
template<int R, int C>
inline Mat<R,C> operator*(double s, const Mat<R,C>& m) noexcept {
    return m * s;
}

// ---------------------------------------------------------------------------
// Cholesky LLᵀ decomposition
// ---------------------------------------------------------------------------
/// Decomposes a symmetric positive-definite (R×R) matrix M into L·Lᵀ.
/// Returns false if M is not positive definite (EKF should jitter and retry).
/// L is lower-triangular and stored in-place in the output parameter.
template<int N>
bool cholesky(const Mat<N,N>& M, Mat<N,N>& L) noexcept {
    L = Mat<N,N>::zero();
    for (int j = 0; j < N; ++j) {
        double sum = M.d[j][j];
        for (int k = 0; k < j; ++k)
            sum -= L.d[j][k] * L.d[j][k];
        if (sum <= 0.0) return false;
        L.d[j][j] = std::sqrt(sum);
        double inv = 1.0 / L.d[j][j];
        for (int i = j + 1; i < N; ++i) {
            double s2 = M.d[i][j];
            for (int k = 0; k < j; ++k)
                s2 -= L.d[i][k] * L.d[j][k];
            L.d[i][j] = s2 * inv;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Forward substitution: solves L·x = b (L lower-triangular)
// ---------------------------------------------------------------------------
template<int N>
Vec<N> forward_substitute(const Mat<N,N>& L, const Vec<N>& b) noexcept {
    Vec<N> x;
    for (int i = 0; i < N; ++i) {
        double sum = b.d[i][0];
        for (int j = 0; j < i; ++j)
            sum -= L.d[i][j] * x.d[j][0];
        x.d[i][0] = sum / L.d[i][i];
    }
    return x;
}

// ---------------------------------------------------------------------------
// Backward substitution: solves Lᵀ·x = b (Lᵀ upper-triangular)
// ---------------------------------------------------------------------------
template<int N>
Vec<N> backward_substitute(const Mat<N,N>& L, const Vec<N>& b) noexcept {
    Vec<N> x;
    for (int i = N - 1; i >= 0; --i) {
        double sum = b.d[i][0];
        for (int j = i + 1; j < N; ++j)
            sum -= L.d[j][i] * x.d[j][0];
        x.d[i][0] = sum / L.d[i][i];
    }
    return x;
}

// ---------------------------------------------------------------------------
// Solve M·x = b via Cholesky (M SPD)
// Equivalent to numpy.linalg.solve for SPD matrices.
// ---------------------------------------------------------------------------
template<int N>
bool chol_solve(const Mat<N,N>& M, const Vec<N>& b, Vec<N>& x) noexcept {
    Mat<N,N> L;
    if (!cholesky(M, L)) return false;
    Vec<N> w = forward_substitute(L, b);
    x = backward_substitute(L, w);
    return true;
}

// ---------------------------------------------------------------------------
// Solve M·X = B (multiple right-hand sides via Cholesky)
// Used in EKF to compute K = (P·Hᵀ)·S⁻¹  as S·Kᵀ = (H·P)
// where K is (N_ERR × m) and S is (m × m).
// ---------------------------------------------------------------------------
template<int M>
bool chol_solve_multi(const Mat<M,M>& S, const Mat<M,15>& B, Mat<M,15>& X) noexcept {
    Mat<M,M> L;
    if (!cholesky(S, L)) return false;
    for (int col = 0; col < 15; ++col) {
        Vec<M> b_col, x_col;
        for (int i = 0; i < M; ++i) b_col.d[i][0] = B.d[i][col];
        Vec<M> w = forward_substitute(L, b_col);
        x_col = backward_substitute(L, w);
        for (int i = 0; i < M; ++i) X.d[i][col] = x_col.d[i][0];
    }
    return true;
}

// ---------------------------------------------------------------------------
// Log determinant via Cholesky diagonal (log|M| = 2 Σ log L_ii)
// ---------------------------------------------------------------------------
template<int N>
double log_det_chol(const Mat<N,N>& L) noexcept {
    double s = 0.0;
    for (int i = 0; i < N; ++i) s += std::log(L.d[i][i]);
    return 2.0 * s;
}

// ---------------------------------------------------------------------------
// Convenience: 3-vector cross product
// ---------------------------------------------------------------------------
inline Vec<3> cross3(const Vec<3>& a, const Vec<3>& b) noexcept {
    Vec<3> c;
    c.d[0][0] = a.d[1][0] * b.d[2][0] - a.d[2][0] * b.d[1][0];
    c.d[1][0] = a.d[2][0] * b.d[0][0] - a.d[0][0] * b.d[2][0];
    c.d[2][0] = a.d[0][0] * b.d[1][0] - a.d[1][0] * b.d[0][0];
    return c;
}

// ---------------------------------------------------------------------------
// Skew-symmetric 3×3 from 3-vector:  skew(v) · a == v × a
// ---------------------------------------------------------------------------
inline Mat<3,3> skew3(const Vec<3>& v) noexcept {
    Mat<3,3> S;
    const double x = v.d[0][0], y = v.d[1][0], z = v.d[2][0];
    S.d[0][0] =  0.0; S.d[0][1] = -z;   S.d[0][2] =  y;
    S.d[1][0] =  z;   S.d[1][1] =  0.0; S.d[1][2] = -x;
    S.d[2][0] = -y;   S.d[2][1] =  x;   S.d[2][2] =  0.0;
    return S;
}

} // namespace nav
