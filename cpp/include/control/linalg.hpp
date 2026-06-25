#pragma once
// Minimal row-major dense matrix utilities for the controllers (no Eigen).
// Small, readable, and enough for LQR (DARE) and condensed-QP MPC.
#include <vector>
#include <cmath>
#include <stdexcept>
#include <cstddef>

namespace ctrl {

struct Mat {
    int r = 0, c = 0;
    std::vector<double> d;
    Mat() {}
    Mat(int r_, int c_, double fill = 0.0) : r(r_), c(c_), d((size_t)r_ * c_, fill) {}
    double& operator()(int i, int j) { return d[(size_t)i * c + j]; }
    double operator()(int i, int j) const { return d[(size_t)i * c + j]; }

    static Mat eye(int n) {
        Mat m(n, n);
        for (int i = 0; i < n; ++i) m(i, i) = 1.0;
        return m;
    }
    static Mat col(const std::vector<double>& v) {
        Mat m((int)v.size(), 1);
        for (int i = 0; i < (int)v.size(); ++i) m(i, 0) = v[i];
        return m;
    }
    std::vector<double> vec() const { return d; }
};

inline Mat operator*(const Mat& A, const Mat& B) {
    if (A.c != B.r) throw std::runtime_error("mat mul dim");
    Mat C(A.r, B.c);
    for (int i = 0; i < A.r; ++i)
        for (int k = 0; k < A.c; ++k) {
            double a = A(i, k);
            if (a == 0.0) continue;
            for (int j = 0; j < B.c; ++j) C(i, j) += a * B(k, j);
        }
    return C;
}
inline Mat operator+(const Mat& A, const Mat& B) {
    Mat C(A.r, A.c);
    for (size_t i = 0; i < A.d.size(); ++i) C.d[i] = A.d[i] + B.d[i];
    return C;
}
inline Mat operator-(const Mat& A, const Mat& B) {
    Mat C(A.r, A.c);
    for (size_t i = 0; i < A.d.size(); ++i) C.d[i] = A.d[i] - B.d[i];
    return C;
}
inline Mat scale(const Mat& A, double s) {
    Mat C(A.r, A.c);
    for (size_t i = 0; i < A.d.size(); ++i) C.d[i] = A.d[i] * s;
    return C;
}
inline Mat T(const Mat& A) {
    Mat C(A.c, A.r);
    for (int i = 0; i < A.r; ++i)
        for (int j = 0; j < A.c; ++j) C(j, i) = A(i, j);
    return C;
}

// Solve A x = b for square A via Gauss-Jordan with partial pivoting. Returns x.
inline Mat solve(Mat A, Mat b) {
    int n = A.r;
    for (int col = 0; col < n; ++col) {
        int piv = col;
        double best = std::fabs(A(col, col));
        for (int i = col + 1; i < n; ++i)
            if (std::fabs(A(i, col)) > best) { best = std::fabs(A(i, col)); piv = i; }
        if (best < 1e-12) { A(col, col) += 1e-9; }  // tiny regularisation
        if (piv != col)
            for (int j = 0; j < n; ++j) { std::swap(A(col, j), A(piv, j)); }
        if (piv != col)
            for (int j = 0; j < b.c; ++j) std::swap(b(col, j), b(piv, j));
        double dgn = A(col, col);
        for (int j = 0; j < n; ++j) A(col, j) /= dgn;
        for (int j = 0; j < b.c; ++j) b(col, j) /= dgn;
        for (int i = 0; i < n; ++i) {
            if (i == col) continue;
            double f = A(i, col);
            if (f == 0.0) continue;
            for (int j = 0; j < n; ++j) A(i, j) -= f * A(col, j);
            for (int j = 0; j < b.c; ++j) b(i, j) -= f * b(col, j);
        }
    }
    return b;
}
inline Mat inv(const Mat& A) { return solve(A, Mat::eye(A.r)); }

}  // namespace ctrl
