// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ekf {

template <int R, int C>
struct Mat {
  double data[R * C]{};

  double& operator()(int r, int c) { return data[r * C + c]; }
  double operator()(int r, int c) const { return data[r * C + c]; }

  static Mat<R, C> identity() {
    Mat<R, C> m{};
    for (int i = 0; i < std::min(R, C); ++i) {
      m(i, i) = 1.0;
    }
    return m;
  }

  static Mat<R, C> zeros() { return {}; }
};

template <int N>
using Vec = Mat<N, 1>;

template <int R, int C, int K>
Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) {
  Mat<R, C> out{};
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      double s = 0.0;
      for (int k = 0; k < K; ++k) {
        s += a(i, k) * b(k, j);
      }
      out(i, j) = s;
    }
  }
  return out;
}

template <int R, int C>
Mat<C, R> transpose(const Mat<R, C>& m) {
  Mat<C, R> out{};
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < C; ++j) {
      out(j, i) = m(i, j);
    }
  }
  return out;
}

template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& a, const Mat<R, C>& b) {
  Mat<R, C> out{};
  for (int i = 0; i < R * C; ++i) {
    out.data[i] = a.data[i] + b.data[i];
  }
  return out;
}

template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& a, const Mat<R, C>& b) {
  Mat<R, C> out{};
  for (int i = 0; i < R * C; ++i) {
    out.data[i] = a.data[i] - b.data[i];
  }
  return out;
}

template <int R, int C>
Mat<R, C> operator*(double s, const Mat<R, C>& m) {
  Mat<R, C> out{};
  for (int i = 0; i < R * C; ++i) {
    out.data[i] = s * m.data[i];
  }
  return out;
}

inline bool cholesky_solve_1(const double s, const double b, double& x) {
  if (!(s > 0.0) || !std::isfinite(s) || !std::isfinite(b)) {
    return false;
  }
  const double l = std::sqrt(s);
  x = b / l / l;
  return std::isfinite(x);
}

inline bool cholesky_solve_2(const double s[2][2], const double b[2], double x[2]) {
  double L[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
  L[0][0] = std::sqrt(s[0][0]);
  if (!(L[0][0] > 0.0)) {
    return false;
  }
  L[1][0] = s[1][0] / L[0][0];
  const double d = s[1][1] - L[1][0] * L[1][0];
  if (!(d > 0.0)) {
    return false;
  }
  L[1][1] = std::sqrt(d);

  double y[2];
  y[0] = b[0] / L[0][0];
  y[1] = (b[1] - L[1][0] * y[0]) / L[1][1];

  x[1] = y[1] / L[1][1];
  x[0] = (y[0] - L[1][0] * x[1]) / L[0][0];
  return std::isfinite(x[0]) && std::isfinite(x[1]);
}

inline bool cholesky_solve_5(const double s[5][5], const double b[5], double x[5]) {
  double L[5][5]{};
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = s[i][j];
      for (int k = 0; k < j; ++k) {
        sum -= L[i][k] * L[j][k];
      }
      if (i == j) {
        if (!(sum > 0.0)) {
          return false;
        }
        L[i][j] = std::sqrt(sum);
      } else {
        L[i][j] = sum / L[j][j];
      }
    }
  }
  double y[5];
  for (int i = 0; i < 5; ++i) {
    double sum = b[i];
    for (int k = 0; k < i; ++k) {
      sum -= L[i][k] * y[k];
    }
    y[i] = sum / L[i][i];
  }
  for (int i = 4; i >= 0; --i) {
    double sum = y[i];
    for (int k = i + 1; k < 5; ++k) {
      sum -= L[k][i] * x[k];
    }
    x[i] = sum / L[i][i];
    if (!std::isfinite(x[i])) {
      return false;
    }
  }
  return true;
}

inline double min_eigenvalue_2x2(double a, double b, double c) {
  const double tr = a + c;
  const double det = a * c - b * b;
  const double disc = std::sqrt(std::max(0.0, tr * tr * 0.25 - det));
  return tr * 0.5 - disc;
}

}  // namespace ekf
