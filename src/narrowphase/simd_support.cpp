/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Asensus Surgical
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of Asensus Surgical nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "coal/internal/simd_support.h"

#include <limits>

#if defined(COAL_ENABLE_SIMD) && \
    (defined(__x86_64__) || defined(__i386__) || defined(_M_X64))
#if defined(__GNUC__) || defined(__clang__)
#define COAL_SIMD_SUPPORT_HAS_AVX2_TARGET 1
#include <immintrin.h>
#endif
#endif

namespace coal {
namespace details {
namespace simd {
namespace {

int maxDotScalar(const Vec3s* points, int count, const Vec3s& dir,
                 Scalar& maxdot) {
  if (count <= 0) {
    maxdot = -std::numeric_limits<Scalar>::infinity();
    return -1;
  }

  int best = 0;
  maxdot = points[0].dot(dir);
  for (int i = 1; i < count; ++i) {
    const Scalar dot = points[i].dot(dir);
    if (dot > maxdot) {
      maxdot = dot;
      best = i;
    }
  }
  return best;
}

int maxDotSoAScalar(const Scalar* xs, const Scalar* ys, const Scalar* zs,
                    int count, const Vec3s& dir, Scalar& maxdot) {
  if (count <= 0) {
    maxdot = -std::numeric_limits<Scalar>::infinity();
    return -1;
  }

  int best = 0;
  maxdot = xs[0] * dir[0] + ys[0] * dir[1] + zs[0] * dir[2];
  for (int i = 1; i < count; ++i) {
    const Scalar dot = xs[i] * dir[0] + ys[i] * dir[1] + zs[i] * dir[2];
    if (dot > maxdot) {
      maxdot = dot;
      best = i;
    }
  }
  return best;
}

#if defined(COAL_SIMD_SUPPORT_HAS_AVX2_TARGET)
bool cpuSupportsAvx2() {
  __builtin_cpu_init();
  return __builtin_cpu_supports("avx2");
}

#ifdef COAL_USE_FLOAT_PRECISION
__attribute__((target("avx2"))) int maxDotAvx2(const Vec3s* points, int count,
                                               const Vec3s& dir,
                                               Scalar& maxdot) {
  if (count <= 0) {
    maxdot = -std::numeric_limits<Scalar>::infinity();
    return -1;
  }

  const __m256 dx = _mm256_set1_ps(dir[0]);
  const __m256 dy = _mm256_set1_ps(dir[1]);
  const __m256 dz = _mm256_set1_ps(dir[2]);

  int best = -1;
  maxdot = -std::numeric_limits<Scalar>::infinity();
  int i = 0;
  alignas(32) Scalar dots[8];
  for (; i + 7 < count; i += 8) {
    const __m256 xs = _mm256_set_ps(
        points[i + 7][0], points[i + 6][0], points[i + 5][0], points[i + 4][0],
        points[i + 3][0], points[i + 2][0], points[i + 1][0], points[i][0]);
    const __m256 ys = _mm256_set_ps(
        points[i + 7][1], points[i + 6][1], points[i + 5][1], points[i + 4][1],
        points[i + 3][1], points[i + 2][1], points[i + 1][1], points[i][1]);
    const __m256 zs = _mm256_set_ps(
        points[i + 7][2], points[i + 6][2], points[i + 5][2], points[i + 4][2],
        points[i + 3][2], points[i + 2][2], points[i + 1][2], points[i][2]);
    const __m256 dot = _mm256_add_ps(
        _mm256_add_ps(_mm256_mul_ps(xs, dx), _mm256_mul_ps(ys, dy)),
        _mm256_mul_ps(zs, dz));
    _mm256_store_ps(dots, dot);
    for (int lane = 0; lane < 8; ++lane) {
      if (dots[lane] > maxdot) {
        maxdot = dots[lane];
        best = i + lane;
      }
    }
  }

  for (; i < count; ++i) {
    const Scalar dot = points[i].dot(dir);
    if (dot > maxdot) {
      maxdot = dot;
      best = i;
    }
  }
  return best;
}

__attribute__((target("avx2"))) int maxDotSoAAvx2(const Scalar* xs_in,
                                                  const Scalar* ys_in,
                                                  const Scalar* zs_in,
                                                  int count, const Vec3s& dir,
                                                  Scalar& maxdot) {
  if (count <= 0) {
    maxdot = -std::numeric_limits<Scalar>::infinity();
    return -1;
  }

  const __m256 dx = _mm256_set1_ps(dir[0]);
  const __m256 dy = _mm256_set1_ps(dir[1]);
  const __m256 dz = _mm256_set1_ps(dir[2]);

  int best = -1;
  maxdot = -std::numeric_limits<Scalar>::infinity();
  int i = 0;
  alignas(32) Scalar dots[8];
  for (; i + 7 < count; i += 8) {
    const __m256 xs = _mm256_loadu_ps(xs_in + i);
    const __m256 ys = _mm256_loadu_ps(ys_in + i);
    const __m256 zs = _mm256_loadu_ps(zs_in + i);
    const __m256 dot = _mm256_add_ps(
        _mm256_add_ps(_mm256_mul_ps(xs, dx), _mm256_mul_ps(ys, dy)),
        _mm256_mul_ps(zs, dz));
    _mm256_store_ps(dots, dot);
    for (int lane = 0; lane < 8; ++lane) {
      if (dots[lane] > maxdot) {
        maxdot = dots[lane];
        best = i + lane;
      }
    }
  }

  for (; i < count; ++i) {
    const Scalar dot =
        xs_in[i] * dir[0] + ys_in[i] * dir[1] + zs_in[i] * dir[2];
    if (dot > maxdot) {
      maxdot = dot;
      best = i;
    }
  }
  return best;
}
#else
__attribute__((target("avx2"))) int maxDotAvx2(const Vec3s* points, int count,
                                               const Vec3s& dir,
                                               Scalar& maxdot) {
  if (count <= 0) {
    maxdot = -std::numeric_limits<Scalar>::infinity();
    return -1;
  }

  const __m256d dx = _mm256_set1_pd(dir[0]);
  const __m256d dy = _mm256_set1_pd(dir[1]);
  const __m256d dz = _mm256_set1_pd(dir[2]);

  int best = -1;
  maxdot = -std::numeric_limits<Scalar>::infinity();
  int i = 0;
  alignas(32) Scalar dots[4];
  for (; i + 3 < count; i += 4) {
    const __m256d xs = _mm256_set_pd(points[i + 3][0], points[i + 2][0],
                                     points[i + 1][0], points[i][0]);
    const __m256d ys = _mm256_set_pd(points[i + 3][1], points[i + 2][1],
                                     points[i + 1][1], points[i][1]);
    const __m256d zs = _mm256_set_pd(points[i + 3][2], points[i + 2][2],
                                     points[i + 1][2], points[i][2]);
    const __m256d dot = _mm256_add_pd(
        _mm256_add_pd(_mm256_mul_pd(xs, dx), _mm256_mul_pd(ys, dy)),
        _mm256_mul_pd(zs, dz));
    _mm256_store_pd(dots, dot);
    for (int lane = 0; lane < 4; ++lane) {
      if (dots[lane] > maxdot) {
        maxdot = dots[lane];
        best = i + lane;
      }
    }
  }

  for (; i < count; ++i) {
    const Scalar dot = points[i].dot(dir);
    if (dot > maxdot) {
      maxdot = dot;
      best = i;
    }
  }
  return best;
}

__attribute__((target("avx2"))) int maxDotSoAAvx2(const Scalar* xs_in,
                                                  const Scalar* ys_in,
                                                  const Scalar* zs_in,
                                                  int count, const Vec3s& dir,
                                                  Scalar& maxdot) {
  if (count <= 0) {
    maxdot = -std::numeric_limits<Scalar>::infinity();
    return -1;
  }

  const __m256d dx = _mm256_set1_pd(dir[0]);
  const __m256d dy = _mm256_set1_pd(dir[1]);
  const __m256d dz = _mm256_set1_pd(dir[2]);

  int best = -1;
  maxdot = -std::numeric_limits<Scalar>::infinity();
  int i = 0;
  alignas(32) Scalar dots[4];
  for (; i + 3 < count; i += 4) {
    const __m256d xs = _mm256_loadu_pd(xs_in + i);
    const __m256d ys = _mm256_loadu_pd(ys_in + i);
    const __m256d zs = _mm256_loadu_pd(zs_in + i);
    const __m256d dot = _mm256_add_pd(
        _mm256_add_pd(_mm256_mul_pd(xs, dx), _mm256_mul_pd(ys, dy)),
        _mm256_mul_pd(zs, dz));
    _mm256_store_pd(dots, dot);
    for (int lane = 0; lane < 4; ++lane) {
      if (dots[lane] > maxdot) {
        maxdot = dots[lane];
        best = i + lane;
      }
    }
  }

  for (; i < count; ++i) {
    const Scalar dot =
        xs_in[i] * dir[0] + ys_in[i] * dir[1] + zs_in[i] * dir[2];
    if (dot > maxdot) {
      maxdot = dot;
      best = i;
    }
  }
  return best;
}
#endif
#endif

}  // namespace

bool isAvx2Enabled() {
#if defined(COAL_SIMD_SUPPORT_HAS_AVX2_TARGET)
  static const bool supported = cpuSupportsAvx2();
  return supported;
#else
  return false;
#endif
}

int maxDot(const Vec3s* points, int count, const Vec3s& dir, Scalar& maxdot) {
#if defined(COAL_SIMD_SUPPORT_HAS_AVX2_TARGET)
  if (isAvx2Enabled()) return maxDotAvx2(points, count, dir, maxdot);
#endif
  return maxDotScalar(points, count, dir, maxdot);
}

int maxDotSoA(const Scalar* xs, const Scalar* ys, const Scalar* zs, int count,
              const Vec3s& dir, Scalar& maxdot) {
#if defined(COAL_SIMD_SUPPORT_HAS_AVX2_TARGET)
  if (isAvx2Enabled()) return maxDotSoAAvx2(xs, ys, zs, count, dir, maxdot);
#endif
  return maxDotSoAScalar(xs, ys, zs, count, dir, maxdot);
}

}  // namespace simd
}  // namespace details
}  // namespace coal
