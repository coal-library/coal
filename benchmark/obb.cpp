/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS
 *  Copyright (c) 2023-2026, Inria
 *  Author: Florent Lamiraux
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of CNRS nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <benchmark/benchmark.h>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include "coal/collision_data.h"
#include "coal/data_types.h"
#include "coal/math/transform.h"

using namespace coal;

namespace {

// Number of random OBB and number of random transform per OBB
const std::size_t num_random_obb = 100;
const std::size_t num_transforms_per_obb = 100;

// Extent norm is between 0 and extent_norm on each axis
const Scalar extent_norm = Scalar(1.);

const CollisionRequest request;
const Scalar break_distance = request.break_distance + request.security_margin;
const Scalar break_distance2 = break_distance * break_distance;

typedef bool (*SeparatingAxisTest)(const Matrix3s& B, const Vec3s& T,
                                   const Vec3s& a, const Vec3s& b,
                                   const Scalar& breakDistance2,
                                   Scalar& squaredLowerBoundDistance);

void randomOBBs(Vec3s& a, Vec3s& b) {
  a = extent_norm * Vec3s::Random().cwiseAbs().normalized();
  b = extent_norm * Vec3s::Random().cwiseAbs().normalized();
}

void randomTransform(Matrix3s& B, Vec3s& T, const Vec3s& a, const Vec3s& b) {
  // A translation of norm N ensures there is no collision.
  // Set translation to be between 0 and 2 * N;
  Scalar N = a.norm() + b.norm();
  T = (Vec3s::Random() / std::sqrt(3)) * 1.5 * N;

  Quats q;
  q.coeffs().setRandom();
  q.normalize();
  B = q;
}

// Half extents of the OBBs and the transform of B with respect to A.
struct ObbCase {
  Matrix3s B;  ///< Orientation
  Vec3s T;     ///< Translation
  Vec3s a;     ///< Half dimension of the first box
  Vec3s b;     ///< Half dimension of the second box
};

// Generate the random cases in a static variable to avoid regenerating them
// for each benchmark.
const std::vector<ObbCase>& get_cases() {
  static const std::vector<ObbCase> cases = [] {
    std::vector<ObbCase> cases;
    cases.reserve(num_random_obb * num_transforms_per_obb);
    for (std::size_t iobb = 0; iobb < num_random_obb; ++iobb) {
      ObbCase obb;
      randomOBBs(obb.a, obb.b);
      for (std::size_t itf = 0; itf < num_transforms_per_obb; ++itf) {
        randomTransform(obb.B, obb.T, obb.a, obb.b);
        cases.push_back(obb);
      }
    }
    return cases;
  }();
  return cases;
}

namespace obbDisjoint_impls {

typedef Scalar (*Case1Fn)(const Vec3s& T, const Vec3s& a, const Vec3s& b,
                          const Matrix3s& Bf);
typedef Scalar (*Case2Fn)(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                          const Vec3s& b, const Matrix3s& Bf);

// Corner of the b axis aligned bounding box the closest to the origin
inline Scalar _computeDistanceForCase1_ManualProduct(const Vec3s& T,
                                                     const Vec3s& a,
                                                     const Vec3s& b,
                                                     const Matrix3s& Bf) {
  Vec3s AABB_corner;
  AABB_corner.noalias() = T.cwiseAbs() - a;
  AABB_corner.noalias() -= Bf.col(0) * b[0];
  AABB_corner.noalias() -= Bf.col(1) * b[1];
  AABB_corner.noalias() -= Bf.col(2) * b[2];
  return AABB_corner.array().max(Scalar(0)).matrix().squaredNorm();
}

// Same as _computeDistanceForCase1_ManualProduct, with the Eigen product
inline Scalar _computeDistanceForCase1_EigenProduct(const Vec3s& T,
                                                    const Vec3s& a,
                                                    const Vec3s& b,
                                                    const Matrix3s& Bf) {
  Vec3s AABB_corner;
  AABB_corner.noalias() = T.cwiseAbs() - Bf * b - a;
  return AABB_corner.array().max(Scalar(0)).matrix().squaredNorm();
}

// | B^T T | - b - Bf^T a
inline Scalar _computeDistanceForCase2_ManualProduct(const Matrix3s& B,
                                                     const Vec3s& T,
                                                     const Vec3s& a,
                                                     const Vec3s& b,
                                                     const Matrix3s& Bf) {
  Scalar s, t = 0;
  s = std::abs(B.col(0).dot(T)) - Bf.col(0).dot(a) - b[0];
  if (s > 0) t += s * s;
  s = std::abs(B.col(1).dot(T)) - Bf.col(1).dot(a) - b[1];
  if (s > 0) t += s * s;
  s = std::abs(B.col(2).dot(T)) - Bf.col(2).dot(a) - b[2];
  if (s > 0) t += s * s;
  return t;
}

// Same as _computeDistanceForCase2_ManualProduct, with the Eigen product
inline Scalar _computeDistanceForCase2_EigenProduct(const Matrix3s& B,
                                                    const Vec3s& T,
                                                    const Vec3s& a,
                                                    const Vec3s& b,
                                                    const Matrix3s& Bf) {
  Vec3s AABB_corner((B.transpose() * T).cwiseAbs() - Bf.transpose() * a - b);
  return AABB_corner.array().max(Scalar(0)).matrix().squaredNorm();
}

// ------------------------ 0 --------------------------------------
template <Case1Fn Case1, Case2Fn Case2>
bool withRuntimeLoop(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                     const Vec3s& b, const Scalar& breakDistance2,
                     Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = Case1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  // | B^T T| - b - Bf^T a
  squaredLowerBoundDistance = Case2(B, T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  int ja = 1, ka = 2, jb = 1, kb = 2;
  for (int ia = 0; ia < 3; ++ia) {
    for (int ib = 0; ib < 3; ++ib) {
      const Scalar s = T[ka] * B(ja, ib) - T[ja] * B(ka, ib);

      const Scalar diff = fabs(s) - (a[ja] * Bf(ka, ib) + a[ka] * Bf(ja, ib) +
                                     b[jb] * Bf(ia, kb) + b[kb] * Bf(ia, jb));
      // We need to divide by the norm || Aia x Bib ||
      // As ||Aia|| = ||Bib|| = 1, (Aia | Bib)^2  = cosine^2
      if (diff > 0) {
        Scalar sinus2 = 1 - Bf(ia, ib) * Bf(ia, ib);
        if (sinus2 > 1e-6) {
          squaredLowerBoundDistance = diff * diff / sinus2;
          if (squaredLowerBoundDistance > breakDistance2) {
            return true;
          }
        }
        /* // or
           Scalar sinus2 = 1 - Bf (ia,ib) * Bf (ia,ib);
           squaredLowerBoundDistance = diff * diff;
           if (squaredLowerBoundDistance > breakDistance2 * sinus2) {
           squaredLowerBoundDistance /= sinus2;
           return true;
           }
        // */
      }

      jb = kb;
      kb = ib;
    }
    ja = ka;
    ka = ia;
  }

  return false;
}

// ------------------------ 1 --------------------------------------
bool withManualLoopUnrolling_1(const Matrix3s& B, const Vec3s& T,
                               const Vec3s& a, const Vec3s& b,
                               const Scalar& breakDistance2,
                               Scalar& squaredLowerBoundDistance) {
  Scalar t, s;
  Scalar diff;

  // Matrix3s Bf = abs(B);
  // Bf += reps;
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  Vec3s AABB_corner(T.cwiseAbs() - Bf * b);
  Vec3s diff3(AABB_corner - a);
  diff3 = diff3.cwiseMax(Vec3s::Zero());

  // for (Vec3s::Index i=0; i<3; ++i) diff3 [i] = std::max (0, diff3 [i]);
  squaredLowerBoundDistance = diff3.squaredNorm();
  if (squaredLowerBoundDistance > breakDistance2) return true;

  AABB_corner = (B.transpose() * T).cwiseAbs() - Bf.transpose() * a;
  // diff3 = | B^T T| - b - Bf^T a
  diff3 = AABB_corner - b;
  diff3 = diff3.cwiseMax(Vec3s::Zero());
  squaredLowerBoundDistance = diff3.squaredNorm();

  if (squaredLowerBoundDistance > breakDistance2) return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  Scalar sinus2;
  diff = t - (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) + b[1] * Bf(0, 2) +
              b[2] * Bf(0, 1));
  // We need to divide by the norm || A0 x B0 ||
  // As ||A0|| = ||B0|| = 1,
  //              2            2
  // || A0 x B0 ||  + (A0 | B0)  = 1
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 0) * Bf(0, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) + b[0] * Bf(0, 2) +
              b[2] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 1) * Bf(0, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) + b[0] * Bf(0, 1) +
              b[1] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 2) * Bf(0, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) + b[1] * Bf(1, 2) +
              b[2] * Bf(1, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 0) * Bf(1, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) + b[0] * Bf(1, 2) +
              b[2] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 1) * Bf(1, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) + b[0] * Bf(1, 1) +
              b[1] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 2) * Bf(1, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) + b[1] * Bf(2, 2) +
              b[2] * Bf(2, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 0) * Bf(2, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) + b[0] * Bf(2, 2) +
              b[2] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 1) * Bf(2, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) + b[0] * Bf(2, 1) +
              b[1] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 2) * Bf(2, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  return false;
}

// ------------------------ 2 --------------------------------------
template <Case1Fn Case1, Case2Fn Case2>
bool withManualLoopUnrolling_2(const Matrix3s& B, const Vec3s& T,
                               const Vec3s& a, const Vec3s& b,
                               const Scalar& breakDistance2,
                               Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = Case1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  squaredLowerBoundDistance = Case2(B, T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  // A0 x B0
  Scalar t, s;
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  Scalar sinus2;
  Scalar diff;
  diff = t - (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) + b[1] * Bf(0, 2) +
              b[2] * Bf(0, 1));
  // We need to divide by the norm || A0 x B0 ||
  // As ||A0|| = ||B0|| = 1,
  //              2            2
  // || A0 x B0 ||  + (A0 | B0)  = 1
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 0) * Bf(0, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) + b[0] * Bf(0, 2) +
              b[2] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 1) * Bf(0, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) + b[0] * Bf(0, 1) +
              b[1] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 2) * Bf(0, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) + b[1] * Bf(1, 2) +
              b[2] * Bf(1, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 0) * Bf(1, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) + b[0] * Bf(1, 2) +
              b[2] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 1) * Bf(1, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) + b[0] * Bf(1, 1) +
              b[1] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 2) * Bf(1, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) + b[1] * Bf(2, 2) +
              b[2] * Bf(2, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 0) * Bf(2, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) + b[0] * Bf(2, 2) +
              b[2] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 1) * Bf(2, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);
  assert(t == fabs(s));

  diff = t - (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) + b[0] * Bf(2, 1) +
              b[1] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 2) * Bf(2, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  return false;
}

// ------------------------ 3 --------------------------------------
template <int ia, int ib, int ja = (ia + 1) % 3, int ka = (ia + 2) % 3,
          int jb = (ib + 1) % 3, int kb = (ib + 2) % 3>
struct loop_case_1 {
  static inline bool run(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                         const Vec3s& b, const Matrix3s& Bf,
                         const Scalar& breakDistance2,
                         Scalar& squaredLowerBoundDistance) {
    const Scalar s = T[ka] * B(ja, ib) - T[ja] * B(ka, ib);

    const Scalar diff = fabs(s) - (a[ja] * Bf(ka, ib) + a[ka] * Bf(ja, ib) +
                                   b[jb] * Bf(ia, kb) + b[kb] * Bf(ia, jb));
    // We need to divide by the norm || Aia x Bib ||
    // As ||Aia|| = ||Bib|| = 1, (Aia | Bib)^2  = cosine^2
    if (diff > 0) {
      Scalar sinus2 = 1 - Bf(ia, ib) * Bf(ia, ib);
      if (sinus2 > 1e-6) {
        squaredLowerBoundDistance = diff * diff / sinus2;
        if (squaredLowerBoundDistance > breakDistance2) {
          return true;
        }
      }
      /* // or
         Scalar sinus2 = 1 - Bf (ia,ib) * Bf (ia,ib);
         squaredLowerBoundDistance = diff * diff;
         if (squaredLowerBoundDistance > breakDistance2 * sinus2) {
         squaredLowerBoundDistance /= sinus2;
         return true;
         }
      // */
    }
    return false;
  }
};

template <Case1Fn Case1, Case2Fn Case2>
bool withTemplateLoopUnrolling_1(const Matrix3s& B, const Vec3s& T,
                                 const Vec3s& a, const Vec3s& b,
                                 const Scalar& breakDistance2,
                                 Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = Case1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  squaredLowerBoundDistance = Case2(B, T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  // Ai x Bj
  if (loop_case_1<0, 0>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<0, 1>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<0, 2>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<1, 0>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<1, 1>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<1, 2>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<2, 0>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<2, 1>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;
  if (loop_case_1<2, 2>::run(B, T, a, b, Bf, breakDistance2,
                             squaredLowerBoundDistance))
    return true;

  return false;
}

// ------------------------ 4 --------------------------------------

template <int ib, int jb = (ib + 1) % 3, int kb = (ib + 2) % 3>
struct loop_case_2 {
  static inline bool run(int ia, int ja, int ka, const Matrix3s& B,
                         const Vec3s& T, const Vec3s& a, const Vec3s& b,
                         const Matrix3s& Bf, const Scalar& breakDistance2,
                         Scalar& squaredLowerBoundDistance) {
    const Scalar s = T[ka] * B(ja, ib) - T[ja] * B(ka, ib);

    const Scalar diff = fabs(s) - (a[ja] * Bf(ka, ib) + a[ka] * Bf(ja, ib) +
                                   b[jb] * Bf(ia, kb) + b[kb] * Bf(ia, jb));
    // We need to divide by the norm || Aia x Bib ||
    // As ||Aia|| = ||Bib|| = 1, (Aia | Bib)^2  = cosine^2
    if (diff > 0) {
      Scalar sinus2 = 1 - Bf(ia, ib) * Bf(ia, ib);
      if (sinus2 > 1e-6) {
        squaredLowerBoundDistance = diff * diff / sinus2;
        if (squaredLowerBoundDistance > breakDistance2) {
          return true;
        }
      }
      /* // or
         Scalar sinus2 = 1 - Bf (ia,ib) * Bf (ia,ib);
         squaredLowerBoundDistance = diff * diff;
         if (squaredLowerBoundDistance > breakDistance2 * sinus2) {
         squaredLowerBoundDistance /= sinus2;
         return true;
         }
      // */
    }
    return false;
  }
};

template <Case1Fn Case1, Case2Fn Case2>
bool withPartialTemplateLoopUnrolling_1(const Matrix3s& B, const Vec3s& T,
                                        const Vec3s& a, const Vec3s& b,
                                        const Scalar& breakDistance2,
                                        Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = Case1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  squaredLowerBoundDistance = Case2(B, T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  // Ai x Bj
  int ja = 1, ka = 2;
  for (int ia = 0; ia < 3; ++ia) {
    if (loop_case_2<0>::run(ia, ja, ka, B, T, a, b, Bf, breakDistance2,
                            squaredLowerBoundDistance))
      return true;
    if (loop_case_2<1>::run(ia, ja, ka, B, T, a, b, Bf, breakDistance2,
                            squaredLowerBoundDistance))
      return true;
    if (loop_case_2<2>::run(ia, ja, ka, B, T, a, b, Bf, breakDistance2,
                            squaredLowerBoundDistance))
      return true;
    ja = ka;
    ka = ia;
  }

  return false;
}

// ------------------------ 5 --------------------------------------
bool originalWithLowerBound(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                            const Vec3s& b, const Scalar& breakDistance2,
                            Scalar& squaredLowerBoundDistance) {
  Scalar t, s;
  Scalar diff;

  Matrix3s Bf(B.cwiseAbs());
  squaredLowerBoundDistance = 0;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);

  diff = t - (a[0] + Bf.row(0).dot(b));
  if (diff > 0) {
    squaredLowerBoundDistance = diff * diff;
  }

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  diff = t - (a[1] + Bf.row(1).dot(b));
  if (diff > 0) {
    squaredLowerBoundDistance += diff * diff;
  }

  // A0 x A1 = A2
  t = ((T[2] < 0.0) ? -T[2] : T[2]);

  diff = t - (a[2] + Bf.row(2).dot(b));
  if (diff > 0) {
    squaredLowerBoundDistance += diff * diff;
  }

  if (squaredLowerBoundDistance > breakDistance2) return true;

  squaredLowerBoundDistance = 0;

  // B1 x B2 = B0
  s = B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  diff = t - (b[0] + Bf.col(0).dot(a));
  if (diff > 0) {
    squaredLowerBoundDistance += diff * diff;
  }

  // B2 x B0 = B1
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  diff = t - (b[1] + Bf.col(1).dot(a));
  if (diff > 0) {
    squaredLowerBoundDistance += diff * diff;
  }

  // B0 x B1 = B2
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  diff = t - (b[2] + Bf.col(2).dot(a));
  if (diff > 0) {
    squaredLowerBoundDistance += diff * diff;
  }

  if (squaredLowerBoundDistance > breakDistance2) return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  Scalar sinus2;
  diff = t - (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) + b[1] * Bf(0, 2) +
              b[2] * Bf(0, 1));
  // We need to divide by the norm || A0 x B0 ||
  // As ||A0|| = ||B0|| = 1,
  //              2            2
  // || A0 x B0 ||  + (A0 | B0)  = 1
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 0) * Bf(0, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) + b[0] * Bf(0, 2) +
              b[2] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 1) * Bf(0, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) + b[0] * Bf(0, 1) +
              b[1] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(0, 2) * Bf(0, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) + b[1] * Bf(1, 2) +
              b[2] * Bf(1, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 0) * Bf(1, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) + b[0] * Bf(1, 2) +
              b[2] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 1) * Bf(1, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) + b[0] * Bf(1, 1) +
              b[1] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(1, 2) * Bf(1, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) + b[1] * Bf(2, 2) +
              b[2] * Bf(2, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 0) * Bf(2, 0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) + b[0] * Bf(2, 2) +
              b[2] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 1) * Bf(2, 1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) + b[0] * Bf(2, 1) +
              b[1] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf(2, 2) * Bf(2, 2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
        return true;
      }
    }
  }

  return false;
}

// ------------------------ 6 --------------------------------------
bool originalWithNoLowerBound(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                              const Vec3s& b, const Scalar&,
                              Scalar& squaredLowerBoundDistance) {
  Scalar t, s;
  const Scalar reps = Scalar(1e-6);

  squaredLowerBoundDistance = 0;

  Matrix3s Bf(B.array().abs() + reps);
  // Bf += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);

  // if(t > (a[0] + Bf.dotX(b)))
  if (t > (a[0] + Bf.row(0).dot(b))) return true;

  // B1 x B2 = B0
  // s =  B.transposeDotX(T);
  s = B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[0] + Bf.transposeDotX(a)))
  if (t > (b[0] + Bf.col(0).dot(a))) return true;

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  // if(t > (a[1] + Bf.dotY(b)))
  if (t > (a[1] + Bf.row(1).dot(b))) return true;

  // A0 x A1 = A2
  t = ((T[2] < 0.0) ? -T[2] : T[2]);

  // if(t > (a[2] + Bf.dotZ(b)))
  if (t > (a[2] + Bf.row(2).dot(b))) return true;

  // B2 x B0 = B1
  // s = B.transposeDotY(T);
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[1] + Bf.transposeDotY(a)))
  if (t > (b[1] + Bf.col(1).dot(a))) return true;

  // B0 x B1 = B2
  // s = B.transposeDotZ(T);
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[2] + Bf.transposeDotZ(a)))
  if (t > (b[2] + Bf.col(2).dot(a))) return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) + b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) + b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) + b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) + b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) + b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) + b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) + b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) + b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  if (t >
      (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) + b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;
}
}  // namespace obbDisjoint_impls

using obbDisjoint_impls::_computeDistanceForCase1_EigenProduct;
using obbDisjoint_impls::_computeDistanceForCase1_ManualProduct;
using obbDisjoint_impls::_computeDistanceForCase2_EigenProduct;
using obbDisjoint_impls::_computeDistanceForCase2_ManualProduct;

// Run the benchmark for the given separating axis test implementation
template <SeparatingAxisTest test>
void run(benchmark::State& state) {
  const std::vector<ObbCase>& cases = get_cases();

  Scalar squared_lower_bound_distance = 0;
  for (auto _ : state) {
    for (const ObbCase& obb : cases) {
      bool disjoint = test(obb.B, obb.T, obb.a, obb.b, break_distance2,
                           squared_lower_bound_distance);
      benchmark::DoNotOptimize(disjoint);
    }
    benchmark::DoNotOptimize(squared_lower_bound_distance);
  }

  state.SetItemsProcessed(state.iterations() *
                          static_cast<std::int64_t>(cases.size()));
}

void withRuntimeLoop_ManualProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withRuntimeLoop<
      _computeDistanceForCase1_ManualProduct,
      _computeDistanceForCase2_ManualProduct>>(state);
}

void withRuntimeLoop_EigenProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withRuntimeLoop<
      _computeDistanceForCase1_EigenProduct,
      _computeDistanceForCase2_EigenProduct>>(state);
}

void withManualLoopUnrolling_1(benchmark::State& state) {
  run<obbDisjoint_impls::withManualLoopUnrolling_1>(state);
}

void withManualLoopUnrolling_2_ManualProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withManualLoopUnrolling_2<
      _computeDistanceForCase1_ManualProduct,
      _computeDistanceForCase2_ManualProduct>>(state);
}

void withManualLoopUnrolling_2_EigenProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withManualLoopUnrolling_2<
      _computeDistanceForCase1_EigenProduct,
      _computeDistanceForCase2_EigenProduct>>(state);
}

void withTemplateLoopUnrolling_1_ManualProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withTemplateLoopUnrolling_1<
      _computeDistanceForCase1_ManualProduct,
      _computeDistanceForCase2_ManualProduct>>(state);
}

void withTemplateLoopUnrolling_1_EigenProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withTemplateLoopUnrolling_1<
      _computeDistanceForCase1_EigenProduct,
      _computeDistanceForCase2_EigenProduct>>(state);
}

void withPartialTemplateLoopUnrolling_1_ManualProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withPartialTemplateLoopUnrolling_1<
      _computeDistanceForCase1_ManualProduct,
      _computeDistanceForCase2_ManualProduct>>(state);
}

void withPartialTemplateLoopUnrolling_1_EigenProduct(benchmark::State& state) {
  run<obbDisjoint_impls::withPartialTemplateLoopUnrolling_1<
      _computeDistanceForCase1_EigenProduct,
      _computeDistanceForCase2_EigenProduct>>(state);
}

void originalWithLowerBound(benchmark::State& state) {
  run<obbDisjoint_impls::originalWithLowerBound>(state);
}

void originalWithNoLowerBound(benchmark::State& state) {
  run<obbDisjoint_impls::originalWithNoLowerBound>(state);
}

}  // namespace

BENCHMARK(withRuntimeLoop_ManualProduct);
BENCHMARK(withRuntimeLoop_EigenProduct);
BENCHMARK(withManualLoopUnrolling_1);
BENCHMARK(withManualLoopUnrolling_2_ManualProduct);
BENCHMARK(withManualLoopUnrolling_2_EigenProduct);
BENCHMARK(withTemplateLoopUnrolling_1_ManualProduct);
BENCHMARK(withTemplateLoopUnrolling_1_EigenProduct);
BENCHMARK(withPartialTemplateLoopUnrolling_1_ManualProduct);
BENCHMARK(withPartialTemplateLoopUnrolling_1_EigenProduct);
BENCHMARK(originalWithLowerBound);
BENCHMARK(originalWithNoLowerBound);

BENCHMARK_MAIN();
