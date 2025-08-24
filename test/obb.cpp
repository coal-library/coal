/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS
 *  Copyright (c) 2023, Inria
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

#include <iostream>
#include <fstream>
#include <sstream>

#include <chrono>

#include "coal/narrowphase/narrowphase.h"

#include "../src/BV/OBB.h"
#include "coal/internal/shape_shape_func.h"
#include "utility.h"

using namespace coal;

void randomOBBs(Vec3s& a, Vec3s& b, Scalar extentNorm) {
  // Extent norm is between 0 and extentNorm on each axis
  // a = (Vec3s::Ones()+Vec3s::Random()) * extentNorm / (2*sqrt(3));
  // b = (Vec3s::Ones()+Vec3s::Random()) * extentNorm / (2*sqrt(3));

  a = extentNorm * Vec3s::Random().cwiseAbs().normalized();
  b = extentNorm * Vec3s::Random().cwiseAbs().normalized();
}

void randomTransform(Matrix3s& B, Vec3s& T, const Vec3s& a, const Vec3s& b,
                     const Scalar extentNorm) {
  // TODO Should we scale T to a and b norm ?
  (void)a;
  (void)b;
  (void)extentNorm;

  Scalar N = a.norm() + b.norm();
  // A translation of norm N ensures there is no collision.
  // Set translation to be between 0 and 2 * N;
  T = (Vec3s::Random() / sqrt(3)) * 1.5 * N;
  // T.setZero();

  Quats q;
  q.coeffs().setRandom();
  q.normalize();
  B = q;
}

#define NB_METHODS 7
#define MANUAL_PRODUCT 1

#if MANUAL_PRODUCT
#define PRODUCT(M33, v3) \
  (M33.col(0) * v3[0] + M33.col(1) * v3[1] + M33.col(2) * v3[2])
#else
#define PRODUCT(M33, v3) (M33 * v3)
#endif

typedef std::chrono::high_resolution_clock clock_type;
typedef clock_type::duration duration_type;

const char* sep = ",\t";
const Scalar eps = Scalar(1.5e-7);

const Eigen::IOFormat py_fmt(Eigen::FullPrecision, 0,
                             ", ",   // Coeff separator
                             ",\n",  // Row separator
                             "(",    // row prefix
                             ",)",   // row suffix
                             "( ",   // mat prefix
                             ", )"   // mat suffix
);

namespace obbDisjoint_impls {
/// @return true if OBB are disjoint.
bool distance(const Matrix3s& B, const Vec3s& T, const Vec3s& a, const Vec3s& b,
              Scalar& distance) {
  GJKSolver gjk;
  Box ba(2 * a), bb(2 * b);
  Transform3s tfa, tfb(B, T);

  Vec3s p1, p2, normal;
  bool compute_penetration = true;
  distance =
      gjk.shapeDistance(ba, tfa, bb, tfb, compute_penetration, p1, p2, normal);
  return (distance > gjk.getDistancePrecision(compute_penetration));
}

inline Scalar _computeDistanceForCase1(const Vec3s& T, const Vec3s& a,
                                       const Vec3s& b, const Matrix3s& Bf) {
  Vec3s AABB_corner;
  /* This seems to be slower
 AABB_corner.noalias() = T.cwiseAbs () - a;
 AABB_corner.noalias() -= PRODUCT(Bf,b);
 /*/
#if MANUAL_PRODUCT
  AABB_corner.noalias() = T.cwiseAbs() - a;
  AABB_corner.noalias() -= Bf.col(0) * b[0];
  AABB_corner.noalias() -= Bf.col(1) * b[1];
  AABB_corner.noalias() -= Bf.col(2) * b[2];
#else
  AABB_corner.noalias() = T.cwiseAbs() - Bf * b - a;
#endif
  // */
  return AABB_corner.array().max(Scalar(0)).matrix().squaredNorm();
}

inline Scalar _computeDistanceForCase2(const Matrix3s& B, const Vec3s& T,
                                       const Vec3s& a, const Vec3s& b,
                                       const Matrix3s& Bf) {
  /*
  Vec3s AABB_corner(PRODUCT(B.transpose(), T).cwiseAbs() - b);
  AABB_corner.noalias() -= PRODUCT(Bf.transpose(), a);
  return AABB_corner.array().max(Scalar(0)).matrix().squaredNorm ();
  /*/
#if MANUAL_PRODUCT
  Scalar s, t = 0;
  s = std::abs(B.col(0).dot(T)) - Bf.col(0).dot(a) - b[0];
  if (s > 0) t += s * s;
  s = std::abs(B.col(1).dot(T)) - Bf.col(1).dot(a) - b[1];
  if (s > 0) t += s * s;
  s = std::abs(B.col(2).dot(T)) - Bf.col(2).dot(a) - b[2];
  if (s > 0) t += s * s;
  return t;
#else
  Vec3s AABB_corner((B.transpose() * T).cwiseAbs() - Bf.transpose() * a - b);
  return AABB_corner.array().max(Scalar(0)).matrix().squaredNorm();
#endif
  // */
}

int separatingAxisId(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                     const Vec3s& b, const Scalar& breakDistance2,
                     Scalar& squaredLowerBoundDistance) {
  int id = 0;

  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = _computeDistanceForCase1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return id;
  ++id;

  // | B^T T| - b - Bf^T a
  squaredLowerBoundDistance = _computeDistanceForCase2(B, T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return id;
  ++id;

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
            return id;
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
      ++id;

      jb = kb;
      kb = ib;
    }
    ja = ka;
    ka = ia;
  }

  return id;
}

// ------------------------ 0 --------------------------------------
bool withRuntimeLoop(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                     const Vec3s& b, const Scalar& breakDistance2,
                     Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = _computeDistanceForCase1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  // | B^T T| - b - Bf^T a
  squaredLowerBoundDistance = _computeDistanceForCase2(B, T, a, b, Bf);
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
bool withManualLoopUnrolling_2(const Matrix3s& B, const Vec3s& T,
                               const Vec3s& a, const Vec3s& b,
                               const Scalar& breakDistance2,
                               Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = _computeDistanceForCase1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  squaredLowerBoundDistance = _computeDistanceForCase2(B, T, a, b, Bf);
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

bool withTemplateLoopUnrolling_1(const Matrix3s& B, const Vec3s& T,
                                 const Vec3s& a, const Vec3s& b,
                                 const Scalar& breakDistance2,
                                 Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = _computeDistanceForCase1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  squaredLowerBoundDistance = _computeDistanceForCase2(B, T, a, b, Bf);
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

bool withPartialTemplateLoopUnrolling_1(const Matrix3s& B, const Vec3s& T,
                                        const Vec3s& a, const Vec3s& b,
                                        const Scalar& breakDistance2,
                                        Scalar& squaredLowerBoundDistance) {
  Matrix3s Bf(B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = _computeDistanceForCase1(T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2) return true;

  squaredLowerBoundDistance = _computeDistanceForCase2(B, T, a, b, Bf);
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

struct BenchmarkResult {
  /// The test ID:
  /// - 0-10 identifies a separating axes.
  /// - 11 means no separatins axes could be found. (distance should be <= 0)
  int ifId;
  Scalar distance;
  Scalar squaredLowerBoundDistance;
  duration_type duration[NB_METHODS];
  bool failure;

  static std::ostream& headers(std::ostream& os) {
    const std::string unit = " (us)";
    os << "separating axis" << sep << "distance lower bound" << sep
       << "distance" << sep << "failure" << sep << "Runtime Loop" << unit << sep
       << "Manual Loop Unrolling 1" << unit << sep << "Manual Loop Unrolling 2"
       << unit << sep << "Template Unrolling" << unit << sep
       << "Partial Template Unrolling" << unit << sep << "Original (LowerBound)"
       << unit << sep << "Original (NoLowerBound)" << unit;
    return os;
  }

  std::ostream& print(std::ostream& os) const {
    os << ifId << sep << std::sqrt(squaredLowerBoundDistance) << sep << distance
       << sep << failure;
    for (int i = 0; i < NB_METHODS; ++i)
      os << sep
         << static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    duration[i])
                    .count()) *
                1e-3;
    return os;
  }
};

std::ostream& operator<<(std::ostream& os, const BenchmarkResult& br) {
  return br.print(os);
}

BenchmarkResult benchmark_obb_case(const Matrix3s& B, const Vec3s& T,
                                   const Vec3s& a, const Vec3s& b,
                                   const CollisionRequest& request,
                                   std::size_t N) {
  const Scalar breakDistance(request.break_distance + request.security_margin);
  const Scalar breakDistance2 = breakDistance * breakDistance;

  BenchmarkResult result;
  // First determine which axis provide the answer
  result.ifId = obbDisjoint_impls::separatingAxisId(
      B, T, a, b, breakDistance2, result.squaredLowerBoundDistance);
  bool disjoint = obbDisjoint_impls::distance(B, T, a, b, result.distance);
  assert(0 <= result.ifId && result.ifId <= 11);

  // Sanity check
  result.failure = true;
  bool overlap = (result.ifId == 11);
  Scalar dist_thr = request.break_distance + request.security_margin;
  if (!overlap && result.distance <= 0) {
    std::cerr << "Failure: negative distance for disjoint OBBs.";
  } else if (!overlap && result.squaredLowerBoundDistance < 0) {
    std::cerr << "Failure: negative distance lower bound.";
  } else if (!overlap && eps < std::sqrt(result.squaredLowerBoundDistance) -
                                   result.distance) {
    std::cerr << "Failure: distance is inferior to its lower bound (diff = "
              << std::sqrt(result.squaredLowerBoundDistance) - result.distance
              << ").";
  } else if (overlap != !disjoint && result.distance >= dist_thr - eps) {
    std::cerr << "Failure: overlapping test and distance query mismatch.";
  } else if (overlap && result.distance >= dist_thr - eps) {
    std::cerr << "Failure: positive distance for overlapping OBBs.";
  } else {
    result.failure = false;
  }
  if (result.failure) {
    std::cerr << "\nR = " << Quats(B).coeffs().transpose().format(py_fmt)
              << "\nT = " << T.transpose().format(py_fmt)
              << "\na = " << a.transpose().format(py_fmt)
              << "\nb = " << b.transpose().format(py_fmt)
              << "\nresult = " << result << '\n'
              << std::endl;
  }

  // Compute time
  Scalar tmp;
  clock_type::time_point start, end;

  // ------------------------ 0 --------------------------------------
  start = clock_type::now();
  for (std::size_t i = 0; i < N; ++i)
    obbDisjoint_impls::withRuntimeLoop(B, T, a, b, breakDistance2, tmp);
  end = clock_type::now();
  result.duration[0] = (end - start) / N;

  // ------------------------ 1 --------------------------------------
  start = clock_type::now();
  for (std::size_t i = 0; i < N; ++i)
    obbDisjoint_impls::withManualLoopUnrolling_1(B, T, a, b, breakDistance2,
                                                 tmp);
  end = clock_type::now();
  result.duration[1] = (end - start) / N;

  // ------------------------ 2 --------------------------------------
  start = clock_type::now();
  for (std::size_t i = 0; i < N; ++i)
    obbDisjoint_impls::withManualLoopUnrolling_2(B, T, a, b, breakDistance2,
                                                 tmp);
  end = clock_type::now();
  result.duration[2] = (end - start) / N;

  // ------------------------ 3 --------------------------------------
  start = clock_type::now();
  for (std::size_t i = 0; i < N; ++i)
    obbDisjoint_impls::withTemplateLoopUnrolling_1(B, T, a, b, breakDistance2,
                                                   tmp);
  end = clock_type::now();
  result.duration[3] = (end - start) / N;

  // ------------------------ 4 --------------------------------------
  start = clock_type::now();
  for (std::size_t i = 0; i < N; ++i)
    obbDisjoint_impls::withPartialTemplateLoopUnrolling_1(B, T, a, b,
                                                          breakDistance2, tmp);
  end = clock_type::now();
  result.duration[4] = (end - start) / N;

  // ------------------------ 5 --------------------------------------
  start = clock_type::now();
  for (std::size_t i = 0; i < N; ++i)
    obbDisjoint_impls::originalWithLowerBound(B, T, a, b, breakDistance2, tmp);
  end = clock_type::now();
  result.duration[5] = (end - start) / N;

  // ------------------------ 6 --------------------------------------
  start = clock_type::now();
  // The 2 last arguments are unused.
  for (std::size_t i = 0; i < N; ++i)
    obbDisjoint_impls::originalWithNoLowerBound(B, T, a, b, breakDistance2,
                                                tmp);
  end = clock_type::now();
  result.duration[6] = (end - start) / N;

  return result;
}

std::size_t obb_overlap_and_lower_bound_distance(std::ostream* output) {
  std::size_t nbFailure = 0;

  // Create two OBBs axis
  Vec3s a, b;
  Matrix3s B;
  Vec3s T;
  CollisionRequest request;

#ifndef NDEBUG  // if debug mode
  static const size_t nbRandomOBB = 10;
  static const size_t nbTransformPerOBB = 10;
  static const size_t nbRunForTimeMeas = 10;
#else
  static const size_t nbRandomOBB = 100;
  static const size_t nbTransformPerOBB = 100;
  static const size_t nbRunForTimeMeas = 1000;
#endif
  static const Scalar extentNorm = 1.;

  if (output != NULL) *output << BenchmarkResult::headers << '\n';

  BenchmarkResult result;
  for (std::size_t iobb = 0; iobb < nbRandomOBB; ++iobb) {
    randomOBBs(a, b, extentNorm);
    for (std::size_t itf = 0; itf < nbTransformPerOBB; ++itf) {
      randomTransform(B, T, a, b, extentNorm);
      result = benchmark_obb_case(B, T, a, b, request, nbRunForTimeMeas);
      if (output != NULL) *output << result << '\n';
      if (result.failure) nbFailure++;
    }
  }
  return nbFailure;
}

int main(int argc, char** argv) {
  std::ostream* output = NULL;
  if (argc > 1 && strcmp(argv[1], "--generate-output") == 0) {
    output = &std::cout;
  }

  std::cout << "The benchmark real time measurements may be noisy and "
               "will incur extra overhead."
               "\nUse the following commands to turn ON:"
               "\n\tsudo cpufreq-set --governor performance"
               "\nor OFF:"
               "\n\tsudo cpufreq-set --governor powersave"
               "\n";

  std::size_t nbFailure = obb_overlap_and_lower_bound_distance(output);
  if (nbFailure > INT_MAX) return INT_MAX;
  return (int)nbFailure;
}
