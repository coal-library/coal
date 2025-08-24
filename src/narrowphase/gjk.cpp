/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

/** \author Jia Pan */

#include "coal/shape/geometric_shapes.h"
#include "coal/narrowphase/gjk.h"
#include "coal/internal/intersect.h"
#include "coal/internal/tools.h"
#include "coal/shape/geometric_shapes_traits.h"
#include "coal/narrowphase/narrowphase_defaults.h"

#include "coal/tracy.hh"

namespace coal {

namespace details {

void GJK::initialize() {
  distance_upper_bound = (std::numeric_limits<SolverScalar>::max)();
  gjk_variant = GJKVariant::DefaultGJK;
  convergence_criterion = GJKConvergenceCriterion::Default;
  convergence_criterion_type = GJKConvergenceCriterionType::Relative;
  reset(max_iterations, tolerance);
}

void GJK::reset(size_t max_iterations_, SolverScalar tolerance_) {
  max_iterations = max_iterations_;
  tolerance = tolerance_;
  COAL_ASSERT(tolerance_ > 0, "Tolerance must be positive.",
              std::invalid_argument);
  status = DidNotRun;
  nfree = 0;
  simplex = nullptr;
  iterations = 0;
  iterations_momentum_stop = 0;
}

Vec3ps GJK::getGuessFromSimplex() const { return ray; }

namespace details {

// This function computes the weights associated with projecting the origin
// onto the final simplex of GJK or EPA.
// The origin is always a convex combination of the supports of the simplex.
// The weights are then linearly distributed among the shapes' supports, thanks
// to the following property:
//   if s is a support of the Minkowski difference, then:
//   w = w.w0 - w.w1, with w.w0 a support of shape0 and w.w1 a support of
//   shape1.
// clang-format off
// Suppose the final simplex is of rank 2:
// We have 0 = alpha * w[0] + (1 - alpha) * w[1], with alpha the weight of the convex
// decomposition, then:
//    0 = alpha * (w[0].w0 - w[0].w1) + (1 - alpha) * (w[1].w0 - w[1].w1)
// => 0 = alpha * w[0].w0 + (1 - alpha) * w[1].w0
//      - alpha * w[0].w1 - (1 - alpha) * w[1].w1
// Therefore we have two witness points:
//   w0 = alpha * w[0].w0 + (1 - alpha) * w[1].w0
//   w1 = alpha * w[0].w1 + (1 - alpha) * w[1].w1
// clang-format on
// TODO
void getClosestPoints(const GJK::Simplex& simplex, Vec3ps& w0, Vec3ps& w1) {
  GJK::SimplexV* const* vs = simplex.vertex;

  for (GJK::vertex_id_t i = 0; i < simplex.rank; ++i) {
    assert(vs[i]->w.isApprox(vs[i]->w0 - vs[i]->w1));
  }

  Project<SolverScalar>::ProjectResult projection;
  switch (simplex.rank) {
    case 1:
      w0 = vs[0]->w0;
      w1 = vs[0]->w1;
      return;
    case 2: {
      const Vec3ps &a = vs[0]->w, a0 = vs[0]->w0, a1 = vs[0]->w1, b = vs[1]->w,
                   b0 = vs[1]->w0, b1 = vs[1]->w1;
      SolverScalar la, lb;
      Vec3ps N(b - a);
      la = N.dot(-a);
      if (la <= 0) {
        assert(false);
        w0 = a0;
        w1 = a1;
      } else {
        lb = N.squaredNorm();
        if (la > lb) {
          assert(false);
          w0 = b0;
          w1 = b1;
        } else {
          lb = la / lb;
          la = 1 - lb;
          w0 = la * a0 + lb * b0;
          w1 = la * a1 + lb * b1;
        }
      }
    }
      return;
    case 3:
      // TODO avoid the reprojection
      projection = Project<SolverScalar>::projectTriangleOrigin(
          vs[0]->w, vs[1]->w, vs[2]->w);
      break;
    case 4:  // We are in collision.
      projection = Project<SolverScalar>::projectTetrahedraOrigin(
          vs[0]->w, vs[1]->w, vs[2]->w, vs[3]->w);
      break;
    default:
      COAL_THROW_PRETTY("The simplex rank must be in [ 1, 4 ]",
                        std::logic_error);
  }
  w0.setZero();
  w1.setZero();
  for (GJK::vertex_id_t i = 0; i < simplex.rank; ++i) {
    w0 += projection.parameterization[i] * vs[i]->w0;
    w1 += projection.parameterization[i] * vs[i]->w1;
  }
  return;
}

/// Inflate the points along a normal.
/// The normal is typically the normal of the separating plane found by GJK
/// or the normal found by EPA.
/// The normal should follow coal convention: it points from shape0 to
/// shape1.
template <bool Separated>
void inflate(const MinkowskiDiff& shape, const Vec3ps& normal, Vec3ps& w0,
             Vec3ps& w1) {
#ifndef NDEBUG
  const SolverScalar dummy_precision =
      Eigen::NumTraits<SolverScalar>::dummy_precision();
  assert((normal.norm() - 1) < dummy_precision);
#endif

  const Eigen::Array<SolverScalar, 1, 2>& I(
      shape.swept_sphere_radius.cast<SolverScalar>());
  Eigen::Array<bool, 1, 2> inflate(I > 0);
  if (!inflate.any()) return;

  if (inflate[0]) w0 += I[0] * normal;
  if (inflate[1]) w1 -= I[1] * normal;
}

}  // namespace details

void GJK::getWitnessPointsAndNormal(const MinkowskiDiff& shape, Vec3ps& w0,
                                    Vec3ps& w1, Vec3ps& normal) const {
  details::getClosestPoints(*simplex, w0, w1);
  if ((w1 - w0).norm() > Eigen::NumTraits<SolverScalar>::dummy_precision()) {
    normal = (w1 - w0).normalized();
  } else {
    normal = -this->ray.normalized();
  }
  details::inflate<true>(shape, normal, w0, w1);
}

GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3ps& guess,
                          const support_func_guess_t& supportHint) {
  COAL_TRACY_ZONE_SCOPED_N("coal::details::GJK::evaluate");
  SolverScalar alpha = 0;
  iterations = 0;
  const SolverScalar swept_sphere_radius = shape_.swept_sphere_radius.sum();
  const SolverScalar upper_bound = distance_upper_bound + swept_sphere_radius;

  free_v[0] = &store_v[0];
  free_v[1] = &store_v[1];
  free_v[2] = &store_v[2];
  free_v[3] = &store_v[3];

  nfree = 4;
  status = NoCollision;
  shape = &shape_;
  distance = 0.0;
  current = 0;
  simplices[current].rank = 0;
  support_hint = supportHint;

  SolverScalar rl = guess.norm();
  if (rl < tolerance) {
    ray = Vec3ps(-1, 0, 0);
    rl = 1;
  } else
    ray = guess;

  // Momentum
  GJKVariant current_gjk_variant = gjk_variant;
  Vec3ps w = ray;
  Vec3ps dir = ray;
  Vec3ps y;
  SolverScalar momentum;
  bool normalize_support_direction = shape->normalize_support_direction;
  do {
    vertex_id_t next = (vertex_id_t)(1 - current);
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    if (rl < tolerance)  // mean origin is near the face of original simplex,
                         // return touch
    {
      // At this point, GJK has converged but we don't know if GJK is enough to
      // recover penetration information.
      // EPA needs to be run.
      // Unless the Minkowski difference is degenerated, EPA will run fine even
      // if the final simplex of GJK is not a tetrahedron.
      assert(rl > 0);
      status = Collision;
      // GJK is not enough to recover the penetration depth, hence we ignore the
      // swept-sphere radius for now.
      // EPA needs to be run to recover the penetration depth.
      distance = rl;
      break;
    }

    // Compute direction for support call
    switch (current_gjk_variant) {
      case DefaultGJK:
        dir = ray;
        break;

      case NesterovAcceleration:
        // Normalize heuristic for collision pairs involving convex but not
        // strictly-convex shapes This corresponds to most use cases.
        if (normalize_support_direction) {
          momentum =
              (SolverScalar(iterations) + 2) / (SolverScalar(iterations) + 3);
          y = momentum * ray + (1 - momentum) * w;
          SolverScalar y_norm = y.norm();
          // ray is the point of the Minkowski difference which currently the
          // closest to the origin. Therefore, y.norm() > ray.norm() Hence, if
          // check A above has not stopped the algorithm, we necessarily have
          // y.norm() > tolerance. The following assert is just a safety check.
          assert(y_norm > tolerance);
          dir = momentum * dir / dir.norm() + (1 - momentum) * y / y_norm;
        } else {
          momentum =
              (SolverScalar(iterations) + 1) / (SolverScalar(iterations) + 3);
          y = momentum * ray + (1 - momentum) * w;
          dir = momentum * dir + (1 - momentum) * y;
        }
        break;

      case PolyakAcceleration:
        momentum = 1 / (SolverScalar(iterations) + 1);
        dir = momentum * dir + (1 - momentum) * ray;
        break;

      default:
        COAL_THROW_PRETTY("Invalid momentum variant.", std::logic_error);
    }

    // see below, ray points away from origin
    appendVertex(curr_simplex, -dir, support_hint);

    // check removed (by ?): when the new support point is close to previous
    // support points, stop (as the new simplex is degenerated)
    w = curr_simplex.vertex[curr_simplex.rank - 1]->w;

    // check B: no collision if omega > 0
    SolverScalar omega = dir.dot(w) / dir.norm();
    if (omega > upper_bound) {
      distance = omega - swept_sphere_radius;
      status = NoCollisionEarlyStopped;
      break;
    }

    // Check to remove acceleration
    if (current_gjk_variant != DefaultGJK) {
      SolverScalar frank_wolfe_duality_gap = 2 * ray.dot(ray - w);
      if (frank_wolfe_duality_gap - tolerance <= 0) {
        removeVertex(simplices[current]);
        current_gjk_variant = DefaultGJK;  // move back to classic GJK
        iterations_momentum_stop = iterations;
        continue;  // continue to next iteration
      }
    }

    // check C: when the new support point is close to the sub-simplex where the
    // ray point lies, stop (as the new simplex again is degenerated)
    bool cv_check_passed = checkConvergence(w, rl, alpha, omega);
    if (iterations > 0 && cv_check_passed) {
      if (iterations > 0) removeVertex(simplices[current]);
      if (current_gjk_variant != DefaultGJK) {
        current_gjk_variant = DefaultGJK;  // move back to classic GJK
        iterations_momentum_stop = iterations;
        continue;
      }
      // At this point, GJK has converged and we know that rl > tolerance (see
      // check above). Therefore, penetration information can always be
      // recovered without running EPA.
      distance = rl - swept_sphere_radius;
      if (distance < tolerance) {
        status = CollisionWithPenetrationInformation;
      } else {
        status = NoCollision;
      }
      break;
    }

    // This has been rewritten thanks to the excellent video:
    // https://youtu.be/Qupqu1xe7Io
    bool inside;
    switch (curr_simplex.rank) {
      case 1:  // Only at the first iteration
        assert(iterations == 0);
        ray = w;
        inside = false;
        next_simplex.rank = 1;
        next_simplex.vertex[0] = curr_simplex.vertex[0];
        break;
      case 2:
        inside = projectLineOrigin(curr_simplex, next_simplex);
        break;
      case 3:
        inside = projectTriangleOrigin(curr_simplex, next_simplex);
        break;
      case 4:
        inside = projectTetrahedraOrigin(curr_simplex, next_simplex);
        break;
      default:
        COAL_THROW_PRETTY("Invalid simplex rank", std::logic_error);
    }
    assert(nfree + next_simplex.rank == 4);
    current = next;
    rl = ray.norm();
    if (inside || rl == 0) {
      status = Collision;
      // GJK is not enough to recover the penetration depth, hence we ignore the
      // swept-sphere radius for now.
      // EPA needs to be run to recover the penetration depth.
      distance = rl;
      break;
    }

    status = ((++iterations) < max_iterations) ? status : Failed;

  } while (status == NoCollision);

  simplex = &simplices[current];
  assert(simplex->rank > 0 && simplex->rank < 5);
  return status;
}

bool GJK::checkConvergence(const Vec3ps& w, const SolverScalar& rl,
                           SolverScalar& alpha,
                           const SolverScalar& omega) const {
  // x^* is the optimal solution (projection of origin onto the Minkowski
  // difference).
  //  x^k is the current iterate (x^k = `ray` in the code).
  // Each criterion provides a different guarantee on the distance to the
  // optimal solution.
  switch (convergence_criterion) {
    case Default: {
      // alpha is the distance to the best separating hyperplane found so far
      alpha = std::max(alpha, omega);
      // ||x^*|| - ||x^k|| <= diff
      const SolverScalar diff = rl - alpha;
      return ((diff - (tolerance + tolerance * rl)) <= 0);
    } break;

    case DualityGap: {
      // ||x^* - x^k||^2 <= diff
      const SolverScalar diff = 2 * ray.dot(ray - w);
      switch (convergence_criterion_type) {
        case Absolute:
          return ((diff - tolerance) <= 0);
          break;
        case Relative:
          return (((diff / tolerance * rl) - tolerance * rl) <= 0);
          break;
        default:
          COAL_THROW_PRETTY("Invalid convergence criterion type.",
                            std::logic_error);
      }
    } break;

    case Hybrid: {
      // alpha is the distance to the best separating hyperplane found so far
      alpha = std::max(alpha, omega);
      // ||x^* - x^k||^2 <= diff
      const SolverScalar diff = rl * rl - alpha * alpha;
      switch (convergence_criterion_type) {
        case Absolute:
          return ((diff - tolerance) <= 0);
          break;
        case Relative:
          return (((diff / tolerance * rl) - tolerance * rl) <= 0);
          break;
        default:
          COAL_THROW_PRETTY("Invalid convergence criterion type.",
                            std::logic_error);
      }
    } break;

    default:
      COAL_THROW_PRETTY("Invalid convergence criterion.", std::logic_error);
  }
}

inline void GJK::removeVertex(Simplex& simplex) {
  free_v[nfree++] = simplex.vertex[--simplex.rank];
}

inline void GJK::appendVertex(Simplex& simplex, const Vec3ps& v,
                              support_func_guess_t& hint) {
  simplex.vertex[simplex.rank] = free_v[--nfree];  // set the memory
  getSupport(v, *simplex.vertex[simplex.rank++], hint);
}

bool GJK::encloseOrigin() {
  Vec3ps axis(Vec3ps::Zero());
  support_func_guess_t hint = support_func_guess_t::Zero();
  switch (simplex->rank) {
    case 1:
      for (int i = 0; i < 3; ++i) {
        axis[i] = 1;
        appendVertex(*simplex, axis, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
        axis[i] = -1;
        appendVertex(*simplex, -axis, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
        axis[i] = 0;
      }
      break;
    case 2: {
      Vec3ps d = simplex->vertex[1]->w - simplex->vertex[0]->w;
      for (int i = 0; i < 3; ++i) {
        axis[i] = 1;
        Vec3ps p = d.cross(axis);
        if (!p.isZero()) {
          appendVertex(*simplex, p, hint);
          if (encloseOrigin()) return true;
          removeVertex(*simplex);
          appendVertex(*simplex, -p, hint);
          if (encloseOrigin()) return true;
          removeVertex(*simplex);
        }
        axis[i] = 0;
      }
    } break;
    case 3:
      axis.noalias() =
          (simplex->vertex[1]->w - simplex->vertex[0]->w)
              .cross(simplex->vertex[2]->w - simplex->vertex[0]->w);
      if (!axis.isZero()) {
        appendVertex(*simplex, axis, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -axis, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
      }
      break;
    case 4:
      if (std::abs(triple(simplex->vertex[0]->w - simplex->vertex[3]->w,
                          simplex->vertex[1]->w - simplex->vertex[3]->w,
                          simplex->vertex[2]->w - simplex->vertex[3]->w)) > 0)
        return true;
      break;
  }

  return false;
}

inline void originToPoint(const GJK::Simplex& current, GJK::vertex_id_t a,
                          const Vec3ps& A, GJK::Simplex& next, Vec3ps& ray) {
  // A is the closest to the origin
  ray = A;
  next.vertex[0] = current.vertex[a];
  next.rank = 1;
}

inline void originToSegment(const GJK::Simplex& current, GJK::vertex_id_t a,
                            GJK::vertex_id_t b, const Vec3ps& A,
                            const Vec3ps& B, const Vec3ps& AB,
                            const SolverScalar& ABdotAO, GJK::Simplex& next,
                            Vec3ps& ray) {
  // ray = - ( AB ^ AO ) ^ AB = (AB.B) A + (-AB.A) B
  ray = AB.dot(B) * A + ABdotAO * B;

  next.vertex[0] = current.vertex[b];
  next.vertex[1] = current.vertex[a];
  next.rank = 2;

  // To ensure backward compatibility
  ray /= AB.squaredNorm();
}

inline bool originToTriangle(const GJK::Simplex& current, GJK::vertex_id_t a,
                             GJK::vertex_id_t b, GJK::vertex_id_t c,
                             const Vec3ps& ABC, const SolverScalar& ABCdotAO,
                             GJK::Simplex& next, Vec3ps& ray) {
  next.rank = 3;
  next.vertex[2] = current.vertex[a];

  if (ABCdotAO == 0) {
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
    ray.setZero();
    return true;
  }
  if (ABCdotAO > 0) {  // Above triangle
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
  } else {
    next.vertex[0] = current.vertex[b];
    next.vertex[1] = current.vertex[c];
  }

  // To ensure backward compatibility
  ray = -ABCdotAO / ABC.squaredNorm() * ABC;
  return false;
}

bool GJK::projectLineOrigin(const Simplex& current, Simplex& next) {
  const vertex_id_t a = 1, b = 0;
  // A is the last point we added.
  const Vec3ps& A = current.vertex[a]->w;
  const Vec3ps& B = current.vertex[b]->w;

  const Vec3ps AB = B - A;
  const SolverScalar d = AB.dot(-A);
  assert(d <= AB.squaredNorm());

  if (d == 0) {
    // Two extremely unlikely cases:
    // - AB is orthogonal to A: should never happen because it means the support
    //   function did not do any progress and GJK should have stopped.
    // - A == origin
    // In any case, A is the closest to the origin
    originToPoint(current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
    return A.isZero();
  } else if (d < 0) {
    // A is the closest to the origin
    originToPoint(current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
  } else
    originToSegment(current, a, b, A, B, AB, d, next, ray);
  return false;
}

bool GJK::projectTriangleOrigin(const Simplex& current, Simplex& next) {
  const vertex_id_t a = 2, b = 1, c = 0;
  // A is the last point we added.
  const Vec3ps &A = current.vertex[a]->w, B = current.vertex[b]->w,
               C = current.vertex[c]->w;

  const Vec3ps AB = B - A, AC = C - A, ABC = AB.cross(AC);

  SolverScalar edgeAC2o = ABC.cross(AC).dot(-A);
  if (edgeAC2o >= 0) {
    SolverScalar towardsC = AC.dot(-A);
    if (towardsC >= 0) {  // Region 1
      originToSegment(current, a, c, A, C, AC, towardsC, next, ray);
      free_v[nfree++] = current.vertex[b];
    } else {  // Region 4 or 5
      SolverScalar towardsB = AB.dot(-A);
      if (towardsB < 0) {  // Region 5
        // A is the closest to the origin
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else  // Region 4
        originToSegment(current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    }
  } else {
    SolverScalar edgeAB2o = AB.cross(ABC).dot(-A);
    if (edgeAB2o >= 0) {  // Region 4 or 5
      SolverScalar towardsB = AB.dot(-A);
      if (towardsB < 0) {  // Region 5
        // A is the closest to the origin
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else  // Region 4
        originToSegment(current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    } else {
      return originToTriangle(current, a, b, c, ABC, ABC.dot(-A), next, ray);
    }
  }
  return false;
}

bool GJK::projectTetrahedraOrigin(const Simplex& current, Simplex& next) {
  // The code of this function was generated using doc/gjk.py
  const vertex_id_t a = 3, b = 2, c = 1, d = 0;
  const Vec3ps& A(current.vertex[a]->w);
  const Vec3ps& B(current.vertex[b]->w);
  const Vec3ps& C(current.vertex[c]->w);
  const Vec3ps& D(current.vertex[d]->w);
  const SolverScalar aa = A.squaredNorm();
  const SolverScalar da = D.dot(A);
  const SolverScalar db = D.dot(B);
  const SolverScalar dc = D.dot(C);
  const SolverScalar dd = D.dot(D);
  const SolverScalar da_aa = da - aa;
  const SolverScalar ca = C.dot(A);
  const SolverScalar cb = C.dot(B);
  const SolverScalar cc = C.dot(C);
  const SolverScalar& cd = dc;
  const SolverScalar ca_aa = ca - aa;
  const SolverScalar ba = B.dot(A);
  const SolverScalar bb = B.dot(B);
  const SolverScalar& bc = cb;
  const SolverScalar& bd = db;
  const SolverScalar ba_aa = ba - aa;
  const SolverScalar ba_ca = ba - ca;
  const SolverScalar ca_da = ca - da;
  const SolverScalar da_ba = da - ba;
  const Vec3ps a_cross_b = A.cross(B);
  const Vec3ps a_cross_c = A.cross(C);

  const SolverScalar dummy_precision(
      3 * std::sqrt(std::numeric_limits<SolverScalar>::epsilon()));
  COAL_UNUSED_VARIABLE(dummy_precision);

#define REGION_INSIDE()               \
  ray.setZero();                      \
  next.vertex[0] = current.vertex[d]; \
  next.vertex[1] = current.vertex[c]; \
  next.vertex[2] = current.vertex[b]; \
  next.vertex[3] = current.vertex[a]; \
  next.rank = 4;                      \
  return true;

  // clang-format off
  if (ba_aa <= 0) {                // if AB.AO >= 0 / a10
    if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / a10.a3
      if (ba * da_ba + bd * ba_aa - bb * da_aa <=
          0) {             // if (ADB ^ AB).AO >= 0 / a10.a3.a9
        if (da_aa <= 0) {  // if AD.AO >= 0 / a10.a3.a9.a12
          assert(da * da_ba + dd * ba_aa - db * da_aa <=
                 dummy_precision);  // (ADB ^ AD).AO >= 0 / a10.a3.a9.a12.a8
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
              0) {  // if (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.a4
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          } else {  // not (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.!a4
            // Region AB
            originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AB).AO >= 0
        } else {  // not AD.AO >= 0 / a10.a3.a9.!a12
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
              0) {  // if (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.a4
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
                0) {  // if (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5
              if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                  0) {  // if (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.a6
                // Region ACD
                originToTriangle(current, a, c, d, (C - A).cross(D - A),
                                 -D.dot(a_cross_c), next, ray);
                free_v[nfree++] = current.vertex[b];
              } else {  // not (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.!a6
                // Region AC
                originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              }  // end of (ACD ^ AC).AO >= 0
            } else {  // not (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.!a5
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            }  // end of (ABC ^ AC).AO >= 0
          } else {  // not (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.!a4
            // Region AB
            originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AB).AO >= 0
        }  // end of AD.AO >= 0
      } else {  // not (ADB ^ AB).AO >= 0 / a10.a3.!a9
        if (da * da_ba + dd * ba_aa - db * da_aa <=
            0) {  // if (ADB ^ AD).AO >= 0 / a10.a3.!a9.a8
          // Region ADB
          originToTriangle(current, a, d, b, (D - A).cross(B - A),
                           D.dot(a_cross_b), next, ray);
          free_v[nfree++] = current.vertex[c];
        } else {  // not (ADB ^ AD).AO >= 0 / a10.a3.!a9.!a8
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.!a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.!a7
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AD).AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        }  // end of (ADB ^ AD).AO >= 0
      }  // end of (ADB ^ AB).AO >= 0
    } else {                        // not ADB.AO >= 0 / a10.!a3
      if (C.dot(a_cross_b) <= 0) {  // if ABC.AO >= 0 / a10.!a3.a1
        if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
            0) {  // if (ABC ^ AB).AO >= 0 / a10.!a3.a1.a4
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                0) {  // if (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.!a6
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AC).AO >= 0
          } else {  // not (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.!a5
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AC).AO >= 0
        } else {  // not (ABC ^ AB).AO >= 0 / a10.!a3.a1.!a4
          // Region AB
          originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
          free_v[nfree++] = current.vertex[c];
          free_v[nfree++] = current.vertex[d];
        }  // end of (ABC ^ AB).AO >= 0
      } else {                        // not ABC.AO >= 0 / a10.!a3.!a1
        if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / a10.!a3.!a1.a2
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {             // not (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.!a6
            if (ca_aa <= 0) {  // if AC.AO >= 0 / a10.!a3.!a1.a2.!a6.a11
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {  // not AC.AO >= 0 / a10.!a3.!a1.a2.!a6.!a11
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of AC.AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        } else {  // not ACD.AO >= 0 / a10.!a3.!a1.!a2
          // Region Inside
          REGION_INSIDE()
        }  // end of ACD.AO >= 0
      }  // end of ABC.AO >= 0
    }  // end of ADB.AO >= 0
  } else {                          // not AB.AO >= 0 / !a10
    if (ca_aa <= 0) {               // if AC.AO >= 0 / !a10.a11
      if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / !a10.a11.a2
        if (da_aa <= 0) {           // if AD.AO >= 0 / !a10.a11.a2.a12
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7
              if (da * da_ba + dd * ba_aa - db * da_aa <=
                  0) {  // if (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.a8
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {  // not (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.!a8
                // Region AD
                originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }  // end of (ADB ^ AD).AO >= 0
            } else {  // not (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6
            assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                     -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                          // !a10.a11.a2.a12.!a6.!a7
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
                0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.a5
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.!a5
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            }  // end of (ABC ^ AC).AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        } else {  // not AD.AO >= 0 / !a10.a11.a2.!a12
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                0) {  // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.a6
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                       -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                            // !a10.a11.a2.!a12.a5.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.!a6
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AC).AO >= 0
          } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.!a5
            if (C.dot(a_cross_b) <=
                0) {  // if ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.a1
              assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                     dummy_precision);  // (ABC ^ AB).AO >= 0 /
                                        // !a10.a11.a2.!a12.!a5.a1.a4
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            } else {  // not ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.!a1
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                       -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                            // !a10.a11.a2.!a12.!a5.!a1.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of ABC.AO >= 0
          }  // end of (ABC ^ AC).AO >= 0
        }  // end of AD.AO >= 0
      } else {                        // not ACD.AO >= 0 / !a10.a11.!a2
        if (C.dot(a_cross_b) <= 0) {  // if ABC.AO >= 0 / !a10.a11.!a2.a1
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.a5
            // Region AC
            originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
            free_v[nfree++] = current.vertex[b];
            free_v[nfree++] = current.vertex[d];
          } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.!a5
            assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                   dummy_precision);  // (ABC ^ AB).AO >= 0 /
                                      // !a10.a11.!a2.a1.!a5.a4
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AC).AO >= 0
        } else {                         // not ABC.AO >= 0 / !a10.a11.!a2.!a1
          if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / !a10.a11.!a2.!a1.a3
            if (da * da_ba + dd * ba_aa - db * da_aa <=
                0) {  // if (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.a8
              // Region ADB
              originToTriangle(current, a, d, b, (D - A).cross(B - A),
                               D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.!a8
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of (ADB ^ AD).AO >= 0
          } else {  // not ADB.AO >= 0 / !a10.a11.!a2.!a1.!a3
            // Region Inside
            REGION_INSIDE()
          }  // end of ADB.AO >= 0
        }  // end of ABC.AO >= 0
      }  // end of ACD.AO >= 0
    } else {                           // not AC.AO >= 0 / !a10.!a11
      if (da_aa <= 0) {                // if AD.AO >= 0 / !a10.!a11.a12
        if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / !a10.!a11.a12.a3
          if (da * ca_da + dc * da_aa - dd * ca_aa <=
              0) {  // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7
            if (da * da_ba + dd * ba_aa - db * da_aa <=
                0) {  // if (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.a8
              assert(!(ba * da_ba + bd * ba_aa - bb * da_aa <=
                       -dummy_precision));  // Not (ADB ^ AB).AO >= 0 /
                                            // !a10.!a11.a12.a3.a7.a8.!a9
              // Region ADB
              originToTriangle(current, a, d, b, (D - A).cross(B - A),
                               D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.!a8
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of (ADB ^ AD).AO >= 0
          } else {  // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.!a7
            if (D.dot(a_cross_c) <=
                0) {  // if ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.a2
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <=
                     dummy_precision);  // (ACD ^ AC).AO >= 0 /
                                        // !a10.!a11.a12.a3.!a7.a2.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2
              if (C.dot(a_cross_b) <=
                  0) {  // if ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.a1
                assert(!(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                         -dummy_precision));  // Not (ABC ^ AB).AO >= 0 /
                                              // !a10.!a11.a12.a3.!a7.!a2.a1.!a4
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {  // not ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.!a1
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              }  // end of ABC.AO >= 0
            }  // end of ACD.AO >= 0
          }  // end of (ACD ^ AD).AO >= 0
        } else {                        // not ADB.AO >= 0 / !a10.!a11.a12.!a3
          if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / !a10.!a11.a12.!a3.a2
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.!a7
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <=
                     dummy_precision);  // (ACD ^ AC).AO >= 0 /
                                        // !a10.!a11.a12.!a3.a2.!a7.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not ACD.AO >= 0 / !a10.!a11.a12.!a3.!a2
            // Region Inside
            REGION_INSIDE()
          }  // end of ACD.AO >= 0
        }  // end of ADB.AO >= 0
      } else {  // not AD.AO >= 0 / !a10.!a11.!a12
        // Region A
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
        free_v[nfree++] = current.vertex[c];
        free_v[nfree++] = current.vertex[d];
      }  // end of AD.AO >= 0
    }  // end of AC.AO >= 0
  }  // end of AB.AO >= 0
  // clang-format on

#undef REGION_INSIDE
  return false;
}

void EPA::initialize() { reset(max_iterations, tolerance); }

void EPA::reset(size_t max_iterations_, SolverScalar tolerance_) {
  max_iterations = max_iterations_;
  tolerance = tolerance_;
  // EPA creates only 2 faces and 1 vertex per iteration.
  // (+ the 4 (or 8 in the future) faces at the beginning
  //  + the 4 vertices (or 6 in the future) at the beginning)
  sv_store.resize(max_iterations + 4);
  fc_store.resize(2 * max_iterations + 4);
  status = DidNotRun;
  normal.setZero();
  support_hint.setZero();
  depth = 0;
  closest_face = nullptr;
  result.reset();
  hull.reset();
  num_vertices = 0;
  stock.reset();
  // The stock is initialized with the faces in reverse order so that the
  // hull and the stock do not overlap (the stock will shring as the hull will
  // grow).
  for (size_t i = 0; i < fc_store.size(); ++i)
    stock.append(&fc_store[fc_store.size() - i - 1]);
  iterations = 0;
}

bool EPA::getEdgeDist(SimplexFace* face, const SimplexVertex& a,
                      const SimplexVertex& b, SolverScalar& dist) {
  Vec3ps ab = b.w - a.w;
  Vec3ps n_ab = ab.cross(face->n);
  SolverScalar a_dot_nab = a.w.dot(n_ab);

  if (a_dot_nab < 0)  // the origin is on the outside part of ab
  {
    // following is similar to projectOrigin for two points
    // however, as we dont need to compute the parameterization, dont need to
    // compute 0 or 1
    SolverScalar a_dot_ab = a.w.dot(ab);
    SolverScalar b_dot_ab = b.w.dot(ab);

    if (a_dot_ab > 0)
      dist = a.w.norm();
    else if (b_dot_ab < 0)
      dist = b.w.norm();
    else {
      dist = std::sqrt(
          std::max(a.w.squaredNorm() - a_dot_ab * a_dot_ab / ab.squaredNorm(),
                   SolverScalar(0)));
    }

    return true;
  }

  return false;
}

EPA::SimplexFace* EPA::newFace(size_t id_a, size_t id_b, size_t id_c,
                               bool force) {
  if (stock.root != nullptr) {
    SimplexFace* face = stock.root;
    stock.remove(face);
    hull.append(face);
    face->pass = 0;
    face->vertex_id[0] = id_a;
    face->vertex_id[1] = id_b;
    face->vertex_id[2] = id_c;
    const SimplexVertex& a = sv_store[id_a];
    const SimplexVertex& b = sv_store[id_b];
    const SimplexVertex& c = sv_store[id_c];
    face->n = (b.w - a.w).cross(c.w - a.w);

    if (face->n.norm() > Eigen::NumTraits<SolverScalar>::epsilon()) {
      face->n.normalize();

      // If the origin projects outside the face, skip it in the
      // `findClosestFace` method.
      // The origin always projects inside the closest face.
      SolverScalar a_dot_nab = a.w.dot((b.w - a.w).cross(face->n));
      SolverScalar b_dot_nbc = b.w.dot((c.w - b.w).cross(face->n));
      SolverScalar c_dot_nca = c.w.dot((a.w - c.w).cross(face->n));
      if (a_dot_nab >= -tolerance &&  //
          b_dot_nbc >= -tolerance &&  //
          c_dot_nca >= -tolerance) {
        face->d = a.w.dot(face->n);
        face->ignore = false;
      } else {
        // We will never check this face, so we don't care about
        // its true distance to the origin.
        face->d = std::numeric_limits<SolverScalar>::max();
        face->ignore = true;
      }

      // For the initialization of EPA, we need to force the addition of the
      // face. This is because the origin can lie outside the initial
      // tetrahedron. This is expected. GJK can converge in two different ways:
      // either by enclosing the origin with a simplex, or by converging
      // sufficiently close to the origin.
      // The thing is, "sufficiently close to the origin" depends on the
      // tolerance of GJK. So in this case, GJK **cannot** guarantee that the
      // shapes are indeed in collision. If it turns out they are not in
      // collision, the origin will lie outside of the Minkowski difference and
      // thus outside the initial tetrahedron. But EPA is ultimately a
      // projection algorithm, so it will work fine!
      //
      // Actually, the `NonConvex` status is badly named. There should not be
      // such a status! Again, if the origin lies outside the Minkowski
      // difference, EPA will work fine, and will find the right (signed)
      // distance between the shapes as well as the right normal. This is a
      // very nice mathematical property, making GJK + EPA a **very** robust set
      // of algorithms. :)
      // TODO(louis): remove the `NonConvex` status.
      if (face->d >= -tolerance || force)
        return face;
      else
        status = NonConvex;
    } else
      status = Degenerated;

    hull.remove(face);
    stock.append(face);
    return nullptr;
  }

  assert(hull.count >= fc_store.size() && "EPA: should not be out of faces.");
  status = OutOfFaces;
  return nullptr;
}

/** @brief Find the best polytope face to split */
EPA::SimplexFace* EPA::findClosestFace() {
  SimplexFace* minf = hull.root;
  SolverScalar mind = std::numeric_limits<SolverScalar>::max();
  for (SimplexFace* f = minf; f; f = f->next_face) {
    if (f->ignore) continue;
    SolverScalar sqd = f->d * f->d;
    if (sqd < mind) {
      minf = f;
      mind = sqd;
    }
  }
  assert(minf && !(minf->ignore) && "EPA: minf should not be flagged ignored.");
  return minf;
}

EPA::Status EPA::evaluate(GJK& gjk, const Vec3ps& guess) {
  COAL_TRACY_ZONE_SCOPED_N("coal::details::EPA::evaluate");
  GJK::Simplex& simplex = *gjk.getSimplex();
  support_hint = gjk.support_hint;

  // TODO(louis): we might want to start with a hexahedron if the
  // simplex given by GJK is of rank <= 3.
  bool enclosed_origin = gjk.encloseOrigin();
  if ((simplex.rank > 1) && enclosed_origin) {
    assert(simplex.rank == 4 &&
           "When starting EPA, simplex should be of rank 4.");
    while (hull.root) {
      SimplexFace* f = hull.root;
      hull.remove(f);
      stock.append(f);
    }
    assert(hull.count == 0);
    assert(stock.count == fc_store.size());

    status = Valid;
    num_vertices = 0;

    // Make sure the tetrahedron has its normals pointing outside.
    if ((simplex.vertex[0]->w - simplex.vertex[3]->w)
            .dot((simplex.vertex[1]->w - simplex.vertex[3]->w)
                     .cross(simplex.vertex[2]->w - simplex.vertex[3]->w)) < 0) {
      SimplexVertex* tmp = simplex.vertex[0];
      simplex.vertex[0] = simplex.vertex[1];
      simplex.vertex[1] = tmp;
    }

    // Add the 4 vertices to sv_store
    for (size_t i = 0; i < 4; ++i) {
      sv_store[num_vertices++] = *simplex.vertex[i];
    }

    SimplexFace* tetrahedron[] = {newFace(0, 1, 2, true),  //
                                  newFace(1, 0, 3, true),  //
                                  newFace(2, 1, 3, true),  //
                                  newFace(0, 2, 3, true)};

    if (hull.count == 4) {
      // set the face connectivity
      bind(tetrahedron[0], 0, tetrahedron[1], 0);
      bind(tetrahedron[0], 1, tetrahedron[2], 0);
      bind(tetrahedron[0], 2, tetrahedron[3], 0);
      bind(tetrahedron[1], 1, tetrahedron[3], 2);
      bind(tetrahedron[1], 2, tetrahedron[2], 1);
      bind(tetrahedron[2], 2, tetrahedron[3], 1);

      closest_face =
          findClosestFace();  // find the best face (the face with the
                              // minimum distance to origin) to split
      SimplexFace outer = *closest_face;

      status = Valid;
      iterations = 0;
      size_t pass = 0;
      for (; iterations < max_iterations; ++iterations) {
        if (num_vertices >= sv_store.size()) {
          status = OutOfVertices;
          break;
        }

        // Step 1: find the support point in the direction of the closest_face
        // normal.
        // --------------------------------------------------------------------------
        SimplexHorizon horizon;
        SimplexVertex& w = sv_store[num_vertices++];
        bool valid = true;
        closest_face->pass = ++pass;
        // At the moment, SimplexF.n is always normalized. This could be revised
        // in the future...
        gjk.getSupport(closest_face->n, w, support_hint);

        // Step 2: check for convergence.
        // ------------------------------
        // Preambule to understand the convergence criterion of EPA:
        // the support we just added is in the direction of the normal of
        // the closest_face. Therefore, the support point will **always**
        // lie "after" the closest_face, i.e closest_face.n.dot(w.w) > 0.
        if (iterations > 0) {
          assert(closest_face->n.dot(w.w) > -tolerance &&
                 "The support is not in the right direction.");
        }
        //
        // 1) First check: `fdist` (see below) is an upper bound of how much
        // more penetration depth we can expect to "gain" by adding `w` to EPA's
        // polytope. This first check, as any convergence check, should be both
        // absolute and relative. This allows to adapt the tolerance to the
        // scale of the objects.
        const SimplexVertex& vf1 = sv_store[closest_face->vertex_id[0]];
        const SimplexVertex& vf2 = sv_store[closest_face->vertex_id[1]];
        const SimplexVertex& vf3 = sv_store[closest_face->vertex_id[2]];
        SolverScalar fdist = closest_face->n.dot(w.w - vf1.w);
        SolverScalar wnorm = w.w.norm();
        // TODO(louis): we might want to use tol_abs and tol_rel; this might
        // obfuscate the code for the user though.
        if (fdist <= tolerance + tolerance * wnorm) {
          status = AccuracyReached;
          break;
        }
        // 2) Second check: the expand function **assumes** that the support we
        // just computed is not a vertex of the face. We make sure that this
        // is the case:
        // TODO(louis): should we use squaredNorm everywhere instead of norm?
        if ((w.w - vf1.w).norm() <= tolerance + tolerance * wnorm ||
            (w.w - vf2.w).norm() <= tolerance + tolerance * wnorm ||
            (w.w - vf3.w).norm() <= tolerance + tolerance * wnorm) {
          status = AccuracyReached;
          break;
        }

        // Step 3: expand the polytope
        // ---------------------------
        for (size_t j = 0; (j < 3) && valid; ++j)
          valid &= expand(pass, w, closest_face->adjacent_faces[j],
                          closest_face->adjacent_edge[j], horizon);

        if (!valid || horizon.num_faces < 3) {
          // The status has already been set by the expand function.
          assert(!(status & Valid));
          break;
        }
        // need to add the edge connectivity between first and last faces
        bind(horizon.first_face, 2, horizon.current_face, 1);
        hull.remove(closest_face);
        stock.append(closest_face);
        closest_face = findClosestFace();
        outer = *closest_face;
      }

      status = ((iterations) < max_iterations) ? status : Failed;
      normal = outer.n;
      depth = outer.d + gjk.shape->swept_sphere_radius.sum();
      result.rank = 3;
      result.vertex[0] = &sv_store[outer.vertex_id[0]];
      result.vertex[1] = &sv_store[outer.vertex_id[1]];
      result.vertex[2] = &sv_store[outer.vertex_id[2]];
      return status;
    }
    assert(false && "The tetrahedron with which EPA started is degenerated.");
  }

  // FallBack when the simplex given by GJK is of rank 1.
  // Since the simplex only contains support points which convex
  // combination describe the origin, the point in the simplex is actually
  // the origin.
  status = FallBack;
  // TODO: define a better normal
  assert(simplex.rank == 1 && simplex.vertex[0]->w.isZero(gjk.getTolerance()));
  normal = -guess;
  SolverScalar nl = normal.norm();
  if (nl > 0)
    normal /= nl;
  else
    normal = Vec3ps(1, 0, 0);
  depth = 0;
  result.rank = 1;
  result.vertex[0] = simplex.vertex[0];
  return status;
}

// Use this function to debug `EPA::expand` if needed.
// void EPA::PrintExpandLooping(const SimplexFace* f, const SimplexVertex& w) {
//     std::cout << "Vertices:\n";
//     for (size_t i = 0; i < num_vertices; ++i) {
//       std::cout << "[";
//       std::cout << sv_store[i].w(0) << ", ";
//       std::cout << sv_store[i].w(1) << ", ";
//       std::cout << sv_store[i].w(2) << "]\n";
//     }
//     //
//     std::cout << "\nTriangles:\n";
//     SimplexFace* face = hull.root;
//     for (size_t i = 0; i < hull.count; ++i) {
//       std::cout << "[";
//       std::cout << face->vertex_id[0] << ", ";
//       std::cout << face->vertex_id[1] << ", ";
//       std::cout << face->vertex_id[2] << "]\n";
//       face = face->next_face;
//     }
//     //
//     std::cout << "\nNormals:\n";
//     face = hull.root;
//     for (size_t i = 0; i < hull.count; ++i) {
//       std::cout << "[";
//       std::cout << face->n(0) << ", ";
//       std::cout << face->n(1) << ", ";
//       std::cout << face->n(2) << "]\n";
//       face = face->next_face;
//     }
//     //
//     std::cout << "\nClosest face:\n";
//     face = hull.root;
//     for (size_t i = 0; i < hull.count; ++i) {
//       if (face == closest_face) {
//         std::cout << i << "\n";
//       }
//       face = face->next_face;
//     }
//     std::cout << "\nSupport point:\n";
//     std::cout << "[" << w.w(0) << ", " << w.w(1) << ", " << w.w(2) << "]\n";
// }

/** @brief the goal is to add a face connecting vertex w and face edge f[e] */
bool EPA::expand(size_t pass, const SimplexVertex& w, SimplexFace* f, size_t e,
                 SimplexHorizon& horizon) {
  static const size_t nexti[] = {1, 2, 0};
  static const size_t previ[] = {2, 0, 1};
  const size_t id_w =
      num_vertices - 1;  // w is always the last vertex added to sv_store

  // Check if we loop through expand indefinitely.
  if (f->pass == pass) {
    // Uncomment the following line and the associated EPA method
    // to debug the infinite loop if needed.
    // EPAPrintExpandLooping(this, f);
    assert(f != closest_face && "EPA is looping indefinitely.");
    status = InvalidHull;
    return false;
  }

  const size_t e1 = nexti[e];

  // Preambule: when expanding the polytope, the `closest_face` is always
  // deleted. This is handled in EPA::evaluate after calling the expand
  // function. This function handles how the neighboring face `f` of the
  // `closest_face` is connected to the new support point. (Note: because
  // `expand` is recursive, `f` can also denote a face of a face of the
  // `closest_face`, and so on. But the reasoning is the same.)
  //
  // EPA can handle `f` in two ways, depending on where the new support point
  // is located:
  // 1) If it is "below" `f`, then `f` is preserved. A new face is created
  //    and connects to the edge `e` of `f`. This new face is made of the
  //    two points of the edge `e` of `f` and the new support point `w`.
  //    Geometrically, this corresponds to the case where the projection of
  //    the origin on the `closest_face` is **inside** the triangle defined by
  //    the `closest_face`.
  // 2) If it is "above" `f`, then `f` has to be deleted, simply because the
  //    edge `e` of `f` is not part of the convex hull anymore.
  //    The two faces adjacent to `f` are thus expanded following
  //    either 1) or 2).
  //    Geometrically, this corresponds to the case where the projection of
  //    the origin on the `closest_face` is on an edge of the triangle defined
  //    by the `closest_face`. The projection of the origin cannot lie on a
  //    vertex of the `closest_face` because EPA should have exited before
  //    reaching this point.
  //
  // The following checks for these two cases.
  // This check is however subject to numerical precision and due to the
  // recursive nature of `expand`, it is safer to go through the first case.
  // This is because `expand` can potentially loop indefinitly if the
  // Minkowski difference is very flat (hence the check above).
  const SolverScalar dummy_precision(
      3 * std::sqrt(std::numeric_limits<SolverScalar>::epsilon()));
  const SimplexVertex& vf = sv_store[f->vertex_id[e]];
  if (f->n.dot(w.w - vf.w) < dummy_precision) {
    // case 1: the support point is "below" `f`.
    SimplexFace* new_face = newFace(f->vertex_id[e1], f->vertex_id[e], id_w);
    if (new_face != nullptr) {
      // add face-face connectivity
      bind(new_face, 0, f, e);

      // if there is last face in the horizon, then need to add another
      // connectivity, i.e. the edge connecting the current new add edge and the
      // last new add edge. This does not finish all the connectivities because
      // the final face need to connect with the first face, this will be
      // handled in the evaluate function. Notice the face is anti-clockwise, so
      // the edges are 0 (bottom), 1 (right), 2 (left)
      if (horizon.current_face != nullptr) {
        bind(new_face, 2, horizon.current_face, 1);
      } else {
        horizon.first_face = new_face;
      }

      horizon.current_face = new_face;
      ++horizon.num_faces;
      return true;
    }
    return false;
  }

  // case 2: the support point is "above" `f`.
  const size_t e2 = previ[e];
  f->pass = pass;
  if (expand(pass, w, f->adjacent_faces[e1], f->adjacent_edge[e1], horizon) &&
      expand(pass, w, f->adjacent_faces[e2], f->adjacent_edge[e2], horizon)) {
    hull.remove(f);
    stock.append(f);
    return true;
  }
  return false;
}

void EPA::getWitnessPointsAndNormal(const MinkowskiDiff& shape, Vec3ps& w0,
                                    Vec3ps& w1, Vec3ps& normal) const {
  details::getClosestPoints(result, w0, w1);
  if ((w0 - w1).norm() > Eigen::NumTraits<SolverScalar>::dummy_precision()) {
    if (this->depth >= 0) {
      // The shapes are in collision.
      normal = (w0 - w1).normalized();
    } else {
      // The shapes are not in collision.
      normal = (w1 - w0).normalized();
    }
  } else {
    normal = this->normal;
  }
  details::inflate<false>(shape, normal, w0, w1);
}

}  // namespace details

template <typename IndexType>
void ConvexBaseTpl<IndexType>::buildSupportWarmStart() {
  typedef ConvexBaseTpl<IndexType> ConvexBaseType;
  if (this->points->size() <
      ConvexBaseType::num_vertices_large_convex_threshold) {
    return;
  }

  this->support_warm_starts.points.reserve(
      ConvexBaseType::num_support_warm_starts);
  this->support_warm_starts.indices.reserve(
      ConvexBaseType::num_support_warm_starts);

  Vec3s axiis(0, 0, 0);
  details::ShapeSupportData support_data;
  int support_hint = 0;
  for (int i = 0; i < 3; ++i) {
    axiis(i) = 1;
    {
      Vec3s support;
      coal::details::getShapeSupport<false, IndexType>(
          this, axiis, support, support_hint, support_data);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_hint);
    }

    axiis(i) = -1;
    {
      Vec3s support;
      coal::details::getShapeSupport<false, IndexType>(
          this, axiis, support, support_hint, support_data);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_hint);
    }

    axiis(i) = 0;
  }

  std::array<Vec3s, 4> eis = {Vec3s(1, 1, 1),    //
                              Vec3s(-1, 1, 1),   //
                              Vec3s(-1, -1, 1),  //
                              Vec3s(1, -1, 1)};

  for (size_t ei_index = 0; ei_index < 4; ++ei_index) {
    {
      Vec3s support;
      coal::details::getShapeSupport<false, IndexType>(
          this, eis[ei_index], support, support_hint, support_data);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_hint);
    }

    {
      Vec3s support;
      coal::details::getShapeSupport<false, IndexType>(
          this, -eis[ei_index], support, support_hint, support_data);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_hint);
    }
  }

  if (this->support_warm_starts.points.size() !=
          ConvexBaseType::num_support_warm_starts ||
      this->support_warm_starts.indices.size() !=
          ConvexBaseType::num_support_warm_starts) {
    COAL_THROW_PRETTY("Wrong number of support warm starts.",
                      std::runtime_error);
  }
}

template void COAL_DLLAPI
ConvexBaseTpl<Triangle16::IndexType>::buildSupportWarmStart();
template void COAL_DLLAPI
ConvexBaseTpl<Triangle32::IndexType>::buildSupportWarmStart();

}  // namespace coal
