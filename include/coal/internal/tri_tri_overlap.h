/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Asensus Surgical
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
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 */

#ifndef COAL_INTERNAL_TRI_TRI_OVERLAP_H
#define COAL_INTERNAL_TRI_TRI_OVERLAP_H

/// @cond INTERNAL

#include "coal/data_types.h"

#include <cmath>
#include <algorithm>

namespace coal {
namespace details {

/// @brief Möller 1997 tri-tri overlap (boolean only).
///
/// Reference: Tomas Möller, "A Fast Triangle-Triangle Intersection Test",
/// Journal of Graphics Tools 2(2):25-30, 1997.
///
/// Returns true if triangles V0V1V2 and U0U1U2 share at least one point.
/// Sets `coplanar = true` if both triangles lie in (approximately) the same
/// plane: in that case the result is conservative-false (no overlap reported)
/// and the caller should fall back to a full method (e.g. GJK) since the
/// 2D-in-plane test is not implemented here. Coplanar pairs are extremely
/// rare in practical motion-validation workloads.
template <typename Vec3>
inline bool triTriOverlap(const Vec3& V0, const Vec3& V1, const Vec3& V2,
                          const Vec3& U0, const Vec3& U1, const Vec3& U2,
                          bool& coplanar) {
  using Scalar = typename Vec3::Scalar;
  coplanar = false;

  // Compute T1's supporting plane.
  const Vec3 E1 = V1 - V0;
  const Vec3 E2 = V2 - V0;
  const Vec3 N1 = E1.cross(E2);
  const Scalar d1 = -N1.dot(V0);

  // Signed distances of T2's vertices to T1's plane.
  Scalar du0 = N1.dot(U0) + d1;
  Scalar du1 = N1.dot(U1) + d1;
  Scalar du2 = N1.dot(U2) + d1;

  // Snap near-zero values to zero so the sign tests below behave robustly.
  // The tolerance scales with the largest |signed-distance| because the
  // numbers we compare against zero are products of (mesh extent) * (normal
  // magnitude), both of which can be large.
  const Scalar eps_u =
      std::numeric_limits<Scalar>::epsilon() *
      (std::max)({Scalar(1), std::abs(du0), std::abs(du1), std::abs(du2)});
  if (std::abs(du0) < eps_u) du0 = 0;
  if (std::abs(du1) < eps_u) du1 = 0;
  if (std::abs(du2) < eps_u) du2 = 0;

  const Scalar du0du1 = du0 * du1;
  const Scalar du0du2 = du0 * du2;

  // All three vertices on the same side of T1's plane → no intersection.
  if (du0du1 > 0 && du0du2 > 0) return false;

  // Compute T2's supporting plane.
  const Vec3 F1 = U1 - U0;
  const Vec3 F2 = U2 - U0;
  const Vec3 N2 = F1.cross(F2);
  const Scalar d2 = -N2.dot(U0);

  // Signed distances of T1's vertices to T2's plane.
  Scalar dv0 = N2.dot(V0) + d2;
  Scalar dv1 = N2.dot(V1) + d2;
  Scalar dv2 = N2.dot(V2) + d2;

  const Scalar eps_v =
      std::numeric_limits<Scalar>::epsilon() *
      (std::max)({Scalar(1), std::abs(dv0), std::abs(dv1), std::abs(dv2)});
  if (std::abs(dv0) < eps_v) dv0 = 0;
  if (std::abs(dv1) < eps_v) dv1 = 0;
  if (std::abs(dv2) < eps_v) dv2 = 0;

  const Scalar dv0dv1 = dv0 * dv1;
  const Scalar dv0dv2 = dv0 * dv2;

  if (dv0dv1 > 0 && dv0dv2 > 0) return false;

  // Direction of the line of intersection of the two planes.
  const Vec3 D = N1.cross(N2);

  // Pick the largest |D[i]| as the projection axis to avoid divisions by
  // small numbers when computing the interval endpoints. If D is (almost)
  // zero, the planes are (approximately) parallel — meaning either the
  // triangles are coplanar, or one's plane is parallel to and disjoint from
  // the other's. The earlier same-sign rejections already handle the
  // disjoint case, so a remaining tiny |D| means coplanar.
  Scalar abs_d0 = std::abs(D[0]);
  Scalar abs_d1 = std::abs(D[1]);
  Scalar abs_d2 = std::abs(D[2]);
  Scalar maxAxis = abs_d0;
  int axis = 0;
  if (abs_d1 > maxAxis) {
    maxAxis = abs_d1;
    axis = 1;
  }
  if (abs_d2 > maxAxis) {
    maxAxis = abs_d2;
    axis = 2;
  }

  if (maxAxis <= std::numeric_limits<Scalar>::epsilon() *
                     (std::max)({Scalar(1), N1.cwiseAbs().maxCoeff(),
                                 N2.cwiseAbs().maxCoeff()})) {
    coplanar = true;
    return false;  // caller falls back
  }

  // Project T1 and T2 vertices onto the dominant axis.
  const Scalar vp0 = V0[axis], vp1 = V1[axis], vp2 = V2[axis];
  const Scalar up0 = U0[axis], up1 = U1[axis], up2 = U2[axis];

  // Compute the two intervals on the dominant axis. For each triangle, the
  // line of intersection enters and exits along two edges (the lone-vertex
  // edges); the interval endpoints are the parameterized crossings.
  //
  // We could do this more directly with a small helper, but compilers
  // produce tighter scalar code from a flat sequence of branches than from
  // a function call here.
  Scalar isect1[2], isect2[2];

  auto compute_interval =
      [](const Scalar& wp0, const Scalar& wp1, const Scalar& wp2,
         const Scalar& d0, const Scalar& d1, const Scalar& d2,
         const Scalar& d0d1, const Scalar& d0d2, Scalar out[2]) -> bool {
    // Identify the lone vertex (whose signed-distance has a sign different
    // from the other two) and parameterize the two edges incident to it.
    if (d0d1 > 0) {
      // d0, d1 have the same sign; d2 is the lone one.
      out[0] = wp2 + (wp0 - wp2) * d2 / (d2 - d0);
      out[1] = wp2 + (wp1 - wp2) * d2 / (d2 - d1);
      return true;
    }
    if (d0d2 > 0) {
      // d0, d2 same sign; d1 is the lone one.
      out[0] = wp1 + (wp0 - wp1) * d1 / (d1 - d0);
      out[1] = wp1 + (wp2 - wp1) * d1 / (d1 - d2);
      return true;
    }
    if (d1 * d2 > 0 || d0 != 0) {
      // d1, d2 same sign; d0 is the lone one.
      out[0] = wp0 + (wp1 - wp0) * d0 / (d0 - d1);
      out[1] = wp0 + (wp2 - wp0) * d0 / (d0 - d2);
      return true;
    }
    if (d1 != 0) {
      out[0] = wp1 + (wp0 - wp1) * d1 / (d1 - d0);
      out[1] = wp1 + (wp2 - wp1) * d1 / (d1 - d2);
      return true;
    }
    if (d2 != 0) {
      out[0] = wp2 + (wp0 - wp2) * d2 / (d2 - d0);
      out[1] = wp2 + (wp1 - wp2) * d2 / (d2 - d1);
      return true;
    }
    return false;  // all signed distances zero → coplanar, caller handles
  };

  if (!compute_interval(vp0, vp1, vp2, dv0, dv1, dv2, dv0dv1, dv0dv2, isect1) ||
      !compute_interval(up0, up1, up2, du0, du1, du2, du0du1, du0du2, isect2)) {
    coplanar = true;
    return false;
  }

  if (isect1[0] > isect1[1]) std::swap(isect1[0], isect1[1]);
  if (isect2[0] > isect2[1]) std::swap(isect2[0], isect2[1]);

  // Intervals overlap iff the larger of the two minima is ≤ the smaller of
  // the two maxima.
  if (isect1[1] < isect2[0] || isect2[1] < isect1[0]) return false;
  return true;
}

}  // namespace details
}  // namespace coal

/// @endcond

#endif  // COAL_INTERNAL_TRI_TRI_OVERLAP_H
