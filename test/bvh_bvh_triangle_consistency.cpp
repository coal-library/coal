/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Louis Montaut */

#define BOOST_TEST_MODULE COAL_BVH_BVH_TRIANGLE_CONSISTENCY
#include <boost/test/included/unit_test.hpp>

#include <cstdlib>
#include <cmath>

#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/intersect.h"

using namespace coal;

// Verifies consistency between triangleTriangleOverlap, sqrTriDistance, and
// GJK for both separated and intersecting triangle pairs.
BOOST_AUTO_TEST_CASE(triangle_triangle_consistency) {
  std::srand(42);

#ifndef NDEBUG
  const std::size_t n = 10;
#else
  const std::size_t n = 1000;
#endif

  GJKSolver solver;
  const Transform3s tf_id = Transform3s::Identity();
  const bool compute_penetration = false;
  // 1e-4% relative tolerance ≈ 1e-6 absolute, matching GJK_DEFAULT_TOLERANCE
  const Scalar tol_pct = Scalar(1e-4);

  for (std::size_t i = 0; i < n; ++i) {
    // tri1 in [-1,1]³, tri2 in [2,4]³ — AABBs are guaranteed non-overlapping.
    const Vec3s P1 = Vec3s::Random();
    const Vec3s P2 = Vec3s::Random();
    const Vec3s P3 = Vec3s::Random();
    const Vec3s Q1 = Vec3s::Random() + Vec3s::Constant(3);
    const Vec3s Q2 = Vec3s::Random() + Vec3s::Constant(3);
    const Vec3s Q3 = Vec3s::Random() + Vec3s::Constant(3);

    // ---- Non-colliding case ----

    // sqrTriDistance: exact squared distance between the two separated
    // triangles
    Vec3s p, q;
    const Scalar sqrTriDist =
        TriangleDistance::sqrTriDistance(P1, P2, P3, Q1, Q2, Q3, p, q);
    BOOST_CHECK(sqrTriDist > 0);

    // triangleTriangleOverlap: SAT boolean + lower bound on squared distance
    Scalar sqrDistLB = 0;
    const bool overlap =
        internal::triangleTriangleOverlap(P1, P2, P3, Q1, Q2, Q3, sqrDistLB);
    BOOST_CHECK(!overlap);
    // sqrDistLB is a valid (possibly loose) lower bound on the exact value.
    // Small relative tolerance absorbs floating point rounding when the SAT
    // axis happens to find the tight distance.
    BOOST_CHECK(sqrDistLB <= sqrTriDist * (Scalar(1) + Scalar(1e-10)));

    // GJK distance must agree with sqrTriDistance
    TriangleP tri1(P1, P2, P3);
    TriangleP tri2(Q1, Q2, Q3);
    Vec3s p1_gjk, p2_gjk, normal;
    const Scalar gjk_dist = solver.shapeDistance<TriangleP, TriangleP>(
        tri1, tf_id, tri2, tf_id, compute_penetration, p1_gjk, p2_gjk, normal);
    BOOST_CHECK(gjk_dist > 0);
    BOOST_CHECK_CLOSE(gjk_dist, std::sqrt(sqrTriDist), tol_pct);

    // ---- Colliding case: centroid-pierce construction ----

    // Build tri2' whose edge R1-R2 passes through the centroid of tri1,
    // guaranteeing intersection for any non-degenerate tri1.
    const Vec3s raw_n = (P2 - P1).cross(P3 - P1);
    if (raw_n.squaredNorm() < Scalar(1e-10)) continue;  // skip degenerate tri1
    const Vec3s n_hat = raw_n.normalized();
    const Vec3s C = (P1 + P2 + P3) / Scalar(3);
    const Vec3s R1 = C + n_hat;
    const Vec3s R2 = C - n_hat;
    const Vec3s R3 = P1 + Scalar(5) * (P2 - P1);

    // GJK should report collision and return zero distance.
    Vec3s p1_gjk_col, p2_gjk_col, normal_col;
    const Scalar gjk_dist_col = solver.shapeDistance<TriangleP, TriangleP>(
        TriangleP(P1, P2, P3), tf_id, TriangleP(R1, R2, R3), tf_id,
        compute_penetration, p1_gjk_col, p2_gjk_col, normal_col);
    BOOST_CHECK(gjk_dist_col <= 0);

    Vec3s pc, qc;
    const Scalar sqrTriDist_col =
        TriangleDistance::sqrTriDistance(P1, P2, P3, R1, R2, R3, pc, qc);
    BOOST_CHECK_EQUAL(sqrTriDist_col, Scalar(0));

    Scalar sqrDistLB_col = 0;
    const bool overlap_col = internal::triangleTriangleOverlap(
        P1, P2, P3, R1, R2, R3, sqrDistLB_col);
    BOOST_CHECK(overlap_col);
  }
}
