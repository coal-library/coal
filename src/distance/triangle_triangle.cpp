/*
 * Software License Agreement (BSD License)
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

/** \author Louis Montaut */

#include "coal/shape/geometric_shapes.h"

#include "coal/internal/shape_shape_func.h"
#include "../narrowphase/details.h"

#include "coal/tracy.hh"

namespace coal {

namespace internal {
template <>
Scalar ShapeShapeDistance<TriangleP, TriangleP>(
    const CollisionGeometry* o1, const Transform3s& tf1,
    const CollisionGeometry* o2, const Transform3s& tf2,
    const GJKSolver* solver, const bool, Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::internal::ShapeShapeDistance<TriangleP, TriangleP>");
  // Transform the triangles in world frame
  const TriangleP& s1 = static_cast<const TriangleP&>(*o1);
  const TriangleP t1(tf1.transform(s1.a), tf1.transform(s1.b),
                     tf1.transform(s1.c));

  const TriangleP& s2 = static_cast<const TriangleP&>(*o2);
  const TriangleP t2(tf2.transform(s2.a), tf2.transform(s2.b),
                     tf2.transform(s2.c));

  // Reset GJK algorithm
  //   We don't need to take into account swept-sphere radius in GJK iterations;
  //   the result will be corrected after GJK terminates.
  solver->minkowski_difference
      .set<::coal::details::SupportOptions::NoSweptSphere>(&t1, &t2);
  solver->gjk.reset(solver->gjk_max_iterations, solver->gjk_tolerance);

  // Get GJK initial guess
  Vec3s guess;
  if (solver->gjk_initial_guess == GJKInitialGuess::CachedGuess ||
      solver->enable_cached_guess) {
    guess = solver->cached_guess;
  } else {
    guess = (t1.a + t1.b + t1.c - t2.a - t2.b - t2.c) / 3;
  }
  support_func_guess_t support_hint;
  solver->epa.status =
      details::EPA::DidNotRun;  // EPA is never called in this function

  Vec3ps guess_ = guess.cast<SolverScalar>();
  details::GJK::Status gjk_status =
      solver->gjk.evaluate(solver->minkowski_difference, guess_, support_hint);

  solver->cached_guess = solver->gjk.getGuessFromSimplex().cast<Scalar>();
  solver->support_func_cached_guess = solver->gjk.support_hint;

  // Retrieve witness points and normal
  Vec3ps p1_, p2_, normal_;
  solver->gjk.getWitnessPointsAndNormal(solver->minkowski_difference, p1_, p2_,
                                        normal_);
  p1 = p1_.cast<Scalar>();
  p2 = p2_.cast<Scalar>();
  normal = normal_.cast<Scalar>();
  Scalar distance = Scalar(solver->gjk.distance);

  if (gjk_status == details::GJK::Collision) {
    Scalar penetrationDepth =
        details::computePenetration(t1.a, t1.b, t1.c, t2.a, t2.b, t2.c, normal);
    distance = -penetrationDepth;
  } else {
    // No collision
    // TODO On degenerated case, the closest point may be wrong
    // (i.e. an object face normal is colinear to gjk.ray
    // assert (dist == (w0 - w1).norm());
    assert(solver->gjk.ray.norm() > solver->gjk.getTolerance());
  }
  // assert(false && "should not reach this point");
  // return false;

  return distance;
}
}  // namespace internal

}  // namespace coal
