/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Jelle Feringa
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

/// Tests of DistanceRequest::distance_upper_bound on the mesh-vs-shape
/// distance query.

#define BOOST_TEST_MODULE COAL_DISTANCE_UPPER_BOUND
#include <boost/test/included/unit_test.hpp>

#include "coal/BV/OBBRSS.h"
#include "coal/BVH/BVH_model.h"
#include "coal/distance.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"

#include "../src/collision_node.h"
#include "coal/internal/traversal_node_bvh_shape.h"
#include "coal/internal/traversal_node_setup.h"

using namespace coal;

namespace {

// Bound of the tests, in metres.
constexpr Scalar BOUND = Scalar(0.5);
// Radius of the sphere, in metres.
constexpr Scalar RADIUS = Scalar(0.05);
// The two triangles lie in the plane z = 0, centred at x = -TRIANGLE_X and
// x = +TRIANGLE_X (metres).
constexpr Scalar TRIANGLE_X = Scalar(1.5);
// Gaps between the sphere and the plane of the triangles, in metres: one
// below the bound, one above it.
constexpr Scalar GAP_BELOW_BOUND = Scalar(0.25);
constexpr Scalar GAP_ABOVE_BOUND = Scalar(3);

/// Triangle 0 around (-TRIANGLE_X, 0, 0), triangle 1 around (TRIANGLE_X, 0, 0).
BVHModel<OBBRSS> twoTriangleMesh() {
  BVHModel<OBBRSS> mesh;
  mesh.beginModel();
  for (const Scalar x : {-TRIANGLE_X, TRIANGLE_X})
    mesh.addTriangle(Vec3s(x - Scalar(0.5), Scalar(-0.5), 0),
                     Vec3s(x + Scalar(0.5), Scalar(-0.5), 0),
                     Vec3s(x, Scalar(0.5), 0));
  mesh.endModel();
  return mesh;
}

/// Pose of the sphere at abscissa `x`, `gap` above the plane of the triangles.
Transform3s sphereAbove(Scalar x, Scalar gap) {
  return Transform3s(Vec3s(x, 0, gap + RADIUS));
}

DistanceRequest boundedRequest() {
  DistanceRequest request;
  request.distance_upper_bound = BOUND;
  return request;
}

/// Runs the bounded query on the traversal node coal::distance uses for an
/// OBBRSS mesh, with statistics enabled, and returns its number of leaf tests.
int countedDistance(const BVHModel<OBBRSS>& mesh, const Sphere& sphere,
                    const Transform3s& tf_sphere, DistanceResult& result) {
  const DistanceRequest request = boundedRequest();
  const GJKSolver solver(request);
  MeshShapeDistanceTraversalNodeOBBRSS<Sphere> node;
  BOOST_REQUIRE(initialize(node, mesh, Transform3s(), sphere, tf_sphere,
                           &solver, request, result));
  node.enableStatistics(true);
  boundedDistance(&node);
  return node.num_leaf_tests;
}

}  // namespace

// With the default bound the standard distance runs: a distance above BOUND
// is reported.
BOOST_AUTO_TEST_CASE(default_bound) {
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Sphere sphere(RADIUS);
  DistanceResult result;
  distance(&mesh, Transform3s(), &sphere,
           sphereAbove(-TRIANGLE_X, GAP_ABOVE_BOUND), DistanceRequest(),
           result);
  BOOST_CHECK_SMALL(result.min_distance - GAP_ABOVE_BOUND,
                    GJK_DEFAULT_TOLERANCE);
  BOOST_CHECK(result.b1 == 0);
}

// The root bounding volume is farther than the bound: the result is left
// untouched.
BOOST_AUTO_TEST_CASE(root_beyond_bound) {
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Sphere sphere(RADIUS);
  DistanceResult result;
  distance(&mesh, Transform3s(), &sphere,
           sphereAbove(-TRIANGLE_X, GAP_ABOVE_BOUND), boundedRequest(), result);
  const DistanceResult untouched;
  BOOST_CHECK(result.min_distance == untouched.min_distance);
  BOOST_CHECK(result.b1 == DistanceResult::NONE);
  BOOST_CHECK(result.o1 == NULL);
}

// The root bounding volume is closer than the bound: a subtree farther than
// the bound is skipped.
BOOST_AUTO_TEST_CASE(subtree_beyond_bound) {
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Sphere sphere(RADIUS);

  // Above triangle 0: its distance is reported, triangle 1 is skipped.
  {
    DistanceResult result;
    const int leaf_tests = countedDistance(
        mesh, sphere, sphereAbove(-TRIANGLE_X, GAP_BELOW_BOUND), result);
    BOOST_CHECK_SMALL(result.min_distance - GAP_BELOW_BOUND,
                      GJK_DEFAULT_TOLERANCE);
    BOOST_CHECK(result.b1 == 0);
    BOOST_CHECK(leaf_tests == 1);
  }

  // Between the triangles, both farther than the bound: both are skipped and
  // the minimum distance stays at the bound.
  {
    DistanceResult result;
    const int leaf_tests =
        countedDistance(mesh, sphere, sphereAbove(0, GAP_BELOW_BOUND), result);
    BOOST_CHECK(result.min_distance == BOUND);
    BOOST_CHECK(result.b1 == DistanceResult::NONE);
    BOOST_CHECK(leaf_tests == 0);
  }
}
