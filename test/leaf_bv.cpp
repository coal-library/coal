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

/// Tests of CollisionRequest::test_leaf_bv: testing the bounding volumes of a
/// leaf pair before the narrow-phase test on that pair.

#define BOOST_TEST_MODULE COAL_LEAF_BV
#include <boost/test/included/unit_test.hpp>

#include <random>
#include <vector>

#include "coal/BV/OBBRSS.h"
#include "coal/BVH/BVH_model.h"
#include "coal/collision.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/shape/geometric_shapes.h"

#include "../src/collision_node.h"
#include "coal/internal/traversal_node_bvh_shape.h"
#include "coal/internal/traversal_node_setup.h"

using namespace coal;

namespace {

// Seed of the randomized sweep; fixed so that a failure is reproducible.
constexpr std::uint32_t SWEEP_SEED = 20261111u;
// Random shape poses per (shape, margin) pair.
constexpr int SWEEP_POSES = 50;
// Large enough that no traversal of the sweep mesh stops early, so the full
// contact set is compared.
constexpr size_t ALL_CONTACTS = 100000;
// Security margins of the sweep, in metres (0 to 5 cm).
const std::vector<Scalar> SWEEP_MARGINS = {0, 1e-3, 5e-2};

struct LeafCounts {
  int leaf_tests;
  int bv_tests;
};

CollisionRequest request(Scalar margin, size_t max_contacts, bool leaf_bv) {
  CollisionRequest r(CONTACT, max_contacts);
  r.security_margin = margin;
  r.test_leaf_bv = leaf_bv;
  return r;
}

/// Runs the oriented mesh-vs-shape traversal (the path coal::collide takes for
/// an OBBRSS mesh) with statistics enabled and returns its counters.
template <typename S>
LeafCounts countedCollide(const BVHModel<OBBRSS>& mesh,
                          const Transform3s& tf_mesh, const S& shape,
                          const Transform3s& tf_shape,
                          const CollisionRequest& req, CollisionResult& result,
                          bool recursive) {
  GJKSolver solver(req);
  MeshShapeCollisionTraversalNode<OBBRSS, S, 0> node(req);
  initialize(node, mesh, tf_mesh, shape, tf_shape, &solver, result);
  node.enableStatistics(true);
  coal::collide(&node, req, result, NULL, recursive);
  return LeafCounts{node.num_leaf_tests, node.num_bv_tests};
}

/// Two triangles: A lies 5 mm above the top face of a unit box centred at the
/// origin, B lies more than a metre away from it.
BVHModel<OBBRSS> twoTriangleMesh() {
  BVHModel<OBBRSS> mesh;
  mesh.beginModel();
  const Scalar z_near = Scalar(0.505);
  mesh.addTriangle(Vec3s(-0.2, -0.2, z_near), Vec3s(0.2, -0.2, z_near),
                   Vec3s(0.0, 0.2, z_near));
  const Scalar z_far = Scalar(2.0);
  mesh.addTriangle(Vec3s(3.0, 0.0, z_far), Vec3s(3.5, 0.0, z_far),
                   Vec3s(3.0, 0.5, z_far));
  mesh.endModel();
  return mesh;
}

void checkSameContacts(const CollisionResult& expected,
                       const CollisionResult& actual) {
  BOOST_REQUIRE_EQUAL(expected.isCollision(), actual.isCollision());
  BOOST_REQUIRE_EQUAL(expected.numContacts(), actual.numContacts());
  for (size_t i = 0; i < expected.numContacts(); ++i) {
    const Contact& e = expected.getContact(i);
    const Contact& a = actual.getContact(i);
    BOOST_CHECK_EQUAL(e.b1, a.b1);
    BOOST_CHECK_EQUAL(e.b2, a.b2);
    BOOST_CHECK(e.normal == a.normal);
    BOOST_CHECK(e.pos == a.pos);
    BOOST_CHECK_EQUAL(e.penetration_depth, a.penetration_depth);
  }
}

Transform3s randomPose(std::mt19937& rng, Scalar half_range) {
  std::uniform_real_distribution<Scalar> pos(-half_range, half_range);
  std::normal_distribution<Scalar> gauss(0, 1);
  Quats q(gauss(rng), gauss(rng), gauss(rng), gauss(rng));
  q.normalize();
  return Transform3s(q, Vec3s(pos(rng), pos(rng), pos(rng)));
}

/// Random poses of `shape` around a sphere mesh: the flag must not change the
/// contacts. Returns the total leaf tests without and with the flag.
template <typename S>
std::pair<long, long> sweep(const S& shape, std::mt19937& rng) {
  BVHModel<OBBRSS> mesh;
  generateBVHModel(mesh, Sphere(0.3), Transform3s(), 12, 12);

  long leaves_default = 0, leaves_flag = 0;
  for (Scalar margin : SWEEP_MARGINS) {
    for (int k = 0; k < SWEEP_POSES; ++k) {
      const Transform3s tf_mesh = randomPose(rng, 0.05);
      const Transform3s tf_shape = randomPose(rng, 0.7);
      CollisionResult r_default, r_flag;
      leaves_default +=
          countedCollide(mesh, tf_mesh, shape, tf_shape,
                         request(margin, ALL_CONTACTS, false), r_default, true)
              .leaf_tests;
      leaves_flag +=
          countedCollide(mesh, tf_mesh, shape, tf_shape,
                         request(margin, ALL_CONTACTS, true), r_flag, true)
              .leaf_tests;
      checkSameContacts(r_default, r_flag);
    }
  }
  return {leaves_default, leaves_flag};
}

}  // namespace

BOOST_AUTO_TEST_CASE(one_triangle_outside_the_margin_is_pruned) {
  BOOST_CHECK(!CollisionRequest().test_leaf_bv);

  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Box box(1, 1, 1);
  for (bool recursive : {true, false}) {
    CollisionResult r_default, r_flag;
    const LeafCounts by_default =
        countedCollide(mesh, Transform3s(), box, Transform3s(),
                       request(Scalar(0.01), 10, false), r_default, recursive);
    const LeafCounts flag =
        countedCollide(mesh, Transform3s(), box, Transform3s(),
                       request(Scalar(0.01), 10, true), r_flag, recursive);
    // Default: both triangles reach the narrow phase, only the root's BV is
    // tested. Flag: root plus both leaves' BVs, and B is skipped.
    BOOST_CHECK_EQUAL(by_default.leaf_tests, 2);
    BOOST_CHECK_EQUAL(by_default.bv_tests, 1);
    BOOST_CHECK_EQUAL(flag.leaf_tests, 1);
    BOOST_CHECK_EQUAL(flag.bv_tests, 3);
    BOOST_REQUIRE_EQUAL(r_default.numContacts(), 1);
    BOOST_CHECK_EQUAL(r_default.getContact(0).b1, 0);
    checkSameContacts(r_default, r_flag);
  }
}

BOOST_AUTO_TEST_CASE(pruned_leaves_keep_the_lower_bound_invariant) {
  // No contact: both leaves are pruned and the result's lower bound comes
  // from their BVs. checkResultLowerBound (src/collision_node.cpp) throws or
  // asserts if it disagrees with the traversal's squared lower bound.
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Box box(1, 1, 1);
  const Transform3s tf_box(Vec3s(0, 0, -0.5));
  for (bool recursive : {true, false}) {
    CollisionResult r_default, r_flag;
    countedCollide(mesh, Transform3s(), box, tf_box,
                   request(Scalar(0.01), 10, false), r_default, recursive);
    const LeafCounts flag =
        countedCollide(mesh, Transform3s(), box, tf_box,
                       request(Scalar(0.01), 10, true), r_flag, recursive);
    BOOST_CHECK(!r_flag.isCollision());
    BOOST_CHECK_EQUAL(flag.leaf_tests, 0);
    BOOST_CHECK_GT(r_flag.distance_lower_bound, 0);
    BOOST_CHECK_LE(
        r_flag.distance_lower_bound,
        r_default.distance_lower_bound + request(0, 1, false).gjk_tolerance);
  }
}

BOOST_AUTO_TEST_CASE(negative_margin_with_the_flag_is_refused) {
  // The leaf test bounds the distance to collision only for margins >= 0.
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Box box(1, 1, 1);
  for (bool recursive : {true, false}) {
    CollisionResult r_flag, r_default;
    BOOST_CHECK_THROW(
        countedCollide(mesh, Transform3s(), box, Transform3s(),
                       request(Scalar(-1e-3), 10, true), r_flag, recursive),
        std::invalid_argument);
    BOOST_CHECK_NO_THROW(countedCollide(mesh, Transform3s(), box, Transform3s(),
                                        request(Scalar(-1e-3), 10, false),
                                        r_default, recursive));
  }
}

BOOST_AUTO_TEST_CASE(randomized_sweep_keeps_the_contacts) {
  std::mt19937 rng(SWEEP_SEED);
  const std::pair<long, long> box = sweep(Box(0.2, 0.3, 0.1), rng);
  const std::pair<long, long> cyl = sweep(Cylinder(0.1, 0.4), rng);
  // Not vacuous: the flag skips leaves.
  BOOST_CHECK_LT(box.second, box.first);
  BOOST_CHECK_LT(cyl.second, cyl.first);
}
