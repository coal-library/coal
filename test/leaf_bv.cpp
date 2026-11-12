/// Tests of CollisionRequest::test_leaf_bv: testing the bounding volume of a
/// leaf pair before the narrow-phase test on that pair.

#define BOOST_TEST_MODULE COAL_LEAF_BV
#include <boost/test/included/unit_test.hpp>

#include <random>
#include <sstream>
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
// Number of random shape poses per (shape, margin) pair.
constexpr int SWEEP_POSES = 400;
// Upper bound on the number of contacts: large enough that no traversal of the
// sweep mesh stops early, so the full contact set is compared.
constexpr size_t ALL_CONTACTS = 100000;
// Security margins of the sweep, in metres (0 to 5 cm).
const std::vector<Scalar> SWEEP_MARGINS = {0, 1e-4, 1e-3, 1e-2, 5e-2};

struct LeafCounts {
  int leaf_tests;
  int bv_tests;
};

/// Runs the oriented mesh-vs-shape traversal (the path coal::collide takes for
/// an OBBRSS mesh) with statistics enabled and returns its counters.
template <typename S>
LeafCounts countedCollide(const BVHModel<OBBRSS>& mesh,
                          const Transform3s& tf_mesh, const S& shape,
                          const Transform3s& tf_shape,
                          const CollisionRequest& request,
                          CollisionResult& result, bool recursive) {
  GJKSolver solver(request);
  MeshShapeCollisionTraversalNode<OBBRSS, S, 0> node(request);
  initialize(node, mesh, tf_mesh, shape, tf_shape, &solver, result);
  node.enableStatistics(true);
  coal::collide(&node, request, result, NULL, recursive);
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
    BOOST_CHECK(e.nearest_points[0] == a.nearest_points[0]);
    BOOST_CHECK(e.nearest_points[1] == a.nearest_points[1]);
    BOOST_CHECK_EQUAL(e.penetration_depth, a.penetration_depth);
  }
}

/// Requests compared everywhere: Coal as released, the released traversal with
/// `break_distance = collision_distance_threshold`, the leaf test alone, and
/// both changes together (the ablation arm).
struct RequestArms {
  CollisionRequest as_built;
  CollisionRequest tight_break;
  CollisionRequest leaf_bv;
  CollisionRequest leaf_bv_tight_break;

  RequestArms(Scalar margin, size_t max_contacts)
      : as_built(CONTACT, max_contacts) {
    as_built.security_margin = margin;
    tight_break = as_built;
    tight_break.break_distance = tight_break.collision_distance_threshold;
    leaf_bv = as_built;
    leaf_bv.test_leaf_bv = true;
    leaf_bv_tight_break = tight_break;
    leaf_bv_tight_break.test_leaf_bv = true;
  }
};

Transform3s randomPose(std::mt19937& rng, Scalar half_range) {
  std::uniform_real_distribution<Scalar> pos(-half_range, half_range);
  std::normal_distribution<Scalar> gauss(0, 1);
  Quats q(gauss(rng), gauss(rng), gauss(rng), gauss(rng));
  q.normalize();
  return Transform3s(q, Vec3s(pos(rng), pos(rng), pos(rng)));
}

/// Sweeps random poses of `shape` around a sphere mesh and checks that every
/// arm reports the contacts of Coal as released. Returns the total leaf tests
/// of the as-built and of the ablation arm.
template <typename S>
std::pair<long, long> sweep(const S& shape, std::mt19937& rng) {
  BVHModel<OBBRSS> mesh;
  generateBVHModel(mesh, Sphere(0.3), Transform3s(), 12, 12);

  long as_built_leaves = 0, ablation_leaves = 0;
  for (Scalar margin : SWEEP_MARGINS) {
    for (int k = 0; k < SWEEP_POSES; ++k) {
      const Transform3s tf_mesh = randomPose(rng, 0.05);
      const Transform3s tf_shape = randomPose(rng, 0.7);
      for (size_t max_contacts : {ALL_CONTACTS, size_t(1)}) {
        const RequestArms arms(margin, max_contacts);

        CollisionResult r_built, r_tight, r_leaf, r_both;
        coal::collide(&mesh, tf_mesh, &shape, tf_shape, arms.as_built, r_built);
        coal::collide(&mesh, tf_mesh, &shape, tf_shape, arms.tight_break,
                      r_tight);
        coal::collide(&mesh, tf_mesh, &shape, tf_shape, arms.leaf_bv, r_leaf);
        coal::collide(&mesh, tf_mesh, &shape, tf_shape,
                      arms.leaf_bv_tight_break, r_both);
        checkSameContacts(r_built, r_tight);
        checkSameContacts(r_built, r_leaf);
        checkSameContacts(r_built, r_both);

        // A pruned leaf contributes a bounding-volume bound, never larger
        // than the leaf's own distance to collision (up to GJK's tolerance).
        if (!r_built.isCollision()) {
          BOOST_CHECK_LE(
              r_both.distance_lower_bound,
              r_built.distance_lower_bound + arms.as_built.gjk_tolerance);
        }

        if (max_contacts == ALL_CONTACTS) {
          CollisionResult c_built, c_both;
          as_built_leaves += countedCollide(mesh, tf_mesh, shape, tf_shape,
                                            arms.as_built, c_built, true)
                                 .leaf_tests;
          ablation_leaves +=
              countedCollide(mesh, tf_mesh, shape, tf_shape,
                             arms.leaf_bv_tight_break, c_both, true)
                  .leaf_tests;
          checkSameContacts(c_built, c_both);
          checkSameContacts(r_built, c_built);
        }
      }
    }
  }
  return {as_built_leaves, ablation_leaves};
}

}  // namespace

BOOST_AUTO_TEST_CASE(default_is_coal_as_released) {
  BOOST_CHECK(!CollisionRequest().test_leaf_bv);
  BOOST_CHECK(!CollisionRequest(CONTACT, 10).test_leaf_bv);

  // Released Coal sends a leaf pair straight to the narrow phase: both
  // triangles are tested and only the root's bounding volume is.
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Box box(1, 1, 1);
  CollisionRequest request(CONTACT, 10);
  request.security_margin = Scalar(0.01);
  for (bool recursive : {true, false}) {
    CollisionResult result;
    const LeafCounts counts = countedCollide(
        mesh, Transform3s(), box, Transform3s(), request, result, recursive);
    BOOST_CHECK_EQUAL(counts.leaf_tests, 2);
    BOOST_CHECK_EQUAL(counts.bv_tests, 1);
    BOOST_CHECK_EQUAL(result.numContacts(), 1);
  }
}

BOOST_AUTO_TEST_CASE(one_triangle_outside_the_margin_is_pruned) {
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Box box(1, 1, 1);
  const RequestArms arms(Scalar(0.01), 10);

  for (bool recursive : {true, false}) {
    CollisionResult r_built, r_both;
    const LeafCounts built =
        countedCollide(mesh, Transform3s(), box, Transform3s(), arms.as_built,
                       r_built, recursive);
    const LeafCounts both =
        countedCollide(mesh, Transform3s(), box, Transform3s(),
                       arms.leaf_bv_tight_break, r_both, recursive);
    BOOST_CHECK_EQUAL(built.leaf_tests, 2);
    BOOST_CHECK_EQUAL(both.leaf_tests, 1);
    // Root plus both leaves.
    BOOST_CHECK_EQUAL(both.bv_tests, 3);
    BOOST_REQUIRE_EQUAL(r_built.numContacts(), 1);
    BOOST_CHECK_EQUAL(r_built.getContact(0).b1, 0);
    checkSameContacts(r_built, r_both);
  }
}

BOOST_AUTO_TEST_CASE(pruned_leaves_keep_the_lower_bound_invariant) {
  // No contact: both leaves are pruned when the flag is set, and the result's
  // lower bound comes from their bounding volumes. checkResultLowerBound
  // (src/collision_node.cpp) throws or asserts if it disagrees with the
  // traversal's squared lower bound.
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Box box(1, 1, 1);
  const Transform3s tf_box(Vec3s(0, 0, -0.5));
  const RequestArms arms(Scalar(0.01), 10);
  for (bool recursive : {true, false}) {
    CollisionResult r_built, r_both;
    countedCollide(mesh, Transform3s(), box, tf_box, arms.as_built, r_built,
                   recursive);
    const LeafCounts both =
        countedCollide(mesh, Transform3s(), box, tf_box,
                       arms.leaf_bv_tight_break, r_both, recursive);
    BOOST_CHECK(!r_built.isCollision());
    BOOST_CHECK(!r_both.isCollision());
    BOOST_CHECK_EQUAL(both.leaf_tests, 0);
    BOOST_CHECK_GT(r_both.distance_lower_bound, 0);
    BOOST_CHECK_LE(r_both.distance_lower_bound,
                   r_built.distance_lower_bound + arms.as_built.gjk_tolerance);
  }
}

BOOST_AUTO_TEST_CASE(randomized_sweep_matches_coal_as_released) {
  std::mt19937 rng(SWEEP_SEED);
  const std::pair<long, long> box_leaves = sweep(Box(0.2, 0.3, 0.1), rng);
  const std::pair<long, long> cyl_leaves = sweep(Cylinder(0.1, 0.4), rng);
  // The sweep is not vacuous: the ablation prunes leaves, and never runs more
  // narrow-phase tests than the released traversal.
  BOOST_CHECK_LT(box_leaves.second, box_leaves.first);
  BOOST_CHECK_LT(cyl_leaves.second, cyl_leaves.first);
  BOOST_TEST_MESSAGE("leaf tests box as built/ablation: "
                     << box_leaves.first << "/" << box_leaves.second);
  BOOST_TEST_MESSAGE("leaf tests cylinder as built/ablation: "
                     << cyl_leaves.first << "/" << cyl_leaves.second);
}

BOOST_AUTO_TEST_CASE(equality_and_printer_include_the_flag) {
  CollisionRequest a(CONTACT, 10), b(CONTACT, 10);
  BOOST_CHECK(a == b);
  b.test_leaf_bv = true;
  BOOST_CHECK(a != b);

  std::ostringstream os;
  os << b;
  BOOST_CHECK(os.str().find("test_leaf_bv: 1") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(negative_margin_with_the_flag_is_refused) {
  // The leaf test bounds the distance to collision only for margins >= 0.
  const BVHModel<OBBRSS> mesh = twoTriangleMesh();
  const Box box(1, 1, 1);
  CollisionRequest request(CONTACT, 10);
  request.security_margin = Scalar(-1e-3);
  request.test_leaf_bv = true;
  for (bool recursive : {true, false}) {
    CollisionResult result;
    BOOST_CHECK_THROW(countedCollide(mesh, Transform3s(), box, Transform3s(),
                                     request, result, recursive),
                      std::invalid_argument);
  }
  // Released Coal's public mesh-vs-shape entry already refuses a negative
  // margin (src/collision_func_matrix.cpp), flag or not.
  CollisionResult result;
  BOOST_CHECK_THROW(
      coal::collide(&mesh, Transform3s(), &box, Transform3s(), request, result),
      std::invalid_argument);

  // Without the flag the traversal itself accepts a negative margin, as
  // released: the guard applies to the flag only.
  request.test_leaf_bv = false;
  for (bool recursive : {true, false}) {
    CollisionResult released;
    BOOST_CHECK_NO_THROW(countedCollide(mesh, Transform3s(), box, Transform3s(),
                                        request, released, recursive));
  }
}
