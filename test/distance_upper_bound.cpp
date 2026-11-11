/// Tests of DistanceRequest::distance_upper_bound on the mesh-vs-shape
/// distance query.

#define BOOST_TEST_MODULE COAL_DISTANCE_UPPER_BOUND
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <type_traits>
#include <vector>

#include "coal/distance.h"
#include "coal/BVH/BVH_model.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/internal/traversal_node_bvh_shape.h"
#include "coal/internal/traversal_node_setup.h"
#include "../src/collision_node.h"

using namespace coal;

namespace {

const Scalar kUnbounded = (std::numeric_limits<Scalar>::max)();

// By value: BOOST_CHECK_EQUAL binds its arguments by reference.
const int kNone = DistanceResult::NONE;

// Bound used by the tests, in metres.
const Scalar kBound = Scalar(0.5);

template <typename BV>
std::shared_ptr<BVHModel<BV> > oneTriangleMesh() {
  std::shared_ptr<BVHModel<BV> > mesh(new BVHModel<BV>);
  mesh->beginModel();
  mesh->addTriangle(Vec3s(-1, -1, 0), Vec3s(1, -1, 0), Vec3s(0, 1, 0));
  mesh->endModel();
  return mesh;
}

// A rippled n x n grid of quads (2 n^2 triangles) spanning [-1, 1]^2.
template <typename BV>
std::shared_ptr<BVHModel<BV> > rippledGridMesh(int n) {
  std::vector<Vec3s> vertices;
  std::vector<Triangle32> triangles;
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n; ++j) {
      const Scalar x = Scalar(-1) + Scalar(2 * i) / Scalar(n);
      const Scalar y = Scalar(-1) + Scalar(2 * j) / Scalar(n);
      const Scalar z = Scalar(0.05) * std::sin(Scalar(3) * x) *
                       std::cos(Scalar(2) * y);
      vertices.push_back(Vec3s(x, y, z));
    }
  }
  const auto index = [n](int i, int j) {
    return static_cast<Triangle32::IndexType>(i * (n + 1) + j);
  };
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      triangles.push_back(
          Triangle32(index(i, j), index(i + 1, j), index(i + 1, j + 1)));
      triangles.push_back(
          Triangle32(index(i, j), index(i + 1, j + 1), index(i, j + 1)));
    }
  }
  std::shared_ptr<BVHModel<BV> > mesh(new BVHModel<BV>);
  mesh->beginModel();
  mesh->addSubModel(vertices, triangles);
  mesh->endModel();
  return mesh;
}

Transform3s translation(Scalar x, Scalar y, Scalar z) {
  Transform3s tf;
  tf.setTranslation(Vec3s(x, y, z));
  return tf;
}

DistanceRequest boundedRequest(Scalar bound) {
  DistanceRequest request;
  request.distance_upper_bound = bound;
  return request;
}

void checkUntouched(const DistanceResult& result) {
  const DistanceResult fresh;
  BOOST_CHECK_EQUAL(result.min_distance, fresh.min_distance);
  BOOST_CHECK(result.o1 == NULL);
  BOOST_CHECK(result.o2 == NULL);
  BOOST_CHECK_EQUAL(result.b1, kNone);
  BOOST_CHECK_EQUAL(result.b2, kNone);
  BOOST_CHECK(result.nearest_points[0] == fresh.nearest_points[0]);
  BOOST_CHECK(result.nearest_points[1] == fresh.nearest_points[1]);
  BOOST_CHECK(result.normal == fresh.normal);
}

// A deterministic sweep of shape poses above, beside and below the grid.
std::vector<Transform3s> sweepPoses() {
  std::vector<Transform3s> poses;
  const Scalar xs[] = {-1.4, -0.7, -0.13, 0.0, 0.41, 0.9, 1.3};
  const Scalar zs[] = {0.09, 0.17, 0.3, 0.45, 0.52, 0.8};
  for (Scalar x : xs) {
    for (Scalar z : zs) {
      Transform3s tf = translation(x, Scalar(0.6) * x - Scalar(0.1), z);
      tf.setQuatRotation(
          Quats(Eigen::AngleAxis<Scalar>(x + z, Vec3s(1, 2, 3).normalized())));
      poses.push_back(tf);
    }
  }
  return poses;
}

template <typename BV, typename Shape>
void checkBoundedMatchesUnboundedOnGrid(const Shape& shape) {
  const std::shared_ptr<BVHModel<BV> > mesh = rippledGridMesh<BV>(12);
  const Transform3s mesh_tf;
  int bounded_hits = 0;
  int beyond_bound = 0;
  for (const Transform3s& shape_tf : sweepPoses()) {
    DistanceResult unbounded;
    const Scalar d_unbounded = distance(mesh.get(), mesh_tf, &shape, shape_tf,
                                        DistanceRequest(), unbounded);
    DistanceResult bounded;
    const Scalar d_bounded = distance(mesh.get(), mesh_tf, &shape, shape_tf,
                                      boundedRequest(kBound), bounded);
    BOOST_CHECK_EQUAL(d_bounded, bounded.min_distance);
    if (d_unbounded < kBound) {
      ++bounded_hits;
      BOOST_CHECK_EQUAL(d_bounded, d_unbounded);
      BOOST_CHECK(bounded.b1 != kNone);
      BOOST_CHECK_EQUAL(bounded.b2, kNone);
      // The generic (OBB) path reports a temporary copy of the mesh as o1,
      // with or without a bound; only the oriented paths report the mesh.
      if (!std::is_same<BV, OBB>::value) {
        BOOST_CHECK(bounded.o1 == mesh.get());
      }
      BOOST_CHECK(bounded.o2 == &shape);
    } else {
      ++beyond_bound;
      // No triangle is closer than the bound: either the root box already
      // excluded the mesh (result untouched) or the traversal found nothing
      // below the bound it started from.
      BOOST_CHECK_EQUAL(bounded.b1, kNone);
      BOOST_CHECK(bounded.min_distance == kUnbounded ||
                  bounded.min_distance == kBound);
    }
  }
  // The sweep must exercise both sides of the bound.
  BOOST_CHECK(bounded_hits > 0);
  BOOST_CHECK(beyond_bound > 0);
}

template <typename BV>
void checkGridAllShapes() {
  checkBoundedMatchesUnboundedOnGrid<BV>(Sphere(0.05));
  checkBoundedMatchesUnboundedOnGrid<BV>(Box(0.1, 0.2, 0.05));
  checkBoundedMatchesUnboundedOnGrid<BV>(Capsule(0.04, 0.2));
}

}  // namespace

BOOST_AUTO_TEST_CASE(default_bound_is_largest_scalar) {
  const DistanceRequest request;
  BOOST_CHECK_EQUAL(request.distance_upper_bound, kUnbounded);
}

BOOST_AUTO_TEST_CASE(equality_and_printer_include_bound) {
  const DistanceRequest a;
  DistanceRequest b;
  BOOST_CHECK(a == b);
  b.distance_upper_bound = kBound;
  BOOST_CHECK(a != b);
  DistanceRequest c(b);
  BOOST_CHECK(b == c);

  std::ostringstream os;
  os << b;
  BOOST_CHECK(os.str().find("distance_upper_bound: 0.5") != std::string::npos);
}

// (1) One triangle: the bounded query returns the unbounded distance.
BOOST_AUTO_TEST_CASE(one_triangle_bounded_equals_unbounded) {
  const std::shared_ptr<BVHModel<OBBRSS> > mesh = oneTriangleMesh<OBBRSS>();
  const Sphere sphere(0.05);
  const Transform3s mesh_tf;
  const Transform3s sphere_tf = translation(0.1, 0.2, 0.2);

  DistanceResult unbounded;
  const Scalar d_unbounded = distance(mesh.get(), mesh_tf, &sphere, sphere_tf,
                                      DistanceRequest(), unbounded);
  DistanceResult bounded;
  const Scalar d_bounded = distance(mesh.get(), mesh_tf, &sphere, sphere_tf,
                                    boundedRequest(kBound), bounded);

  BOOST_CHECK_CLOSE(d_unbounded, 0.15, 1e-6);
  BOOST_CHECK_EQUAL(d_bounded, d_unbounded);
  BOOST_CHECK_EQUAL(bounded.b1, 0);
  BOOST_CHECK_EQUAL(bounded.b1, unbounded.b1);
  BOOST_CHECK(bounded.nearest_points[0] == unbounded.nearest_points[0]);
  BOOST_CHECK(bounded.nearest_points[1] == unbounded.nearest_points[1]);
  BOOST_CHECK(bounded.normal == unbounded.normal);
}

// (1, node level) The bounded query tests the root box once and runs one leaf
// test; the unbounded query tests no box. The unbounded preprocess GJK run is
// not counted by the node's statistics (it is not a leafComputeDistance call).
BOOST_AUTO_TEST_CASE(one_triangle_node_statistics) {
  const std::shared_ptr<BVHModel<OBBRSS> > mesh = oneTriangleMesh<OBBRSS>();
  const Sphere sphere(0.05);
  const Transform3s mesh_tf;
  const Transform3s sphere_tf = translation(0.1, 0.2, 0.2);

  for (const Scalar bound : {kUnbounded, kBound}) {
    const DistanceRequest request = boundedRequest(bound);
    const GJKSolver solver(request);
    DistanceResult result;
    MeshShapeDistanceTraversalNodeOBBRSS<Sphere> node;
    BOOST_REQUIRE(initialize(node, *mesh, mesh_tf, sphere, sphere_tf, &solver,
                             request, result));
    node.enable_statistics = true;
    boundedDistance(&node);
    BOOST_CHECK_CLOSE(result.min_distance, 0.15, 1e-6);
    BOOST_CHECK_EQUAL(node.num_leaf_tests, 1);
    BOOST_CHECK_EQUAL(node.num_bv_tests, bound == kUnbounded ? 0 : 1);
  }
}

// (2) A shape farther than the bound: the result is untouched.
BOOST_AUTO_TEST_CASE(shape_beyond_bound_leaves_result_untouched) {
  const Sphere sphere(0.05);
  const Transform3s mesh_tf;
  const Transform3s far_tf = translation(0.0, 0.0, 3.0);
  {
    const std::shared_ptr<BVHModel<OBBRSS> > mesh = oneTriangleMesh<OBBRSS>();
    DistanceResult result;
    const Scalar d = distance(mesh.get(), mesh_tf, &sphere, far_tf,
                              boundedRequest(kBound), result);
    BOOST_CHECK_EQUAL(d, kUnbounded);
    checkUntouched(result);
  }
  {
    const std::shared_ptr<BVHModel<OBBRSS> > mesh = rippledGridMesh<OBBRSS>(12);
    DistanceResult result;
    distance(mesh.get(), mesh_tf, &sphere, far_tf, boundedRequest(kBound),
             result);
    checkUntouched(result);

    // Node level: one box test (the root), no leaf test.
    const DistanceRequest request = boundedRequest(kBound);
    const GJKSolver solver(request);
    DistanceResult node_result;
    MeshShapeDistanceTraversalNodeOBBRSS<Sphere> node;
    BOOST_REQUIRE(initialize(node, *mesh, mesh_tf, sphere, far_tf, &solver,
                             request, node_result));
    node.enable_statistics = true;
    boundedDistance(&node);
    checkUntouched(node_result);
    BOOST_CHECK_EQUAL(node.num_bv_tests, 1);
    BOOST_CHECK_EQUAL(node.num_leaf_tests, 0);
  }
}

// A bound never raises a minimum the caller's result already holds.
BOOST_AUTO_TEST_CASE(bound_never_raises_existing_minimum) {
  const std::shared_ptr<BVHModel<OBBRSS> > mesh = rippledGridMesh<OBBRSS>(12);
  const Sphere sphere(0.05);
  DistanceResult result;
  result.min_distance = Scalar(0.1);
  distance(mesh.get(), Transform3s(), &sphere, translation(0.0, 0.0, 0.4),
           boundedRequest(kBound), result);
  BOOST_CHECK_EQUAL(result.min_distance, Scalar(0.1));
  BOOST_CHECK_EQUAL(result.b1, kNone);
}

// (3) A separated multi-triangle mesh: bounded == unbounded whenever the true
// distance is below the bound, for every bounding-volume type and both call
// orders.
BOOST_AUTO_TEST_CASE(grid_bounded_equals_unbounded_below_bound_obbrss) {
  checkGridAllShapes<OBBRSS>();
}

BOOST_AUTO_TEST_CASE(grid_bounded_equals_unbounded_below_bound_rss) {
  checkGridAllShapes<RSS>();
}

BOOST_AUTO_TEST_CASE(grid_bounded_equals_unbounded_below_bound_kios) {
  checkGridAllShapes<kIOS>();
}

// OBB takes the generic (non-oriented) mesh-shape path. Its bounding-volume
// distance is a stub returning 0, so the root test never excludes the mesh and
// nothing is pruned, but the seeded traversal must still agree. (AABB has no
// mesh-shape distance function registered.)
BOOST_AUTO_TEST_CASE(grid_bounded_equals_unbounded_below_bound_obb) {
  checkGridAllShapes<OBB>();
}

BOOST_AUTO_TEST_CASE(grid_shape_first_call_order) {
  const std::shared_ptr<BVHModel<OBBRSS> > mesh = rippledGridMesh<OBBRSS>(12);
  const Capsule capsule(0.04, 0.2);
  int hits = 0;
  for (const Transform3s& shape_tf : sweepPoses()) {
    DistanceResult unbounded;
    const Scalar d_unbounded =
        distance(&capsule, shape_tf, mesh.get(), Transform3s(),
                 DistanceRequest(), unbounded);
    DistanceResult bounded;
    const Scalar d_bounded = distance(&capsule, shape_tf, mesh.get(),
                                      Transform3s(), boundedRequest(kBound),
                                      bounded);
    if (d_unbounded < kBound) {
      ++hits;
      BOOST_CHECK_EQUAL(d_bounded, d_unbounded);
      BOOST_CHECK(bounded.nearest_points[0] == unbounded.nearest_points[0]);
      BOOST_CHECK(bounded.nearest_points[1] == unbounded.nearest_points[1]);
    }
  }
  BOOST_CHECK(hits > 0);
}

// A NaN bound is refused.
BOOST_AUTO_TEST_CASE(nan_bound_is_refused) {
  const std::shared_ptr<BVHModel<OBBRSS> > mesh = oneTriangleMesh<OBBRSS>();
  const Sphere sphere(0.05);
  DistanceResult result;
  BOOST_CHECK_THROW(
      distance(mesh.get(), Transform3s(), &sphere, translation(0, 0, 0.2),
               boundedRequest(std::numeric_limits<Scalar>::quiet_NaN()),
               result),
      std::invalid_argument);
}
