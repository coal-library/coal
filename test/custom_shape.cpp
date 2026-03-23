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
 *   * Neither the name of the copyright holder nor the names of its
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

/// @file custom_shape.cpp
/// @brief Tests for custom shape support via ShapeBase virtual dispatch.
///        Demonstrates how to implement a custom shape outside the Coal
///        library, specifically in the style of the Tesseract implementation of
///        a Trajopt CastHullShape for continuous collision detection
///        (originally using Bullet) (Schulman et al., 2014).

#define BOOST_TEST_MODULE COAL_CUSTOM_SHAPE
#include <boost/test/included/unit_test.hpp>

#include "coal/collision.h"
#include "coal/collision_object.h"
#include "coal/distance.h"
#include "coal/math/transform.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/narrowphase/support_data.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/shape/geometric_shapes_utility.h"
#include "coal/BVH/BVH_model.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/hfield.h"
#include "coal/contact_patch.h"

#ifdef COAL_HAS_OCTOMAP
#include "coal/octree.h"
#endif

using namespace coal;

// ============================================================================
// CustomSphere: a sphere implemented as a custom ShapeBase subclass.
// This demonstrates the pattern for extending Coal with custom shapes
// without modifying Coal source code.
// ============================================================================
class CustomSphere : public ShapeBase {
 public:
  explicit CustomSphere(Scalar radius) : ShapeBase(), radius(radius) {}

  CustomSphere* clone() const override { return new CustomSphere(*this); }

  NODE_TYPE getNodeType() const override { return GEOM_CUSTOM; }

  void computeLocalAABB() override {
    const Scalar r = radius + this->getSweptSphereRadius();
    aabb_local.min_ = Vec3s::Constant(-r);
    aabb_local.max_ = Vec3s::Constant(r);
    aabb_center = Vec3s::Zero();
    aabb_radius = r;
  }

  // Support function: for a sphere the support is simply radius * dir
  // (direction is already unit-length as required by Coal).
  // Note: do NOT add swept sphere radius here; Coal handles it separately.
  void computeShapeSupport(const Vec3s& dir, Vec3s& support, int& /*hint*/,
                           details::ShapeSupportData& /*data*/) const override {
    support = radius * dir;
  }

  // A sphere is smooth/strictly convex, so Nesterov normalization is helpful.
  bool needNesterovNormalizeHeuristic() const override { return true; }

  bool isEqual(const CollisionGeometry& other) const override {
    const CustomSphere* other_ = dynamic_cast<const CustomSphere*>(&other);
    if (other_ == nullptr) return false;
    return radius == other_->radius;
  }

  Scalar radius;
};

// ============================================================================
// CastSphere: swept volume of a sphere moving from the identity transform to
// a given cast transform. This implements the Schulman et al. (2013) swept
// volume support function: support_cast(d) = max(support_T0(d), support_T1(d))
// where T0 is identity and T1 is the cast_tf.
//
// This is the Coal equivalent of Tesseract Bullet's CastHullShape, using
// the virtual dispatch extension mechanism added to ShapeBase.
// ============================================================================
class CastSphere : public ShapeBase {
 public:
  /// @param radius sphere radius
  /// @param cast_tf the transform from pose 0 (identity) to pose 1,
  ///                expressed in the local frame of this shape.
  CastSphere(Scalar radius, const Transform3s& cast_tf)
      : ShapeBase(), radius(radius), cast_tf(cast_tf) {}

  CastSphere* clone() const override { return new CastSphere(*this); }

  NODE_TYPE getNodeType() const override { return GEOM_CUSTOM; }

  void computeLocalAABB() override {
    // AABB must bound the sphere at both pose 0 (identity) and pose 1.
    const Scalar r = radius + this->getSweptSphereRadius();
    // Pose 0: sphere at origin
    Vec3s min0 = Vec3s::Constant(-r);
    Vec3s max0 = Vec3s::Constant(r);
    // Pose 1: sphere at cast_tf.translation()
    const Vec3s& t1 = cast_tf.getTranslation();
    Vec3s min1 = t1 - Vec3s::Constant(r);
    Vec3s max1 = t1 + Vec3s::Constant(r);
    aabb_local.min_ = min0.cwiseMin(min1);
    aabb_local.max_ = max0.cwiseMax(max1);
    aabb_center = (aabb_local.min_ + aabb_local.max_) / 2;
    aabb_radius = (aabb_local.max_ - aabb_local.min_).norm() / 2;
  }

  /// @brief Support of the swept volume (convex hull of shape at pose 0 and
  /// pose 1). General formula (Schulman et al. 2013):
  ///   s0 = support_shape(dir)                          -- pose 0 (identity)
  ///   s1 = T * support_shape(T.rotation^{-1} * dir)    -- pose 1
  ///   support_cast(dir) = argmax_{s0, s1} dot(dir, ·)
  /// For a sphere, T.rotation^{-1} * dir == dir (rotationally symmetric),
  /// so s1 simplifies to t + radius * dir.
  void computeShapeSupport(const Vec3s& dir, Vec3s& support, int& /*hint*/,
                           details::ShapeSupportData& /*data*/) const override {
    // Pose 0: shape in its local frame (identity transform)
    const Vec3s s0 = radius * dir;

    // Pose 1: rotate dir into pose-1 local frame, get support, transform back.
    // For a sphere the rotation is irrelevant (radius * dir regardless of
    // orientation), but we keep the general structure here. A non-symmetric
    // shape would replace `radius * dir_local1` with its own local support.
    const Vec3s dir_local1 = cast_tf.getRotation().transpose() * dir;
    const Vec3s s1 = cast_tf.transform(radius * dir_local1);

    // Take the one that is furthest in direction dir. (Prefer pose 1 on tie,
    // matching Tesseract.)
    support = (dir.dot(s0) > dir.dot(s1)) ? s0 : s1;
  }

  bool needNesterovNormalizeHeuristic() const override { return true; }

  bool isEqual(const CollisionGeometry& other) const override {
    const CastSphere* other_ = dynamic_cast<const CastSphere*>(&other);
    if (other_ == nullptr) return false;
    return radius == other_->radius && cast_tf == other_->cast_tf;
  }

  Scalar radius;
  Transform3s cast_tf;  ///< relative transform from pose 0 to pose 1
};

// ============================================================================
// Tests
// ============================================================================

/// Verify that GEOM_CUSTOM is the default node type for ShapeBase subclasses.
BOOST_AUTO_TEST_CASE(test_geom_custom_node_type) {
  CustomSphere sphere(1.0);
  BOOST_CHECK_EQUAL(sphere.getNodeType(), GEOM_CUSTOM);
  BOOST_CHECK_EQUAL(sphere.getObjectType(), OT_GEOM);

  CastSphere cast_sphere(1.0, Transform3s());
  BOOST_CHECK_EQUAL(cast_sphere.getNodeType(), GEOM_CUSTOM);
}

/// Test that a custom sphere gives the same distance result as Coal's
/// built-in Sphere, using both the low-level GJK API and the high-level
/// distance() API.
BOOST_AUTO_TEST_CASE(test_custom_sphere_vs_builtin_sphere_distance) {
  const Scalar radius = 1.0;
  const Scalar tol = Scalar(1e-6);

  // Built-in sphere
  Sphere builtin_sphere(radius);
  // Custom sphere (same geometry, implemented via virtual dispatch)
  CustomSphere custom_sphere(radius);

  GJKSolver solver;
  solver.gjk_tolerance = tol;
  solver.epa_tolerance = tol;

  // Test a range of separations
  std::vector<Scalar> separations = {0.1, 0.5, 1.0, 2.0, 5.0};
  for (Scalar sep : separations) {
    Transform3s tf1;  // identity
    Transform3s tf2(Quats::Identity(), Vec3s(2 * radius + sep, 0, 0));

    // Distance between two built-in spheres
    Sphere s2_builtin(radius);
    DistanceRequest request(true);
    DistanceResult result_builtin;
    Scalar d_builtin = distance(&builtin_sphere, tf1, &s2_builtin, tf2, request,
                                result_builtin);

    // Distance between custom sphere and built-in sphere
    DistanceResult result_custom;
    Scalar d_custom =
        distance(&custom_sphere, tf1, &s2_builtin, tf2, request, result_custom);

    BOOST_CHECK_CLOSE(d_custom, d_builtin, Scalar(1e-3));

    // Also test the symmetric pair (built-in vs. custom)
    DistanceResult result_sym;
    Scalar d_sym =
        distance(&s2_builtin, tf1, &custom_sphere, tf2, request, result_sym);
    BOOST_CHECK_CLOSE(d_sym, d_builtin, Scalar(1e-3));
  }
}

/// Test collision detection between a custom sphere and a built-in Box.
BOOST_AUTO_TEST_CASE(test_custom_sphere_collide_with_box) {
  const Scalar radius = 1.0;
  CustomSphere custom_sphere(radius);
  Box box(2.0, 2.0, 2.0);

  CollisionRequest request;
  CollisionResult result;

  // Overlapping: sphere center at distance 0.5 from box surface
  Transform3s tf1;  // custom sphere at origin
  Transform3s tf2(Quats::Identity(), Vec3s(1.5, 0, 0));

  std::size_t n_contacts =
      collide(&custom_sphere, tf1, &box, tf2, request, result);
  BOOST_CHECK_GT(n_contacts, 0u);

  // Separated: sphere center at distance 2.0 from box surface
  result.clear();
  tf2.setTranslation(Vec3s(4.0, 0, 0));
  n_contacts = collide(&custom_sphere, tf1, &box, tf2, request, result);
  BOOST_CHECK_EQUAL(n_contacts, 0u);
}

/// Test custom-vs-custom collision (both shapes use virtual dispatch).
BOOST_AUTO_TEST_CASE(test_custom_sphere_vs_custom_sphere) {
  const Scalar radius = 1.0;
  CustomSphere s1(radius);
  CustomSphere s2(radius);

  DistanceRequest dist_req(true);
  DistanceResult dist_res;

  // Touching: distance should be ~0
  Transform3s tf1;
  Transform3s tf2(Quats::Identity(), Vec3s(2 * radius, 0, 0));
  Scalar d = distance(&s1, tf1, &s2, tf2, dist_req, dist_res);
  BOOST_CHECK_SMALL(d, Scalar(1e-5));

  // Separated
  dist_res.clear();
  tf2.setTranslation(Vec3s(3 * radius, 0, 0));
  d = distance(&s1, tf1, &s2, tf2, dist_req, dist_res);
  BOOST_CHECK_CLOSE(d, Scalar(radius), Scalar(1e-3));

  // Overlapping
  CollisionRequest coll_req;
  CollisionResult coll_res;
  tf2.setTranslation(Vec3s(radius, 0, 0));
  std::size_t n = collide(&s1, tf1, &s2, tf2, coll_req, coll_res);
  BOOST_CHECK_GT(n, 0u);
}

/// Test a CastSphere (swept volume of a sphere between two poses).
/// This demonstrates Tesseract-style CCD via Coal's custom shape API.
///
/// The CastSphere represents the convex hull of a sphere moving from position
/// (0,0,0) to (d,0,0). Its support function maximises over both endpoints.
BOOST_AUTO_TEST_CASE(test_cast_sphere_swept_volume) {
  const Scalar radius = 1.0;
  const Scalar sweep_dist = 3.0;

  // Cast transform: sphere moves sweep_dist along X
  Transform3s cast_tf(Quats::Identity(), Vec3s(sweep_dist, 0, 0));
  CastSphere cast_sphere(radius, cast_tf);
  cast_sphere.computeLocalAABB();

  // A box placed just beyond the swept path end
  Box box(1.0, 1.0, 1.0);
  Transform3s tf_cast;  // cast sphere at origin
  Transform3s tf_box;

  CollisionRequest coll_req;
  CollisionResult coll_res;

  // Box at (sweep_dist + 0.5, 0, 0): box edge at sweep_dist + 0.0, just
  // touching the end sphere → should collide.
  tf_box.setTranslation(Vec3s(sweep_dist + radius + Scalar(0.4), 0, 0));
  std::size_t n =
      collide(&cast_sphere, tf_cast, &box, tf_box, coll_req, coll_res);
  BOOST_CHECK_GT(n, 0u);

  // Box far beyond the swept path: no collision.
  coll_res.clear();
  tf_box.setTranslation(Vec3s(sweep_dist + radius + Scalar(2.0), 0, 0));
  n = collide(&cast_sphere, tf_cast, &box, tf_box, coll_req, coll_res);
  BOOST_CHECK_EQUAL(n, 0u);

  // Box inside the swept path (mid-point): should collide.
  coll_res.clear();
  tf_box.setTranslation(Vec3s(sweep_dist / 2, 0, 0));
  n = collide(&cast_sphere, tf_cast, &box, tf_box, coll_req, coll_res);
  BOOST_CHECK_GT(n, 0u);

  // Verify distance API also works for the cast shape
  DistanceRequest dist_req(true);
  DistanceResult dist_res;
  tf_box.setTranslation(Vec3s(sweep_dist + radius + Scalar(2.0), 0, 0));
  Scalar d = distance(&cast_sphere, tf_cast, &box, tf_box, dist_req, dist_res);
  BOOST_CHECK_GT(d, Scalar(0));
  // Expected: gap = 2.0 - 0.5 (box half-extent) = 1.5
  BOOST_CHECK_CLOSE(d, Scalar(1.5), Scalar(1));
}

/// Verify that getNodeType() returning GEOM_CUSTOM does NOT affect built-in
/// shapes (they must still return their specific GEOM_* type).
BOOST_AUTO_TEST_CASE(test_builtin_shapes_keep_their_node_type) {
  BOOST_CHECK_EQUAL(Sphere(1.0).getNodeType(), GEOM_SPHERE);
  BOOST_CHECK_EQUAL(Box(1, 1, 1).getNodeType(), GEOM_BOX);
  BOOST_CHECK_EQUAL(Capsule(1, 2).getNodeType(), GEOM_CAPSULE);
  BOOST_CHECK_EQUAL(Cylinder(1, 2).getNodeType(), GEOM_CYLINDER);
  BOOST_CHECK_EQUAL(Cone(1, 2).getNodeType(), GEOM_CONE);
  BOOST_CHECK_EQUAL(Ellipsoid(1, 1, 1).getNodeType(), GEOM_ELLIPSOID);
}

/// Test computeBV<AABB, ShapeBase> produces tight AABBs matching built-in.
BOOST_AUTO_TEST_CASE(test_computeBV_AABB_ShapeBase) {
  const Scalar radius = 1.5;
  CustomSphere custom(radius);
  Sphere builtin(radius);

  // Identity transform
  {
    AABB bv_custom, bv_builtin;
    computeBV<AABB, ShapeBase>(custom, Transform3s(), bv_custom);
    computeBV<AABB, Sphere>(builtin, Transform3s(), bv_builtin);
    BOOST_CHECK(bv_custom.min_.isApprox(bv_builtin.min_, Scalar(1e-10)));
    BOOST_CHECK(bv_custom.max_.isApprox(bv_builtin.max_, Scalar(1e-10)));
  }

  // Non-identity transform (rotation + translation)
  {
    Transform3s tf;
    tf.setTranslation(Vec3s(1.0, 2.0, 3.0));
    Quats q(Eigen::AngleAxis<Scalar>(Scalar(0.7), Vec3s::UnitZ()));
    tf.setQuatRotation(q);

    AABB bv_custom, bv_builtin;
    computeBV<AABB, ShapeBase>(custom, tf, bv_custom);
    computeBV<AABB, Sphere>(builtin, tf, bv_builtin);
    // Sphere AABB is rotation-invariant, so both should match
    BOOST_CHECK(bv_custom.min_.isApprox(bv_builtin.min_, Scalar(1e-10)));
    BOOST_CHECK(bv_custom.max_.isApprox(bv_builtin.max_, Scalar(1e-10)));
  }
}

#ifdef COAL_HAS_OCTOMAP

/// Helper: create a simple occupied octree centered at the origin.
static OcTree makeSimpleOctree(Scalar resolution = 0.1) {
  auto octree_ptr =
      coal::shared_ptr<octomap::OcTree>(new octomap::OcTree(resolution));
  // Fill a small 1x1x1 cube centered at origin
  const Scalar half = Scalar(0.5);
  for (Scalar x = -half; x < half; x += resolution) {
    for (Scalar y = -half; y < half; y += resolution) {
      for (Scalar z = -half; z < half; z += resolution) {
        octomap::point3d p(static_cast<float>(x + resolution * Scalar(0.5)),
                           static_cast<float>(y + resolution * Scalar(0.5)),
                           static_cast<float>(z + resolution * Scalar(0.5)));
        octree_ptr->updateNode(p, true);
      }
    }
  }
  octree_ptr->updateInnerOccupancy();
  return OcTree(octree_ptr);
}

/// Test collision: CustomSphere vs OcTree (both directions).
BOOST_AUTO_TEST_CASE(test_custom_shape_octree_collision) {
  const Scalar radius = 0.5;
  CustomSphere custom(radius);
  custom.computeLocalAABB();

  OcTree octree = makeSimpleOctree();
  octree.computeLocalAABB();

  CollisionRequest request;
  CollisionResult result;

  // Overlapping: custom sphere at origin overlaps octree at origin
  Transform3s tf1;
  Transform3s tf2;
  std::size_t n = collide(&custom, tf1, &octree, tf2, request, result);
  BOOST_CHECK_GT(n, 0u);

  // Reversed order: octree vs custom
  result.clear();
  n = collide(&octree, tf2, &custom, tf1, request, result);
  BOOST_CHECK_GT(n, 0u);

  // Separated: move custom sphere far away
  result.clear();
  tf1.setTranslation(Vec3s(5.0, 0, 0));
  n = collide(&custom, tf1, &octree, tf2, request, result);
  BOOST_CHECK_EQUAL(n, 0u);

  // Reversed separated
  result.clear();
  n = collide(&octree, tf2, &custom, tf1, request, result);
  BOOST_CHECK_EQUAL(n, 0u);
}

/// Test distance: CustomSphere vs OcTree (both directions).
BOOST_AUTO_TEST_CASE(test_custom_shape_octree_distance) {
  const Scalar radius = 0.5;
  CustomSphere custom(radius);
  custom.computeLocalAABB();

  OcTree octree = makeSimpleOctree();
  octree.computeLocalAABB();

  DistanceRequest request(true);
  DistanceResult result;

  // Separated: custom sphere well away from octree
  Transform3s tf1(Quats::Identity(), Vec3s(3.0, 0, 0));
  Transform3s tf2;
  Scalar d = distance(&custom, tf1, &octree, tf2, request, result);
  BOOST_CHECK_GT(d, Scalar(0));

  // Reversed: octree vs custom
  result.clear();
  Scalar d_rev = distance(&octree, tf2, &custom, tf1, request, result);
  BOOST_CHECK_GT(d_rev, Scalar(0));

  // Both directions should give similar distances
  BOOST_CHECK_CLOSE(d, d_rev, Scalar(1));
}

/// Test that ComputeCollision does not throw for GEOM_CUSTOM <-> GEOM_OCTREE.
BOOST_AUTO_TEST_CASE(test_custom_octree_no_throw) {
  CustomSphere custom(1.0);
  custom.computeLocalAABB();

  OcTree octree = makeSimpleOctree();
  octree.computeLocalAABB();

  CollisionRequest request;
  CollisionResult result;

  // Should not throw unsupported-pair exception
  BOOST_CHECK_NO_THROW(
      collide(&custom, Transform3s(), &octree, Transform3s(), request, result));
  result.clear();
  BOOST_CHECK_NO_THROW(
      collide(&octree, Transform3s(), &custom, Transform3s(), request, result));
}

#endif  // COAL_HAS_OCTOMAP

// ============================================================================
// BVH mesh tests
// ============================================================================

/// Helper: create a simple BVHModel<OBBRSS> box mesh (unit cube centered at
/// origin).
static coal::shared_ptr<BVHModel<OBBRSS>> makeBoxMesh() {
  auto model = coal::make_shared<BVHModel<OBBRSS>>();
  generateBVHModel(*model, Box(Vec3s::Ones()), Transform3s());
  return model;
}

/// Collision: CustomSphere vs BVHModel<OBBRSS> mesh
BOOST_AUTO_TEST_CASE(test_custom_shape_bvh_collision) {
  const Scalar radius = 0.5;
  CustomSphere custom(radius);
  custom.computeLocalAABB();

  auto mesh = makeBoxMesh();

  CollisionRequest request;
  CollisionResult result;

  // Overlapping: sphere at origin, mesh at origin
  Transform3s tf1, tf2;
  std::size_t n = collide(&custom, tf1, mesh.get(), tf2, request, result);
  BOOST_CHECK_GT(n, 0u);

  // Reversed order
  result.clear();
  n = collide(mesh.get(), tf2, &custom, tf1, request, result);
  BOOST_CHECK_GT(n, 0u);

  // Separated: move sphere far away
  result.clear();
  tf1.setTranslation(Vec3s(5.0, 0, 0));
  n = collide(&custom, tf1, mesh.get(), tf2, request, result);
  BOOST_CHECK_EQUAL(n, 0u);
}

/// Distance: CustomSphere vs BVHModel<OBBRSS> mesh
BOOST_AUTO_TEST_CASE(test_custom_shape_bvh_distance) {
  const Scalar radius = 0.5;
  CustomSphere custom(radius);
  custom.computeLocalAABB();

  auto mesh = makeBoxMesh();

  DistanceRequest request(true);
  DistanceResult result;

  // Separated: sphere at x=3, mesh at origin. Expected distance ~2.0
  // (sphere surface at 2.5, box surface at 0.5)
  Transform3s tf1(Quats::Identity(), Vec3s(3.0, 0, 0));
  Transform3s tf2;
  Scalar d = distance(&custom, tf1, mesh.get(), tf2, request, result);
  BOOST_CHECK_GT(d, Scalar(0));
  BOOST_CHECK_CLOSE(d, Scalar(2.0), Scalar(5));

  // Symmetry: mesh vs custom should give similar distance
  result.clear();
  Scalar d_rev = distance(mesh.get(), tf2, &custom, tf1, request, result);
  BOOST_CHECK_GT(d_rev, Scalar(0));
  BOOST_CHECK_CLOSE(d, d_rev, Scalar(1));
}

/// Collision: CustomSphere vs HeightField<AABB>
BOOST_AUTO_TEST_CASE(test_custom_shape_heightfield_collision) {
  const Scalar radius = 0.5;
  CustomSphere custom(radius);
  custom.computeLocalAABB();

  // Create a simple flat 2x2 heightfield at z=0, spanning [-1,1] x [-1,1]
  const Eigen::DenseIndex nx = 3, ny = 3;
  MatrixXs heights = MatrixXs::Zero(ny, nx);
  HeightField<AABB> hf(Scalar(2.0), Scalar(2.0), heights,
                       Scalar(-0.1));  // min_height
  hf.computeLocalAABB();

  CollisionRequest request;
  CollisionResult result;

  // Sphere at z=0.3 (overlaps with heightfield surface at z=0)
  Transform3s tf1(Quats::Identity(), Vec3s(0, 0, Scalar(0.3)));
  Transform3s tf2;
  std::size_t n = collide(&custom, tf1, &hf, tf2, request, result);
  BOOST_CHECK_GT(n, 0u);

  // Sphere far above: no collision
  result.clear();
  tf1.setTranslation(Vec3s(0, 0, 5.0));
  n = collide(&custom, tf1, &hf, tf2, request, result);
  BOOST_CHECK_EQUAL(n, 0u);
}

/// HeightField distance is unimplemented in Coal for ALL shape types (the
/// matrix entry exists but the function throws std::invalid_argument).
/// This test documents that limitation and ensures GEOM_CUSTOM behaves
/// consistently with built-in shapes.
BOOST_AUTO_TEST_CASE(test_custom_shape_heightfield_distance_not_implemented) {
  const Scalar radius = 0.5;
  CustomSphere custom(radius);
  custom.computeLocalAABB();

  const Eigen::DenseIndex nx = 3, ny = 3;
  MatrixXs heights = MatrixXs::Zero(ny, nx);
  HeightField<AABB> hf(Scalar(2.0), Scalar(2.0), heights, Scalar(-0.1));
  hf.computeLocalAABB();

  DistanceRequest request(true);
  DistanceResult result;

  Transform3s tf1(Quats::Identity(), Vec3s(0, 0, 3.0));
  Transform3s tf2;

  // Matches the behaviour of built-in shapes (e.g. Sphere vs HeightField)
  BOOST_CHECK_THROW(distance(&custom, tf1, &hf, tf2, request, result),
                    std::invalid_argument);
}

/// Verify computeBV<OBB, ShapeBase> produces valid BV containing the shape.
BOOST_AUTO_TEST_CASE(test_computeBV_OBB_ShapeBase) {
  const Scalar radius = 1.5;
  CustomSphere custom(radius);

  Transform3s tf;
  tf.setTranslation(Vec3s(1.0, 2.0, 3.0));
  Quats q(Eigen::AngleAxis<Scalar>(Scalar(0.7), Vec3s::UnitZ()));
  tf.setQuatRotation(q);

  // Compute OBB for custom shape
  OBB obb;
  computeBV<OBB, ShapeBase>(custom, tf, obb);

  // Compute AABB for reference — the OBB should contain the AABB center
  AABB aabb;
  computeBV<AABB, ShapeBase>(custom, tf, aabb);

  // The OBB should at least contain the AABB center
  BOOST_CHECK(obb.contain(aabb.center()));

  // Also test OBBRSS (the one Tesseract uses)
  OBBRSS obbrss;
  computeBV<OBBRSS, ShapeBase>(custom, tf, obbrss);
  // OBBRSS should contain the AABB center too
  BOOST_CHECK(obbrss.contain(aabb.center()));
}

// ============================================================================
// Contact patch tests
// ============================================================================

/// Contact patch: CustomSphere vs built-in Box.
/// Exercises GEOM_CUSTOM entries in contact_patch_func_matrix and
/// the GEOM_CUSTOM case in ContactPatchSolver::makeSupportSetFunction.
BOOST_AUTO_TEST_CASE(test_custom_shape_contact_patch) {
  const Scalar radius = 1.0;
  CustomSphere custom(radius);
  Box box(2.0, 2.0, 2.0);

  // Sphere at origin, box slightly overlapping along +Z
  Transform3s tf1;
  const Scalar overlap = Scalar(0.01);
  Transform3s tf2(Quats::Identity(),
                  Vec3s(0, 0, radius + Scalar(1.0) - overlap));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  const ContactPatchRequest patch_req;

  CollisionResult col_res;
  coal::collide(&custom, tf1, &box, tf2, col_req, col_res);
  BOOST_REQUIRE(col_res.isCollision());

  {
    ContactPatchResult patch_res(patch_req);
    coal::computeContactPatch(&custom, tf1, &box, tf2, col_res, patch_req,
                              patch_res);
    BOOST_REQUIRE(patch_res.numContactPatches() > 0);

    const Contact& contact = col_res.getContact(0);
    const ContactPatch& patch = patch_res.getContactPatch(0);

    // Sphere is strictly convex => single-point contact patch
    BOOST_CHECK_EQUAL(patch.size(), 1u);

    const Scalar tol = Scalar(1e-3);
    BOOST_CHECK_SMALL((patch.getNormal() - contact.normal).norm(), tol);
    BOOST_CHECK_SMALL(
        std::abs(patch.penetration_depth - contact.penetration_depth), tol);
  }

  // Reversed order: Box vs CustomSphere
  {
    CollisionResult col_res2;
    coal::collide(&box, tf2, &custom, tf1, col_req, col_res2);
    BOOST_REQUIRE(col_res2.isCollision());

    ContactPatchResult patch_res(patch_req);
    coal::computeContactPatch(&box, tf2, &custom, tf1, col_res2, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() > 0);
  }

  // GEOM_CUSTOM vs GEOM_CUSTOM
  {
    CustomSphere custom2(radius);
    Transform3s tf_c2(Quats::Identity(), Vec3s(0, 0, 2 * radius - overlap));

    CollisionResult col_res3;
    coal::collide(&custom, tf1, &custom2, tf_c2, col_req, col_res3);
    BOOST_REQUIRE(col_res3.isCollision());

    ContactPatchResult patch_res(patch_req);
    coal::computeContactPatch(&custom, tf1, &custom2, tf_c2, col_res3,
                              patch_req, patch_res);
    BOOST_REQUIRE(patch_res.numContactPatches() > 0);

    const ContactPatch& patch = patch_res.getContactPatch(0);
    BOOST_CHECK_EQUAL(patch.size(), 1u);
  }
}

/// Smoke test: contact patch computation does not throw for various
/// GEOM_CUSTOM pairings.
BOOST_AUTO_TEST_CASE(test_custom_shape_contact_patch_no_throw) {
  const Scalar radius = 1.0;
  CustomSphere custom(radius);

  Transform3s tf1;
  const Scalar overlap = Scalar(0.01);
  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);

  auto test_pair = [&](CollisionGeometry* o1, const Transform3s& t1,
                       CollisionGeometry* o2, const Transform3s& t2) {
    CollisionResult col_res;
    coal::collide(o1, t1, o2, t2, col_req, col_res);
    BOOST_REQUIRE(col_res.isCollision());
    const ContactPatchRequest patch_req;
    ContactPatchResult patch_res(patch_req);
    BOOST_CHECK_NO_THROW(coal::computeContactPatch(o1, t1, o2, t2, col_res,
                                                   patch_req, patch_res));
  };

  Transform3s tf_near(Quats::Identity(), Vec3s(0, 0, 2 * radius - overlap));

  Sphere sphere(radius);
  test_pair(&custom, tf1, &sphere, tf_near);

  Capsule capsule(radius, 2.0);
  test_pair(&custom, tf1, &capsule, tf_near);

  Cylinder cylinder(radius, 2.0);
  test_pair(&custom, tf1, &cylinder, tf_near);

  Ellipsoid ellipsoid(radius, radius, radius);
  test_pair(&custom, tf1, &ellipsoid, tf_near);
}
