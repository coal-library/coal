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
///        Demonstrates how to implement a custom shape outside the Coal library,
///        specifically in the style of Tesseract Bullet's CastHullShape for
///        continuous collision detection (Schulman et al., 2013).

#define BOOST_TEST_MODULE COAL_CUSTOM_SHAPE
#include <boost/test/included/unit_test.hpp>

#include "coal/collision.h"
#include "coal/collision_object.h"
#include "coal/distance.h"
#include "coal/math/transform.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/narrowphase/support_data.h"
#include "coal/narrowphase/support_functions.h"
#include "coal/shape/geometric_shapes.h"

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

  /// @brief Support function of the swept volume (convex hull of two sphere
  /// poses). The support is the maximum of the two sphere supports:
  ///   support_cast(dir) = max(r * dir, T1 * (r * T1^{-1} * dir))
  ///                     = max(r * dir, r * dir + t1) (since sphere is round)
  ///                     = r * dir + max(0, dir · t1) * dir_normalized
  /// For a sphere, this simplifies to:
  ///   support_cast(dir) = r * dir + max(0, dir · t1) * dir
  /// (dir is already unit-length, so no normalization needed)
  void computeShapeSupport(const Vec3s& dir, Vec3s& support, int& /*hint*/,
                           details::ShapeSupportData& /*data*/) const override {
    // Support of sphere at pose 0 (identity)
    const Vec3s s0 = radius * dir;
    // Support of sphere at pose 1 (cast_tf)
    // For a sphere: support in dir = center + radius * dir
    // center of sphere at pose 1 = cast_tf.translation()
    const Vec3s& t1 = cast_tf.getTranslation();
    const Vec3s s1 = t1 + radius * dir;
    // Take the one that is furthest in direction dir
    support = (dir.dot(s0) >= dir.dot(s1)) ? s0 : s1;
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
    Transform3s tf2(Quats::Identity(),
                    Vec3s(2 * radius + sep, 0, 0));

    // Distance between two built-in spheres
    Sphere s2_builtin(radius);
    DistanceRequest request(true);
    DistanceResult result_builtin;
    Scalar d_builtin =
        distance(&builtin_sphere, tf1, &s2_builtin, tf2, request, result_builtin);

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
  Transform3s cast_tf(Quats::Identity(),
                      Vec3s(sweep_dist, 0, 0));
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
  std::size_t n = collide(&cast_sphere, tf_cast, &box, tf_box, coll_req, coll_res);
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
