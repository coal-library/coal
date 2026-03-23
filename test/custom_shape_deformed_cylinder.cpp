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

/// @file custom_shape_deformed_cylinder.cpp
/// @brief Demonstrates how GEOM_CUSTOM enables the "deformed cylinder" shape
///        from coal-library/coal#792 without any changes to Coal internals.
///        A deformed cylinder is the convex hull of two arbitrarily oriented
///        disks, useful for modelling joints in articulated cylindrical
///        structures (e.g. robotic fingers).

#define BOOST_TEST_MODULE COAL_CUSTOM_SHAPE_DEFORMED_CYLINDER
#include <boost/test/included/unit_test.hpp>

#include "coal/collision.h"
#include "coal/contact_patch.h"
#include "coal/distance.h"
#include "coal/math/transform.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/shape/geometric_shapes_utility.h"

using namespace coal;

// ============================================================================
// DeformedCylinder: convex hull of two arbitrarily oriented disks.
//
// Each disk is defined by a center (in local frame), a unit normal, and a
// radius. This models the joint piece between two tilted cylindrical segments
// (see coal-library/coal#792).
//
// The support function is closed-form: for each disk, the farthest point in
// direction d is center_i + r_i * normalize(d - (d·n_i)n_i). We return
// whichever disk yields the larger dot(d, support).
// ============================================================================
class DeformedCylinder : public ShapeBase {
 public:
  DeformedCylinder(const Vec3s& p1, const Vec3s& n1, Scalar r1,
                   const Vec3s& p2, const Vec3s& n2, Scalar r2)
      : ShapeBase(),
        p1(p1), n1(n1.normalized()), r1(r1),
        p2(p2), n2(n2.normalized()), r2(r2) {}

  DeformedCylinder* clone() const override {
    return new DeformedCylinder(*this);
  }

  void computeLocalAABB() override {
    // Delegate to the generic ShapeBase AABB computation, which calls
    // computeShapeSupport in each axis direction.
    AABB bv;
    computeBV<AABB, ShapeBase>(*this, Transform3s(), bv);
    const Scalar ssr = this->getSweptSphereRadius();
    aabb_local.min_ = bv.min_ - Vec3s::Constant(ssr);
    aabb_local.max_ = bv.max_ + Vec3s::Constant(ssr);
    aabb_center = (aabb_local.min_ + aabb_local.max_) / 2;
    aabb_radius = (aabb_local.max_ - aabb_local.min_).norm() / 2;
  }

  void computeShapeSupport(const Vec3s& dir, Vec3s& support, int& /*hint*/,
                           details::ShapeSupportData& /*data*/) const override {
    // When dir is parallel to a disk normal, the projection is zero and
    // the support degenerates to the disk center.
    auto disk_support = [](const Vec3s& p, const Vec3s& n, Scalar r,
                           const Vec3s& d) -> Vec3s {
      Vec3s proj = d - d.dot(n) * n;
      Scalar len = proj.norm();
      if (len > Scalar(1e-12))
        return p + r * (proj / len);
      return p;
    };

    Vec3s s1 = disk_support(p1, n1, r1, dir);
    Vec3s s2 = disk_support(p2, n2, r2, dir);
    support = (dir.dot(s1) >= dir.dot(s2)) ? s1 : s2;
  }

  // Smooth surface => Nesterov normalization is helpful
  bool needNesterovNormalizeHeuristic() const override { return true; }

  bool isEqual(const CollisionGeometry& other) const override {
    const auto* o = dynamic_cast<const DeformedCylinder*>(&other);
    if (o == nullptr) return false;
    return p1 == o->p1 && n1 == o->n1 && r1 == o->r1 &&
           p2 == o->p2 && n2 == o->n2 && r2 == o->r2;
  }

  Vec3s p1;       ///< disk 1 center (local frame)
  Vec3s n1;       ///< disk 1 normal (unit)
  Scalar r1;      ///< disk 1 radius
  Vec3s p2;       ///< disk 2 center (local frame)
  Vec3s n2;       ///< disk 2 normal (unit)
  Scalar r2;      ///< disk 2 radius
};

// ============================================================================
// Tests
// ============================================================================

BOOST_AUTO_TEST_CASE(test_deformed_cylinder_node_type) {
  DeformedCylinder dc(Vec3s(0, 0, 0), Vec3s::UnitZ(), 1.0,
                      Vec3s(0, 0, 2), Vec3s::UnitZ(), 1.0);
  BOOST_CHECK_EQUAL(dc.getObjectType(), OT_GEOM);
  BOOST_CHECK_EQUAL(dc.getNodeType(), GEOM_CUSTOM);
}

/// Collision and distance: 30° joint deformed cylinder vs Box.
BOOST_AUTO_TEST_CASE(test_deformed_cylinder_vs_box) {
  // Two disks at a 30° joint angle, radius 0.5, separated by 2 units
  const Scalar angle = M_PI / 6;  // 30°
  const Scalar r = 0.5;
  Vec3s n1 = Vec3s::UnitZ();
  Vec3s n2(std::sin(angle), 0, std::cos(angle));
  DeformedCylinder dc(Vec3s(0, 0, 0), n1, r,
                      Vec3s(0, 0, 2), n2, r);
  dc.computeLocalAABB();

  Box box(1.0, 1.0, 1.0);

  // Collision: box overlapping the deformed cylinder
  {
    CollisionRequest request;
    CollisionResult result;
    Transform3s tf1;
    Transform3s tf2(Quats::Identity(), Vec3s(0, 0, 1));
    std::size_t n = collide(&dc, tf1, &box, tf2, request, result);
    BOOST_CHECK_GT(n, 0u);
  }

  // No collision: box far away
  {
    CollisionRequest request;
    CollisionResult result;
    Transform3s tf1;
    Transform3s tf2(Quats::Identity(), Vec3s(5, 0, 1));
    std::size_t n = collide(&dc, tf1, &box, tf2, request, result);
    BOOST_CHECK_EQUAL(n, 0u);
  }

  // Distance: box separated from deformed cylinder
  {
    DistanceRequest request(true);
    DistanceResult result;
    Transform3s tf1;
    Transform3s tf2(Quats::Identity(), Vec3s(3, 0, 1));
    Scalar d = distance(&dc, tf1, &box, tf2, request, result);
    BOOST_CHECK_GT(d, Scalar(0));
    BOOST_CHECK_CLOSE(d, Scalar(2.0), Scalar(10));
  }
}

/// When both disks are parallel with the same radius, the deformed cylinder
/// degenerates to a standard cylinder. Verify distance matches Coal's built-in
/// Cylinder.
BOOST_AUTO_TEST_CASE(test_deformed_cylinder_degenerates_to_cylinder) {
  const Scalar r = 0.5;
  const Scalar half_h = 1.0;

  // DeformedCylinder with parallel disks along Z
  DeformedCylinder dc(Vec3s(0, 0, -half_h), Vec3s::UnitZ(), r,
                      Vec3s(0, 0, half_h), Vec3s::UnitZ(), r);
  dc.computeLocalAABB();

  // Coal built-in Cylinder (centered at origin, total height = 2*half_h)
  Cylinder cyl(r, 2 * half_h);

  Sphere probe(0.1);

  // Test distance from several directions
  Vec3s offsets[] = {
    Vec3s(3, 0, 0),       // radial
    Vec3s(0, 0, 3),       // axial above
    Vec3s(0, 0, -3),      // axial below
    Vec3s(2, 2, 0.5),     // diagonal
  };

  for (const auto& offset : offsets) {
    DistanceRequest request(true);
    Transform3s tf1;
    Transform3s tf2(Quats::Identity(), offset);

    DistanceResult res_dc;
    Scalar d_dc = distance(&dc, tf1, &probe, tf2, request, res_dc);

    DistanceResult res_cyl;
    Scalar d_cyl = distance(&cyl, tf1, &probe, tf2, request, res_cyl);

    // Both should be positive (probe is far enough)
    BOOST_CHECK_GT(d_dc, Scalar(0));
    BOOST_CHECK_GT(d_cyl, Scalar(0));

    // Distances should be very close (not exact due to GJK tolerance)
    BOOST_CHECK_CLOSE(d_dc, d_cyl, Scalar(1));  // 1% tolerance
  }
}

/// Contact patch: deformed cylinder vs Box.
BOOST_AUTO_TEST_CASE(test_deformed_cylinder_contact_patch) {
  const Scalar r = 0.5;
  DeformedCylinder dc(Vec3s(0, 0, 0), Vec3s::UnitZ(), r,
                      Vec3s(0, 0, 2), Vec3s::UnitZ(), r);
  dc.computeLocalAABB();

  Box box(2.0, 2.0, 2.0);

  Transform3s tf1;
  Transform3s tf2(Quats::Identity(), Vec3s(0, 0, 1));

  const CollisionRequest col_req(CollisionRequestFlag::CONTACT, 1);
  CollisionResult col_res;
  coal::collide(&dc, tf1, &box, tf2, col_req, col_res);
  BOOST_REQUIRE(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);
  BOOST_CHECK_NO_THROW(coal::computeContactPatch(
      &dc, tf1, &box, tf2, col_res, patch_req, patch_res));
  BOOST_CHECK(patch_res.numContactPatches() > 0);
}
