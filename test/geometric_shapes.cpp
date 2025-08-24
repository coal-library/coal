/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

/** \author Jia Pan */

#define BOOST_TEST_MODULE COAL_GEOMETRIC_SHAPES
#include <boost/test/included/unit_test.hpp>

#include "coal/narrowphase/narrowphase.h"
#include "coal/collision.h"
#include "coal/distance.h"
#include "utility.h"
#include <iostream>
#include "coal/internal/tools.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/internal/shape_shape_func.h"

using namespace coal;

Scalar extents[6] = {0, 0, 0, 10, 10, 10};

Scalar tol_gjk = Scalar(0.01);
GJKSolver solver1;
GJKSolver solver2;

int line;
#define SET_LINE line = __LINE__
#define FCL_CHECK(cond) \
  BOOST_CHECK_MESSAGE(cond, "from line " << line << ": " #cond)
#define FCL_CHECK_EQUAL(a, b)                                                \
  BOOST_CHECK_MESSAGE((a) == (b), "from line " << line << ": " #a "[" << (a) \
                                               << "] != " #b "[" << (b)      \
                                               << "].")
#define BOOST_CHECK_FALSE(p) BOOST_CHECK(!(p))

namespace coal {
std::ostream& operator<<(std::ostream& os, const ShapeBase&) {
  return os << "a_shape";
}

std::ostream& operator<<(std::ostream& os, const Box& b) {
  return os << "Box(" << 2 * b.halfSide.transpose() << ')';
}
}  // namespace coal

template <typename S1, typename S2>
void printComparisonError(const std::string& comparison_type, const S1& s1,
                          const Transform3s& tf1, const S2& s2,
                          const Transform3s& tf2,
                          const Vec3s& contact_or_normal,
                          const Vec3s& expected_contact_or_normal,
                          bool check_opposite_normal, Scalar tol) {
  std::cout << "Disagreement between " << comparison_type << " and expected_"
            << comparison_type << " for " << getNodeTypeName(s1.getNodeType())
            << " and " << getNodeTypeName(s2.getNodeType()) << ".\n"
            << "tf1.quaternion: " << tf1.getQuatRotation() << std::endl
            << "tf1.translation: " << tf1.getTranslation().transpose()
            << std::endl
            << "tf2.quaternion: " << tf2.getQuatRotation() << std::endl
            << "tf2.translation: " << tf2.getTranslation().transpose()
            << std::endl
            << comparison_type << ": " << contact_or_normal.transpose()
            << std::endl
            << "expected_" << comparison_type << ": "
            << expected_contact_or_normal.transpose();

  if (check_opposite_normal)
    std::cout << " or " << -expected_contact_or_normal.transpose();

  std::cout << std::endl
            << "difference: "
            << (contact_or_normal - expected_contact_or_normal).norm()
            << std::endl
            << "tolerance: " << tol << std::endl;
}

template <typename S1, typename S2>
void printComparisonError(const std::string& comparison_type, const S1& s1,
                          const Transform3s& tf1, const S2& s2,
                          const Transform3s& tf2, Scalar depth,
                          Scalar expected_depth, Scalar tol) {
  std::cout << "Disagreement between " << comparison_type << " and expected_"
            << comparison_type << " for " << getNodeTypeName(s1.getNodeType())
            << " and " << getNodeTypeName(s2.getNodeType()) << ".\n"
            << "tf1.quaternion: " << tf1.getQuatRotation() << std::endl
            << "tf1.translation: " << tf1.getTranslation() << std::endl
            << "tf2.quaternion: " << tf2.getQuatRotation() << std::endl
            << "tf2.translation: " << tf2.getTranslation() << std::endl
            << "depth: " << depth << std::endl
            << "expected_depth: " << expected_depth << std::endl
            << "difference: " << std::fabs(depth - expected_depth) << std::endl
            << "tolerance: " << tol << std::endl;
}

template <typename S1, typename S2>
void compareContact(const S1& s1, const Transform3s& tf1, const S2& s2,
                    const Transform3s& tf2, const Vec3s& contact,
                    Vec3s* expected_point, Scalar depth, Scalar* expected_depth,
                    const Vec3s& normal, Vec3s* expected_normal,
                    bool check_opposite_normal, Scalar tol) {
  if (expected_point) {
    bool contact_equal = isEqual(contact, *expected_point, tol);
    FCL_CHECK(contact_equal);
    if (!contact_equal)
      printComparisonError("contact", s1, tf1, s2, tf2, contact,
                           *expected_point, false, tol);
  }

  if (expected_depth) {
    bool depth_equal = std::fabs(depth - *expected_depth) < tol;
    FCL_CHECK(depth_equal);
    if (!depth_equal)
      printComparisonError("depth", s1, tf1, s2, tf2, depth, *expected_depth,
                           tol);
  }

  if (expected_normal) {
    bool normal_equal = isEqual(normal, *expected_normal, tol);

    if (!normal_equal && check_opposite_normal)
      normal_equal = isEqual(normal, -(*expected_normal), tol);

    FCL_CHECK(normal_equal);
    if (!normal_equal)
      printComparisonError("normal", s1, tf1, s2, tf2, normal, *expected_normal,
                           check_opposite_normal, tol);
  }
}

template <typename S1, typename S2>
void testShapeCollide(const S1& s1, const Transform3s& tf1, const S2& s2,
                      const Transform3s& tf2, bool expect_collision,
                      Vec3s* expected_point = NULL,
                      Scalar* expected_depth = NULL,
                      Vec3s* expected_normal = NULL,
                      bool check_opposite_normal = false,
                      Scalar tol = Scalar(1e-9)) {
  CollisionRequest request;
  CollisionResult result;

  Vec3s contact;
  Vec3s normal;  // normal direction should be from object 1 to object 2
  bool collision;
  bool check_failed = false;

  request.enable_contact = false;
  result.clear();
  collision = (collide(&s1, tf1, &s2, tf2, request, result) > 0);
  FCL_CHECK_EQUAL(collision, expect_collision);
  check_failed = check_failed || (collision != expect_collision);

  request.enable_contact = true;
  result.clear();
  collision = (collide(&s1, tf1, &s2, tf2, request, result) > 0);
  FCL_CHECK_EQUAL(collision, expect_collision);
  check_failed = check_failed || (collision != expect_collision);

  if (check_failed) {
    BOOST_TEST_MESSAGE("Failure occured between " << s1 << " and " << s2
                                                  << " at transformations\n"
                                                  << tf1 << '\n'
                                                  << tf2);
  }

  if (expect_collision) {
    FCL_CHECK_EQUAL(result.numContacts(), 1);
    if (result.numContacts() == 1) {
      Contact contact = result.getContact(0);
      compareContact(s1, tf1, s2, tf2, contact.pos, expected_point,
                     contact.penetration_depth, expected_depth, contact.normal,
                     expected_normal, check_opposite_normal, tol);
    }
  }
}

BOOST_AUTO_TEST_CASE(box_to_bvh) {
  Box shape(1, 1, 1);
  BVHModel<OBB> bvh;
  generateBVHModel(bvh, shape, Transform3s());
}

BOOST_AUTO_TEST_CASE(sphere_to_bvh) {
  Sphere shape(1);
  BVHModel<OBB> bvh;
  generateBVHModel(bvh, shape, Transform3s(), 10, 10);
  generateBVHModel(bvh, shape, Transform3s(), 50);
}

BOOST_AUTO_TEST_CASE(cylinder_to_bvh) {
  Cylinder shape(1, 1);
  BVHModel<OBB> bvh;
  generateBVHModel(bvh, shape, Transform3s(), 10, 10);
  generateBVHModel(bvh, shape, Transform3s(), 50);
}

BOOST_AUTO_TEST_CASE(cone_to_bvh) {
  Cone shape(1, 1);
  BVHModel<OBB> bvh;
  generateBVHModel(bvh, shape, Transform3s(), 10, 10);
  generateBVHModel(bvh, shape, Transform3s(), 50);
}

BOOST_AUTO_TEST_CASE(collide_spheresphere) {
  Sphere s1(20);
  Sphere s2(10);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  // Vec3s point;
  // Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(40, 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(40, 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(30, 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(30.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(30.01), 0, 0));
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(29.9), 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(29.9), 0, 0));
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s();
  normal << 1, 0, 0;  // If the centers of two sphere are at the same position,
                      // the normal is (1, 0, 0)
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform;
  normal << 1, 0, 0;  // If the centers of two sphere are at the same position,
                      // the normal is (1, 0, 0)
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-29.9), 0, 0));
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-29.9), 0, 0));
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-30.0, 0, 0));
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-30.01), 0, 0));
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-30.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);
}

bool compareContactPoints(const Vec3s& c1, const Vec3s& c2) {
  return c1[2] < c2[2];
}  // Ascending order

void testBoxBoxContactPoints(const Matrix3s& R) {
  Box s1(100, 100, 100);
  Box s2(10, 20, 30);

  // Vertices of s2
  std::vector<Vec3s> vertices(8);
  vertices[0] << 1, 1, 1;
  vertices[1] << 1, 1, -1;
  vertices[2] << 1, -1, 1;
  vertices[3] << 1, -1, -1;
  vertices[4] << -1, 1, 1;
  vertices[5] << -1, 1, -1;
  vertices[6] << -1, -1, 1;
  vertices[7] << -1, -1, -1;

  for (std::size_t i = 0; i < 8; ++i) {
    vertices[i].array() *= s2.halfSide.array();
  }

  Transform3s tf1 = Transform3s(Vec3s(0, 0, -50));
  Transform3s tf2 = Transform3s(R);

  Vec3s normal;
  Vec3s p1, p2;

  // Make sure the two boxes are colliding
  solver1.gjk_tolerance = Scalar(1e-5);
  solver1.epa_tolerance = Scalar(1e-5);
  const bool compute_penetration = true;
  Scalar distance = solver1.shapeDistance(s1, tf1, s2, tf2, compute_penetration,
                                          p1, p2, normal);
  FCL_CHECK(distance <= 0);

  // Compute global vertices
  for (std::size_t i = 0; i < 8; ++i) vertices[i] = tf2.transform(vertices[i]);

  // Sort the vertices so that the lowest vertex along z-axis comes first
  std::sort(vertices.begin(), vertices.end(), compareContactPoints);

  // The lowest vertex along z-axis should be the contact point
  FCL_CHECK(normal.isApprox(Vec3s(0, 0, 1), Scalar(1e-6)));
  Vec3s point = Scalar(0.5) * (p1 + p2);
  FCL_CHECK(vertices[0].head<2>().isApprox(point.head<2>(), Scalar(1e-6)));
  FCL_CHECK(vertices[0][2] <= point[2] && point[2] < 0);
}

BOOST_AUTO_TEST_CASE(collide_boxbox) {
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  // Vec3s point;
  // Scalar depth;
  Vec3s normal;

  Quats q;
  q = AngleAxis((Scalar)3.140 / 6, UnitZ);

  tf1 = Transform3s();
  tf2 = Transform3s();
  // TODO: Need convention for normal when the centers of two objects are at
  // same position. The current result is (1, 0, 0).
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at
  // same position. The current result is (1, 0, 0).
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(15, 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false,
                   Scalar(1e-8));

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(15.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(15.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(q);
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3s(q);
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  int numTests = 100;
  for (int i = 0; i < numTests; ++i) {
    Transform3s tf;
    generateRandomTransform(extents, tf);
    SET_LINE;
    testBoxBoxContactPoints(tf.getRotation());
  }
}

BOOST_AUTO_TEST_CASE(collide_spherebox) {
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  // Vec3s point;
  // Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  // TODO: Need convention for normal when the centers of two objects are at
  // same position. The current result is (-1, 0, 0).
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(22.50001), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(22.501), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(22.4), 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(22.4), 0, 0));
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);
}

BOOST_AUTO_TEST_CASE(distance_spherebox) {
  coal::Matrix3s rotSphere;
  rotSphere << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  coal::Vec3s trSphere(0.0, 0.0, 0.0);

  coal::Matrix3s rotBox;
  rotBox << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  coal::Vec3s trBox(0.0, 5.0, 3.0);

  coal::Sphere sphere(1);
  coal::Box box(10, 2, 10);

  coal::DistanceResult result;
  distance(&sphere, Transform3s(rotSphere, trSphere), &box,
           Transform3s(rotBox, trBox), DistanceRequest(true), result);

  Scalar eps = Eigen::NumTraits<Scalar>::epsilon();
  BOOST_CHECK_CLOSE(result.min_distance, 3., eps);
  EIGEN_VECTOR_IS_APPROX(result.nearest_points[0], Vec3s(0, 1, 0), eps);
  EIGEN_VECTOR_IS_APPROX(result.nearest_points[1], Vec3s(0, 4, 0), eps);
  EIGEN_VECTOR_IS_APPROX(result.normal, Vec3s(0, 1, 0), eps);
}

BOOST_AUTO_TEST_CASE(collide_spherecapsule) {
  Sphere s1(20);
  Capsule s2(5, 10);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  // Vec3s point;
  // Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(24.9), 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(24.9), 0, 0));
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(25, 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(24.999999), 0, 0));
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(25.1), 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(25.1), 0, 0));
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_cylindercylinder) {
  Cylinder s1(5, 15);
  Cylinder s2(5, 15);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  // Vec3s point;
  // Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(9.9), 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(9.9), 0));
  normal << 0, 1, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(9.9), 0, 0));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(9.9), 0, 0));
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(10.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(10.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_conecone) {
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  // Vec3s point;
  // Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3s();
  // z=0 is a singular points. Two normals could be returned.
  tf2 = Transform3s(Vec3s(Scalar(9.9), 0, Scalar(0.00001)));
  normal = Vec3s(2 * (s1.halfLength + s2.halfLength), 0, s1.radius + s2.radius)
               .normalized();
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform * tf1;
  tf2 = transform * tf2;
  normal = transform.getRotation() * normal;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(9.9), 0, Scalar(0.00001)));
  normal = Vec3s(2 * (s1.halfLength + s2.halfLength), 0, s1.radius + s2.radius)
               .normalized();
  normal = transform.getRotation() * normal;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, true, tol_gjk);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(10.1), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(10.001), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(10.001), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(9.9)));
  normal << 0, 0, 1;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(9.9)));
  normal = transform.getRotation() * Vec3s(0, 0, 1);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);
}

BOOST_AUTO_TEST_CASE(collide_conecylinder) {
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  // Vec3s point;
  // Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at
  // same position.
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(9.9), 0, 0));
  normal =
      Vec3s(2 * (s1.halfLength + s2.halfLength), 0, -(s1.radius + s2.radius))
          .normalized();
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(9.9), 0, 0));
  normal = transform.getRotation() * normal;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(9.9), 0, Scalar(0.1)));
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(9.9), 0, Scalar(0.1)));
  normal = transform.getRotation() * normal;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(10.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(10.01), 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(10, 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(10, 0, 0));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(9.9)));
  normal << 0, 0, 1;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(9.9)));
  normal = transform.getRotation() * Vec3s(0, 0, 1);
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(10.01)));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(10.01)));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, 10));
  normal << 0, 0, 1;
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(10.1)));
  SET_LINE;
  testShapeCollide(s1, tf1, s2, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_spheretriangle) {
  Sphere s(10);

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s normal;

  //
  // Testing collision x, y, z
  //
  {
    Vec3s t[3];
    t[0] << 20, 0, 0;
    t[1] << -20, 0, 0;
    t[2] << 0, 20, 0;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();  // identity

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(0.001)));
    normal << 0, 0, 1;
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(-0.001)));
    normal << 0, 0, -1;
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);
  }

  {
    Vec3s t[3];
    t[0] << 30, 0, 0;
    t[1] << Scalar(9.9), -20, 0;
    t[2] << Scalar(9.9), 20, 0;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(0.001)));
    normal << Scalar(9.9), 0, Scalar(0.001);
    normal.normalize();
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(-0.001)));
    normal << Scalar(9.9), 0, Scalar(-0.001);
    normal.normalize();
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);
  }

  {
    Vec3s t[3];
    t[0] << 30, 0, 0;
    t[1] << -20, 0, 0;
    t[2] << 0, 0, 20;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();

    tf_tri.setTranslation(Vec3s(0, Scalar(0.001), 0));
    normal << 0, 1, 0;
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, Scalar(-0.001), 0));
    normal << 0, -1, 0;
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);
  }

  {
    Vec3s t[3];
    t[0] << 0, 30, 0;
    t[1] << 0, -10, 0;
    t[2] << 0, 0, 20;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();

    tf_tri.setTranslation(Vec3s(Scalar(0.001), 0, 0));
    normal << 1, 0, 0;
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(Scalar(-0.001), 0, 0));
    normal << -1, 0, 0;
    testShapeCollide(s, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    SET_LINE;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);
  }

  //
  // Testing no collision x, y, z
  //
  {
    Vec3s t[3];
    t[0] << 20, 0, 0;
    t[1] << -20, 0, 0;
    t[2] << 0, 20, 0;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(10.1)));
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(-10.1)));
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, false);
  }

  {
    Vec3s t[3];
    t[0] << 20, 0, 0;
    t[1] << -20, 0, 0;
    t[2] << 0, 0, 20;
    TriangleP tri(t[0], t[1], t[2]);

    Transform3s tf_tri = Transform3s();
    tf_tri.setTranslation(Vec3s(0, Scalar(10.1), 0));
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(0, Scalar(-10.1), 0));
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, false);
  }

  {
    Vec3s t[3];
    t[0] << 0, 20, 0;
    t[1] << 0, -20, 0;
    t[2] << 0, 0, 20;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();

    tf_tri.setTranslation(Vec3s(Scalar(10.1), 0, 0));
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(Scalar(-10.1), 0, 0));
    SET_LINE;
    testShapeCollide(s, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(s, transform, tri, transform * tf_tri, false);
  }
}

BOOST_AUTO_TEST_CASE(collide_halfspacetriangle) {
  Halfspace hs(Vec3s(0, 0, 1), 0);

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s normal;
  normal = hs.n;  // with halfspaces, it's simple: normal is always
                  // the normal of the halfspace.

  {
    Vec3s t[3];
    t[0] << 20, 0, 0;
    t[1] << -20, 0, 0;
    t[2] << 0, 20, 0;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();  // identity

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(-0.001)));
    normal = hs.n;
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(1, 1, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(-1, -1, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);
  }

  {
    Vec3s t[3];
    t[0] << 30, 0, 0;
    t[1] << -20, 0, 0;
    t[2] << 0, 0, 20;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(-0.001)));
    normal = hs.n;
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(1, 1, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(-1, -1, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);
  }

  {
    Vec3s t[3];
    t[0] << 0, 30, 0;
    t[1] << 0, -10, 0;
    t[2] << 0, 0, 20;
    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(-0.001)));
    normal = hs.n;
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(1, 1, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(-1, -1, Scalar(0.001)));
    SET_LINE;
    testShapeCollide(hs, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(hs, transform, tri, transform * tf_tri, false);
  }
}

BOOST_AUTO_TEST_CASE(collide_planetriangle) {
  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s normal;

  {
    Vec3s t[3];
    t[0] << 20, 0, Scalar(0.05);
    t[1] << -20, 0, Scalar(0.05);
    t[2] << 0, 20, Scalar(-0.1);
    Plane pl(Vec3s(0, 0, 1), 0);

    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();  // identity
    normal = -pl.n;
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(0.05)));
    normal = pl.n;
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(-0.06)));
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(0, 0, Scalar(0.11)));
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, false);
  }

  {
    Vec3s t[3];
    t[0] << 30, Scalar(0.05), 0;
    t[1] << -20, Scalar(0.05), 0;
    t[2] << 0, Scalar(-0.1), 20;
    Plane pl(Vec3s(0, 1, 0), 0);

    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();  // identity
    normal = -pl.n;
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, Scalar(0.05), 0));
    normal = pl.n;
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(0, Scalar(-0.06), 0));
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(0, Scalar(0.11), 0));
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, false);
  }

  {
    Vec3s t[3];
    t[0] << Scalar(0.05), 30, 0;
    t[1] << Scalar(0.05), -10, 0;
    t[2] << Scalar(-0.1), 0, 20;
    Plane pl(Vec3s(1, 0, 0), 0);

    TriangleP tri(t[0], t[1], t[2]);
    Transform3s tf_tri = Transform3s();  // identity
    normal = -pl.n;
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(Scalar(0.05), 0, 0));
    normal = pl.n;
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, true, NULL, NULL, &normal);
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, true, NULL, NULL,
                     &normal);

    tf_tri.setTranslation(Vec3s(Scalar(-0.06), 0, 0));
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, false);

    tf_tri.setTranslation(Vec3s(Scalar(0.11), 0, 0));
    SET_LINE;
    testShapeCollide(pl, Transform3s(), tri, tf_tri, false);
    SET_LINE;
    testShapeCollide(pl, transform, tri, transform * tf_tri, false);
  }
}

BOOST_AUTO_TEST_CASE(collide_halfspacesphere) {
  Sphere s(10);
  Halfspace hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << -5, 0, 0;
  depth = -10;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(-5, 0, 0));
  depth = -10;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(5, 0, 0));
  contact << -2.5, 0, 0;
  depth = -15;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(5, 0, 0));
  contact = transform.transform(Vec3s(-2.5, 0, 0));
  depth = -15;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-5, 0, 0));
  contact << -7.5, 0, 0;
  depth = -5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-5, 0, 0));
  contact = transform.transform(Vec3s(-7.5, 0, 0));
  depth = -5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-10.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-10.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(10.1), 0, 0));
  contact << Scalar(0.05), 0, 0;
  depth = Scalar(-20.1);
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(10.1), 0, 0));
  contact = transform.transform(Vec3s(Scalar(0.05), 0, 0));
  depth = Scalar(-20.1);
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);
}

BOOST_AUTO_TEST_CASE(collide_planesphere) {
  Sphere s(10);
  Plane hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;
  Vec3s p1, p2;

  Scalar eps = Scalar(1e-6);
  tf1 = Transform3s(Vec3s(eps, 0, 0));
  tf2 = Transform3s();
  depth = -10 + eps;
  p1 << -10 + eps, 0, 0;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  normal << -1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  normal =
      transform.getRotation() * Vec3s(-1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  eps = Scalar(-1e-6);
  tf1 = Transform3s(Vec3s(eps, 0, 0));
  tf2 = Transform3s();
  depth = -10 - eps;
  p1 << 10 + eps, 0, 0;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  normal = transform.getRotation() * Vec3s(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(5, 0, 0));
  p1 << 10, 0, 0;
  p2 << 5, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5;
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(5, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -5;
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-5, 0, 0));
  p1 << -10, 0, 0;
  p2 << -5, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-5, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-10.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-10.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(10.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(10.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_halfspacebox) {
  Box s(5, 10, 20);
  Halfspace hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << -1.25, 0, 0;
  depth = -2.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(-1.25, 0, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(1.25, 0, 0));
  contact << -0.625, 0, 0;
  depth = -3.75;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(1.25, 0, 0));
  contact = transform.transform(Vec3s(-0.625, 0, 0));
  depth = -3.75;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-1.25, 0, 0));
  contact << -1.875, 0, 0;
  depth = -1.25;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-1.25, 0, 0));
  contact = transform.transform(Vec3s(-1.875, 0, 0));
  depth = -1.25;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(2.51), 0, 0));
  contact << Scalar(0.005), 0, 0;
  depth = Scalar(-5.01);
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(2.51), 0, 0));
  contact = transform.transform(Vec3s(Scalar(0.005), 0, 0));
  depth = Scalar(-5.01);
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-2.51), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-2.51), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s(transform.getRotation());
  tf2 = Transform3s();
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true);
}

BOOST_AUTO_TEST_CASE(collide_planebox) {
  Box s(5, 10, 20);
  Plane hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  Vec3s p1(2.5, 0, 0);
  Vec3s p2(0, 0, 0);
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(1.25, 0, 0));
  p2 << 1.25, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -1.25;
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(1.25, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -1.25;
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-1.25, 0, 0));
  p1 << -2.5, 0, 0;
  p2 << -1.25, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -1.25;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-1.25, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -1.25;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(2.51), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(2.51), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-2.51), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-2.51), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s(transform.getRotation());
  tf2 = Transform3s();
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true);
}

BOOST_AUTO_TEST_CASE(collide_halfspacecapsule) {
  Capsule s(5, 10);
  Halfspace hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << -2.5, 0, 0;
  depth = -5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(-2.5, 0, 0));
  depth = -5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(2.5, 0, 0));
  contact << -1.25, 0, 0;
  depth = -7.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(2.5, 0, 0));
  contact = transform.transform(Vec3s(-1.25, 0, 0));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-2.5, 0, 0));
  contact << -3.75, 0, 0;
  depth = -2.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-2.5, 0, 0));
  contact = transform.transform(Vec3s(-3.75, 0, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(5.1), 0, 0));
  contact << Scalar(0.05), 0, 0;
  depth = Scalar(-10.1);
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(5.1), 0, 0));
  contact = transform.transform(Vec3s(Scalar(0.05), 0, 0));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Halfspace(Vec3s(0, 1, 0), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, -2.5, 0;
  depth = -5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, -2.5, 0));
  depth = -5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 2.5, 0));
  contact << 0, -1.25, 0;
  depth = -7.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 2.5, 0));
  contact = transform.transform(Vec3s(0, -1.25, 0));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, -2.5, 0));
  contact << 0, -3.75, 0;
  depth = -2.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, -2.5, 0));
  contact = transform.transform(Vec3s(0, -3.75, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(5.1), 0));
  contact << 0, Scalar(0.05), 0;
  depth = Scalar(-10.1);
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(5.1), 0));
  contact = transform.transform(Vec3s(0, Scalar(0.05), 0));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Halfspace(Vec3s(0, 0, 1), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, 0, -5;
  depth = -10;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, 0, -5));
  depth = -10;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, 2.5));
  contact << 0, 0, -3.75;
  depth = -12.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, 2.5));
  contact = transform.transform(Vec3s(0, 0, -3.75));
  depth = -12.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, -2.5));
  contact << 0, 0, -6.25;
  depth = -7.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, -2.5));
  contact = transform.transform(Vec3s(0, 0, -6.25));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(10.1)));
  contact << 0, 0, Scalar(0.05);
  depth = Scalar(-20.1);
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(10.1)));
  contact = transform.transform(Vec3s(0, 0, Scalar(0.05)));
  depth = Scalar(-20.1);
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_planecapsule) {
  Capsule s(5, 10);
  Plane hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, 0, 0;
  depth = -5;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, 0x0, 0x0, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, 0, 0));
  depth = -5;
  normal = transform.getRotation() * Vec3s(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, 0x0, 0x0, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(2.5, 0, 0));
  contact << 2.5, 0, 0;
  depth = -2.5;
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(2.5, 0, 0));
  contact = transform.transform(Vec3s(2.5, 0, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-2.5, 0, 0));
  contact << -2.5, 0, 0;
  depth = -2.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-2.5, 0, 0));
  contact = transform.transform(Vec3s(-2.5, 0, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Plane(Vec3s(0, 1, 0), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, 0, 0;
  depth = -5;
  normal << 0, 1, 0;  // (0, 1, 0) or (0, -1, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, 0, 0));
  depth = -5;
  normal = transform.getRotation() * Vec3s(0, 1, 0);  // (0, 1, 0) or (0, -1, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 2.5, 0));
  contact << 0, 2.5, 0;
  depth = -2.5;
  normal << 0, 1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 2.5, 0));
  contact = transform.transform(Vec3s(0, 2.5, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, -2.5, 0));
  contact << 0, -2.5, 0;
  depth = -2.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, -2.5, 0));
  contact = transform.transform(Vec3s(0, -2.5, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Plane(Vec3s(0, 0, 1), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, 0, 0;
  depth = -10;
  normal << 0, 0, 1;  // (0, 0, 1) or (0, 0, -1)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, 0x0, 0x0, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, 0, 0));
  depth = -10;
  normal = transform.getRotation() * Vec3s(0, 0, 1);  // (0, 0, 1) or (0, 0, -1)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, 2.5));
  contact << 0, 0, 2.5;
  depth = -7.5;
  normal << 0, 0, 1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, 2.5));
  contact = transform.transform(Vec3s(0, 0, 2.5));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, 0, 1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, -2.5));
  contact << 0, 0, -2.5;
  depth = -7.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, -2.5));
  contact = transform.transform(Vec3s(0, 0, -2.5));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_halfspacecylinder) {
  Cylinder s(5, 10);
  Halfspace hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << -2.5, 0, 0;
  depth = -5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(-2.5, 0, 0));
  depth = -5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(2.5, 0, 0));
  contact << -1.25, 0, 0;
  depth = -7.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(2.5, 0, 0));
  contact = transform.transform(Vec3s(-1.25, 0, 0));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-2.5, 0, 0));
  contact << -3.75, 0, 0;
  depth = -2.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-2.5, 0, 0));
  contact = transform.transform(Vec3s(-3.75, 0, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(5.1), 0, 0));
  contact << Scalar(0.05), 0, 0;
  depth = Scalar(-10.1);
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(5.1), 0, 0));
  contact = transform.transform(Vec3s(Scalar(0.05), 0, 0));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Halfspace(Vec3s(0, 1, 0), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, -2.5, 0;
  depth = -5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, -2.5, 0));
  depth = -5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 2.5, 0));
  contact << 0, -1.25, 0;
  depth = -7.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 2.5, 0));
  contact = transform.transform(Vec3s(0, -1.25, 0));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, -2.5, 0));
  contact << 0, -3.75, 0;
  depth = -2.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, -2.5, 0));
  contact = transform.transform(Vec3s(0, -3.75, 0));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(5.1), 0));
  contact << 0, Scalar(0.05), 0;
  depth = Scalar(-10.1);
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(5.1), 0));
  contact = transform.transform(Vec3s(0, Scalar(0.05), 0));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Halfspace(Vec3s(0, 0, 1), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, 0, -2.5;
  depth = -5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, 0, -2.5));
  depth = -5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, 2.5));
  contact << 0, 0, -1.25;
  depth = -7.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, 2.5));
  contact = transform.transform(Vec3s(0, 0, -1.25));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, -2.5));
  contact << 0, 0, -3.75;
  depth = -2.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, -2.5));
  contact = transform.transform(Vec3s(0, 0, -3.75));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(5.1)));
  contact << 0, 0, Scalar(0.05);
  depth = Scalar(-10.1);
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(5.1)));
  contact = transform.transform(Vec3s(0, 0, Scalar(0.05)));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(-5.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(-5.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_planecylinder) {
  Cylinder s(5, 10);
  Plane hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;
  Vec3s p1, p2;

  Scalar eps = Scalar(1e-6);
  tf1 = Transform3s(Vec3s(eps, 0, 0));
  tf2 = Transform3s();
  p1 << -5 + eps, 0, 0;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 + eps;
  normal << -1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 + eps;
  normal =
      transform.getRotation() * Vec3s(-1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  eps = Scalar(-1e-6);
  tf1 = Transform3s(Vec3s(eps, 0, 0));
  tf2 = Transform3s();
  p1 << 5 + eps, 0, 0;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 - eps;
  normal << -1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 - eps;
  normal =
      transform.getRotation() * Vec3s(-1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(2.5, 0, 0));
  p1 << 5, 0, 0;
  p2 << 2.5, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(2.5, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-2.5, 0, 0));
  p1 << -5, 0, 0;
  p2 << -2.5, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-2.5, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Plane(Vec3s(0, 1, 0), 0);

  eps = Scalar(1e-6);
  tf1 = Transform3s(Vec3s(0, eps, 0));
  tf2 = Transform3s();
  p1 << 0, -5 + eps, 0;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 + eps;
  normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 + eps;
  normal = transform.getRotation() * Vec3s(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  eps = Scalar(-1e-6);
  tf1 = Transform3s(Vec3s(0, eps, 0));
  tf2 = Transform3s();
  p1 << 0, 5 + eps, 0;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 - eps;
  normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 - eps;
  normal = transform.getRotation() * Vec3s(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 2.5, 0));
  p1 << 0, 5, 0;
  p2 << 0, 2.5, 0;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, 1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 2.5, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, -2.5, 0));
  p1 << 0, -5, 0;
  p2 << 0, -2.5, 0;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, -2.5, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Plane(Vec3s(0, 0, 1), 0);

  eps = Scalar(1e-6);
  tf1 = Transform3s(Vec3s(0, 0, eps));
  tf2 = Transform3s();
  p1 << 0, 0, -5 + eps;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 + eps;
  normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 + eps;
  normal = transform.getRotation() * Vec3s(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  eps = Scalar(-1e-6);
  tf1 = Transform3s(Vec3s(0, 0, eps));
  tf2 = Transform3s();
  p1 << 0, 0, 5 + eps;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 - eps;
  normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 - eps;
  normal = transform.getRotation() * Vec3s(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, 2.5));
  p1 << 0, 0, 5;
  p2 << 0, 0, 2.5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, 0, 1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, 2.5));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 0, 1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, -2.5));
  p1 << 0, 0, -5.;
  p2 << 0, 0, -2.5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, -2.5));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_halfspacecone) {
  Cone s(5, 10);
  Halfspace hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << -2.5, 0, -5;
  depth = -5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(-2.5, 0, -5));
  depth = -5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(2.5, 0, 0));
  contact << -1.25, 0, -5;
  depth = -7.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(2.5, 0, 0));
  contact = transform.transform(Vec3s(-1.25, 0, -5));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-2.5, 0, 0));
  contact << -3.75, 0, -5;
  depth = -2.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-2.5, 0, 0));
  contact = transform.transform(Vec3s(-3.75, 0, -5));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(5.1), 0, 0));
  contact << Scalar(0.05), 0, -5;
  depth = Scalar(-10.1);
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(5.1), 0, 0));
  contact = transform.transform(Vec3s(Scalar(0.05), 0, -5));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Halfspace(Vec3s(0, 1, 0), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, -2.5, -5;
  depth = -5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, -2.5, -5));
  depth = -5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 2.5, 0));
  contact << 0, -1.25, -5;
  depth = -7.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 2.5, 0));
  contact = transform.transform(Vec3s(0, -1.25, -5));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, -2.5, 0));
  contact << 0, -3.75, -5;
  depth = -2.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, -2.5, 0));
  contact = transform.transform(Vec3s(0, -3.75, -5));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(5.1), 0));
  contact << 0, Scalar(0.05), -5;
  depth = Scalar(-10.1);
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(5.1), 0));
  contact = transform.transform(Vec3s(0, Scalar(0.05), -5));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Halfspace(Vec3s(0, 0, 1), 0);

  tf1 = Transform3s();
  tf2 = Transform3s();
  contact << 0, 0, -2.5;
  depth = -5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3s(0, 0, -2.5));
  depth = -5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, 2.5));
  contact << 0, 0, -1.25;
  depth = -7.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, 2.5));
  contact = transform.transform(Vec3s(0, 0, -1.25));
  depth = -7.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, -2.5));
  contact << 0, 0, -3.75;
  depth = -2.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, -2.5));
  contact = transform.transform(Vec3s(0, 0, -3.75));
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(5.1)));
  contact << 0, 0, Scalar(0.05);
  depth = Scalar(-10.1);
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(5.1)));
  contact = transform.transform(Vec3s(0, 0, Scalar(0.05)));
  depth = Scalar(-10.1);
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(-5.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(-5.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_planecone) {
  Cone s(5, 10);
  Plane hs(Vec3s(1, 0, 0), 0);

  Transform3s tf1;
  Transform3s tf2;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Vec3s contact;
  Scalar depth;
  Vec3s normal;
  Vec3s p1, p2;

  Scalar eps = Scalar(1e-6);
  tf1 = Transform3s(Vec3s(eps, 0, 0));
  tf2 = Transform3s();
  p1 << -5 + eps, 0, -5;
  p2 << 0, 0, -5;
  contact << (p1 + p2) / 2;
  depth = -5 + eps;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 + eps;
  normal =
      transform.getRotation() * Vec3s(-1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  eps = Scalar(-1e-6);
  tf1 = Transform3s(Vec3s(eps, 0, 0));
  tf2 = Transform3s();
  p1 << 5 + eps, 0, -5;
  p2 << 0, 0, -5;
  contact << (p1 + p2) / 2;
  depth = -5 - eps;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 - eps;
  normal =
      transform.getRotation() * Vec3s(-1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(2.5, 0, 0));
  p1 << 5, 0, -5;
  p2 << 2.5, 0, -5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(2.5, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(-2.5, 0, 0));
  p1 << -5, 0, -5;
  p2 << -2.5, 0, -5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << -1, 0, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(-2.5, 0, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(-1, 0, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(Scalar(-5.1), 0, 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Plane(Vec3s(0, 1, 0), 0);

  eps = Scalar(1e-6);
  tf1 = Transform3s(Vec3s(0, eps, 0));
  tf2 = Transform3s();
  p1 << 0, -5 + eps, -5;
  p2 << 0, 0, -5;
  contact << (p1 + p2) / 2;
  depth = -5 + eps;
  normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 + eps;
  normal = transform.getRotation() * Vec3s(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  eps = Scalar(-1e-6);
  tf1 = Transform3s(Vec3s(0, eps, 0));
  tf2 = Transform3s();
  p1 << 0, 5 + eps, -5;
  p2 << 0, 0, -5;
  contact << (p1 + p2) / 2;
  depth = -5 - eps;
  normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 - eps;
  normal = transform.getRotation() * Vec3s(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 2.5, 0));
  p1 << 0, 5, -5;
  p2 << 0, 2.5, -5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, 1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 2.5, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, -2.5, 0));
  p1 << 0, -5, -5;
  p2 << 0, -2.5, -5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, -1, 0;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, -2.5, 0));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, -1, 0);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, Scalar(-5.1), 0));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  hs = Plane(Vec3s(0, 0, 1), 0);

  eps = Scalar(1e-6);
  tf1 = Transform3s(Vec3s(0, 0, eps));
  tf2 = Transform3s();
  p1 << 0, 0, -5 + eps;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 + eps;
  normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 + eps;
  normal = transform.getRotation() * Vec3s(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  eps = Scalar(-1e-6);
  tf1 = Transform3s(Vec3s(0, 0, eps));
  tf2 = Transform3s();
  p1 << 0, 0, 5 + eps;
  p2 << 0, 0, 0;
  contact << (p1 + p2) / 2;
  depth = -5 - eps;
  normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform * tf1;
  tf2 = transform;
  contact = transform.transform((p1 + p2) / 2);
  depth = -5 - eps;
  normal = transform.getRotation() * Vec3s(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, 2.5));
  p1 << 0, 0, 5;
  p2 << 0, 0, 2.5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, 0, 1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, 2.5));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 0, 1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, -2.5));
  p1 << 0, 0, -5;
  p2 << 0, 0, -2.5;
  contact << (p1 + p2) / 2;
  depth = -2.5;
  normal << 0, 0, -1;
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, -2.5));
  contact = transform.transform((p1 + p2) / 2);
  depth = -2.5;
  normal = transform.getRotation() * Vec3s(0, 0, -1);
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = Transform3s();
  tf2 = Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3s(Vec3s(0, 0, Scalar(-10.1)));
  SET_LINE;
  testShapeCollide(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(collide_planeplane) {
  Transform3s tf1;
  Transform3s tf2;

  Vec3s normal;
  Vec3s contact;
  Scalar distance;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset = Scalar(3.14);
    Plane plane1(n, offset);
    Plane plane2(n, offset);

    tf1.setIdentity();
    tf2.setIdentity();
    normal = n;
    contact = plane1.n * plane1.d;
    distance = 0.;
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, true, &contact, &distance,
                     &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    contact =
        transform.getRotation() * plane1.n *
        (plane1.d +
         (transform.getRotation() * plane1.n).dot(transform.getTranslation()));
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, true, &contact, &distance,
                     &normal);
  }

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset1 = Scalar(3.14);
    Scalar offset2 = offset1 + Scalar(1.19841);
    Plane plane1(n, offset1);
    Plane plane2(n, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, false);

    tf1 = transform;
    tf2 = transform;
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, false);
  }

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset1 = Scalar(3.14);
    Scalar offset2 = offset1 - Scalar(1.19841);
    Plane plane1(n, offset1);
    Plane plane2(n, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, false);

    tf1 = transform;
    tf2 = transform;
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, false);
  }

  {
    Vec3s n1(1, 0, 0);
    Scalar offset1 = Scalar(3.14);
    Plane plane1(n1, offset1);
    Vec3s n2(0, 0, 1);
    Scalar offset2 = Scalar(-2.13);
    Plane plane2(n2, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal << 0, -1, 0;
    contact << offset1, 0, offset2;
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, true, &contact, NULL, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, true, NULL, NULL, &normal);
  }

  {
    Vec3s n1(1, 0, 0);
    Scalar offset1 = Scalar(3.14);
    Plane plane1(n1, offset1);
    Vec3s n2(1, 1, 1);
    Scalar offset2 = Scalar(-2.13);
    Plane plane2(n2, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal << 0, Scalar(-0.5774), Scalar(0.5774);
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, true, NULL, NULL, &normal, false,
                     Scalar(1e-3));

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(plane1, tf1, plane2, tf2, true, NULL, NULL, &normal, false,
                     Scalar(1e-3));
  }
}

BOOST_AUTO_TEST_CASE(collide_halfspacehalfspace) {
  Transform3s tf1;
  Transform3s tf2;

  Vec3s normal;
  Vec3s contact;
  Scalar distance;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset = Scalar(3.14);
    Halfspace hf1(n, offset);
    Halfspace hf2(n, offset);

    tf1.setIdentity();
    tf2.setIdentity();
    normal = n;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal);
  }

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset1 = Scalar(3.14);
    Scalar offset2 = offset1 + Scalar(1.19841);
    Halfspace hf1(n, offset1);
    Halfspace hf2(n, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal = n;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal);
  }

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset1 = Scalar(3.14);
    Scalar offset2 = offset1 - Scalar(1.19841);
    Halfspace hf1(n, offset1);
    Halfspace hf2(-n, -offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal = n;
    distance = offset2 - offset1;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, &distance, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, &distance, &normal);
  }

  {
    Vec3s n1(1, 0, 0);
    Scalar offset1 = Scalar(3.14);
    Halfspace hf1(n1, offset1);
    Vec3s n2(0, 0, 1);
    Scalar offset2 = Scalar(-2.13);
    Halfspace hf2(n2, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal << 0, -1, 0;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal);
  }

  {
    Vec3s n1(1, 0, 0);
    Scalar offset1 = Scalar(3.14);
    Halfspace hf1(n1, offset1);
    Vec3s n2(1, 1, 1);
    Scalar offset2 = Scalar(-2.13);
    Halfspace hf2(n2, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal << 0, Scalar(-0.5774), Scalar(0.5774);
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal, false,
                     Scalar(1e-3));

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf1, tf1, hf2, tf2, true, NULL, NULL, &normal, false,
                     Scalar(1e-3));
  }
}

BOOST_AUTO_TEST_CASE(collide_halfspaceplane) {
  Transform3s tf1;
  Transform3s tf2;

  Vec3s normal;
  Vec3s contact;
  Scalar distance;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset = Scalar(3.14);
    Halfspace hf(n, offset);
    Plane plane(n, offset);

    tf1.setIdentity();
    tf2.setIdentity();
    normal = n;
    distance = 0;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, &distance, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, &distance, &normal);
  }

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset1 = Scalar(3.14);
    Scalar offset2 = offset1 + Scalar(1.19841);
    Halfspace hf(n, offset1);
    Plane plane(n, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal = n;
    distance = offset2 - offset1;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, false);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, false);
  }

  {
    Vec3s n = Vec3s::Random().normalized();
    Scalar offset1 = Scalar(3.14);
    Scalar offset2 = offset1 - Scalar(1.19841);
    Halfspace hf(n, offset1);
    Plane plane(n, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal = n;
    distance = offset2 - offset1;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, &distance, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, &distance, &normal);
  }

  {
    Vec3s n1(1, 0, 0);
    Scalar offset1 = Scalar(3.14);
    Halfspace hf(n1, offset1);
    Vec3s n2(0, 0, 1);
    Scalar offset2 = Scalar(-2.13);
    Plane plane(n2, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal << 0, -1, 0;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, NULL, &normal);

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, NULL, &normal);
  }

  {
    Vec3s n1(1, 0, 0);
    Scalar offset1 = Scalar(3.14);
    Halfspace hf(n1, offset1);
    Vec3s n2(1, 1, 1);
    Scalar offset2 = Scalar(-2.13);
    Plane plane(n2, offset2);

    tf1.setIdentity();
    tf2.setIdentity();
    normal << 0, Scalar(-0.5774), Scalar(0.5774);
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, NULL, &normal, false,
                     Scalar(1e-3));

    tf1 = transform;
    tf2 = transform;
    normal = transform.getRotation() * normal;
    SET_LINE;
    testShapeCollide(hf, tf1, plane, tf2, true, NULL, NULL, &normal, false,
                     Scalar(1e-3));
  }
}

BOOST_AUTO_TEST_CASE(GJKSolver_shapeDistance_spheresphere) {
  Sphere s1(20);
  Sphere s2(10);
  Vec3s closest_p1, closest_p2, normal;
  bool compute_penetration = true;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Scalar dist = -1;

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(40, 0, 0)), compute_penetration,
      closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(Scalar(30.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(Scalar(29.9), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);

  dist = solver1.shapeDistance(s1, Transform3s(Vec3s(40, 0, 0)), s2,
                               Transform3s(), compute_penetration, closest_p1,
                               closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);

  dist = solver1.shapeDistance(s1, Transform3s(Vec3s(Scalar(30.1), 0, 0)), s2,
                               Transform3s(), compute_penetration, closest_p1,
                               closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(s1, Transform3s(Vec3s(Scalar(29.9), 0, 0)), s2,
                               Transform3s(), compute_penetration, closest_p1,
                               closest_p2, normal);
  BOOST_CHECK(dist < 0);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(40, 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(Scalar(30.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(Scalar(29.9), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist < 0);

  dist = solver1.shapeDistance(s1, transform * Transform3s(Vec3s(40, 0, 0)), s2,
                               transform, compute_penetration, closest_p1,
                               closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform * Transform3s(Vec3s(Scalar(30.1), 0, 0)), s2, transform,
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform * Transform3s(Vec3s(Scalar(29.9), 0, 0)), s2, transform,
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist < 0);
}

BOOST_AUTO_TEST_CASE(GJKSolver_shapeDistance_boxbox) {
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);
  Vec3s closest_p1, closest_p2, normal;
  bool compute_penetration = true;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Scalar dist;

  dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                               compute_penetration, closest_p1, closest_p2,
                               normal);
  BOOST_CHECK(dist <= 0);

  dist =
      solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                            closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);

  dist = solver1.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(Scalar(20.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);

  dist = solver1.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(0, Scalar(20.2), 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.2) < 0.001);

  dist = solver1.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(Scalar(10.1), Scalar(10.1), 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1 * 1.414) < 0.001);

  dist = solver2.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver2.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(Scalar(20.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);

  dist = solver2.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(0, Scalar(20.1), 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);

  dist = solver2.shapeDistance(
      s2, Transform3s(), s2, Transform3s(Vec3s(Scalar(10.1), Scalar(10.1), 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1 * 1.414) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(Scalar(15.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(20, 0, 0)), compute_penetration,
      closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 5) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(20, 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
}

BOOST_AUTO_TEST_CASE(GJKSolver_shapeDistance_cylinderbox) {
  Cylinder s1(Scalar(0.029), Scalar(0.1));
  Box s2(Scalar(1.6), Scalar(0.6), Scalar(0.025));

  using Quat = Eigen::Quaternion<Scalar>;
  Transform3s tf1(
      Quat(Scalar(0.5279170511703305), Scalar(-0.50981118132505521),
           Scalar(-0.67596178682051911), Scalar(0.0668715876735793)),
      Vec3s(Scalar(0.041218354748013122), Scalar(1.2022554710435607),
            Scalar(0.77338855025700015)));

  Transform3s tf2(
      Quat(Scalar(0.70738826916719977), 0, 0, Scalar(0.70682518110536596)),
      Vec3s(Scalar(-0.29936284351096382), Scalar(0.80023864435868775),
            Scalar(0.71750000000000003)));

  GJKSolver solver;
  Vec3s p1, p2, normal;
  bool compute_penetration = true;
  solver.shapeDistance(s1, tf1, s2, tf2, compute_penetration, p1, p2, normal);
  // If objects are not colliding, p2 should be outside the cylinder and
  // p1 should be outside the box
  Vec3s p2Loc(tf1.inverse().transform(p2));
  bool p2_in_cylinder((fabs(p2Loc[2]) <= s1.halfLength) &&
                      (p2Loc[0] * p2Loc[0] + p2Loc[1] * p2Loc[1] <= s1.radius));
  Vec3s p1Loc(tf2.inverse().transform(p1));
  bool p1_in_box = (p1Loc.array().abs() <= s2.halfSide.array()).all();
  std::cout << "p2 in cylinder = (" << p2Loc.transpose() << ")" << std::endl;
  std::cout << "p1 in box = (" << p1Loc.transpose() << ")" << std::endl;

  BOOST_CHECK((!p2_in_cylinder && !p1_in_box) || (p2_in_cylinder && p1_in_box));

  solver.shapeDistance(s2, tf2, s1, tf1, compute_penetration, p2, p1, normal);
  // If objects are not colliding, p2 should be outside the cylinder and
  // p1 should be outside the box

  p2Loc = tf1.inverse().transform(p2);
  p2_in_cylinder = (fabs(p2Loc[2]) <= s1.halfLength) &&
                   (p2Loc[0] * p2Loc[0] + p2Loc[1] * p2Loc[1] <= s1.radius);
  p1Loc = tf2.inverse().transform(p1);
  p1_in_box = (p1Loc.array().abs() <= s2.halfSide.array()).all();

  std::cout << "p2 in cylinder = (" << p2Loc.transpose() << ")" << std::endl;
  std::cout << "p1 in box = (" << p1.transpose() << ")" << std::endl;

  BOOST_CHECK((!p2_in_cylinder && !p1_in_box) || (p2_in_cylinder && p1_in_box));

  s1 = Cylinder(Scalar(0.06), Scalar(0.1));
  tf1.setTranslation(Vec3s(Scalar(-0.66734052046473924),
                           Scalar(0.22219183277457269),
                           Scalar(0.76825248755616293)));
  using Quat = Eigen::Quaternion<Scalar>;
  tf1.setQuatRotation(
      Quat(Scalar(0.52613359459338371), Scalar(0.32189408354839893),
           Scalar(0.70415587451837913), Scalar(-0.35175580165512249)));
  solver.shapeDistance(s1, tf1, s2, tf2, compute_penetration, p1, p2, normal);
}

BOOST_AUTO_TEST_CASE(GJKSolver_shapeDistance_boxsphere) {
  Sphere s1(20);
  Box s2(5, 5, 5);
  Vec3s closest_p1, closest_p2, normal;
  bool compute_penetration = true;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Scalar dist;

  int N = 10;
  for (int i = 0; i < N + 1; ++i) {
    Scalar dbox = Scalar(0.0001) + (s1.radius + s2.halfSide(0)) * Scalar(i) *
                                       4 / (3 * Scalar(N));
    dist = solver1.shapeDistance(s1, Transform3s(Vec3s(dbox, 0., 0.)), s2,
                                 Transform3s(), compute_penetration, closest_p1,
                                 closest_p2, normal);
    BOOST_CHECK_CLOSE(dist, (dbox - s1.radius - s2.halfSide(0)), Scalar(1e-6));
    EIGEN_VECTOR_IS_APPROX(normal, -Vec3s(1, 0, 0), Scalar(1e-6));

    dist =
        solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                              closest_p1, closest_p2, normal);
    dist = solver1.shapeDistance(
        s1, transform * Transform3s(Vec3s(dbox, 0., 0.)), s2, transform,
        compute_penetration, closest_p1, closest_p2, normal);
    BOOST_CHECK_CLOSE(dist, (dbox - s1.radius - s2.halfSide(0)), Scalar(1e-6));
    EIGEN_VECTOR_IS_APPROX(normal, -transform.getRotation().col(0),
                           Scalar(1e-6));
  }

  dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                               compute_penetration, closest_p1, closest_p2,
                               normal);
  BOOST_CHECK(dist <= 0);

  dist =
      solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                            closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(Scalar(22.6), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(Scalar(22.6), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(40, 0, 0)), compute_penetration,
      closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(40, 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
}

BOOST_AUTO_TEST_CASE(GJKSolver_shapeDistance_cylindercylinder) {
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);
  Vec3s closest_p1, closest_p2, normal;
  bool compute_penetration = true;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Scalar dist;

  {
    // The following situations corresponds to the case where the two cylinders
    // are exactly superposed. This is the worst case for EPA which will take
    // forever to converge with default parameters.
    dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                                 compute_penetration, closest_p1, closest_p2,
                                 normal);
    BOOST_CHECK(dist <= 0);

    dist =
        solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                              closest_p1, closest_p2, normal);
    BOOST_CHECK(dist <= 0);

    // To handle the superposing case, we have to decrease the tolerance of EPA
    // and allow it to work with more vertices and faces.
    Scalar epa_tolerance_backup = solver1.epa_tolerance;
    size_t epa_max_iterations_backup = solver1.epa_max_iterations;
    solver1.epa_tolerance = Scalar(1e-2);
    solver1.epa_max_iterations = 1000;

    dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                                 compute_penetration, closest_p1, closest_p2,
                                 normal);
    BOOST_CHECK(dist <= 0);

    dist =
        solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                              closest_p1, closest_p2, normal);
    BOOST_CHECK(dist <= 0);

    // We restore the original values of the EPA parameters
    solver1.epa_tolerance = epa_tolerance_backup;
    solver1.epa_max_iterations = epa_max_iterations_backup;
  }

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(40, 0, 0)), compute_penetration,
      closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(40, 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
}

BOOST_AUTO_TEST_CASE(GJKSolver_shapeDistance_conecone) {
  Cone s1(5, 10);
  Cone s2(5, 10);
  Vec3s closest_p1, closest_p2, normal;
  bool compute_penetration = true;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Scalar dist;

  {
    // The following situations corresponds to the case where the two cones
    // are exactly superposed. This is the worst case for EPA which will take
    // forever to converge with default parameters.
    dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                                 compute_penetration, closest_p1, closest_p2,
                                 normal);
    BOOST_CHECK(dist <= 0);

    dist =
        solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                              closest_p1, closest_p2, normal);
    BOOST_CHECK(dist <= 0);

    // To handle the superposing case, we have to decrease the tolerance of EPA
    // and allow it to work with more vertices and faces.
    Scalar epa_tolerance_backup = solver1.epa_tolerance;
    size_t epa_max_iterations_backup = solver1.epa_max_iterations;
    solver1.epa_tolerance = Scalar(1e-2);
    solver1.epa_max_iterations = 1000;

    dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                                 compute_penetration, closest_p1, closest_p2,
                                 normal);
    BOOST_CHECK(dist <= 0);

    dist =
        solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                              closest_p1, closest_p2, normal);
    BOOST_CHECK(dist <= 0);

    // We restore the original values of the EPA parameters
    solver1.epa_tolerance = epa_tolerance_backup;
    solver1.epa_max_iterations = epa_max_iterations_backup;
  }

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(0, 0, 40)), compute_penetration,
      closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 1);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(0, 0, 40)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 1);
}

BOOST_AUTO_TEST_CASE(GJKSolver_shapeDistance_conecylinder) {
  Cylinder s1(5, 10);
  Cone s2(5, 10);
  Vec3s closest_p1, closest_p2, normal;
  bool compute_penetration = true;

  Transform3s transform;
  generateRandomTransform(extents, transform);

  Scalar dist;

  {
    // The following situations corresponds to the case where the two cones
    // are exactly superposed. This is the worst case for EPA which will take
    // forever to converge with default parameters.
    dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                                 compute_penetration, closest_p1, closest_p2,
                                 normal);
    BOOST_CHECK(dist <= 0);

    dist =
        solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                              closest_p1, closest_p2, normal);
    BOOST_CHECK(dist <= 0);

    // To handle the superposing case, we have to decrease the tolerance of EPA
    // and allow it to work with more vertices and faces.
    Scalar epa_tolerance_backup = solver1.epa_tolerance;
    size_t epa_max_iterations_backup = solver1.epa_max_iterations;
    solver1.epa_tolerance = Scalar(1e-2);
    solver1.epa_max_iterations = 1000;

    dist = solver1.shapeDistance(s1, Transform3s(), s2, Transform3s(),
                                 compute_penetration, closest_p1, closest_p2,
                                 normal);
    BOOST_CHECK(dist <= 0);

    dist =
        solver1.shapeDistance(s1, transform, s2, transform, compute_penetration,
                              closest_p1, closest_p2, normal);
    BOOST_CHECK(dist <= 0);

    // We restore the original values of the EPA parameters
    solver1.epa_tolerance = epa_tolerance_backup;
    solver1.epa_max_iterations = epa_max_iterations_backup;
  }

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(Scalar(10.1), 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.02);

  dist = solver1.shapeDistance(
      s1, Transform3s(), s2, Transform3s(Vec3s(40, 0, 0)), compute_penetration,
      closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.01);

  dist = solver1.shapeDistance(
      s1, transform, s2, transform * Transform3s(Vec3s(40, 0, 0)),
      compute_penetration, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.1);
}

template <typename S1, typename S2>
void testReversibleShapeDistance(const S1& s1, const S2& s2, Scalar distance) {
  const Scalar half = Scalar(0.5);
  Transform3s tf1(Vec3s(-half * distance, 0.0, 0.0));
  Transform3s tf2(Vec3s(+half * distance, 0.0, 0.0));

  Scalar distA;
  Scalar distB;
  Vec3s p1A;
  Vec3s p1B;
  Vec3s p2A;
  Vec3s p2B;
  Vec3s normalA, normalB;
  bool compute_penetration = true;

  const Scalar tol = Scalar(1e-6);

  distA = solver1.shapeDistance(s1, tf1, s2, tf2, compute_penetration, p1A, p2A,
                                normalA);
  distB = solver1.shapeDistance(s2, tf2, s1, tf1, compute_penetration, p1B, p2B,
                                normalB);

  assert((distA <= 0 && distB <= 0) || (distA > 0 && distB > 0));
  BOOST_CHECK_CLOSE(distA, distB, tol);  // distances should be same
  BOOST_CHECK(
      isEqual(p1A, p2B, tol));  // closest points should in reverse order
  BOOST_CHECK(isEqual(p2A, p1B, tol));

  distA = solver2.shapeDistance(s1, tf1, s2, tf2, compute_penetration, p1A, p2A,
                                normalA);
  distB = solver2.shapeDistance(s2, tf2, s1, tf1, compute_penetration, p1B, p2B,
                                normalB);

  assert((distA <= 0 && distB <= 0) || (distA > 0 && distB > 0));
  BOOST_CHECK(solver1.gjk.status == solver2.gjk.status);
  BOOST_CHECK(solver1.epa.status == solver2.epa.status);
  BOOST_CHECK_CLOSE(distA, distB, tol);
  BOOST_CHECK(isEqual(p1A, p2B, tol));
  BOOST_CHECK(isEqual(p2A, p1B, tol));
}

BOOST_AUTO_TEST_CASE(reversibleShapeDistance_allshapes) {
  // This test check whether a shape distance algorithm is called for the
  // reverse case as well. For example, if FCL has sphere-capsule distance
  // algorithm, then this algorithm should be called for capsule-sphere case.

  // Prepare all kinds of primitive shapes (7) -- box, sphere, capsule, cone,
  // cylinder, plane, halfspace
  Box box(10, 10, 10);
  Sphere sphere(5);
  Capsule capsule(5, 10);
  Cone cone(5, 10);
  Cylinder cylinder(5, 10);
  Plane plane(Vec3s(0, 0, 0), 0.0);
  Halfspace halfspace(Vec3s(0, 0, 0), 0.0);

  // Use sufficiently long distance so that all the primitive shapes CANNOT
  // intersect
  Scalar distance = 15.0;

  // If new shape distance algorithm is added for two distinct primitive
  // shapes, uncomment associated lines. For example, box-sphere intersection
  // algorithm is added, then uncomment box-sphere.

  //  testReversibleShapeDistance(box, sphere, distance);
  //  testReversibleShapeDistance(box, capsule, distance);
  //  testReversibleShapeDistance(box, cone, distance);
  //  testReversibleShapeDistance(box, cylinder, distance);
  //  testReversibleShapeDistance(box, plane, distance);
  //  testReversibleShapeDistance(box, halfspace, distance);

  SET_LINE;
  testReversibleShapeDistance(sphere, capsule, distance);
  //  testReversibleShapeDistance(sphere, cone, distance);
  //  testReversibleShapeDistance(sphere, cylinder, distance);
  //  testReversibleShapeDistance(sphere, plane, distance);
  //  testReversibleShapeDistance(sphere, halfspace, distance);

  //  testReversibleShapeDistance(capsule, cone, distance);
  //  testReversibleShapeDistance(capsule, cylinder, distance);
  //  testReversibleShapeDistance(capsule, plane, distance);
  //  testReversibleShapeDistance(capsule, halfspace, distance);

  //  testReversibleShapeDistance(cone, cylinder, distance);
  //  testReversibleShapeDistance(cone, plane, distance);
  //  testReversibleShapeDistance(cone, halfspace, distance);

  //  testReversibleShapeDistance(cylinder, plane, distance);
  //  testReversibleShapeDistance(cylinder, halfspace, distance);

  //  testReversibleShapeDistance(plane, halfspace, distance);
}
