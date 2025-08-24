/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, INRIA.
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

#define BOOST_TEST_MODULE COAL_SECURITY_MARGIN
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <iostream>
#include "coal/distance.h"
#include "coal/math/transform.h"
#include "coal/collision.h"
#include "coal/collision_object.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/shape/geometric_shapes_utility.h"

#include "utility.h"

using namespace coal;
using coal::CollisionGeometryPtr_t;
using coal::CollisionObject;
using coal::CollisionRequest;
using coal::CollisionResult;
using coal::DistanceRequest;
using coal::DistanceResult;
using coal::Transform3s;
using coal::Vec3s;

#define MATH_SQUARED(x) (x * x)

template <typename Shape>
bool isApprox(const Shape &s1, const Shape &s2, const Scalar tol);

bool isApprox(const Scalar &v1, const Scalar &v2, const Scalar tol) {
  typedef Eigen::Matrix<Scalar, 1, 1> Matrix;
  Matrix m1;
  m1 << v1;
  Matrix m2;
  m2 << v2;
  return m1.isApprox(m2, tol);
}

bool isApprox(const Box &s1, const Box &s2, const Scalar tol) {
  return s1.halfSide.isApprox(s2.halfSide, tol);
}

bool isApprox(const Sphere &s1, const Sphere &s2, const Scalar tol) {
  return isApprox(s1.radius, s2.radius, tol);
}

bool isApprox(const Ellipsoid &s1, const Ellipsoid &s2, const Scalar tol) {
  return s1.radii.isApprox(s2.radii, tol);
}

bool isApprox(const Capsule &s1, const Capsule &s2, const Scalar tol) {
  return isApprox(s1.radius, s2.radius, tol) &&
         isApprox(s1.halfLength, s2.halfLength, tol);
}

bool isApprox(const Cylinder &s1, const Cylinder &s2, const Scalar tol) {
  return isApprox(s1.radius, s2.radius, tol) &&
         isApprox(s1.halfLength, s2.halfLength, tol);
}

bool isApprox(const Cone &s1, const Cone &s2, const Scalar tol) {
  return isApprox(s1.radius, s2.radius, tol) &&
         isApprox(s1.halfLength, s2.halfLength, tol);
}

bool isApprox(const TriangleP &s1, const TriangleP &s2, const Scalar tol) {
  return s1.a.isApprox(s2.a, tol) && s1.b.isApprox(s2.b, tol) &&
         s1.c.isApprox(s2.c, tol);
}

bool isApprox(const Halfspace &s1, const Halfspace &s2, const Scalar tol) {
  return isApprox(s1.d, s2.d, tol) && s1.n.isApprox(s2.n, tol);
}

template <typename Shape>
void test(const Shape &original_shape, const Scalar inflation,
          const Scalar tol = Scalar(1e-8)) {
  // Zero inflation
  {
    const Scalar inflation = 0.;
    const auto &inflation_result = original_shape.inflated(inflation);
    const Transform3s &shift = inflation_result.second;
    const Shape &inflated_shape = inflation_result.first;

    BOOST_CHECK(isApprox(original_shape, inflated_shape, tol));
    BOOST_CHECK(shift.isIdentity(tol));
  }

  // Positive inflation
  {
    const auto &inflation_result = original_shape.inflated(inflation);
    const Shape &inflated_shape = inflation_result.first;
    const Transform3s &inflation_shift = inflation_result.second;

    BOOST_CHECK(!isApprox(original_shape, inflated_shape, tol));

    const auto &deflation_result = inflated_shape.inflated(-inflation);
    const Shape &deflated_shape = deflation_result.first;
    const Transform3s &deflation_shift = deflation_result.second;

    BOOST_CHECK(isApprox(original_shape, deflated_shape, tol));
    BOOST_CHECK((inflation_shift * deflation_shift).isIdentity(tol));
  }

  // Negative inflation
  {
    const auto &inflation_result = original_shape.inflated(-inflation);
    const Shape &inflated_shape = inflation_result.first;
    const Transform3s &inflation_shift = inflation_result.second;

    BOOST_CHECK(!isApprox(original_shape, inflated_shape, tol));

    const auto &deflation_result = inflated_shape.inflated(+inflation);
    const Shape &deflated_shape = deflation_result.first;
    const Transform3s &deflation_shift = deflation_result.second;

    BOOST_CHECK(isApprox(original_shape, deflated_shape, tol));
    BOOST_CHECK((inflation_shift * deflation_shift).isIdentity(tol));
  }
}

template <typename Shape>
void test_throw(const Shape &shape, const Scalar inflation) {
  BOOST_REQUIRE_THROW(shape.inflated(inflation), std::invalid_argument);
}

template <typename Shape>
void test_no_throw(const Shape &shape, const Scalar inflation) {
  BOOST_REQUIRE_NO_THROW(shape.inflated(inflation));
}

BOOST_AUTO_TEST_CASE(test_inflate) {
  const coal::Sphere sphere(1);
  test(sphere, Scalar(0.01), Scalar(1e-8));
  test_throw(sphere, Scalar(-1.1));
  test_no_throw(sphere, 1.);

  const coal::Box box(1, 1, 1);
  test(box, Scalar(0.01), Scalar(1e-8));
  test_throw(box, Scalar(-0.6));

  const coal::Ellipsoid ellipsoid(1, 2, 3);
  test(ellipsoid, Scalar(0.01), Scalar(1e-8));
  test_throw(ellipsoid, Scalar(-1.1));

  const coal::Capsule capsule(1, 2);
  test(capsule, Scalar(0.01), Scalar(1e-8));
  test_throw(capsule, Scalar(-1.1));

  const coal::Cylinder cylinder(1, 2);
  test(cylinder, Scalar(0.01), Scalar(1e-8));
  test_throw(cylinder, Scalar(-1.1));

  const coal::Cone cone(1, 4);
  test(cone, Scalar(0.01), Scalar(1e-8));
  test_throw(cone, Scalar(-1.1));

  const coal::Halfspace halfspace(Vec3s::UnitZ(), 0);
  test(halfspace, Scalar(0.01), Scalar(1e-8));

  //  const coal::TriangleP triangle(Vec3s::UnitX(), Vec3s::UnitY(),
  //                                     Vec3s::UnitZ());
  //  test(triangle, 0.01, 1e-8);
}
