/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

#define _USE_MATH_DEFINES
#include <cmath>

#define BOOST_TEST_MODULE COAL_MATH
#include <boost/test/included/unit_test.hpp>

#include "coal/data_types.h"
#include "coal/math/transform.h"

#include "coal/internal/intersect.h"
#include "coal/internal/tools.h"

using namespace coal;

BOOST_AUTO_TEST_CASE(vec_test_eigen_vec64) {
  Vec3s v1(1, 2, 3);
  BOOST_CHECK(v1[0] == Scalar(1));
  BOOST_CHECK(v1[1] == Scalar(2));
  BOOST_CHECK(v1[2] == Scalar(3));

  Vec3s v2 = v1;
  Vec3s v3(Scalar(3.3), Scalar(4.3), Scalar(5.3));
  v1 += v3;
  BOOST_CHECK(isEqual(v1, v2 + v3));
  v1 -= v3;
  BOOST_CHECK(isEqual(v1, v2));
  v1 -= v3;
  BOOST_CHECK(isEqual(v1, v2 - v3));
  v1 += v3;

  v1.array() *= v3.array();
  BOOST_CHECK(isEqual(v1, v2.cwiseProduct(v3)));
  v1.array() /= v3.array();
  BOOST_CHECK(isEqual(v1, v2));
  v1.array() /= v3.array();
  BOOST_CHECK(isEqual(v1, v2.cwiseQuotient(v3)));
  v1.array() *= v3.array();

  v1 *= 2.0;
  BOOST_CHECK(isEqual(v1, v2 * 2.0));
  v1 /= 2.0;
  BOOST_CHECK(isEqual(v1, v2));
  v1 /= 2.0;
  BOOST_CHECK(isEqual(v1, v2 / 2.0));
  v1 *= 2.0;

  v1.array() += 2.0;
  BOOST_CHECK(isEqual(v1, (v2.array() + 2.0).matrix()));
  v1.array() -= 2.0;
  BOOST_CHECK(isEqual(v1, v2));
  v1.array() -= 2.0;
  BOOST_CHECK(isEqual(v1, (v2.array() - 2.0).matrix()));
  v1.array() += 2.0;

  BOOST_CHECK(isEqual((-Vec3s(1.0, 2.0, 3.0)), Vec3s(-1.0, -2.0, -3.0)));

  v1 = Vec3s(1.0, 2.0, 3.0);
  v2 = Vec3s(3.0, 4.0, 5.0);
  BOOST_CHECK(isEqual((v1.cross(v2)), Vec3s(-2.0, 4.0, -2.0)));
  BOOST_CHECK(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3s(3.0, 4.0, 5.0);
  BOOST_CHECK(std::abs(v1.squaredNorm() - 50) < 1e-5);
  BOOST_CHECK(std::abs(v1.norm() - sqrt(50)) < 1e-5);
  BOOST_CHECK(isEqual(v1.normalized(), v1 / v1.norm()));

  v1 = Vec3s(1.0, 2.0, 3.0);
  v2 = Vec3s(3.0, 4.0, 5.0);
  BOOST_CHECK(isEqual(v1.cross(v2), Vec3s(-2.0, 4.0, -2.0)));
  BOOST_CHECK(v1.dot(v2) == 26);
}

Vec3s rotate(Vec3s input, Scalar w, Vec3s vec) {
  return 2 * vec.dot(input) * vec + (w * w - vec.dot(vec)) * input +
         2 * w * vec.cross(input);
}

BOOST_AUTO_TEST_CASE(quaternion) {
  Quats q1(Quats::Identity()), q2, q3;
  q2 = fromAxisAngle(Vec3s(0, 0, 1), Scalar(M_PI / 2));
  q3 = q2.inverse();

  Vec3s v(1, -1, 0);

  BOOST_CHECK(isEqual(v, q1 * v));
  BOOST_CHECK(isEqual(Vec3s(1, 1, 0), q2 * v));
  BOOST_CHECK(
      isEqual(rotate(v, q3.w(), Vec3s(q3.x(), q3.y(), q3.z())), q3 * v));
}

BOOST_AUTO_TEST_CASE(transform) {
  Quats q = fromAxisAngle(Vec3s(0, 0, 1), Scalar(M_PI / 2));
  Vec3s T(0, 1, 2);
  Transform3s tf(q, T);

  Vec3s v(1, -1, 0);

  BOOST_CHECK(isEqual(q * v + T, q * v + T));

  Vec3s rv(q * v);
  // typename Transform3s::transform_return_type<Vec3s>::type output =
  // tf * v;
  // std::cout << rv << std::endl;
  // std::cout << output.lhs() << std::endl;
  BOOST_CHECK(isEqual(rv + T, tf.transform(v)));
}

BOOST_AUTO_TEST_CASE(random_transform) {
  for (size_t i = 0; i < 100; ++i) {
    const Transform3s tf = Transform3s::Random();
    BOOST_CHECK((tf.inverseTimes(tf)).isIdentity());
  }
}
