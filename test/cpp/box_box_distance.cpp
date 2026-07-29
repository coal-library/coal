/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS.
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

/** \author Florent Lamiraux <florent@laas.fr> */

#define _USE_MATH_DEFINES
#include <cmath>

#define BOOST_TEST_MODULE COAL_BOX_BOX
#include <boost/test/included/unit_test.hpp>

#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE((x + 1.0), (1.0), (eps))

#include <cmath>
#include <iostream>
#include "coal/distance.h"
#include "coal/math/transform.h"
#include "coal/collision.h"
#include "coal/collision_object.h"
#include "coal/shape/geometric_shapes.h"

#include "utility.h"

using coal::CollisionGeometryPtr_t;
using coal::CollisionObject;
using coal::DistanceRequest;
using coal::DistanceResult;
using coal::Scalar;
using coal::Transform3s;
using coal::Vec3s;

BOOST_AUTO_TEST_CASE(distance_box_box_1) {
  CollisionGeometryPtr_t s1(new coal::Box(6, 10, 2));
  CollisionGeometryPtr_t s2(new coal::Box(2, 2, 2));

  Transform3s tf1;
  Transform3s tf2(Vec3s(25, 20, 5));

  CollisionObject o1(s1, tf1);
  CollisionObject o2(s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true, true, 0, 0);
  DistanceResult distanceResult;

  coal::distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
            << " R1 = " << tf1.getRotation() << std::endl
            << " T2 = " << tf2.getTranslation() << std::endl
            << " R2 = " << tf2.getRotation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points[0]
            << ", p2 = " << distanceResult.nearest_points[1]
            << ", distance = " << distanceResult.min_distance << std::endl;
  Scalar dx = 25 - 3 - 1;
  Scalar dy = 20 - 5 - 1;
  Scalar dz = 5 - 1 - 1;

  const Vec3s& p1 = distanceResult.nearest_points[0];
  const Vec3s& p2 = distanceResult.nearest_points[1];
  BOOST_CHECK_CLOSE(distanceResult.min_distance,
                    sqrt(dx * dx + dy * dy + dz * dz), 1e-4);

  BOOST_CHECK_CLOSE(p1[0], 3, 1e-6);
  BOOST_CHECK_CLOSE(p1[1], 5, 1e-6);
  BOOST_CHECK_CLOSE(p1[2], 1, 1e-6);
  BOOST_CHECK_CLOSE(p2[0], 24, 1e-6);
  BOOST_CHECK_CLOSE(p2[1], 19, 1e-6);
  BOOST_CHECK_CLOSE(p2[2], 4, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_box_box_2) {
  CollisionGeometryPtr_t s1(new coal::Box(6, 10, 2));
  CollisionGeometryPtr_t s2(new coal::Box(2, 2, 2));
  static Scalar pi = Scalar(M_PI);
  Transform3s tf1;
  Transform3s tf2(
      coal::makeQuat(cos(Scalar(pi / 8)), sin(Scalar(pi / 8)) / sqrt(Scalar(3)),
                     sin(Scalar(pi / 8)) / sqrt(Scalar(3)),
                     sin(Scalar(pi / 8)) / sqrt(Scalar(3))),
      Vec3s(0, 0, 10));

  CollisionObject o1(s1, tf1);
  CollisionObject o2(s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true, true, 0, 0);
  DistanceResult distanceResult;

  coal::distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
            << " R1 = " << tf1.getRotation() << std::endl
            << " T2 = " << tf2.getTranslation() << std::endl
            << " R2 = " << tf2.getRotation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points[0]
            << ", p2 = " << distanceResult.nearest_points[1]
            << ", distance = " << distanceResult.min_distance << std::endl;

  const Vec3s& p1 = distanceResult.nearest_points[0];
  const Vec3s& p2 = distanceResult.nearest_points[1];
  Scalar distance = Scalar(-1.62123444 + 10 - 1);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  BOOST_CHECK_CLOSE(p1[0], 0.60947571, 1e-4);
  BOOST_CHECK_CLOSE(p1[1], 0.01175873, 1e-4);
  BOOST_CHECK_CLOSE(p1[2], 1, 1e-6);
  BOOST_CHECK_CLOSE(p2[0], 0.60947571, 1e-4);
  BOOST_CHECK_CLOSE(p2[1], 0.01175873, 1e-4);
  BOOST_CHECK_CLOSE(p2[2], -1.62123444 + 10, 1e-4);
}

BOOST_AUTO_TEST_CASE(distance_box_box_3) {
  CollisionGeometryPtr_t s1(new coal::Box(1, 1, 1));
  CollisionGeometryPtr_t s2(new coal::Box(1, 1, 1));
  static Scalar pi = Scalar(M_PI);
  Transform3s tf1(coal::makeQuat(cos(pi / 8), 0, 0, sin(pi / 8)),
                  Vec3s(-2, 1, .5));
  Transform3s tf2(coal::makeQuat(cos(pi / 8), 0, sin(pi / 8), 0),
                  Vec3s(2, .5, .5));

  CollisionObject o1(s1, tf1);
  CollisionObject o2(s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true, 0, 0);
  DistanceResult distanceResult;

  coal::distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
            << " R1 = " << tf1.getRotation() << std::endl
            << " T2 = " << tf2.getTranslation() << std::endl
            << " R2 = " << tf2.getRotation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points[0]
            << ", p2 = " << distanceResult.nearest_points[1]
            << ", distance = " << distanceResult.min_distance << std::endl;

  const Vec3s& p1 = distanceResult.nearest_points[0];
  const Vec3s& p2 = distanceResult.nearest_points[1];
  Scalar distance = Scalar(4 - sqrt(2));
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  const Vec3s p1Ref(sqrt(Scalar(2)) / 2 - 2, 1, .5);
  const Vec3s p2Ref(2 - sqrt(Scalar(2)) / 2, 1, .5);
  BOOST_CHECK_CLOSE(p1[0], p1Ref[0], 1e-4);
  BOOST_CHECK_CLOSE(p1[1], p1Ref[1], 1e-4);
  BOOST_CHECK_CLOSE(p1[2], p1Ref[2], 1e-4);
  BOOST_CHECK_CLOSE(p2[0], p2Ref[0], 1e-4);
  BOOST_CHECK_CLOSE(p2[1], p2Ref[1], 1e-4);
  BOOST_CHECK_CLOSE(p2[2], p2Ref[2], 1e-4);

  // Apply the same global transform to both objects and recompute
  Transform3s tf3(
      coal::makeQuat(Scalar(0.435952844074), Scalar(-0.718287018243),
                     Scalar(0.310622451066), Scalar(0.444435113443)),
      Vec3s(4, 5, 6));
  tf1 = tf3 * tf1;
  tf2 = tf3 * tf2;
  o1 = CollisionObject(s1, tf1);
  o2 = CollisionObject(s2, tf2);

  distanceResult.clear();
  coal::distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
            << " R1 = " << tf1.getRotation() << std::endl
            << " T2 = " << tf2.getTranslation() << std::endl
            << " R2 = " << tf2.getRotation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points[0]
            << ", p2 = " << distanceResult.nearest_points[1]
            << ", distance = " << distanceResult.min_distance << std::endl;

  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  const Vec3s p1Moved = tf3.transform(p1Ref);
  const Vec3s p2Moved = tf3.transform(p2Ref);
  BOOST_CHECK_CLOSE(p1[0], p1Moved[0], 1e-4);
  BOOST_CHECK_CLOSE(p1[1], p1Moved[1], 1e-4);
  BOOST_CHECK_CLOSE(p1[2], p1Moved[2], 1e-4);
  BOOST_CHECK_CLOSE(p2[0], p2Moved[0], 1e-4);
  BOOST_CHECK_CLOSE(p2[1], p2Moved[1], 1e-4);
  BOOST_CHECK_CLOSE(p2[2], p2Moved[2], 1e-4);
}

BOOST_AUTO_TEST_CASE(distance_box_box_4) {
  coal::Box s1(1, 1, 1);
  coal::Box s2(1, 1, 1);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true, true, 0, 0);
  DistanceResult distanceResult;
  Scalar distance;

  Transform3s tf1(Vec3s(2, 0, 0));
  Transform3s tf2;
  coal::distance(&s1, tf1, &s2, tf2, distanceRequest, distanceResult);

  distance = 1.;
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  tf1.setTranslation(Vec3s(Scalar(1.01), 0, 0));
  distanceResult.clear();
  coal::distance(&s1, tf1, &s2, tf2, distanceRequest, distanceResult);

  distance = Scalar(0.01);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 2e-3);

  tf1.setTranslation(Vec3s(Scalar(0.99), 0, 0));
  distanceResult.clear();
  coal::distance(&s1, tf1, &s2, tf2, distanceRequest, distanceResult);

  distance = Scalar(-0.01);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 2e-3);

  tf1.setTranslation(Vec3s(0, 0, 0));
  distanceResult.clear();
  coal::distance(&s1, tf1, &s2, tf2, distanceRequest, distanceResult);

  distance = -1;
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 2e-3);
}
