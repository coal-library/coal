/*
 *  Software License Agreement (BSD License)
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

/** \author Karsten Knese <Karsten.Knese@googlemail.com> */

#define BOOST_TEST_MODULE COAL_CAPSULE_CAPSULE
#include <boost/test/included/unit_test.hpp>

#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE((x + 1.0), (1.0), (eps))

#include <cmath>
#include <iostream>
#include "coal/distance.h"
#include "coal/collision.h"
#include "coal/math/transform.h"
#include "coal/collision.h"
#include "coal/collision_object.h"
#include "coal/shape/geometric_shapes.h"

#include "utility.h"

using namespace coal;
using Quat = Eigen::Quaternion<Scalar>;
using Vec4s = Eigen::Matrix<Scalar, 4, 1>;

BOOST_AUTO_TEST_CASE(collision_capsule_capsule_trivial) {
  const Scalar radius = 1.;

  CollisionGeometryPtr_t c1(new Capsule(radius, 0.));
  CollisionGeometryPtr_t c2(new Capsule(radius, 0.));

  CollisionGeometryPtr_t s1(new Sphere(radius));
  CollisionGeometryPtr_t s2(new Sphere(radius));

#ifndef NDEBUG
  int num_tests = 1e3;
#else
  int num_tests = 1e6;
#endif

  Transform3s tf1;
  Transform3s tf2;

  for (int i = 0; i < num_tests; ++i) {
    Vec3s p1 = Vec3s::Random() * (2. * radius);
    Vec3s p2 = Vec3s::Random() * (2. * radius);

    Matrix3s rot1 = Quat(Vec4s::Random().normalized()).toRotationMatrix();
    Matrix3s rot2 = Quat(Vec4s::Random().normalized()).toRotationMatrix();

    tf1.setTranslation(p1);
    tf1.setRotation(rot1);
    tf2.setTranslation(p2);
    tf2.setRotation(rot2);

    CollisionObject capsule_o1(c1, tf1);
    CollisionObject capsule_o2(c2, tf2);

    CollisionObject sphere_o1(s1, tf1);
    CollisionObject sphere_o2(s2, tf2);

    // Enable computation of nearest points
    CollisionRequest collisionRequest;
    CollisionResult capsule_collisionResult, sphere_collisionResult;

    size_t sphere_num_collisions = collide(
        &sphere_o1, &sphere_o2, collisionRequest, sphere_collisionResult);
    size_t capsule_num_collisions = collide(
        &capsule_o1, &capsule_o2, collisionRequest, capsule_collisionResult);

    BOOST_CHECK_EQUAL(sphere_num_collisions, capsule_num_collisions);
    if (sphere_num_collisions == 0 && capsule_num_collisions == 0)
      BOOST_CHECK_CLOSE(sphere_collisionResult.distance_lower_bound,
                        capsule_collisionResult.distance_lower_bound, 1e-6);
  }
}

BOOST_AUTO_TEST_CASE(collision_capsule_capsule_aligned) {
  const Scalar radius = Scalar(0.01);
  const Scalar length = Scalar(0.2);

  CollisionGeometryPtr_t c1(new Capsule(radius, length));
  CollisionGeometryPtr_t c2(new Capsule(radius, length));
#ifndef NDEBUG
  int num_tests = 1e3;
#else
  int num_tests = 1e6;
#endif

  Transform3s tf1;
  Transform3s tf2;

  Vec3s p1 = Vec3s::Zero();
  Vec3s p2_no_collision =
      Vec3s(0, 0,
            Scalar(2 * (length / 2. + radius) +
                   1e-3));  // because capsule are along the Z axis

  for (int i = 0; i < num_tests; ++i) {
    Matrix3s rot = Quat(Vec4s::Random().normalized()).toRotationMatrix();

    tf1.setTranslation(p1);
    tf1.setRotation(rot);
    tf2.setTranslation(p2_no_collision);
    tf2.setRotation(rot);

    CollisionObject capsule_o1(c1, tf1);
    CollisionObject capsule_o2(c2, tf2);

    // Enable computation of nearest points
    CollisionRequest collisionRequest;
    CollisionResult capsule_collisionResult;

    size_t capsule_num_collisions = collide(
        &capsule_o1, &capsule_o2, collisionRequest, capsule_collisionResult);

    BOOST_CHECK(capsule_num_collisions == 0);
  }

  Vec3s p2_with_collision =
      Vec3s(0, 0, Scalar(std::min(length / 2, radius) * (1. - 1e-2)));
  for (int i = 0; i < num_tests; ++i) {
    Matrix3s rot = Quat(Vec4s::Random().normalized()).toRotationMatrix();

    tf1.setTranslation(p1);
    tf1.setRotation(rot);
    tf2.setTranslation(p2_with_collision);
    tf2.setRotation(rot);

    CollisionObject capsule_o1(c1, tf1);
    CollisionObject capsule_o2(c2, tf2);

    // Enable computation of nearest points
    CollisionRequest collisionRequest;
    CollisionResult capsule_collisionResult;

    size_t capsule_num_collisions = collide(
        &capsule_o1, &capsule_o2, collisionRequest, capsule_collisionResult);

    BOOST_CHECK(capsule_num_collisions > 0);
  }

  p2_no_collision = Vec3s(0, 0, Scalar(2 * (length / 2. + radius) + 1e-3));

  Transform3s geom1_placement(Matrix3s::Identity(), Vec3s::Zero());
  Transform3s geom2_placement(Matrix3s::Identity(), p2_no_collision);

  for (int i = 0; i < num_tests; ++i) {
    Matrix3s rot = Quat(Vec4s::Random().normalized()).toRotationMatrix();
    Vec3s trans = Vec3s::Random();

    Transform3s displacement(rot, trans);
    Transform3s tf1 = displacement * geom1_placement;
    Transform3s tf2 = displacement * geom2_placement;

    CollisionObject capsule_o1(c1, tf1);
    CollisionObject capsule_o2(c2, tf2);

    // Enable computation of nearest points
    CollisionRequest collisionRequest;
    CollisionResult capsule_collisionResult;

    size_t capsule_num_collisions = collide(
        &capsule_o1, &capsule_o2, collisionRequest, capsule_collisionResult);

    BOOST_CHECK(capsule_num_collisions == 0);
  }

  //  p2_with_collision =
  //  Vec3s(0.,0.,std::min(length/2.,radius)*(1.-1e-2));
  p2_with_collision = Vec3s(0, 0, Scalar(0.01));
  geom2_placement.setTranslation(p2_with_collision);

  for (int i = 0; i < num_tests; ++i) {
    Matrix3s rot = Quat(Vec4s::Random().normalized()).toRotationMatrix();
    Vec3s trans = Vec3s::Random();

    Transform3s displacement(rot, trans);
    Transform3s tf1 = displacement * geom1_placement;
    Transform3s tf2 = displacement * geom2_placement;

    CollisionObject capsule_o1(c1, tf1);
    CollisionObject capsule_o2(c2, tf2);

    // Enable computation of nearest points
    CollisionRequest collisionRequest;
    CollisionResult capsule_collisionResult;

    size_t capsule_num_collisions = collide(
        &capsule_o1, &capsule_o2, collisionRequest, capsule_collisionResult);

    BOOST_CHECK(capsule_num_collisions > 0);
  }
}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_origin) {
  CollisionGeometryPtr_t s1(new Capsule(5, 10));
  CollisionGeometryPtr_t s2(new Capsule(5, 10));

  Transform3s tf1;
  Transform3s tf2(Vec3s(Scalar(20.1), 0, 0));

  CollisionObject o1(s1, tf1);
  CollisionObject o2(s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true);
  DistanceResult distanceResult;

  distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied translation on two capsules";
  std::cerr << " T1 = " << tf1.getTranslation()
            << ", T2 = " << tf2.getTranslation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points[0]
            << ", p2 = " << distanceResult.nearest_points[1]
            << ", distance = " << distanceResult.min_distance << std::endl;

  BOOST_CHECK_CLOSE(distanceResult.min_distance, 10.1, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformXY) {
  CollisionGeometryPtr_t s1(new Capsule(5, 10));
  CollisionGeometryPtr_t s2(new Capsule(5, 10));

  Transform3s tf1;
  Transform3s tf2(Vec3s(20, 20, 0));

  CollisionObject o1(s1, tf1);
  CollisionObject o2(s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true);
  DistanceResult distanceResult;

  distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied translation on two capsules";
  std::cerr << " T1 = " << tf1.getTranslation()
            << ", T2 = " << tf2.getTranslation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points[0]
            << ", p2 = " << distanceResult.nearest_points[1]
            << ", distance = " << distanceResult.min_distance << std::endl;

  Scalar expected = sqrt(Scalar(800)) - 10;
  BOOST_CHECK_CLOSE(distanceResult.min_distance, expected, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformZ) {
  CollisionGeometryPtr_t s1(new Capsule(5, 10));
  CollisionGeometryPtr_t s2(new Capsule(5, 10));

  Transform3s tf1;
  Transform3s tf2(Vec3s(0, 0, Scalar(20.1)));

  CollisionObject o1(s1, tf1);
  CollisionObject o2(s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true);
  DistanceResult distanceResult;

  distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied translation on two capsules";
  std::cerr << " T1 = " << tf1.getTranslation()
            << ", T2 = " << tf2.getTranslation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points[0]
            << ", p2 = " << distanceResult.nearest_points[1]
            << ", distance = " << distanceResult.min_distance << std::endl;

  BOOST_CHECK_CLOSE(distanceResult.min_distance, 0.1, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformZ2) {
  CollisionGeometryPtr_t s1(new Capsule(5, 10));
  CollisionGeometryPtr_t s2(new Capsule(5, 10));

  Transform3s tf1;
  Transform3s tf2(makeQuat(sqrt(Scalar(2)) / 2, 0, sqrt(Scalar(2)) / 2, 0),
                  Vec3s(0, 0, Scalar(25.1)));

  CollisionObject o1(s1, tf1);
  CollisionObject o2(s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest(true);
  DistanceResult distanceResult;

  distance(&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied rotation and translation on two capsules" << std::endl;
  std::cerr << "R1 = " << tf1.getRotation() << std::endl
            << "T1 = " << tf1.getTranslation().transpose() << std::endl
            << "R2 = " << tf2.getRotation() << std::endl
            << "T2 = " << tf2.getTranslation().transpose() << std::endl;
  std::cerr << "Closest points:" << std::endl
            << "p1 = " << distanceResult.nearest_points[0].transpose()
            << std::endl
            << "p2 = " << distanceResult.nearest_points[1].transpose()
            << std::endl
            << "distance = " << distanceResult.min_distance << std::endl;

  const Vec3s& p1 = distanceResult.nearest_points[0];
  const Vec3s& p2 = distanceResult.nearest_points[1];

  BOOST_CHECK_CLOSE(distanceResult.min_distance, 10.1, 1e-6);
  CHECK_CLOSE_TO_0(p1[0], 1e-4);
  CHECK_CLOSE_TO_0(p1[1], 1e-4);
  BOOST_CHECK_CLOSE(p1[2], 10, 1e-4);
  CHECK_CLOSE_TO_0(p2[0], 1e-4);
  CHECK_CLOSE_TO_0(p2[1], 1e-4);
  BOOST_CHECK_CLOSE(p2[2], 20.1, 1e-4);
}
