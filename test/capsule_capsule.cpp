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

BOOST_AUTO_TEST_CASE(collision_capsule_capsule_trivial) {
  const double radius = 1.;

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
    Eigen::Vector3d p1 = Eigen::Vector3d::Random() * (2. * radius);
    Eigen::Vector3d p2 = Eigen::Vector3d::Random() * (2. * radius);

    Eigen::Matrix3d rot1 =
        Eigen::Quaterniond(Eigen::Vector4d::Random().normalized())
            .toRotationMatrix();
    Eigen::Matrix3d rot2 =
        Eigen::Quaterniond(Eigen::Vector4d::Random().normalized())
            .toRotationMatrix();

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
  const double radius = 0.01;
  const double length = 0.2;

  CollisionGeometryPtr_t c1(new Capsule(radius, length));
  CollisionGeometryPtr_t c2(new Capsule(radius, length));
#ifndef NDEBUG
  int num_tests = 1e3;
#else
  int num_tests = 1e6;
#endif

  Transform3s tf1;
  Transform3s tf2;

  Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p2_no_collision =
      Eigen::Vector3d(0., 0.,
                      2 * (length / 2. + radius) +
                          1e-3);  // because capsule are along the Z axis

  for (int i = 0; i < num_tests; ++i) {
    Eigen::Matrix3d rot =
        Eigen::Quaterniond(Eigen::Vector4d::Random().normalized())
            .toRotationMatrix();

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

  Eigen::Vector3d p2_with_collision =
      Eigen::Vector3d(0., 0., std::min(length / 2., radius) * (1. - 1e-2));
  for (int i = 0; i < num_tests; ++i) {
    Eigen::Matrix3d rot =
        Eigen::Quaterniond(Eigen::Vector4d::Random().normalized())
            .toRotationMatrix();

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

  p2_no_collision = Eigen::Vector3d(0., 0., 2 * (length / 2. + radius) + 1e-3);

  Transform3s geom1_placement(Eigen::Matrix3d::Identity(),
                              Eigen::Vector3d::Zero());
  Transform3s geom2_placement(Eigen::Matrix3d::Identity(), p2_no_collision);

  for (int i = 0; i < num_tests; ++i) {
    Eigen::Matrix3d rot =
        Eigen::Quaterniond(Eigen::Vector4d::Random().normalized())
            .toRotationMatrix();
    Eigen::Vector3d trans = Eigen::Vector3d::Random();

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
  //  Eigen::Vector3d(0.,0.,std::min(length/2.,radius)*(1.-1e-2));
  p2_with_collision = Eigen::Vector3d(0., 0., 0.01);
  geom2_placement.setTranslation(p2_with_collision);

  for (int i = 0; i < num_tests; ++i) {
    Eigen::Matrix3d rot =
        Eigen::Quaterniond(Eigen::Vector4d::Random().normalized())
            .toRotationMatrix();
    Eigen::Vector3d trans = Eigen::Vector3d::Random();

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
  Transform3s tf2(Vec3s(20.1, 0, 0));

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

  CoalScalar expected = sqrt(800) - 10;
  BOOST_CHECK_CLOSE(distanceResult.min_distance, expected, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformZ) {
  CollisionGeometryPtr_t s1(new Capsule(5, 10));
  CollisionGeometryPtr_t s2(new Capsule(5, 10));

  Transform3s tf1;
  Transform3s tf2(Vec3s(0, 0, 20.1));

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
  Transform3s tf2(makeQuat(sqrt(2) / 2, 0, sqrt(2) / 2, 0), Vec3s(0, 0, 25.1));

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
