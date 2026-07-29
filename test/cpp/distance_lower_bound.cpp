/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS
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
 *   * Neither the name of CNRS-LAAS nor the names of its
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

#define BOOST_TEST_MODULE COAL_DISTANCE_LOWER_BOUND
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "coal/fwd.hh"
#include "coal/data_types.h"
#include "coal/BV/OBBRSS.h"
#include "coal/BVH/BVH_model.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/collision.h"
#include "coal/distance.h"
#include "utility.h"
#include "fcl_resources/config.h"

using coal::BVHModel;
using coal::CollisionGeometryPtr_t;
using coal::CollisionObject;
using coal::CollisionRequest;
using coal::CollisionResult;
using coal::DistanceRequest;
using coal::DistanceResult;
using coal::OBBRSS;
using coal::Scalar;
using coal::shared_ptr;
using coal::Transform3s;
using coal::Triangle32;
using coal::Vec3s;

bool testDistanceLowerBound(const Transform3s& tf,
                            const CollisionGeometryPtr_t& m1,
                            const CollisionGeometryPtr_t& m2,
                            Scalar& distance) {
  Transform3s pose1(tf), pose2;

  CollisionRequest request;

  CollisionResult result;
  CollisionObject co1(m1, pose1);
  CollisionObject co2(m2, pose2);

  coal::collide(&co1, &co2, request, result);
  distance = result.distance_lower_bound;

  return result.isCollision();
}

bool testCollide(const Transform3s& tf, const CollisionGeometryPtr_t& m1,
                 const CollisionGeometryPtr_t& m2) {
  Transform3s pose1(tf), pose2;

  CollisionRequest request(coal::NO_REQUEST, 1);
  request.enable_distance_lower_bound = false;

  CollisionResult result;
  CollisionObject co1(m1, pose1);
  CollisionObject co2(m2, pose2);

  coal::collide(&co1, &co2, request, result);
  return result.isCollision();
}

bool testDistance(const Transform3s& tf, const CollisionGeometryPtr_t& m1,
                  const CollisionGeometryPtr_t& m2, Scalar& distance) {
  Transform3s pose1(tf), pose2;

  DistanceRequest request;
  DistanceResult result;
  CollisionObject co1(m1, pose1);
  CollisionObject co2(m2, pose2);

  coal::distance(&co1, &co2, request, result);
  distance = result.min_distance;

  if (result.min_distance <= 0) {
    return true;
  } else {
    return false;
  }
}

BOOST_AUTO_TEST_CASE(mesh_mesh) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle32> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  shared_ptr<BVHModel<OBBRSS> > m1(new BVHModel<OBBRSS>);
  shared_ptr<BVHModel<OBBRSS> > m2(new BVHModel<OBBRSS>);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  m2->beginModel();
  m2->addSubModel(p2, t2);
  m2->endModel();

  std::vector<Transform3s> transforms;
  Scalar extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  std::size_t n = 100;

  generateRandomTransforms(extents, transforms, n);

  // collision
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    Scalar distanceLowerBound, distance;
    bool col1, col2, col3;
    col1 = testDistanceLowerBound(transforms[i], m1, m2, distanceLowerBound);
    col2 = testDistance(transforms[i], m1, m2, distance);
    col3 = testCollide(transforms[i], m1, m2);
    std::cout << "col1 = " << col1 << ", col2 = " << col2 << ", col3 = " << col3
              << std::endl;
    std::cout << "distance lower bound = " << distanceLowerBound << std::endl;
    std::cout << "distance             = " << distance << std::endl;
    BOOST_CHECK(col1 == col3);
    if (!col1) {
      BOOST_CHECK_MESSAGE(distanceLowerBound <= distance,
                          "distance = " << distance << ", lower bound = "
                                        << distanceLowerBound);
    }
  }
}

BOOST_AUTO_TEST_CASE(box_sphere) {
  shared_ptr<coal::Sphere> sphere(new coal::Sphere(0.5));
  shared_ptr<coal::Box> box(new coal::Box(1., 1., 1.));

  Transform3s M1;
  M1.setIdentity();
  Transform3s M2;
  M2.setIdentity();

  std::vector<Transform3s> transforms;
  Scalar extents[] = {-2., -2., -2., 2., 2., 2.};
  const std::size_t n = 1000;

  generateRandomTransforms(extents, transforms, n);

  Scalar distanceLowerBound, distance;
  bool col1, col2;
  col1 = testDistanceLowerBound(M1, sphere, box, distanceLowerBound);
  col2 = testDistance(M1, sphere, box, distance);
  BOOST_CHECK(col1 == col2);
  BOOST_CHECK(distanceLowerBound <= distance);

  for (std::size_t i = 0; i < transforms.size(); ++i) {
    Scalar distanceLowerBound, distance;
    bool col1, col2;
    col1 =
        testDistanceLowerBound(transforms[i], sphere, box, distanceLowerBound);
    col2 = testDistance(transforms[i], sphere, box, distance);
    BOOST_CHECK(col1 == col2);
    if (!col1) {
      BOOST_CHECK_MESSAGE(distanceLowerBound <= distance,
                          "distance = " << distance << ", lower bound = "
                                        << distanceLowerBound);
    }
  }
}

BOOST_AUTO_TEST_CASE(sphere_sphere) {
  shared_ptr<coal::Sphere> sphere1(new coal::Sphere(0.5));
  shared_ptr<coal::Sphere> sphere2(new coal::Sphere(1.));

  Transform3s M1;
  M1.setIdentity();
  Transform3s M2;
  M2.setIdentity();

  std::vector<Transform3s> transforms;
  Scalar extents[] = {-2., -2., -2., 2., 2., 2.};
  const std::size_t n = 1000;

  generateRandomTransforms(extents, transforms, n);

  Scalar distanceLowerBound, distance;
  bool col1, col2;
  col1 = testDistanceLowerBound(M1, sphere1, sphere2, distanceLowerBound);
  col2 = testDistance(M1, sphere1, sphere2, distance);
  BOOST_CHECK(col1 == col2);
  BOOST_CHECK(distanceLowerBound <= distance);

  for (std::size_t i = 0; i < transforms.size(); ++i) {
    Scalar distanceLowerBound, distance;
    bool col1, col2;
    col1 = testDistanceLowerBound(transforms[i], sphere1, sphere2,
                                  distanceLowerBound);
    col2 = testDistance(transforms[i], sphere1, sphere2, distance);
    BOOST_CHECK(col1 == col2);
    if (!col1) {
      BOOST_CHECK_MESSAGE(distanceLowerBound <= distance,
                          "distance = " << distance << ", lower bound = "
                                        << distanceLowerBound);
    }
  }
}

BOOST_AUTO_TEST_CASE(box_mesh) {
  std::vector<Vec3s> p1;
  std::vector<Triangle32> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  shared_ptr<BVHModel<OBBRSS> > m1(new BVHModel<OBBRSS>);
  shared_ptr<coal::Box> m2(new coal::Box(500, 200, 150));

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  std::vector<Transform3s> transforms;
  Scalar extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  std::size_t n = 100;

  generateRandomTransforms(extents, transforms, n);

  // collision
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    Scalar distanceLowerBound, distance;
    bool col1, col2;
    col1 = testDistanceLowerBound(transforms[i], m1, m2, distanceLowerBound);
    col2 = testDistance(transforms[i], m1, m2, distance);
    BOOST_CHECK(col1 == col2);
    if (!col1) {
      BOOST_CHECK_MESSAGE(distanceLowerBound <= distance,
                          "distance = " << distance << ", lower bound = "
                                        << distanceLowerBound);
    }
  }
}
