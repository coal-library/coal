/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020. Toyota Research Institute
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/** @author Damrong Guoy (Damrong.Guoy@tri.global) */

/** Tests the dynamic axis-aligned bounding box tree.*/

#define BOOST_TEST_MODULE COAL_BROADPHASE_DYNAMIC_AABB_TREE
#include <boost/test/included/unit_test.hpp>

// #include "coal/data_types.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/broadphase/broadphase_dynamic_AABB_tree.h"
#include "coal/broadphase/default_broadphase_callbacks.h"
#include "coal/distance.h"

#include <iostream>
#include <memory>
#include <random>

using namespace coal;

// Pack the data for callback function.
struct CallBackData {
  bool expect_object0_then_object1;
  std::vector<CollisionObject*>* objects;
};

// This callback function tests the order of the two collision objects from
// the dynamic tree against the `data`. We assume that the first two
// parameters are always objects[0] and objects[1] in two possible orders,
// so we can safely ignore the second parameter. We do not use the last
// Scalar& parameter, which specifies the distance beyond which the
// pair of objects will be skipped.

struct DistanceCallBackDerived : DistanceCallBackBase {
  bool distance(CollisionObject* o1, CollisionObject* o2, Scalar& dist) {
    return distance_callback(o1, o2, &data, dist);
  }

  bool distance_callback(CollisionObject* a, CollisionObject*,
                         void* callback_data, Scalar&) {
    // Unpack the data.
    CallBackData* data = static_cast<CallBackData*>(callback_data);
    const std::vector<CollisionObject*>& objects = *(data->objects);
    const bool object0_first = a == objects[0];
    BOOST_CHECK_EQUAL(data->expect_object0_then_object1, object0_first);
    // TODO(DamrongGuoy): Remove the statement below when we solve the
    //  repeatability problem as mentioned in:
    //  https://github.com/flexible-collision-library/fcl/issues/368
    // Expect to switch the order next time.
    data->expect_object0_then_object1 = !data->expect_object0_then_object1;
    // Return true to stop the tree traversal.
    return true;
  }

  CallBackData data;
};

// Tests repeatability of a dynamic tree of two spheres when we call update()
// and distance() again and again without changing the poses of the objects.
// We only use the distance() method to invoke a hierarchy traversal.
// The distance-callback function in this test does not compute the signed
// distance between the two objects; it only checks their order.
//
// Currently every call to update() switches the order of the two objects.
// TODO(DamrongGuoy): Remove the above comment when we solve the
//  repeatability problem as mentioned in:
//  https://github.com/flexible-collision-library/fcl/issues/368
//
BOOST_AUTO_TEST_CASE(DynamicAABBTreeCollisionManager_class) {
  CollisionGeometryPtr_t sphere0 = make_shared<Sphere>(0.1);
  CollisionGeometryPtr_t sphere1 = make_shared<Sphere>(0.2);
  CollisionObject object0(sphere0);
  CollisionObject object1(sphere1);
  const Vec3s position0(Scalar(0.1), Scalar(0.2), Scalar(0.3));
  const Vec3s position1(Scalar(0.11), Scalar(0.21), Scalar(0.31));

  // We will use `objects` to check the order of the two collision objects in
  // our callback function.
  //
  // We use std::vector that contains *pointers* to CollisionObject,
  // instead of std::vector that contains CollisionObject's.
  // Previously we used std::vector<CollisionObject>, and it failed the
  // Eigen alignment assertion on Win32. We also tried, without success, the
  // custom allocator:
  //     std::vector<CollisionObject,
  //                 Eigen::aligned_allocator<CollisionObject>>,
  // but some platforms failed to build.
  std::vector<CollisionObject*> objects;
  objects.push_back(&object0);
  objects.push_back(&object1);

  std::vector<Vec3s> positions;
  positions.push_back(position0);
  positions.push_back(position1);

  DynamicAABBTreeCollisionManager dynamic_tree;
  for (size_t i = 0; i < objects.size(); ++i) {
    objects[i]->setTranslation(positions[i]);
    objects[i]->computeAABB();
    dynamic_tree.registerObject(objects[i]);
  }

  DistanceCallBackDerived callback;
  callback.data.expect_object0_then_object1 = false;
  callback.data.objects = &objects;

  // We repeat update() and distance() many times.  Each time, in the
  // callback function, we check the order of the two objects.
  for (int count = 0; count < 8; ++count) {
    dynamic_tree.update();
    dynamic_tree.distance(&callback);
  }
}

// Specialized (non-GJK) shape pairs must merge into a shared DistanceResult
// with the same min semantics as the GJK path (DistanceResult::update).
BOOST_AUTO_TEST_CASE(specialized_shape_distance_keeps_minimum) {
  const Box box(0.2, 0.2, 0.2);
  const Sphere sphere(0.05);
  const Transform3s identity;
  Transform3s tf_near;
  tf_near.setTranslation(Vec3s(0.3, 0., 0.));
  Transform3s tf_far;
  tf_far.setTranslation(Vec3s(1., 0., 0.));

  DistanceRequest request;
  DistanceResult result;
  const Scalar d_near =
      distance(&box, identity, &sphere, tf_near, request, result);
  const Vec3s p_near0 = result.nearest_points[0];
  const Vec3s p_near1 = result.nearest_points[1];
  const Scalar d_far =
      distance(&box, identity, &sphere, tf_far, request, result);

  BOOST_CHECK_CLOSE(d_near, 0.15, 1e-6);
  BOOST_CHECK_CLOSE(d_far, 0.85, 1e-6);
  BOOST_CHECK_EQUAL(result.min_distance, d_near);
  BOOST_CHECK(result.nearest_points[0] == p_near0);
  BOOST_CHECK(result.nearest_points[1] == p_near1);
}

// Broadphase distance over a mix of specialized (sphere-box, sphere-capsule)
// and GJK (capsule-box) pairs must return the pairwise minimum.
BOOST_AUTO_TEST_CASE(
    DynamicAABBTreeCollisionManager_distance_specialized_pairs) {
  std::vector<CollisionObjectPtr_t> objs1, objs2;
  for (int i = 0; i < 4; ++i)
    objs1.push_back(
        std::make_shared<CollisionObject>(std::make_shared<Sphere>(0.05)));
  for (int i = 0; i < 4; ++i)
    objs1.push_back(std::make_shared<CollisionObject>(
        std::make_shared<Capsule>(0.04, 0.2)));
  for (int i = 0; i < 6; ++i)
    objs2.push_back(std::make_shared<CollisionObject>(
        std::make_shared<Box>(0.3, 0.3, 0.3)));

  DynamicAABBTreeCollisionManager m1, m2;
  for (const auto& o : objs1) m1.registerObject(o.get());
  for (const auto& o : objs2) m2.registerObject(o.get());
  m1.setup();
  m2.setup();

  std::mt19937 gen(0);
  std::uniform_real_distribution<Scalar> uni(-1., 1.);
  const DistanceRequest request;
  int n_checked = 0;
  for (int trial = 0; trial < 300; ++trial) {
    for (const auto& o : objs1)
      o->setTranslation(Vec3s(uni(gen), uni(gen), uni(gen)));
    for (const auto& o : objs2)
      o->setTranslation(Vec3s(uni(gen), uni(gen), uni(gen)));
    m1.update();
    m2.update();

    DistanceResult pairwise;
    for (const auto& a : objs1)
      for (const auto& b : objs2) {
        DistanceResult r;
        distance(a.get(), b.get(), request, r);
        pairwise.update(r);
      }
    if (pairwise.min_distance <= 0) continue;  // broadphase stops on contact
    ++n_checked;

    DistanceCallBackDefault callback;
    m1.distance(&m2, &callback);
    const DistanceResult& res = callback.data.result;
    BOOST_CHECK_CLOSE(res.min_distance, pairwise.min_distance, 1e-6);
    BOOST_CHECK_EQUAL(res.o1, pairwise.o1);
    BOOST_CHECK_EQUAL(res.o2, pairwise.o2);
    BOOST_CHECK_SMALL((res.nearest_points[0] - res.nearest_points[1]).norm() -
                          pairwise.min_distance,
                      Scalar(1e-6));
  }
  BOOST_CHECK(n_checked > 100);
}
