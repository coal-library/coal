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

#include <iostream>
#include <memory>

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
