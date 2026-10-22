/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026. Toyota Research Institute
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

/** \author Sean Curtis */

/** Tests CollisionObject's own API -- in particular that the OBJECT_TYPE and
 NODE_TYPE tags it reports always agree with the geometry it currently holds.
 Those tags are cached in CollisionObject::init() rather than forwarded to the
 geometry on every call, so every path that can change the geometry pointer has
 to refresh them. */

#define BOOST_TEST_MODULE COAL_COLLISION_OBJECT
#include <boost/test/included/unit_test.hpp>

#include "coal/BV/BV.h"
#include "coal/BVH/BVH_model.h"
#include "coal/collision_object.h"
#include "coal/math/transform.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/shape/geometric_shapes.h"

#include "utility.h"

#include <string>
#include <vector>

using namespace coal;

namespace {

template <typename BV>
CollisionGeometryPtr_t makeMesh() {
  shared_ptr<BVHModel<BV>> mesh = make_shared<BVHModel<BV>>();
  generateBVHModel(*mesh, Box(1, 1, 1), Transform3s());
  return mesh;
}

// The invariant the cache exists to preserve: what the object reports is what
// the geometry it holds reports (which matches our expectation).
void checkObjectsTypesTags(const CollisionObject& object,
                           const CollisionGeometry& expected_geometry) {
  const CollisionGeometry* geometry = object.collisionGeometryPtr();
  BOOST_REQUIRE(geometry != nullptr);
  // Matches expected.
  BOOST_CHECK_EQUAL(object.getObjectType(), expected_geometry.getObjectType());
  BOOST_CHECK_EQUAL(object.getNodeType(), expected_geometry.getNodeType());
  // Internally consistent.
  BOOST_CHECK_EQUAL(object.getObjectType(), geometry->getObjectType());
  BOOST_CHECK_EQUAL(object.getNodeType(), geometry->getNodeType());
}

}  // namespace

// CollisionObject caches the node and object types from its geometry upon
// geometry and mutation.
//
// We'll assume that geometries are well behaved, so we'll simply alternate
// between two different, arbitrary geometries.
BOOST_AUTO_TEST_CASE(caching_node_and_object_types) {
  // Two CollisionGeometries with different object and node types.
  const auto g1 = make_shared<Box>(1, 2, 3);
  const auto g2 = makeMesh<OBBRSS>();
  const Transform3s tf = Transform3s::Identity();

  // All constructors.
  {
    checkObjectsTypesTags(CollisionObject(g1, true), *g1);
    checkObjectsTypesTags(CollisionObject(g2, false), *g2);
    checkObjectsTypesTags(CollisionObject(g1, tf, true), *g1);
    checkObjectsTypesTags(CollisionObject(g2, tf, false), *g2);
    checkObjectsTypesTags(
        CollisionObject(g1, tf.getRotation(), tf.getTranslation(), true), *g1);
    checkObjectsTypesTags(
        CollisionObject(g2, tf.getRotation(), tf.getTranslation(), false), *g2);
    CollisionObject nulled(nullptr);
    BOOST_CHECK_EQUAL(nulled.getObjectType(), OT_UNKNOWN);
    BOOST_CHECK_EQUAL(nulled.getNodeType(), BV_UNKNOWN);
  }

  // SetCollisionGeometry()
  {
    CollisionObject object(g1);
    checkObjectsTypesTags(object, *g1);
    object.setCollisionGeometry(g2);
    checkObjectsTypesTags(object, *g2);

    // Null geometry should result in unknown tags.
    object.setCollisionGeometry(nullptr);
    BOOST_CHECK_EQUAL(object.getObjectType(), OT_UNKNOWN);
    BOOST_CHECK_EQUAL(object.getNodeType(), BV_UNKNOWN);
  }

  // Copying
  {
    CollisionObject object(g1);
    CollisionObject copy_constructed(object);
    checkObjectsTypesTags(copy_constructed, *g1);

    CollisionObject assigned(g2);
    assigned = object;
    checkObjectsTypesTags(assigned, *g1);
  }
}
