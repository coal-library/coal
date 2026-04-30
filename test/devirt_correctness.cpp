/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026 Coal Library
 *  All rights reserved.
 */

// Locks down the dispatch contract introduced by the BVH-traversal devirt
// commit: the templated `collide<Node>(Node*, ...)` and `distance<Node>(Node*,
// ...)` overloads in src/collision_node.h must produce *exactly* the same
// CollisionResult / DistanceResult as the existing virtual-dispatch
// `collide(CollisionTraversalNodeBase*, ...)` / `distance(...)` overloads.
//
// Without this, a regression in either path would only surface as a perf
// shift in benchmark_devirt — never as a wrong answer in the test suite.

#define _USE_MATH_DEFINES
#include <cmath>

#define BOOST_TEST_MODULE COAL_DEVIRT_CORRECTNESS
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "coal/internal/traversal_node_setup.h"
#include "coal/internal/traversal_node_bvhs.h"
#include "coal/internal/BV_splitter.h"
#include "../src/collision_node.h"

#include "utility.h"
#include "fcl_resources/config.h"

using namespace coal;

namespace {

template <typename BV>
void makeModel(const std::vector<Vec3s>& vertices,
               const std::vector<Triangle32>& triangles, BVHModel<BV>& model) {
  model.bv_splitter.reset(new BVSplitter<BV>(SPLIT_METHOD_MEAN));
  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

std::vector<Transform3s> sampleTransforms() {
  std::vector<Transform3s> tfs;
  Scalar extents[] = {Scalar(-3000), Scalar(-3000), Scalar(-3000),
                      Scalar(3000),  Scalar(3000),  Scalar(3000)};
  generateRandomTransforms(extents, tfs, 16);
  return tfs;
}

template <typename BV, typename CollisionNode>
void check_collide_devirt(const BVHModel<BV>& m1, const BVHModel<BV>& m2,
                          const std::vector<Transform3s>& tfs) {
  for (const Transform3s& tf : tfs) {
    CollisionRequest request(CONTACT, /*num_max_contacts*/ 1);

    CollisionResult result_virt, result_devirt;
    CollisionNode node_virt(request), node_devirt(request);
    node_virt.enable_statistics = false;
    node_devirt.enable_statistics = false;

    Transform3s tf2;
    BOOST_REQUIRE(initialize(node_virt, m1, tf, m2, tf2, result_virt));
    BOOST_REQUIRE(initialize(node_devirt, m1, tf, m2, tf2, result_devirt));

    // Force the virtual-dispatch path with an explicit upcast.
    collide(static_cast<CollisionTraversalNodeBase*>(&node_virt), request,
            result_virt);

    // Templated overload resolution selects collide<CollisionNode> for the
    // exact-type pointer.
    collide(&node_devirt, request, result_devirt);

    BOOST_CHECK_EQUAL(result_virt.isCollision(), result_devirt.isCollision());
    BOOST_CHECK_EQUAL(result_virt.numContacts(), result_devirt.numContacts());
    BOOST_CHECK_CLOSE(result_virt.distance_lower_bound,
                      result_devirt.distance_lower_bound, Scalar(1e-9));
  }
}

template <typename BV, typename DistanceNode>
void check_distance_devirt(const BVHModel<BV>& m1, const BVHModel<BV>& m2,
                           const std::vector<Transform3s>& tfs) {
  for (const Transform3s& tf : tfs) {
    DistanceRequest request(/*enable_nearest_points*/ true);

    DistanceResult result_virt, result_devirt;
    DistanceNode node_virt, node_devirt;
    node_virt.enable_statistics = false;
    node_devirt.enable_statistics = false;

    Transform3s tf2;
    BOOST_REQUIRE(initialize(node_virt, m1, tf, m2, tf2, request, result_virt));
    BOOST_REQUIRE(
        initialize(node_devirt, m1, tf, m2, tf2, request, result_devirt));

    distance(static_cast<DistanceTraversalNodeBase*>(&node_virt));
    distance(&node_devirt);

    BOOST_CHECK_CLOSE(result_virt.min_distance, result_devirt.min_distance,
                      Scalar(1e-9));
  }
}

}  // namespace

BOOST_AUTO_TEST_SUITE(devirt_correctness)

BOOST_AUTO_TEST_CASE(collide_results_match) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle32> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  const std::vector<Transform3s> tfs = sampleTransforms();

  BVHModel<RSS> m1_rss, m2_rss;
  makeModel(p1, t1, m1_rss);
  makeModel(p2, t2, m2_rss);
  check_collide_devirt<RSS, MeshCollisionTraversalNodeRSS>(m1_rss, m2_rss, tfs);

  BVHModel<kIOS> m1_kios, m2_kios;
  makeModel(p1, t1, m1_kios);
  makeModel(p2, t2, m2_kios);
  check_collide_devirt<kIOS, MeshCollisionTraversalNodekIOS>(m1_kios, m2_kios,
                                                             tfs);

  BVHModel<OBB> m1_obb, m2_obb;
  makeModel(p1, t1, m1_obb);
  makeModel(p2, t2, m2_obb);
  check_collide_devirt<OBB, MeshCollisionTraversalNodeOBB>(m1_obb, m2_obb, tfs);

  BVHModel<OBBRSS> m1_obbrss, m2_obbrss;
  makeModel(p1, t1, m1_obbrss);
  makeModel(p2, t2, m2_obbrss);
  check_collide_devirt<OBBRSS, MeshCollisionTraversalNodeOBBRSS>(
      m1_obbrss, m2_obbrss, tfs);
}

BOOST_AUTO_TEST_CASE(distance_results_match) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle32> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  const std::vector<Transform3s> tfs = sampleTransforms();

  BVHModel<RSS> m1_rss, m2_rss;
  makeModel(p1, t1, m1_rss);
  makeModel(p2, t2, m2_rss);
  check_distance_devirt<RSS, MeshDistanceTraversalNodeRSS>(m1_rss, m2_rss, tfs);

  BVHModel<kIOS> m1_kios, m2_kios;
  makeModel(p1, t1, m1_kios);
  makeModel(p2, t2, m2_kios);
  check_distance_devirt<kIOS, MeshDistanceTraversalNodekIOS>(m1_kios, m2_kios,
                                                             tfs);

  // OBB has no MeshDistanceTraversalNode; skip.

  BVHModel<OBBRSS> m1_obbrss, m2_obbrss;
  makeModel(p1, t1, m1_obbrss);
  makeModel(p2, t2, m2_obbrss);
  check_distance_devirt<OBBRSS, MeshDistanceTraversalNodeOBBRSS>(
      m1_obbrss, m2_obbrss, tfs);
}

BOOST_AUTO_TEST_SUITE_END()
