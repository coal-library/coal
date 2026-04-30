/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026 Coal Library
 *  All rights reserved.
 */

// Locks down the no-state-pollution invariant introduced when the per-leaf
// GJKSolver was hoisted to a `mutable GJKSolver solver_` member of
// MeshCollisionTraversalNode.
//
// The hoist saves heap traffic (EPA::reset() / SimplexVertex resize) by
// allocating the solver's working buffers once per query rather than once per
// triangle-pair leaf. The risk is that leftover state from one leaf call
// leaks into the next.
//
// This test exercises the worst case: run collide() *twice on the same
// traversal-node instance* with two different transforms, then run collide()
// on a *fresh* traversal-node instance with the second transform alone.
// If the member solver had pollution, the same-node second run would diverge
// from the fresh-node run.

#define _USE_MATH_DEFINES
#include <cmath>

#define BOOST_TEST_MODULE COAL_GJK_HOIST_NO_STATE_POLLUTION
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

}  // namespace

BOOST_AUTO_TEST_SUITE(gjk_hoist_no_state_pollution)

BOOST_AUTO_TEST_CASE(reused_node_matches_fresh_node) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle32> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  BVHModel<RSS> m1, m2;
  makeModel(p1, t1, m1);
  makeModel(p2, t2, m2);

  // Four deterministic transforms covering colliding and non-colliding
  // configurations.
  std::vector<Transform3s> tfs;
  Scalar extents[] = {Scalar(-3000), Scalar(-3000), Scalar(-3000),
                      Scalar(3000),  Scalar(3000),  Scalar(3000)};
  generateRandomTransforms(extents, tfs, 4);

  CollisionRequest request(CONTACT, /*num_max_contacts*/ 1);

  // Reused-node path: walk all 4 transforms on the *same* node instance.
  std::vector<CollisionResult> results_reused(tfs.size());
  MeshCollisionTraversalNodeRSS reused_node(request);
  reused_node.enable_statistics = false;

  for (std::size_t i = 0; i < tfs.size(); ++i) {
    Transform3s tf2;
    BOOST_REQUIRE(
        initialize(reused_node, m1, tfs[i], m2, tf2, results_reused[i]));
    collide(&reused_node, request, results_reused[i]);
  }

  // Fresh-node path: a new node instance per transform.
  for (std::size_t i = 0; i < tfs.size(); ++i) {
    CollisionResult result_fresh;
    MeshCollisionTraversalNodeRSS fresh_node(request);
    fresh_node.enable_statistics = false;

    Transform3s tf2;
    BOOST_REQUIRE(initialize(fresh_node, m1, tfs[i], m2, tf2, result_fresh));
    collide(&fresh_node, request, result_fresh);

    BOOST_TEST_INFO("transform index " << i);
    BOOST_CHECK_EQUAL(results_reused[i].isCollision(),
                      result_fresh.isCollision());
    BOOST_CHECK_EQUAL(results_reused[i].numContacts(),
                      result_fresh.numContacts());
    BOOST_CHECK_CLOSE(results_reused[i].distance_lower_bound,
                      result_fresh.distance_lower_bound, Scalar(1e-9));
  }
}

BOOST_AUTO_TEST_SUITE_END()
