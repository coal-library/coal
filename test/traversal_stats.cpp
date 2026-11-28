/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Jelle Feringa
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

#define BOOST_TEST_MODULE COAL_TRAVERSAL_STATS
#include <boost/test/included/unit_test.hpp>

#include <thread>
#include <vector>

#include "coal/BV/OBBRSS.h"
#include "coal/BVH/BVH_model.h"
#include "coal/collision.h"
#include "coal/distance.h"
#include "coal/traversal_stats.h"
#include "coal/shape/geometric_shapes.h"

using coal::Box;
using coal::BVHModel;
using coal::CollisionObject;
using coal::CollisionRequest;
using coal::CollisionResult;
using coal::DistanceRequest;
using coal::DistanceResult;
using coal::make_shared;
using coal::OBBRSS;
using coal::Scalar;
using coal::shared_ptr;
using coal::Sphere;
using coal::Transform3s;
using coal::Triangle32;
using coal::Vec3s;

namespace {

// Grid of GRID_CELLS x GRID_CELLS squares, two triangles each, in the plane
// z = 0 over [-GRID_HALF_WIDTH, GRID_HALF_WIDTH]^2 (metres).
constexpr int GRID_CELLS = 10;
constexpr Scalar GRID_HALF_WIDTH = 0.4;
// Box half-extent (metres): the box holds the whole grid, so every mesh BV
// overlaps it and every triangle is in contact.
constexpr Scalar BOX_HALF_EXTENT = 0.5;
// Contacts asked for: more than the grid has triangles, so the request is
// never satisfied early and the traversal visits every leaf.
constexpr std::size_t MAX_CONTACTS = 1000;

// The count a test expects for `n` traversal tests: `n` when Coal counts
// (built with COAL_ENABLE_TRAVERSAL_STATS), zero otherwise.
std::uint64_t counted(std::uint64_t n) {
  return coal::traversalStatsEnabled() ? n : 0;
}

shared_ptr<BVHModel<OBBRSS> > gridMesh() {
  std::vector<Vec3s> vertices;
  std::vector<Triangle32> triangles;
  const Scalar step = 2 * GRID_HALF_WIDTH / GRID_CELLS;
  for (int i = 0; i <= GRID_CELLS; ++i)
    for (int j = 0; j <= GRID_CELLS; ++j)
      vertices.push_back(
          Vec3s(-GRID_HALF_WIDTH + i * step, -GRID_HALF_WIDTH + j * step, 0));
  const auto at = [](int i, int j) {
    return static_cast<Triangle32::IndexType>(i * (GRID_CELLS + 1) + j);
  };
  for (int i = 0; i < GRID_CELLS; ++i)
    for (int j = 0; j < GRID_CELLS; ++j) {
      triangles.push_back(Triangle32(at(i, j), at(i + 1, j), at(i + 1, j + 1)));
      triangles.push_back(Triangle32(at(i, j), at(i + 1, j + 1), at(i, j + 1)));
    }
  shared_ptr<BVHModel<OBBRSS> > mesh = make_shared<BVHModel<OBBRSS> >();
  mesh->beginModel();
  mesh->addSubModel(vertices, triangles);
  mesh->endModel();
  return mesh;
}

shared_ptr<BVHModel<OBBRSS> > oneTriangleMesh() {
  std::vector<Vec3s> vertices{Vec3s(0, 0, 0), Vec3s(1, 0, 0), Vec3s(0, 1, 0)};
  std::vector<Triangle32> triangles{Triangle32(0, 1, 2)};
  shared_ptr<BVHModel<OBBRSS> > mesh = make_shared<BVHModel<OBBRSS> >();
  mesh->beginModel();
  mesh->addSubModel(vertices, triangles);
  mesh->endModel();
  return mesh;
}

// Collides the grid with the box that holds it. Returns whether they collide,
// for the caller to require on its own thread.
bool gridCollidesWithBox(const shared_ptr<BVHModel<OBBRSS> >& mesh) {
  const shared_ptr<Box> box = make_shared<Box>(
      2 * BOX_HALF_EXTENT, 2 * BOX_HALF_EXTENT, 2 * BOX_HALF_EXTENT);
  CollisionObject o1(mesh, Transform3s());
  CollisionObject o2(box, Transform3s());
  CollisionRequest request;
  request.num_max_contacts = MAX_CONTACTS;
  CollisionResult result;
  coal::collide(&o1, &o2, request, result);
  return result.isCollision();
}

}  // namespace

// The probe reports the option the library was built with. The macro is a
// PUBLIC compile definition of the library, so this test sees the same value.
BOOST_AUTO_TEST_CASE(enabled_probe_matches_the_build_option) {
#ifdef COAL_ENABLE_TRAVERSAL_STATS
  BOOST_CHECK(coal::traversalStatsEnabled());
#else
  BOOST_CHECK(!coal::traversalStatsEnabled());
#endif
}

// Every inner BV overlaps the box shape and the request is never satisfied
// early, so the traversal visits every node of the mesh's binary tree: one
// leaf test per triangle, one BVDisjoints per inner node (a binary tree with
// n leaves has n - 1 inner nodes; the leaf's BV test is not run).
BOOST_AUTO_TEST_CASE(mesh_box_collide_counts_every_triangle) {
  const shared_ptr<BVHModel<OBBRSS> > mesh = gridMesh();
  const std::uint64_t n_tris = static_cast<std::uint64_t>(mesh->num_tris);
  BOOST_REQUIRE_EQUAL(n_tris, 2u * GRID_CELLS * GRID_CELLS);
  BOOST_REQUIRE_EQUAL(static_cast<std::uint64_t>(mesh->getNumBVs()),
                      2 * n_tris - 1);

  coal::resetTraversalStats();
  BOOST_REQUIRE(gridCollidesWithBox(mesh));

  BOOST_CHECK_EQUAL(coal::traversalStats().leaf_tests, counted(n_tris));
  BOOST_CHECK_EQUAL(coal::traversalStats().bv_tests, counted(n_tris - 1));
}

// One-triangle mesh: one BV node, the root is a leaf. coal::distance runs
// preprocess (one call on triangle 0), then distanceRecurse(0, 0) finds both
// nodes leaves and calls leafComputeDistance once, before any
// BVDistanceLowerBound: two leaf tests, no BV test.
BOOST_AUTO_TEST_CASE(one_triangle_unbounded_distance_counts_preprocess) {
  const shared_ptr<BVHModel<OBBRSS> > mesh = oneTriangleMesh();
  BOOST_REQUIRE_EQUAL(mesh->getNumBVs(), 1u);
  BOOST_REQUIRE(mesh->getBV(0).isLeaf());

  const shared_ptr<Sphere> sphere = make_shared<Sphere>(Scalar(0.1));
  Transform3s far_away;
  far_away.setTranslation(Vec3s(0, 0, 5));
  CollisionObject o1(mesh, Transform3s());
  CollisionObject o2(sphere, far_away);
  DistanceRequest request;
  DistanceResult result;

  coal::resetTraversalStats();
  coal::distance(&o1, &o2, request, result);

  BOOST_CHECK_CLOSE(result.min_distance, 4.9, 1e-6);
  BOOST_CHECK_EQUAL(coal::traversalStats().leaf_tests, counted(2));
  BOOST_CHECK_EQUAL(coal::traversalStats().bv_tests, 0u);
}

BOOST_AUTO_TEST_CASE(reset_gives_zeros) {
  coal::traversalStats().leaf_tests = 7;
  coal::traversalStats().bv_tests = 11;

  coal::resetTraversalStats();

  BOOST_CHECK_EQUAL(coal::traversalStats().leaf_tests, 0u);
  BOOST_CHECK_EQUAL(coal::traversalStats().bv_tests, 0u);
}

// A worker thread counts its own traversal and leaves this thread's totals
// alone.
BOOST_AUTO_TEST_CASE(other_thread_leaves_this_thread_unchanged) {
  const shared_ptr<BVHModel<OBBRSS> > mesh = gridMesh();
  const std::uint64_t n_tris = static_cast<std::uint64_t>(mesh->num_tris);
  coal::resetTraversalStats();
  coal::traversalStats().leaf_tests = 7;
  coal::traversalStats().bv_tests = 11;

  coal::TraversalStats other;
  bool collided = false;
  std::thread worker([&mesh, &other, &collided]() {
    coal::resetTraversalStats();
    collided = gridCollidesWithBox(mesh);
    other = coal::traversalStats();
  });
  worker.join();

  BOOST_REQUIRE(collided);
  BOOST_CHECK_EQUAL(other.leaf_tests, counted(n_tris));
  BOOST_CHECK_EQUAL(other.bv_tests, counted(n_tris - 1));
  BOOST_CHECK_EQUAL(coal::traversalStats().leaf_tests, 7u);
  BOOST_CHECK_EQUAL(coal::traversalStats().bv_tests, 11u);
}
