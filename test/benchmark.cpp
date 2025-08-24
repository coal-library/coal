// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of Coal.
// Coal is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Coal is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// Coal. If not, see <http://www.gnu.org/licenses/>.

#include <boost/filesystem.hpp>

#include "coal/internal/traversal_node_setup.h"
#include "coal/internal/traversal_node_bvhs.h"
#include "../src/collision_node.h"
#include "coal/internal/BV_splitter.h"

#include "utility.h"
#include "fcl_resources/config.h"

#define RUN_CASE(BV, tf, models, split) \
  run<BV>(tf, models, split, #BV " - " #split ":\t")

using namespace coal;

bool verbose = false;
Scalar DELTA = Scalar(0.001);

template <typename BV>
void makeModel(const std::vector<Vec3s>& vertices,
               const std::vector<Triangle32>& triangles,
               SplitMethodType split_method, BVHModel<BV>& model);

template <typename BV, typename TraversalNode>
double distance(const std::vector<Transform3s>& tf, const BVHModel<BV>& m1,
                const BVHModel<BV>& m2, bool verbose);

template <typename BV, typename TraversalNode>
double collide(const std::vector<Transform3s>& tf, const BVHModel<BV>& m1,
               const BVHModel<BV>& m2, bool verbose);

template <typename BV>
double run(const std::vector<Transform3s>& tf,
           const BVHModel<BV> (&models)[2][3], int split_method,
           const char* sm_name);

template <typename BV>
struct traits {};

template <>
struct traits<RSS> {
  typedef MeshCollisionTraversalNodeRSS CollisionTraversalNode;
  typedef MeshDistanceTraversalNodeRSS DistanceTraversalNode;
};

template <>
struct traits<kIOS> {
  typedef MeshCollisionTraversalNodekIOS CollisionTraversalNode;
  typedef MeshDistanceTraversalNodekIOS DistanceTraversalNode;
};

template <>
struct traits<OBB> {
  typedef MeshCollisionTraversalNodeOBB CollisionTraversalNode;
  // typedef MeshDistanceTraversalNodeOBB  DistanceTraversalNode;
};

template <>
struct traits<OBBRSS> {
  typedef MeshCollisionTraversalNodeOBBRSS CollisionTraversalNode;
  typedef MeshDistanceTraversalNodeOBBRSS DistanceTraversalNode;
};

template <typename BV>
void makeModel(const std::vector<Vec3s>& vertices,
               const std::vector<Triangle32>& triangles,
               SplitMethodType split_method, BVHModel<BV>& model) {
  model.bv_splitter.reset(new BVSplitter<BV>(split_method));
  model.bv_splitter.reset(new BVSplitter<BV>(split_method));

  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

template <typename BV, typename TraversalNode>
double distance(const std::vector<Transform3s>& tf, const BVHModel<BV>& m1,
                const BVHModel<BV>& m2, bool verbose) {
  Transform3s pose2;

  DistanceResult local_result;
  DistanceRequest request(true);
  TraversalNode node;

  node.enable_statistics = verbose;

  BenchTimer timer;
  timer.start();

  for (std::size_t i = 0; i < tf.size(); ++i) {
    if (!initialize(node, m1, tf[i], m2, pose2, request, local_result))
      std::cout << "initialize error" << std::endl;

    distance(&node, NULL);
  }
  timer.stop();
  return timer.getElapsedTimeInMicroSec();
}

template <typename BV, typename TraversalNode>
double collide(const std::vector<Transform3s>& tf, const BVHModel<BV>& m1,
               const BVHModel<BV>& m2, bool verbose) {
  Transform3s pose2;

  CollisionResult local_result;
  CollisionRequest request;
  TraversalNode node(request);

  node.enable_statistics = verbose;

  BenchTimer timer;
  timer.start();

  for (std::size_t i = 0; i < tf.size(); ++i) {
    bool success(initialize(node, m1, tf[i], m2, pose2, local_result));
    (void)success;
    assert(success);

    CollisionResult result;
    collide(&node, request, result);
  }

  timer.stop();
  return timer.getElapsedTimeInMicroSec();
}

template <typename BV>
double run(const std::vector<Transform3s>& tf,
           const BVHModel<BV> (&models)[2][3], int split_method,
           const char* prefix) {
  double col = collide<BV, typename traits<BV>::CollisionTraversalNode>(
      tf, models[0][split_method], models[1][split_method], verbose);
  double dist = distance<BV, typename traits<BV>::DistanceTraversalNode>(
      tf, models[0][split_method], models[1][split_method], verbose);

  std::cout << prefix << " (" << col << ", " << dist << ")\n";
  return col + dist;
}

template <>
double run<OBB>(const std::vector<Transform3s>& tf,
                const BVHModel<OBB> (&models)[2][3], int split_method,
                const char* prefix) {
  double col = collide<OBB, traits<OBB>::CollisionTraversalNode>(
      tf, models[0][split_method], models[1][split_method], verbose);
  double dist = 0;

  std::cout << prefix << " (\t" << col << ", \tNaN)\n";
  return col + dist;
}

int main(int, char*[]) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle32> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  // Make models
  BVHModel<RSS> ms_rss[2][3];
  makeModel(p1, t1, SPLIT_METHOD_MEAN, ms_rss[0][SPLIT_METHOD_MEAN]);
  makeModel(p1, t1, SPLIT_METHOD_BV_CENTER, ms_rss[0][SPLIT_METHOD_BV_CENTER]);
  makeModel(p1, t1, SPLIT_METHOD_MEDIAN, ms_rss[0][SPLIT_METHOD_MEDIAN]);
  makeModel(p2, t2, SPLIT_METHOD_MEAN, ms_rss[1][SPLIT_METHOD_MEAN]);
  makeModel(p2, t2, SPLIT_METHOD_BV_CENTER, ms_rss[1][SPLIT_METHOD_BV_CENTER]);
  makeModel(p2, t2, SPLIT_METHOD_MEDIAN, ms_rss[1][SPLIT_METHOD_MEDIAN]);

  BVHModel<kIOS> ms_kios[2][3];
  makeModel(p1, t1, SPLIT_METHOD_MEAN, ms_kios[0][SPLIT_METHOD_MEAN]);
  makeModel(p1, t1, SPLIT_METHOD_BV_CENTER, ms_kios[0][SPLIT_METHOD_BV_CENTER]);
  makeModel(p1, t1, SPLIT_METHOD_MEDIAN, ms_kios[0][SPLIT_METHOD_MEDIAN]);
  makeModel(p2, t2, SPLIT_METHOD_MEAN, ms_kios[1][SPLIT_METHOD_MEAN]);
  makeModel(p2, t2, SPLIT_METHOD_BV_CENTER, ms_kios[1][SPLIT_METHOD_BV_CENTER]);
  makeModel(p2, t2, SPLIT_METHOD_MEDIAN, ms_kios[1][SPLIT_METHOD_MEDIAN]);

  BVHModel<OBB> ms_obb[2][3];
  makeModel(p1, t1, SPLIT_METHOD_MEAN, ms_obb[0][SPLIT_METHOD_MEAN]);
  makeModel(p1, t1, SPLIT_METHOD_BV_CENTER, ms_obb[0][SPLIT_METHOD_BV_CENTER]);
  makeModel(p1, t1, SPLIT_METHOD_MEDIAN, ms_obb[0][SPLIT_METHOD_MEDIAN]);
  makeModel(p2, t2, SPLIT_METHOD_MEAN, ms_obb[1][SPLIT_METHOD_MEAN]);
  makeModel(p2, t2, SPLIT_METHOD_BV_CENTER, ms_obb[1][SPLIT_METHOD_BV_CENTER]);
  makeModel(p2, t2, SPLIT_METHOD_MEDIAN, ms_obb[1][SPLIT_METHOD_MEDIAN]);

  BVHModel<OBBRSS> ms_obbrss[2][3];
  makeModel(p1, t1, SPLIT_METHOD_MEAN, ms_obbrss[0][SPLIT_METHOD_MEAN]);
  makeModel(p1, t1, SPLIT_METHOD_BV_CENTER,
            ms_obbrss[0][SPLIT_METHOD_BV_CENTER]);
  makeModel(p1, t1, SPLIT_METHOD_MEDIAN, ms_obbrss[0][SPLIT_METHOD_MEDIAN]);
  makeModel(p2, t2, SPLIT_METHOD_MEAN, ms_obbrss[1][SPLIT_METHOD_MEAN]);
  makeModel(p2, t2, SPLIT_METHOD_BV_CENTER,
            ms_obbrss[1][SPLIT_METHOD_BV_CENTER]);
  makeModel(p2, t2, SPLIT_METHOD_MEDIAN, ms_obbrss[1][SPLIT_METHOD_MEDIAN]);

  std::vector<Transform3s> transforms;  // t0
  Scalar extents[] = {-3000, -3000, -3000, 3000, 3000, 3000};
  std::size_t n = 10000;

  generateRandomTransforms(extents, transforms, n);
  double total_time = 0;

  total_time += RUN_CASE(RSS, transforms, ms_rss, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(RSS, transforms, ms_rss, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(RSS, transforms, ms_rss, SPLIT_METHOD_MEDIAN);

  total_time += RUN_CASE(kIOS, transforms, ms_kios, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(kIOS, transforms, ms_kios, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(kIOS, transforms, ms_kios, SPLIT_METHOD_MEDIAN);

  total_time += RUN_CASE(OBB, transforms, ms_obb, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(OBB, transforms, ms_obb, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(OBB, transforms, ms_obb, SPLIT_METHOD_MEDIAN);

  total_time += RUN_CASE(OBBRSS, transforms, ms_obbrss, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(OBBRSS, transforms, ms_obbrss, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(OBBRSS, transforms, ms_obbrss, SPLIT_METHOD_MEDIAN);

  std::cout << "\n\nTotal time: " << total_time << std::endl;
}
