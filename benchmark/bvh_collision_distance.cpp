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
// received a copy of the GNU General Lesser Public License along with
// Coal. If not, see <http://www.gnu.org/licenses/>.

#include <benchmark/benchmark.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "coal/BV/BV.h"
#include "coal/BVH/BVH_model.h"
#include "coal/collision_data.h"
#include "coal/data_types.h"
#include "coal/internal/BV_splitter.h"
#include "coal/internal/traversal_node_bvhs.h"
#include "coal/internal/traversal_node_setup.h"
#include "coal/math/transform.h"
#include "coal/mesh_loader/assimp.h"
#include "../src/collision_node.h"
#include "utility.h"

using namespace coal;

namespace {

// Number of generated transformation
const std::size_t num_transforms = 10000;
// Transformation range
const Scalar extent = Scalar(3000.0);

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
};

template <>
struct traits<OBBRSS> {
  typedef MeshCollisionTraversalNodeOBBRSS CollisionTraversalNode;
  typedef MeshDistanceTraversalNodeOBBRSS DistanceTraversalNode;
};

// 2D vector with mesh as first dimension and split strategy as second one.
template <typename BV>
using Models = std::array<std::array<shared_ptr<BVHModel<BV>>, 3>, 2>;

// Load all models in a static variable to avoid reloading
// them for each benchmarks.
template <typename BV>
const Models<BV>& get_models() {
  static const Models<BV> models = [] {
    const std::array<std::string, 2> resources = {
        TEST_RESOURCES_DIR "/env.obj", TEST_RESOURCES_DIR "/rob.obj"};

    Models<BV> models;
    for (std::size_t mesh = 0; mesh < models.size(); ++mesh) {
      for (int split_method = 0; split_method < 3; ++split_method) {
        shared_ptr<BVHModel<BV>> model = std::make_shared<BVHModel<BV>>();
        model->bv_splitter =
            std::make_shared<BVSplitter<BV>>(SplitMethodType(split_method));
        loadPolyhedronFromResource<BV>(resources[mesh], Vec3s::Ones(), model);
        models[mesh][split_method] = model;
      }
    }
    return models;
  }();
  return models;
}

// Generate random transformation in a static variable
// to avoid reloading them
const std::vector<Transform3s>& get_transforms() {
  static const std::vector<Transform3s> transforms = [] {
    std::vector<Transform3s> ret_transforms;
    Scalar extents_6d[] = {-extent, -extent, -extent, extent, extent, extent};
    generateRandomTransforms(extents_6d, ret_transforms, num_transforms);
    return ret_transforms;
  }();
  return transforms;
}

// Collision benchmark with custom node traversal
template <typename BV, SplitMethodType SplitMethod>
void run_collision(benchmark::State& state) {
  const Models<BV>& models = get_models<BV>();
  const BVHModel<BV>& o1 = *models[0][int(SplitMethod)];
  const BVHModel<BV>& o2 = *models[1][int(SplitMethod)];
  const std::vector<Transform3s>& transforms = get_transforms();

  const Transform3s tf2 = Transform3s::Identity();

  const CollisionRequest request;
  typename traits<BV>::CollisionTraversalNode node(request);
  CollisionResult result;

  for (auto _ : state) {
    result.clear();

    for (const Transform3s& tf1 : transforms) {
      initialize(node, o1, tf1, o2, tf2, result);
      collide(&node, request, result);
    }
  }

  state.SetItemsProcessed(state.iterations() *
                          static_cast<std::int64_t>(num_transforms));
}

// Distance benchmark with custom node traversal
template <typename BV, SplitMethodType SplitMethod>
void run_distance(benchmark::State& state) {
  const Models<BV>& models = get_models<BV>();
  const BVHModel<BV>& o1 = *models[0][int(SplitMethod)];
  const BVHModel<BV>& o2 = *models[1][int(SplitMethod)];
  const std::vector<Transform3s>& transforms = get_transforms();

  const Transform3s tf2 = Transform3s::Identity();

  const DistanceRequest request(true);
  typename traits<BV>::DistanceTraversalNode node;
  DistanceResult result;

  for (auto _ : state) {
    result.clear();

    for (const Transform3s& tf1 : transforms) {
      initialize(node, o1, tf1, o2, tf2, request, result);
      distance(&node, nullptr);
    }
  }

  state.SetItemsProcessed(state.iterations() *
                          static_cast<std::int64_t>(num_transforms));
}

}  // namespace

BENCHMARK(run_collision<RSS, SPLIT_METHOD_MEAN>);
BENCHMARK(run_collision<RSS, SPLIT_METHOD_BV_CENTER>);
BENCHMARK(run_collision<RSS, SPLIT_METHOD_MEDIAN>);
BENCHMARK(run_distance<RSS, SPLIT_METHOD_MEAN>);
BENCHMARK(run_distance<RSS, SPLIT_METHOD_BV_CENTER>);
BENCHMARK(run_distance<RSS, SPLIT_METHOD_MEDIAN>);

BENCHMARK(run_collision<kIOS, SPLIT_METHOD_MEAN>);
BENCHMARK(run_collision<kIOS, SPLIT_METHOD_BV_CENTER>);
BENCHMARK(run_collision<kIOS, SPLIT_METHOD_MEDIAN>);
BENCHMARK(run_distance<kIOS, SPLIT_METHOD_MEAN>);
BENCHMARK(run_distance<kIOS, SPLIT_METHOD_BV_CENTER>);
BENCHMARK(run_distance<kIOS, SPLIT_METHOD_MEDIAN>);

BENCHMARK(run_collision<OBB, SPLIT_METHOD_MEAN>);
BENCHMARK(run_collision<OBB, SPLIT_METHOD_BV_CENTER>);
BENCHMARK(run_collision<OBB, SPLIT_METHOD_MEDIAN>);

BENCHMARK(run_collision<OBBRSS, SPLIT_METHOD_MEAN>);
BENCHMARK(run_collision<OBBRSS, SPLIT_METHOD_BV_CENTER>);
BENCHMARK(run_collision<OBBRSS, SPLIT_METHOD_MEDIAN>);
BENCHMARK(run_distance<OBBRSS, SPLIT_METHOD_MEAN>);
BENCHMARK(run_distance<OBBRSS, SPLIT_METHOD_BV_CENTER>);
BENCHMARK(run_distance<OBBRSS, SPLIT_METHOD_MEDIAN>);

BENCHMARK_MAIN();
