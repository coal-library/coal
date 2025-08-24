/*
 * Software License Agreement (BSD License)
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

/** \author Jia Pan */

#define BOOST_TEST_MODULE COAL_BROADPHASE
#include <boost/test/included/unit_test.hpp>

#include "coal/config.hh"
#include "coal/broadphase/broadphase.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/math/transform.h"
#include "utility.h"

#if USE_GOOGLEHASH
#include <sparsehash/sparse_hash_map>
#include <sparsehash/dense_hash_map>
#include <hash_map>
#endif

#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <iomanip>

using namespace coal;
using namespace coal::detail;

/// @brief Generate environment with 3 * n objects for self distance, so we try
/// to make sure none of them collide with each other.
void generateSelfDistanceEnvironments(std::vector<CollisionObject*>& env,
                                      Scalar env_scale, std::size_t n);

/// @brief Generate environment with 3 * n objects for self distance, but all in
/// meshes.
void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject*>& env,
                                          Scalar env_scale, std::size_t n);

/// @brief test for broad phase distance
void broad_phase_distance_test(Scalar env_scale, std::size_t env_size,
                               std::size_t query_size, bool use_mesh = false);

/// @brief test for broad phase self distance
void broad_phase_self_distance_test(Scalar env_scale, std::size_t env_size,
                                    bool use_mesh = false);

Scalar DELTA = Scalar(0.01);

#if USE_GOOGLEHASH
template <typename U, typename V>
struct GoogleSparseHashTable
    : public google::sparse_hash_map<U, V, std::tr1::hash<size_t>,
                                     std::equal_to<size_t> > {};

template <typename U, typename V>
struct GoogleDenseHashTable
    : public google::dense_hash_map<U, V, std::tr1::hash<size_t>,
                                    std::equal_to<size_t> > {
  GoogleDenseHashTable()
      : google::dense_hash_map<U, V, std::tr1::hash<size_t>,
                               std::equal_to<size_t> >() {
    this->set_empty_key(NULL);
  }
};
#endif

// TODO(jcarpent): fix this test
//  - test_core_bf_broad_phase_distance

/// check broad phase distance
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_distance) {
#ifndef NDEBUG
  broad_phase_distance_test(200, 100, 10);
  broad_phase_distance_test(200, 1000, 10);
  broad_phase_distance_test(2000, 100, 10);
  broad_phase_distance_test(2000, 1000, 10);
#else
  broad_phase_distance_test(200, 100, 100);
  broad_phase_distance_test(200, 1000, 100);
  broad_phase_distance_test(2000, 100, 100);
  broad_phase_distance_test(2000, 1000, 100);
#endif
}

/// check broad phase self distance
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_self_distance) {
  broad_phase_self_distance_test(200, 512);
  broad_phase_self_distance_test(200, 1000);
  broad_phase_self_distance_test(200, 5000);
}

/// check broad phase distance
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_distance_mesh) {
#ifndef NDEBUG
  broad_phase_distance_test(200, 10, 10, true);
  broad_phase_distance_test(200, 100, 10, true);
  broad_phase_distance_test(2000, 10, 10, true);
  broad_phase_distance_test(2000, 100, 10, true);
#else
  broad_phase_distance_test(200, 100, 100, true);
  broad_phase_distance_test(200, 1000, 100, true);
  broad_phase_distance_test(2000, 100, 100, true);
  broad_phase_distance_test(2000, 1000, 100, true);
#endif
}

/// check broad phase self distance
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_self_distance_mesh) {
  broad_phase_self_distance_test(200, 512, true);
  broad_phase_self_distance_test(200, 1000, true);
  broad_phase_self_distance_test(200, 5000, true);
}

void generateSelfDistanceEnvironments(std::vector<CollisionObject*>& env,
                                      Scalar env_scale, std::size_t n) {
  int n_edge = static_cast<int>(std::floor(std::pow(n, 1 / 3.0)));

  Scalar step_size = env_scale * 2 / Scalar(n_edge);
  Scalar delta_size = step_size * Scalar(0.05);
  Scalar single_size = step_size - 2 * delta_size;

  int i = 0;
  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box* box = new Box(single_size, single_size, single_size);
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(box),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere* sphere = new Sphere(single_size / 2);
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(sphere),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder* cylinder = new Cylinder(single_size / 2, single_size);
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(cylinder),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone* cone = new Cone(single_size / 2, single_size);
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(cone),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }
}

void generateSelfDistanceEnvironmentsMesh(std::vector<CollisionObject*>& env,
                                          Scalar env_scale, std::size_t n) {
  int n_edge = static_cast<int>(std::floor(std::pow(n, 1 / 3.0)));

  Scalar step_size = env_scale * 2 / Scalar(n_edge);
  Scalar delta_size = step_size * Scalar(0.05);
  Scalar single_size = step_size - 2 * delta_size;

  int i = 0;
  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Box box(single_size, single_size, single_size);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, box, Transform3s());
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(model),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Sphere sphere(single_size / 2);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, sphere, Transform3s(), 16, 16);
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(model),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cylinder cylinder(single_size / 2, single_size);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cylinder, Transform3s(), 16, 16);
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(model),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  for (; i < n_edge * n_edge * n_edge / 4; ++i) {
    int x = i % (n_edge * n_edge);
    int y = (i - n_edge * n_edge * x) % n_edge;
    int z = i - n_edge * n_edge * x - n_edge * y;

    Cone cone(single_size / 2, single_size);
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cone, Transform3s(), 16, 16);
    const Scalar half = Scalar(0.5);
    env.push_back(new CollisionObject(
        shared_ptr<CollisionGeometry>(model),
        Transform3s(Vec3s(
            Scalar(x) * step_size + delta_size + half * single_size - env_scale,
            Scalar(y) * step_size + delta_size + half * single_size - env_scale,
            Scalar(z) * step_size + delta_size + half * single_size -
                env_scale))));
    env.back()->collisionGeometry()->computeLocalAABB();
  }
}

void broad_phase_self_distance_test(Scalar env_scale, std::size_t env_size,
                                    bool use_mesh) {
  std::vector<TStruct> ts;
  std::vector<BenchTimer> timers;

  std::vector<CollisionObject*> env;
  if (use_mesh)
    generateSelfDistanceEnvironmentsMesh(env, env_scale, env_size);
  else
    generateSelfDistanceEnvironments(env, env_scale, env_size);

  std::vector<BroadPhaseCollisionManager*> managers;

  managers.push_back(new NaiveCollisionManager());
  managers.push_back(new SSaPCollisionManager());
  managers.push_back(new SaPCollisionManager());
  managers.push_back(new IntervalTreeCollisionManager());

  Vec3s lower_limit, upper_limit;
  SpatialHashingCollisionManager<>::computeBound(env, lower_limit, upper_limit);
  Scalar cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 5,
                                       (upper_limit[1] - lower_limit[1]) / 5),
                              (upper_limit[2] - lower_limit[2]) / 5);
  // managers.push_back(new SpatialHashingCollisionManager<>(cell_size,
  // lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<
                     SparseHashTable<AABB, CollisionObject*, SpatialHash> >(
      cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(
      new SpatialHashingCollisionManager<SparseHashTable<
          AABB, CollisionObject*, SpatialHash, GoogleSparseHashTable> >(
          cell_size, lower_limit, upper_limit));
  managers.push_back(
      new SpatialHashingCollisionManager<SparseHashTable<
          AABB, CollisionObject*, SpatialHash, GoogleDenseHashTable> >(
          cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager());
  managers.push_back(new DynamicAABBTreeArrayCollisionManager());

  {
    DynamicAABBTreeCollisionManager* m = new DynamicAABBTreeCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeArrayCollisionManager* m =
        new DynamicAABBTreeArrayCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  ts.resize(managers.size());
  timers.resize(managers.size());

  for (size_t i = 0; i < managers.size(); ++i) {
    timers[i].start();
    managers[i]->registerObjects(env);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for (size_t i = 0; i < managers.size(); ++i) {
    timers[i].start();
    managers[i]->setup();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  std::vector<DistanceCallBackDefault> self_callbacks(managers.size());

  for (size_t i = 0; i < self_callbacks.size(); ++i) {
    timers[i].start();
    managers[i]->distance(&self_callbacks[i]);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
    // std::cout << self_data[i].result.min_distance << " ";
  }
  // std::cout << std::endl;

  for (size_t i = 1; i < managers.size(); ++i)
    BOOST_CHECK(fabs(self_callbacks[0].data.result.min_distance -
                     self_callbacks[i].data.result.min_distance) < DELTA ||
                fabs(self_callbacks[0].data.result.min_distance -
                     self_callbacks[i].data.result.min_distance) /
                        fabs(self_callbacks[0].data.result.min_distance) <
                    DELTA);

  for (size_t i = 0; i < env.size(); ++i) delete env[i];

  for (size_t i = 0; i < managers.size(); ++i) delete managers[i];

  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  int w = 7;

  std::cout << "self distance timing summary" << std::endl;
  std::cout << env.size() << " objs" << std::endl;
  std::cout << "register time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "self distance time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[2] << " ";
  std::cout << std::endl;

  std::cout << "overall time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;
}

void broad_phase_distance_test(Scalar env_scale, std::size_t env_size,
                               std::size_t query_size, bool use_mesh) {
  std::vector<TStruct> ts;
  std::vector<BenchTimer> timers;

  std::vector<CollisionObject*> env;
  if (use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject*> query;

  BroadPhaseCollisionManager* manager = new NaiveCollisionManager();
  for (std::size_t i = 0; i < env.size(); ++i) manager->registerObject(env[i]);
  manager->setup();

  while (1) {
    std::vector<CollisionObject*> candidates;
    if (use_mesh)
      generateEnvironmentsMesh(candidates, env_scale, query_size);
    else
      generateEnvironments(candidates, env_scale, query_size);

    for (std::size_t i = 0; i < candidates.size(); ++i) {
      CollisionCallBackDefault callback;
      manager->collide(candidates[i], &callback);
      if (callback.data.result.numContacts() == 0)
        query.push_back(candidates[i]);
      else
        delete candidates[i];
      if (query.size() == query_size) break;
    }

    if (query.size() == query_size) break;
  }

  delete manager;

  std::vector<BroadPhaseCollisionManager*> managers;

  managers.push_back(new NaiveCollisionManager());
  managers.push_back(new SSaPCollisionManager());
  managers.push_back(new SaPCollisionManager());
  managers.push_back(new IntervalTreeCollisionManager());

  Vec3s lower_limit, upper_limit;
  SpatialHashingCollisionManager<>::computeBound(env, lower_limit, upper_limit);
  Scalar cell_size = std::min(std::min((upper_limit[0] - lower_limit[0]) / 20,
                                       (upper_limit[1] - lower_limit[1]) / 20),
                              (upper_limit[2] - lower_limit[2]) / 20);
  // managers.push_back(new SpatialHashingCollisionManager<>(cell_size,
  // lower_limit, upper_limit));
  managers.push_back(new SpatialHashingCollisionManager<
                     SparseHashTable<AABB, CollisionObject*, SpatialHash> >(
      cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(
      new SpatialHashingCollisionManager<SparseHashTable<
          AABB, CollisionObject*, SpatialHash, GoogleSparseHashTable> >(
          cell_size, lower_limit, upper_limit));
  managers.push_back(
      new SpatialHashingCollisionManager<SparseHashTable<
          AABB, CollisionObject*, SpatialHash, GoogleDenseHashTable> >(
          cell_size, lower_limit, upper_limit));
#endif
  managers.push_back(new DynamicAABBTreeCollisionManager());
  managers.push_back(new DynamicAABBTreeArrayCollisionManager());

  {
    DynamicAABBTreeCollisionManager* m = new DynamicAABBTreeCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  {
    DynamicAABBTreeArrayCollisionManager* m =
        new DynamicAABBTreeArrayCollisionManager();
    m->tree_init_level = 2;
    managers.push_back(m);
  }

  ts.resize(managers.size());
  timers.resize(managers.size());

  for (size_t i = 0; i < managers.size(); ++i) {
    timers[i].start();
    managers[i]->registerObjects(env);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for (size_t i = 0; i < managers.size(); ++i) {
    timers[i].start();
    managers[i]->setup();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for (size_t i = 0; i < query.size(); ++i) {
    std::vector<DistanceCallBackDefault> query_callbacks(managers.size());
    for (size_t j = 0; j < managers.size(); ++j) {
      timers[j].start();
      managers[j]->distance(query[i], &query_callbacks[j]);
      timers[j].stop();
      ts[j].push_back(timers[j].getElapsedTime());
      std::cout << query_callbacks[j].data.result.min_distance << " ";
    }
    std::cout << std::endl;

    for (size_t j = 1; j < managers.size(); ++j) {
      bool test = fabs(query_callbacks[0].data.result.min_distance -
                       query_callbacks[j].data.result.min_distance) < DELTA ||
                  fabs(query_callbacks[0].data.result.min_distance -
                       query_callbacks[j].data.result.min_distance) /
                          fabs(query_callbacks[0].data.result.min_distance) <
                      DELTA;
      BOOST_CHECK(test);

      if (!test) {
        const BroadPhaseCollisionManager& self = *managers[j];
        std::cout << "j: " << typeid(self).name() << std::endl;
        std::cout << "query_callbacks[0].data.result.min_distance: "
                  << query_callbacks[0].data.result.min_distance << std::endl;
        std::cout << "query_callbacks[j].data.result.min_distance: "
                  << query_callbacks[j].data.result.min_distance << std::endl;
      }
    }
  }

  for (std::size_t i = 0; i < env.size(); ++i) delete env[i];
  for (std::size_t i = 0; i < query.size(); ++i) delete query[i];

  for (size_t i = 0; i < managers.size(); ++i) delete managers[i];

  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  int w = 7;

  std::cout << "distance timing summary" << std::endl;
  std::cout << env_size << " objs, " << query_size << " queries" << std::endl;
  std::cout << "register time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "distance time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i) {
    double tmp = 0;
    for (size_t j = 2; j < ts[i].records.size(); ++j) tmp += ts[i].records[j];
    std::cout << std::setw(w) << tmp << " ";
  }
  std::cout << std::endl;

  std::cout << "overall time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;
}
