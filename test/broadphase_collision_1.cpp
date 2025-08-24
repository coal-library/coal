/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2022, INRIA
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
/** @author Jia Pan */

#define BOOST_TEST_MODULE COAL_BROADPHASE_COLLISION_1
#include <boost/test/included/unit_test.hpp>

#include "coal/broadphase/broadphase_bruteforce.h"
#include "coal/broadphase/broadphase_spatialhash.h"
#include "coal/broadphase/broadphase_SaP.h"
#include "coal/broadphase/broadphase_SSaP.h"
#include "coal/broadphase/broadphase_interval_tree.h"
#include "coal/broadphase/broadphase_dynamic_AABB_tree.h"
#include "coal/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "coal/broadphase/default_broadphase_callbacks.h"
#include "coal/broadphase/detail/sparse_hash_table.h"
#include "coal/broadphase/detail/spatial_hash.h"
#include "utility.h"

#include <boost/math/constants/constants.hpp>

#if USE_GOOGLEHASH
#include <sparsehash/sparse_hash_map>
#include <sparsehash/dense_hash_map>
#include <hash_map>
#endif

#include <iostream>
#include <iomanip>

using namespace coal;

/// @brief make sure if broadphase algorithms doesn't check twice for the same
/// collision object pair
void broad_phase_duplicate_check_test(Scalar env_scale, std::size_t env_size,
                                      bool verbose = false);

/// @brief test for broad phase update
void broad_phase_update_collision_test(Scalar env_scale, std::size_t env_size,
                                       std::size_t query_size,
                                       std::size_t num_max_contacts = 1,
                                       bool exhaustive = false,
                                       bool use_mesh = false);

#if USE_GOOGLEHASH
template <typename U, typename V>
struct GoogleSparseHashTable
    : public google::sparse_hash_map<U, V, std::tr1::hash<size_t>,
                                     std::equal_to<size_t>> {};

template <typename U, typename V>
struct GoogleDenseHashTable
    : public google::dense_hash_map<U, V, std::tr1::hash<size_t>,
                                    std::equal_to<size_t>> {
  GoogleDenseHashTable()
      : google::dense_hash_map<U, V, std::tr1::hash<size_t>,
                               std::equal_to<size_t>>() {
    this->set_empty_key(nullptr);
  }
};
#endif

/// make sure if broadphase algorithms doesn't check twice for the same
/// collision object pair
BOOST_AUTO_TEST_CASE(test_broad_phase_dont_duplicate_check) {
#ifdef NDEBUG
  broad_phase_duplicate_check_test(2000, 1000);
#else
  broad_phase_duplicate_check_test(2000, 100);
#endif
}

/// check the update, only return collision or not
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_update_collision_binary) {
#ifdef NDEBUG
  broad_phase_update_collision_test(2000, 100, 1000, 1, false);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, false);
#else
  broad_phase_update_collision_test(2000, 10, 100, 1, false);
  broad_phase_update_collision_test(2000, 100, 100, 1, false);
#endif
}

/// check the update, return 10 contacts
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_update_collision) {
#ifdef NDEBUG
  broad_phase_update_collision_test(2000, 100, 1000, 10, false);
  broad_phase_update_collision_test(2000, 1000, 1000, 10, false);
#else
  broad_phase_update_collision_test(2000, 10, 100, 10, false);
  broad_phase_update_collision_test(2000, 100, 100, 10, false);
#endif
}

/// check the update, exhaustive
BOOST_AUTO_TEST_CASE(test_core_bf_broad_phase_update_collision_exhaustive) {
#ifdef NDEBUG
  broad_phase_update_collision_test(2000, 100, 1000, 1, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, true);
#else
  broad_phase_update_collision_test(2000, 10, 100, 1, true);
  broad_phase_update_collision_test(2000, 100, 100, 1, true);
#endif
}

/// check broad phase update, in mesh, only return collision or not
BOOST_AUTO_TEST_CASE(
    test_core_mesh_bf_broad_phase_update_collision_mesh_binary) {
#ifdef NDEBUG
  broad_phase_update_collision_test(2000, 100, 1000, 1, false, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, false, true);
#else
  broad_phase_update_collision_test(2000, 2, 4, 1, false, true);
  broad_phase_update_collision_test(2000, 4, 4, 1, false, true);
#endif
}

/// check broad phase update, in mesh, return 10 contacts
BOOST_AUTO_TEST_CASE(test_core_mesh_bf_broad_phase_update_collision_mesh) {
#ifdef NDEBUG
  broad_phase_update_collision_test(2000, 100, 1000, 10, false, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 10, false, true);
#else
  broad_phase_update_collision_test(200, 2, 4, 10, false, true);
  broad_phase_update_collision_test(200, 4, 4, 10, false, true);
#endif
}

/// check broad phase update, in mesh, exhaustive
BOOST_AUTO_TEST_CASE(
    test_core_mesh_bf_broad_phase_update_collision_mesh_exhaustive) {
#ifdef NDEBUG
  broad_phase_update_collision_test(2000, 100, 1000, 1, true, true);
  broad_phase_update_collision_test(2000, 1000, 1000, 1, true, true);
#else
  broad_phase_update_collision_test(2000, 2, 4, 1, true, true);
  broad_phase_update_collision_test(2000, 4, 4, 1, true, true);
#endif
}

//==============================================================================
struct CollisionDataForUniquenessChecking {
  std::set<std::pair<CollisionObject*, CollisionObject*>> checkedPairs;

  bool checkUniquenessAndAddPair(CollisionObject* o1, CollisionObject* o2) {
    auto search = checkedPairs.find(std::make_pair(o1, o2));

    if (search != checkedPairs.end()) return false;

    checkedPairs.emplace(o1, o2);

    return true;
  }
};

//==============================================================================
struct CollisionFunctionForUniquenessChecking : CollisionCallBackBase {
  bool collide(CollisionObject* o1, CollisionObject* o2) {
    BOOST_CHECK(data.checkUniquenessAndAddPair(o1, o2));
    return false;
  }

  CollisionDataForUniquenessChecking data;
};

//==============================================================================
void broad_phase_duplicate_check_test(Scalar env_scale, std::size_t env_size,
                                      bool verbose) {
  std::vector<TStruct> ts;
  std::vector<BenchTimer> timers;

  std::vector<CollisionObject*> env;
  generateEnvironments(env, env_scale, env_size);

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
  managers.push_back(
      new SpatialHashingCollisionManager<
          detail::SparseHashTable<AABB, CollisionObject*, detail::SpatialHash>>(
          cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(
      new SpatialHashingCollisionManager<detail::SparseHashTable<
          AABB, CollisionObject*, detail::SpatialHash, GoogleSparseHashTable>>(
          cell_size, lower_limit, upper_limit));
  managers.push_back(
      new SpatialHashingCollisionManager<detail::SparseHashTable<
          AABB, CollisionObject*, detail::SpatialHash, GoogleDenseHashTable>>(
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

  // update the environment
  Scalar delta_angle_max =
      Scalar(10) / Scalar(360 * 2) * boost::math::constants::pi<Scalar>();
  Scalar delta_trans_max = Scalar(0.01) * env_scale;
  const Scalar half = Scalar(0.5);
  for (size_t i = 0; i < env.size(); ++i) {
    Scalar rand_angle_x =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_angle_max;
    Scalar rand_trans_x =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_trans_max;
    Scalar rand_angle_y =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_angle_max;
    Scalar rand_trans_y =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_trans_max;
    Scalar rand_angle_z =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_angle_max;
    Scalar rand_trans_z =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_trans_max;

    using AngleAxis = Eigen::AngleAxis<Scalar>;
    Matrix3s dR(AngleAxis(rand_angle_x, Vec3s::UnitX()) *
                AngleAxis(rand_angle_y, Vec3s::UnitY()) *
                AngleAxis(rand_angle_z, Vec3s::UnitZ()));
    Vec3s dT(rand_trans_x, rand_trans_y, rand_trans_z);

    Matrix3s R = env[i]->getRotation();
    Vec3s T = env[i]->getTranslation();
    env[i]->setTransform(dR * R, dR * T + dT);
    env[i]->computeAABB();
  }

  for (size_t i = 0; i < managers.size(); ++i) {
    timers[i].start();
    managers[i]->update();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  std::vector<CollisionDataForUniquenessChecking> self_data(managers.size());

  for (size_t i = 0; i < managers.size(); ++i) {
    CollisionFunctionForUniquenessChecking callback;
    timers[i].start();
    managers[i]->collide(&callback);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for (auto obj : env) delete obj;

  if (!verbose) return;

  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  int w = 7;

  std::cout << "collision timing summary" << std::endl;
  std::cout << env_size << " objs" << std::endl;
  std::cout << "register time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "update time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[2] << " ";
  std::cout << std::endl;

  std::cout << "self collision time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[3] << " ";
  std::cout << std::endl;

  std::cout << "collision time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i) {
    double tmp = 0;
    for (size_t j = 4; j < ts[i].records.size(); ++j) tmp += ts[i].records[j];
    std::cout << std::setw(w) << tmp << " ";
  }
  std::cout << std::endl;

  std::cout << "overall time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;
}

void broad_phase_update_collision_test(Scalar env_scale, std::size_t env_size,
                                       std::size_t query_size,
                                       std::size_t num_max_contacts,
                                       bool exhaustive, bool use_mesh) {
  std::vector<TStruct> ts;
  std::vector<BenchTimer> timers;

  std::vector<CollisionObject*> env;
  if (use_mesh)
    generateEnvironmentsMesh(env, env_scale, env_size);
  else
    generateEnvironments(env, env_scale, env_size);

  std::vector<CollisionObject*> query;
  if (use_mesh)
    generateEnvironmentsMesh(query, env_scale, query_size);
  else
    generateEnvironments(query, env_scale, query_size);

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
  // managers.push_back(new SpatialHashingCollisionManager(cell_size,
  // lower_limit, upper_limit));
  managers.push_back(
      new SpatialHashingCollisionManager<
          detail::SparseHashTable<AABB, CollisionObject*, detail::SpatialHash>>(
          cell_size, lower_limit, upper_limit));
#if USE_GOOGLEHASH
  managers.push_back(
      new SpatialHashingCollisionManager<detail::SparseHashTable<
          AABB, CollisionObject*, detail::SpatialHash, GoogleSparseHashTable>>(
          cell_size, lower_limit, upper_limit));
  managers.push_back(
      new SpatialHashingCollisionManager<detail::SparseHashTable<
          AABB, CollisionObject*, detail::SpatialHash, GoogleDenseHashTable>>(
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

  // update the environment
  Scalar delta_angle_max =
      Scalar(10) / Scalar(360 * 2) * boost::math::constants::pi<Scalar>();
  Scalar delta_trans_max = Scalar(0.01) * env_scale;
  const Scalar half = Scalar(0.5);
  for (size_t i = 0; i < env.size(); ++i) {
    Scalar rand_angle_x =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_angle_max;
    Scalar rand_trans_x =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_trans_max;
    Scalar rand_angle_y =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_angle_max;
    Scalar rand_trans_y =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_trans_max;
    Scalar rand_angle_z =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_angle_max;
    Scalar rand_trans_z =
        2 * (Scalar(rand()) / Scalar(RAND_MAX) - half) * delta_trans_max;

    using AngleAxis = Eigen::AngleAxis<Scalar>;
    Matrix3s dR(AngleAxis(rand_angle_x, Vec3s::UnitX()) *
                AngleAxis(rand_angle_y, Vec3s::UnitY()) *
                AngleAxis(rand_angle_z, Vec3s::UnitZ()));
    Vec3s dT(rand_trans_x, rand_trans_y, rand_trans_z);

    Matrix3s R = env[i]->getRotation();
    Vec3s T = env[i]->getTranslation();
    env[i]->setTransform(dR * R, dR * T + dT);
    env[i]->computeAABB();
  }

  for (size_t i = 0; i < managers.size(); ++i) {
    timers[i].start();
    managers[i]->update();
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  std::vector<CollisionData> self_data(managers.size());
  for (size_t i = 0; i < managers.size(); ++i) {
    if (exhaustive)
      self_data[i].request.num_max_contacts = 100000;
    else
      self_data[i].request.num_max_contacts = num_max_contacts;
  }

  for (size_t i = 0; i < managers.size(); ++i) {
    CollisionCallBackDefault callback;
    timers[i].start();
    managers[i]->collide(&callback);
    timers[i].stop();
    ts[i].push_back(timers[i].getElapsedTime());
  }

  for (size_t i = 0; i < managers.size(); ++i)
    std::cout << self_data[i].result.numContacts() << " ";
  std::cout << std::endl;

  if (exhaustive) {
    for (size_t i = 1; i < managers.size(); ++i)
      BOOST_CHECK(self_data[i].result.numContacts() ==
                  self_data[0].result.numContacts());
  } else {
    std::vector<bool> self_res(managers.size());
    for (size_t i = 0; i < self_res.size(); ++i)
      self_res[i] = (self_data[i].result.numContacts() > 0);

    for (size_t i = 1; i < self_res.size(); ++i)
      BOOST_CHECK(self_res[0] == self_res[i]);

    for (size_t i = 1; i < managers.size(); ++i)
      BOOST_CHECK(self_data[i].result.numContacts() ==
                  self_data[0].result.numContacts());
  }

  for (size_t i = 0; i < query.size(); ++i) {
    std::vector<CollisionCallBackDefault> query_callbacks(managers.size());

    for (size_t j = 0; j < query_callbacks.size(); ++j) {
      if (exhaustive)
        query_callbacks[j].data.request.num_max_contacts = 100000;
      else
        query_callbacks[j].data.request.num_max_contacts = num_max_contacts;
    }

    for (size_t j = 0; j < query_callbacks.size(); ++j) {
      timers[j].start();
      managers[j]->collide(query[i], &query_callbacks[j]);
      timers[j].stop();
      ts[j].push_back(timers[j].getElapsedTime());
    }

    // for(size_t j = 0; j < managers.size(); ++j)
    //   std::cout << query_callbacks[j].result.numContacts() << " ";
    // std::cout << std::endl;

    if (exhaustive) {
      for (size_t j = 1; j < managers.size(); ++j)
        BOOST_CHECK(query_callbacks[j].data.result.numContacts() ==
                    query_callbacks[0].data.result.numContacts());
    } else {
      std::vector<bool> query_res(managers.size());
      for (size_t j = 0; j < query_res.size(); ++j)
        query_res[j] = (query_callbacks[j].data.result.numContacts() > 0);
      for (size_t j = 1; j < query_res.size(); ++j)
        BOOST_CHECK(query_res[0] == query_res[j]);

      for (size_t j = 1; j < managers.size(); ++j)
        BOOST_CHECK(query_callbacks[j].data.result.numContacts() ==
                    query_callbacks[0].data.result.numContacts());
    }
  }

  for (size_t i = 0; i < env.size(); ++i) delete env[i];
  for (size_t i = 0; i < query.size(); ++i) delete query[i];

  for (size_t i = 0; i < managers.size(); ++i) delete managers[i];

  std::cout.setf(std::ios_base::left, std::ios_base::adjustfield);
  int w = 7;

  std::cout << "collision timing summary" << std::endl;
  std::cout << env_size << " objs, " << query_size << " queries" << std::endl;
  std::cout << "register time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[0] << " ";
  std::cout << std::endl;

  std::cout << "setup time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[1] << " ";
  std::cout << std::endl;

  std::cout << "update time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[2] << " ";
  std::cout << std::endl;

  std::cout << "self collision time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].records[3] << " ";
  std::cout << std::endl;

  std::cout << "collision time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i) {
    double tmp = 0;
    for (size_t j = 4; j < ts[i].records.size(); ++j) tmp += ts[i].records[j];
    std::cout << std::setw(w) << tmp << " ";
  }
  std::cout << std::endl;

  std::cout << "overall time" << std::endl;
  for (size_t i = 0; i < ts.size(); ++i)
    std::cout << std::setw(w) << ts[i].overall_time << " ";
  std::cout << std::endl;
  std::cout << std::endl;
}
