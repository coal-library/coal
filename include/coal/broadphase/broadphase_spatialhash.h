/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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

#ifndef COAL_BROADPHASE_BROADPAHSESPATIALHASH_H
#define COAL_BROADPHASE_BROADPAHSESPATIALHASH_H

#include <list>
#include <map>
#include "coal/BV/AABB.h"
#include "coal/broadphase/broadphase_collision_manager.h"
#include "coal/broadphase/detail/simple_hash_table.h"
#include "coal/broadphase/detail/sparse_hash_table.h"
#include "coal/broadphase/detail/spatial_hash.h"

namespace coal {

/// @brief spatial hashing collision mananger
template <typename HashTable = detail::SimpleHashTable<AABB, CollisionObject*,
                                                       detail::SpatialHash> >
class SpatialHashingCollisionManager : public BroadPhaseCollisionManager {
 public:
  typedef BroadPhaseCollisionManager Base;
  using Base::getObjects;

  SpatialHashingCollisionManager(Scalar cell_size, const Vec3s& scene_min,
                                 const Vec3s& scene_max,
                                 unsigned int default_table_size = 1000);

  ~SpatialHashingCollisionManager();

  /// @brief add one object to the manager
  void registerObject(CollisionObject* obj);

  /// @brief remove one object from the manager
  void unregisterObject(CollisionObject* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  virtual void update();

  /// @brief update the manager by explicitly given the object updated
  void update(CollisionObject* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  void update(const std::vector<CollisionObject*>& updated_objs);

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /// @brief perform collision test between one object and all the objects
  /// belonging to the manager
  void collide(CollisionObject* obj, CollisionCallBackBase* callback) const;

  /// @brief perform distance computation between one object and all the objects
  /// belonging ot the manager
  void distance(CollisionObject* obj, DistanceCallBackBase* callback) const;

  /// @brief perform collision test for the objects belonging to the manager
  /// (i.e, N^2 self collision)
  void collide(CollisionCallBackBase* callback) const;

  /// @brief perform distance test for the objects belonging to the manager
  /// (i.e., N^2 self distance)
  void distance(DistanceCallBackBase* callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager* other_manager,
               CollisionCallBackBase* callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager* other_manager,
                DistanceCallBackBase* callback) const;

  /// @brief whether the manager is empty
  bool empty() const;

  /// @brief the number of objects managed by the manager
  size_t size() const;

  /// @brief compute the bound for the environent
  static void computeBound(std::vector<CollisionObject*>& objs, Vec3s& l,
                           Vec3s& u);

 protected:
  /// @brief perform collision test between one object and all the objects
  /// belonging to the manager
  bool collide_(CollisionObject* obj, CollisionCallBackBase* callback) const;

  /// @brief perform distance computation between one object and all the objects
  /// belonging ot the manager
  bool distance_(CollisionObject* obj, DistanceCallBackBase* callback,
                 Scalar& min_dist) const;

  /// @brief all objects in the scene
  std::list<CollisionObject*> objs;

  /// @brief objects partially penetrating (not totally inside nor outside) the
  /// scene limit are in another list
  std::list<CollisionObject*> objs_partially_penetrating_scene_limit;

  /// @brief objects outside the scene limit are in another list
  std::list<CollisionObject*> objs_outside_scene_limit;

  /// @brief the size of the scene
  AABB scene_limit;

  /// @brief store the map between objects and their aabbs. will make update
  /// more convenient
  std::map<CollisionObject*, AABB> obj_aabb_map;

  /// @brief objects in the scene limit (given by scene_min and scene_max) are
  /// in the spatial hash table
  HashTable* hash_table;

 private:
  enum ObjectStatus { Inside, PartiallyPenetrating, Outside };

  template <typename Container>
  bool distanceObjectToObjects(CollisionObject* obj, const Container& objs,
                               DistanceCallBackBase* callback,
                               Scalar& min_dist) const;
};

}  // namespace coal

#include "coal/broadphase/broadphase_spatialhash-inl.h"

#endif
