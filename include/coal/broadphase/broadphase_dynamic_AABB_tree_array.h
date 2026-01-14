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

#ifndef COAL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_H
#define COAL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_H

#include <unordered_map>
#include <functional>
#include <limits>

#include "coal/fwd.hh"
// #include "coal/BV/utility.h"
#include "coal/shape/geometric_shapes.h"
// #include "coal/geometry/shape/utility.h"
#include "coal/broadphase/broadphase_collision_manager.h"
#include "coal/broadphase/detail/hierarchy_tree_array.h"

namespace coal {

class COAL_DLLAPI DynamicAABBTreeArrayCollisionManager
    : public BroadPhaseCollisionManager {
 public:
  typedef BroadPhaseCollisionManager Base;
  using Base::getObjects;

  using DynamicAABBNode = detail::implementation_array::NodeBase<AABB>;

  int max_tree_nonbalanced_level;
  int tree_incremental_balance_pass;
  int* tree_topdown_balance_threshold{nullptr};
  int* tree_topdown_level{nullptr};
  int tree_init_level;

  bool octree_as_geometry_collide;
  bool octree_as_geometry_distance;

  DynamicAABBTreeArrayCollisionManager();

  /// @brief add objects to the manager
  void registerObjects(const std::vector<CollisionObject*>& other_objs);

  /// @brief add one object to the manager
  void registerObject(CollisionObject* obj);

  /// @brief add one object to the manager with an AABB expanded by margin. This
  /// is useful when the narrowphase checks use a security margin. margin must
  /// be >= 0.
  void registerObject(CollisionObject* obj, Scalar margin);

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
  /// belonging to the manager
  void distance(CollisionObject* obj, DistanceCallBackBase* callback) const;

  /// @brief perform collision test for the objects belonging to the manager
  /// (i.e., N^2 self collision)
  void collide(CollisionCallBackBase* callback) const;

  /// @brief perform distance test for the objects belonging to the manager
  /// (i.e., N^2 self distance)
  void distance(DistanceCallBackBase* callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager* other_manager_,
               CollisionCallBackBase* callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager* other_manager_,
                DistanceCallBackBase* callback) const;

  /// @brief whether the manager is empty
  bool empty() const;

  /// @brief the number of objects managed by the manager
  size_t size() const;

  const detail::implementation_array::HierarchyTree<AABB>& getTree() const;

 private:
  detail::implementation_array::HierarchyTree<AABB> dtree{};
  struct ObjectInfo {
    size_t index = 0;
    Scalar margin = 0.0;
  };
  using DynamicAABBTable = std::unordered_map<CollisionObject*, ObjectInfo>;
  DynamicAABBTable table;

  bool setup_;

  void update_(CollisionObject* updated_obj);
};

}  // namespace coal

#include "coal/broadphase/broadphase_dynamic_AABB_tree_array-inl.h"

#endif
