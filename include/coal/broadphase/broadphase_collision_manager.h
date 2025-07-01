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

#ifndef COAL_BROADPHASE_BROADPHASECOLLISIONMANAGER_H
#define COAL_BROADPHASE_BROADPHASECOLLISIONMANAGER_H

#include <set>
#include <vector>
#include <functional>

#include "coal/collision_object.h"
#include "coal/broadphase/broadphase_callbacks.h"

namespace coal {

using CollisionCallBackFunctor =
    std::function<bool(CollisionObject*, CollisionObject*)>;

using DistanceCallBackFunctor =
    std::function<bool(CollisionObject*, CollisionObject*, Scalar&)>;

/// @brief Base class for broad phase collision. It helps to accelerate the
/// collision/distance between N objects. Also support self collision, self
/// distance and collision/distance with another M objects.
class COAL_DLLAPI BroadPhaseCollisionManager {
 public:
  BroadPhaseCollisionManager();

  virtual ~BroadPhaseCollisionManager();

  /// @brief add objects to the manager
  virtual void registerObjects(const std::vector<CollisionObject*>& other_objs);

  /// @brief add one object to the manager
  virtual void registerObject(CollisionObject* obj) = 0;

  /// @brief remove one object from the manager
  virtual void unregisterObject(CollisionObject* obj) = 0;

  /// @brief initialize the manager, related with the specific type of manager
  virtual void setup() = 0;

  /// @brief update the condition of manager
  virtual void update() = 0;

  /// @brief update the manager by explicitly given the object updated
  virtual void update(CollisionObject* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  virtual void update(const std::vector<CollisionObject*>& updated_objs);

  /// @brief clear the manager
  virtual void clear() = 0;

  /// @brief return the objects managed by the manager
  virtual void getObjects(std::vector<CollisionObject*>& objs) const = 0;

  /// @brief return the objects managed by the manager
  virtual std::vector<CollisionObject*> getObjects() const {
    std::vector<CollisionObject*> res(size());
    getObjects(res);
    return res;
  };

  /// @brief perform collision test between one object and all the objects
  /// belonging to the manager
  virtual void collide(CollisionObject* obj,
                       CollisionCallBackBase* callback) const = 0;

  /// @copybrief collide(CollisionObject*, CollisionCallBackBase*)
  void collide(CollisionObject* obj, const CollisionCallBackFunctor& fn) const;

  /// @brief perform distance computation between one object and all the objects
  /// belonging to the manager
  virtual void distance(CollisionObject* obj,
                        DistanceCallBackBase* callback) const = 0;

  /// @copybrief distance(CollisionObject*, DistanceCallBackBase*)
  void distance(CollisionObject* obj, const DistanceCallBackFunctor& fn) const;

  /// @brief perform collision test for the objects belonging to the manager
  /// (i.e., N^2 self collision)
  virtual void collide(CollisionCallBackBase* callback) const = 0;

  /// @copybrief collide(CollisionCallBackBase*)
  void collide(const CollisionCallBackFunctor& fn) const;

  /// @brief perform distance test for the objects belonging to the manager
  /// (i.e., N^2 self distance)
  virtual void distance(DistanceCallBackBase* callback) const = 0;

  /// @copybrief distance(DistanceCallBackBase*)
  void distance(const DistanceCallBackFunctor& fn) const;

  /// @brief perform collision test with objects belonging to another manager
  virtual void collide(BroadPhaseCollisionManager* other_manager,
                       CollisionCallBackBase* callback) const = 0;

  /// @copybrief collide(BroadPhaseCollisionManager*, CollisionCallBackBase*)
  void collide(BroadPhaseCollisionManager* other_manager,
               const CollisionCallBackFunctor& fn) const;

  /// @brief perform distance test with objects belonging to another manager
  virtual void distance(BroadPhaseCollisionManager* other_manager,
                        DistanceCallBackBase* callback) const = 0;

  /// @copybrief distance(BroadPhaseCollisionManager*, DistanceCallBackBase*)
  void distance(BroadPhaseCollisionManager* other_manager,
                const DistanceCallBackFunctor& fn) const;

  /// @brief whether the manager is empty
  virtual bool empty() const = 0;

  /// @brief the number of objects managed by the manager
  virtual size_t size() const = 0;

 protected:
  /// @brief tools help to avoid repeating collision or distance callback for
  /// the pairs of objects tested before. It can be useful for some of the
  /// broadphase algorithms.
  mutable std::set<std::pair<CollisionObject*, CollisionObject*> > tested_set;
  mutable bool enable_tested_set_;

  bool inTestedSet(CollisionObject* a, CollisionObject* b) const;

  void insertTestedSet(CollisionObject* a, CollisionObject* b) const;
};

}  // namespace coal

#endif
