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

#ifndef COAL_BROAD_PHASE_SAP_H
#define COAL_BROAD_PHASE_SAP_H

#include <map>
#include <list>

#include "coal/broadphase/broadphase_collision_manager.h"

namespace coal {

/// @brief Rigorous SAP collision manager
class COAL_DLLAPI SaPCollisionManager : public BroadPhaseCollisionManager {
 public:
  typedef BroadPhaseCollisionManager Base;
  using Base::getObjects;

  SaPCollisionManager();

  ~SaPCollisionManager();

  /// @brief add objects to the manager
  void registerObjects(const std::vector<CollisionObject*>& other_objs);

  /// @brief remove one object from the manager
  void registerObject(CollisionObject* obj);

  /// @brief add one object to the manager
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
  void collide(BroadPhaseCollisionManager* other_manager,
               CollisionCallBackBase* callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager* other_manager,
                DistanceCallBackBase* callback) const;

  /// @brief whether the manager is empty
  bool empty() const;

  /// @brief the number of objects managed by the manager
  size_t size() const;

 protected:
  struct EndPoint;

  /// @brief SAP interval for one object
  struct SaPAABB {
    /// @brief object
    CollisionObject* obj;

    /// @brief lower bound end point of the interval
    EndPoint* lo;

    /// @brief higher bound end point of the interval
    EndPoint* hi;

    /// @brief cached AABB value
    AABB cached;
  };

  /// @brief End point for an interval
  struct EndPoint {
    /// @brief tag for whether it is a lower bound or higher bound of an
    /// interval, 0 for lo, and 1 for hi
    char minmax;

    /// @brief back pointer to SAP interval
    SaPAABB* aabb;

    /// @brief the previous end point in the end point list
    EndPoint* prev[3];

    /// @brief the next end point in the end point list
    EndPoint* next[3];

    /// @brief get the value of the end point
    const Vec3s& getVal() const;

    /// @brief set the value of the end point
    Vec3s& getVal();

    Scalar getVal(int i) const;

    Scalar& getVal(int i);
  };

  /// @brief A pair of objects that are not culling away and should further
  /// check collision
  struct SaPPair {
    SaPPair(CollisionObject* a, CollisionObject* b);

    CollisionObject* obj1;
    CollisionObject* obj2;

    bool operator==(const SaPPair& other) const;
  };

  /// @brief Functor to help unregister one object
  class COAL_DLLAPI isUnregistered {
    CollisionObject* obj;

   public:
    isUnregistered(CollisionObject* obj_);

    bool operator()(const SaPPair& pair) const;
  };

  /// @brief Functor to help remove collision pairs no longer valid (i.e.,
  /// should be culled away)
  class COAL_DLLAPI isNotValidPair {
    CollisionObject* obj1;
    CollisionObject* obj2;

   public:
    isNotValidPair(CollisionObject* obj1_, CollisionObject* obj2_);

    bool operator()(const SaPPair& pair);
  };

  void update_(SaPAABB* updated_aabb);

  void updateVelist();

  /// @brief End point list for x, y, z coordinates
  EndPoint* elist[3];

  /// @brief vector version of elist, for acceleration
  std::vector<EndPoint*> velist[3];

  /// @brief SAP interval list
  std::list<SaPAABB*> AABB_arr;

  /// @brief The pair of objects that should further check for collision
  std::list<SaPPair> overlap_pairs;

  int optimal_axis;

  std::map<CollisionObject*, SaPAABB*> obj_aabb_map;

  bool distance_(CollisionObject* obj, DistanceCallBackBase* callback,
                 Scalar& min_dist) const;

  bool collide_(CollisionObject* obj, CollisionCallBackBase* callback) const;

  void addToOverlapPairs(const SaPPair& p);

  void removeFromOverlapPairs(const SaPPair& p);
};

}  // namespace coal

#endif
