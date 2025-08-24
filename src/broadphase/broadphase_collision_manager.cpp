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

#include "coal/broadphase/broadphase_collision_manager.h"

namespace coal {

namespace detail {
struct CollisionCallBackFunctorWrapper : CollisionCallBackBase {
  CollisionCallBackFunctorWrapper(const CollisionCallBackFunctor& functor)
      : m_functor(&functor) {}

  void init() override {}

  bool collide(CollisionObject* o1, CollisionObject* o2) override {
    return (*m_functor)(o1, o2);
  }

  CollisionCallBackFunctor const* m_functor;
};

struct DistanceCallBackFunctorWrapper : DistanceCallBackBase {
  DistanceCallBackFunctorWrapper(const DistanceCallBackFunctor& functor)
      : m_functor(&functor) {}

  void init() override {}

  bool distance(CollisionObject* o1, CollisionObject* o2,
                Scalar& dist) override {
    return (*m_functor)(o1, o2, dist);
  }

  DistanceCallBackFunctor const* m_functor;
};
}  // namespace detail

//==============================================================================
BroadPhaseCollisionManager::BroadPhaseCollisionManager()
    : enable_tested_set_(false) {
  // Do nothing
}

//==============================================================================
BroadPhaseCollisionManager::~BroadPhaseCollisionManager() {
  // Do nothing
}

//==============================================================================
void BroadPhaseCollisionManager::registerObjects(
    const std::vector<CollisionObject*>& other_objs) {
  for (size_t i = 0; i < other_objs.size(); ++i) registerObject(other_objs[i]);
}

//==============================================================================
void BroadPhaseCollisionManager::update(CollisionObject* updated_obj) {
  COAL_UNUSED_VARIABLE(updated_obj);

  update();
}

//==============================================================================
void BroadPhaseCollisionManager::collide(
    CollisionObject* obj, const CollisionCallBackFunctor& fn) const {
  detail::CollisionCallBackFunctorWrapper wrapper{fn};
  this->collide(obj, &wrapper);
}

//==============================================================================
void BroadPhaseCollisionManager::collide(
    const CollisionCallBackFunctor& fn) const {
  detail::CollisionCallBackFunctorWrapper wrapper{fn};
  this->collide(&wrapper);
}

//==============================================================================
void BroadPhaseCollisionManager::collide(
    BroadPhaseCollisionManager* other_manager,
    const CollisionCallBackFunctor& fn) const {
  detail::CollisionCallBackFunctorWrapper wrapper{fn};
  this->collide(other_manager, &wrapper);
}

//==============================================================================
void BroadPhaseCollisionManager::distance(
    CollisionObject* obj, const DistanceCallBackFunctor& fn) const {
  detail::DistanceCallBackFunctorWrapper wrapper{fn};
  this->distance(obj, &wrapper);
}

//==============================================================================
void BroadPhaseCollisionManager::distance(
    const DistanceCallBackFunctor& fn) const {
  detail::DistanceCallBackFunctorWrapper wrapper{fn};
  this->distance(&wrapper);
}

//==============================================================================
void BroadPhaseCollisionManager::distance(
    BroadPhaseCollisionManager* other_manager,
    const DistanceCallBackFunctor& fn) const {
  detail::DistanceCallBackFunctorWrapper wrapper{fn};
  this->distance(other_manager, &wrapper);
}

//==============================================================================
void BroadPhaseCollisionManager::update(
    const std::vector<CollisionObject*>& updated_objs) {
  COAL_UNUSED_VARIABLE(updated_objs);

  update();
}

//==============================================================================
bool BroadPhaseCollisionManager::inTestedSet(CollisionObject* a,
                                             CollisionObject* b) const {
  if (a < b)
    return tested_set.find(std::make_pair(a, b)) != tested_set.end();
  else
    return tested_set.find(std::make_pair(b, a)) != tested_set.end();
}

//==============================================================================
void BroadPhaseCollisionManager::insertTestedSet(CollisionObject* a,
                                                 CollisionObject* b) const {
  if (a < b)
    tested_set.insert(std::make_pair(a, b));
  else
    tested_set.insert(std::make_pair(b, a));
}

}  // namespace coal
