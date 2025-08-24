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

#include "coal/broadphase/broadphase_bruteforce.h"

#include <iterator>
#include <algorithm>
#include "coal/tracy.hh"

namespace coal {

//==============================================================================
NaiveCollisionManager::NaiveCollisionManager() {
  // Do nothing
}

//==============================================================================
void NaiveCollisionManager::registerObjects(
    const std::vector<CollisionObject*>& other_objs) {
  std::copy(other_objs.begin(), other_objs.end(), std::back_inserter(objs));
}

//==============================================================================
void NaiveCollisionManager::unregisterObject(CollisionObject* obj) {
  objs.remove(obj);
}

//==============================================================================
void NaiveCollisionManager::registerObject(CollisionObject* obj) {
  objs.push_back(obj);
}

//==============================================================================
void NaiveCollisionManager::setup() {
  // Do nothing
}

//==============================================================================
void NaiveCollisionManager::update() {
  // Do nothing
}

//==============================================================================
void NaiveCollisionManager::clear() { objs.clear(); }

//==============================================================================
void NaiveCollisionManager::getObjects(
    std::vector<CollisionObject*>& objs_) const {
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

//==============================================================================
void NaiveCollisionManager::collide(CollisionObject* obj,
                                    CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::NaiveCollisionManager::collide(CollisionObject*, "
      "CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  for (auto* obj2 : objs) {
    if ((*callback)(obj, obj2)) return;
  }
}

//==============================================================================
void NaiveCollisionManager::distance(CollisionObject* obj,
                                     DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::NaiveCollisionManager::distance(CollisionObject*, "
      "DistanceCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  for (auto* obj2 : objs) {
    if (obj->getAABB().distance(obj2->getAABB()) < min_dist) {
      if ((*callback)(obj, obj2, min_dist)) return;
    }
  }
}

//==============================================================================
void NaiveCollisionManager::collide(CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::NaiveCollisionManager::collide(CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  for (typename std::list<CollisionObject*>::const_iterator it1 = objs.begin(),
                                                            end = objs.end();
       it1 != end; ++it1) {
    typename std::list<CollisionObject*>::const_iterator it2 = it1;
    it2++;
    for (; it2 != end; ++it2) {
      if ((*it1)->getAABB().overlap((*it2)->getAABB())) {
        if ((*callback)(*it1, *it2)) return;
      }
    }
  }
}

//==============================================================================
void NaiveCollisionManager::distance(DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::NaiveCollisionManager::distance(CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  for (typename std::list<CollisionObject*>::const_iterator it1 = objs.begin(),
                                                            end = objs.end();
       it1 != end; ++it1) {
    typename std::list<CollisionObject*>::const_iterator it2 = it1;
    it2++;
    for (; it2 != end; ++it2) {
      if ((*it1)->getAABB().distance((*it2)->getAABB()) < min_dist) {
        if ((*callback)(*it1, *it2, min_dist)) return;
      }
    }
  }
}

//==============================================================================
void NaiveCollisionManager::collide(BroadPhaseCollisionManager* other_manager_,
                                    CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::NaiveCollisionManager::collide(BroadPhaseCollisionManager*, "
      "CollisionCallBackBase*)");
  callback->init();
  NaiveCollisionManager* other_manager =
      static_cast<NaiveCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    collide(callback);
    return;
  }

  for (auto* obj1 : objs) {
    for (auto* obj2 : other_manager->objs) {
      if (obj1->getAABB().overlap(obj2->getAABB())) {
        if ((*callback)(obj1, obj2)) return;
      }
    }
  }
}

//==============================================================================
void NaiveCollisionManager::distance(BroadPhaseCollisionManager* other_manager_,
                                     DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::NaiveCollisionManager::distance(BroadPhaseCollisionManager*, "
      "DistanceCallBackBase*)");
  callback->init();
  NaiveCollisionManager* other_manager =
      static_cast<NaiveCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    distance(callback);
    return;
  }

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  for (auto* obj1 : objs) {
    for (auto* obj2 : other_manager->objs) {
      if (obj1->getAABB().distance(obj2->getAABB()) < min_dist) {
        if ((*callback)(obj1, obj2, min_dist)) return;
      }
    }
  }
}

//==============================================================================
bool NaiveCollisionManager::empty() const { return objs.empty(); }

//==============================================================================
size_t NaiveCollisionManager::size() const { return objs.size(); }

}  // namespace coal
