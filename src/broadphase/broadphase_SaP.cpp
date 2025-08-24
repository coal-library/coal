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

#include "coal/broadphase/broadphase_SaP.h"
#include "coal/tracy.hh"

namespace coal {

//==============================================================================
void SaPCollisionManager::unregisterObject(CollisionObject* obj) {
  auto it = AABB_arr.begin();
  for (auto end = AABB_arr.end(); it != end; ++it) {
    if ((*it)->obj == obj) break;
  }

  AABB_arr.erase(it);
  obj_aabb_map.erase(obj);

  if (it == AABB_arr.end()) return;

  SaPAABB* curr = *it;
  *it = nullptr;

  for (int coord = 0; coord < 3; ++coord) {
    // first delete the lo endpoint of the interval.
    if (curr->lo->prev[coord] == nullptr)
      elist[coord] = curr->lo->next[coord];
    else
      curr->lo->prev[coord]->next[coord] = curr->lo->next[coord];

    curr->lo->next[coord]->prev[coord] = curr->lo->prev[coord];

    // then, delete the "hi" endpoint.
    if (curr->hi->prev[coord] == nullptr)
      elist[coord] = curr->hi->next[coord];
    else
      curr->hi->prev[coord]->next[coord] = curr->hi->next[coord];

    if (curr->hi->next[coord] != nullptr)
      curr->hi->next[coord]->prev[coord] = curr->hi->prev[coord];
  }

  delete curr->lo;
  delete curr->hi;
  delete curr;

  overlap_pairs.remove_if(isUnregistered(obj));
}

//==============================================================================
SaPCollisionManager::SaPCollisionManager() {
  elist[0] = nullptr;
  elist[1] = nullptr;
  elist[2] = nullptr;

  optimal_axis = 0;
}

//==============================================================================
SaPCollisionManager::~SaPCollisionManager() { clear(); }

//==============================================================================
void SaPCollisionManager::registerObjects(
    const std::vector<CollisionObject*>& other_objs) {
  if (other_objs.empty()) return;

  if (size() > 0)
    BroadPhaseCollisionManager::registerObjects(other_objs);
  else {
    std::vector<EndPoint*> endpoints(2 * other_objs.size());

    for (size_t i = 0; i < other_objs.size(); ++i) {
      SaPAABB* sapaabb = new SaPAABB();
      sapaabb->obj = other_objs[i];
      sapaabb->lo = new EndPoint();
      sapaabb->hi = new EndPoint();
      sapaabb->cached = other_objs[i]->getAABB();
      endpoints[2 * i] = sapaabb->lo;
      endpoints[2 * i + 1] = sapaabb->hi;
      sapaabb->lo->minmax = 0;
      sapaabb->hi->minmax = 1;
      sapaabb->lo->aabb = sapaabb;
      sapaabb->hi->aabb = sapaabb;
      AABB_arr.push_back(sapaabb);
      obj_aabb_map[other_objs[i]] = sapaabb;
    }

    Scalar scale[3];
    for (int coord = 0; coord < 3; ++coord) {
      std::sort(
          endpoints.begin(), endpoints.end(),
          std::bind(std::less<Scalar>(),
                    std::bind(static_cast<Scalar (EndPoint::*)(int) const>(
                                  &EndPoint::getVal),
                              std::placeholders::_1, coord),
                    std::bind(static_cast<Scalar (EndPoint::*)(int) const>(
                                  &EndPoint::getVal),
                              std::placeholders::_2, coord)));

      endpoints[0]->prev[coord] = nullptr;
      endpoints[0]->next[coord] = endpoints[1];
      for (size_t i = 1; i < endpoints.size() - 1; ++i) {
        endpoints[i]->prev[coord] = endpoints[i - 1];
        endpoints[i]->next[coord] = endpoints[i + 1];
      }
      endpoints[endpoints.size() - 1]->prev[coord] =
          endpoints[endpoints.size() - 2];
      endpoints[endpoints.size() - 1]->next[coord] = nullptr;

      elist[coord] = endpoints[0];

      scale[coord] = endpoints.back()->aabb->cached.max_[coord] -
                     endpoints[0]->aabb->cached.min_[coord];
    }

    int axis = 0;
    if (scale[axis] < scale[1]) axis = 1;
    if (scale[axis] < scale[2]) axis = 2;

    EndPoint* pos = elist[axis];

    while (pos != nullptr) {
      EndPoint* pos_next = nullptr;
      SaPAABB* aabb = pos->aabb;
      EndPoint* pos_it = pos->next[axis];

      while (pos_it != nullptr) {
        if (pos_it->aabb == aabb) {
          if (pos_next == nullptr) pos_next = pos_it;
          break;
        }

        if (pos_it->minmax == 0) {
          if (pos_next == nullptr) pos_next = pos_it;
          if (pos_it->aabb->cached.overlap(aabb->cached))
            overlap_pairs.emplace_back(pos_it->aabb->obj, aabb->obj);
        }
        pos_it = pos_it->next[axis];
      }

      pos = pos_next;
    }
  }

  updateVelist();
}

//==============================================================================
void SaPCollisionManager::registerObject(CollisionObject* obj) {
  // Initialize a new SaPAABB associated with current Collision Object
  SaPAABB* new_sap = new SaPAABB;
  new_sap->cached = obj->getAABB();
  new_sap->obj = obj;

  new_sap->lo = new EndPoint;
  new_sap->lo->minmax = 0;
  new_sap->lo->aabb = new_sap;

  new_sap->hi = new EndPoint;
  new_sap->hi->minmax = 1;
  new_sap->hi->aabb = new_sap;
  for (int coord = 0; coord < 3; ++coord) {
    EndPoint* current = elist[coord];

    // first insert the lo end point
    if (current == nullptr)  // empty list
    {
      elist[coord] = new_sap->lo;
      new_sap->lo->prev[coord] = new_sap->lo->next[coord] = nullptr;
    } else  // otherwise, find the correct location in the list and insert
    {
      EndPoint* curr_lo = new_sap->lo;
      Scalar curr_lo_val = curr_lo->getVal()[coord];
      while ((current->getVal()[coord] < curr_lo_val) &&
             (current->next[coord] != nullptr))
        current = current->next[coord];

      if (current->getVal()[coord] >= curr_lo_val) {
        curr_lo->prev[coord] = current->prev[coord];
        curr_lo->next[coord] = current;
        if (current->prev[coord] == nullptr)  // current was the first box
          elist[coord] =
              curr_lo;  // new_sap->lo becomes the new first box on the axis
        else
          current->prev[coord]->next[coord] = curr_lo;

        current->prev[coord] =
            curr_lo;  // new_sap->lo becomes the predecessor of current
      } else {  // current->next[coord] == nullptr, so the current is just the
                // cell before current->
        curr_lo->prev[coord] = current;
        curr_lo->next[coord] = nullptr;
        current->next[coord] = curr_lo;
      }
    }

    // now insert hi end point
    current = new_sap->lo;

    EndPoint* curr_hi = new_sap->hi;
    Scalar curr_hi_val = curr_hi->getVal()[coord];

    if (coord == 0) {
      while ((current->getVal()[coord] < curr_hi_val) &&
             (current->next[coord] != nullptr)) {
        if (current != new_sap->lo)
          if (current->aabb->cached.overlap(new_sap->cached))
            overlap_pairs.emplace_back(current->aabb->obj, obj);

        current = current->next[coord];
      }
    } else {
      while ((current->getVal()[coord] < curr_hi_val) &&
             (current->next[coord] != nullptr))
        current = current->next[coord];
    }

    if (current->getVal()[coord] >= curr_hi_val) {
      curr_hi->prev[coord] = current->prev[coord];
      curr_hi->next[coord] = current;
      if (current->prev[coord] != nullptr)
        current->prev[coord]->next[coord] = curr_hi;

      current->prev[coord] = curr_hi;
    } else {
      curr_hi->prev[coord] = current;
      curr_hi->next[coord] = nullptr;
      current->next[coord] = curr_hi;
    }
  }

  AABB_arr.push_back(new_sap);

  obj_aabb_map[obj] = new_sap;

  updateVelist();
}

//==============================================================================
void SaPCollisionManager::setup() {
  if (size() == 0) return;

  Scalar scale[3];
  scale[0] = (velist[0].back())->getVal(0) - velist[0][0]->getVal(0);
  scale[1] = (velist[1].back())->getVal(1) - velist[1][0]->getVal(1);
  scale[2] = (velist[2].back())->getVal(2) - velist[2][0]->getVal(2);

  int axis = 0;
  if (scale[axis] < scale[1]) axis = 1;
  if (scale[axis] < scale[2]) axis = 2;
  optimal_axis = axis;
}

//==============================================================================
void SaPCollisionManager::update_(SaPAABB* updated_aabb) {
  if (updated_aabb->cached == updated_aabb->obj->getAABB()) return;

  SaPAABB* current = updated_aabb;

  const AABB current_aabb = current->obj->getAABB();

  const Vec3s& new_min = current_aabb.min_;
  const Vec3s& new_max = current_aabb.max_;

  for (int coord = 0; coord < 3; ++coord) {
    int direction;  // -1 reverse, 0 nochange, 1 forward
    EndPoint* temp;

    if (current->lo->getVal(coord) > new_min[coord])
      direction = -1;
    else if (current->lo->getVal(coord) < new_min[coord])
      direction = 1;
    else
      direction = 0;

    if (direction == -1) {
      // first update the "lo" endpoint of the interval
      if (current->lo->prev[coord] != nullptr) {
        temp = current->lo->prev[coord];
        while ((temp != nullptr) && (temp->getVal(coord) > new_min[coord])) {
          if (temp->minmax == 1)
            if (temp->aabb->cached.overlap(current_aabb))
              addToOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
          temp = temp->prev[coord];
        }

        if (temp == nullptr) {
          current->lo->prev[coord]->next[coord] = current->lo->next[coord];
          current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
          current->lo->prev[coord] = nullptr;
          current->lo->next[coord] = elist[coord];
          elist[coord]->prev[coord] = current->lo;
          elist[coord] = current->lo;
        } else {
          current->lo->prev[coord]->next[coord] = current->lo->next[coord];
          current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
          current->lo->prev[coord] = temp;
          current->lo->next[coord] = temp->next[coord];
          temp->next[coord]->prev[coord] = current->lo;
          temp->next[coord] = current->lo;
        }
      }

      // Update the value of the lower bound along axis coord
      current->lo->getVal(coord) = new_min[coord];

      // update hi end point
      if (current->hi->prev[coord] != nullptr) {
        temp = current->hi->prev[coord];

        while ((temp != nullptr) && (temp->getVal(coord) > new_max[coord])) {
          if ((temp->minmax == 0) &&
              (temp->aabb->cached.overlap(current->cached)))
            removeFromOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
          temp = temp->prev[coord];
        }

        current->hi->prev[coord]->next[coord] = current->hi->next[coord];
        if (current->hi->next[coord] != nullptr)
          current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
        current->hi->prev[coord] = temp;  // Wrong line
        current->hi->next[coord] = temp->next[coord];
        if (temp->next[coord] != nullptr)
          temp->next[coord]->prev[coord] = current->hi;
        temp->next[coord] = current->hi;

        current->hi->getVal(coord) = new_max[coord];
      }

      current->hi->getVal(coord) = new_max[coord];
    } else if (direction == 1) {
      // here, we first update the "hi" endpoint.
      if (current->hi->next[coord] != nullptr) {
        temp = current->hi->next[coord];
        while ((temp->next[coord] != nullptr) &&
               (temp->getVal(coord) < new_max[coord])) {
          if (temp->minmax == 0)
            if (temp->aabb->cached.overlap(current_aabb))
              addToOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
          temp = temp->next[coord];
        }

        if (temp->getVal(coord) < new_max[coord]) {
          current->hi->prev[coord]->next[coord] = current->hi->next[coord];
          current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
          current->hi->prev[coord] = temp;
          current->hi->next[coord] = nullptr;
          temp->next[coord] = current->hi;
        } else {
          current->hi->prev[coord]->next[coord] = current->hi->next[coord];
          current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
          current->hi->prev[coord] = temp->prev[coord];
          current->hi->next[coord] = temp;
          temp->prev[coord]->next[coord] = current->hi;
          temp->prev[coord] = current->hi;
        }
      }

      current->hi->getVal(coord) = new_max[coord];

      // then, update the "lo" endpoint of the interval.
      if (current->lo->next[coord] != nullptr) {
        temp = current->lo->next[coord];

        while ((temp->next[coord] != nullptr) &&
               (temp->getVal(coord) < new_min[coord])) {
          if ((temp->minmax == 1) &&
              (temp->aabb->cached.overlap(current->cached)))
            removeFromOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
          temp = temp->next[coord];
        }

        if (current->lo->prev[coord] != nullptr)
          current->lo->prev[coord]->next[coord] = current->lo->next[coord];
        else
          elist[coord] = current->lo->next[coord];
        current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
        current->lo->prev[coord] = temp->prev[coord];
        current->lo->next[coord] = temp;
        if (temp->prev[coord] != nullptr)
          temp->prev[coord]->next[coord] = current->lo;
        else
          elist[coord] = current->lo;
        temp->prev[coord] = current->lo;
        current->lo->getVal(coord) = new_min[coord];
      }
    }
  }
}

//==============================================================================
void SaPCollisionManager::updateVelist() {
  for (int coord = 0; coord < 3; ++coord) {
    velist[coord].resize(size() * 2);
    EndPoint* current = elist[coord];
    size_t id = 0;
    while (current) {
      velist[coord][id] = current;
      current = current->next[coord];
      id++;
    }
  }
}

//==============================================================================
void SaPCollisionManager::update(CollisionObject* updated_obj) {
  update_(obj_aabb_map[updated_obj]);

  updateVelist();

  setup();
}

//==============================================================================
void SaPCollisionManager::update(
    const std::vector<CollisionObject*>& updated_objs) {
  for (size_t i = 0; i < updated_objs.size(); ++i)
    update_(obj_aabb_map[updated_objs[i]]);

  updateVelist();

  setup();
}

//==============================================================================
void SaPCollisionManager::update() {
  for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it) {
    update_(*it);
  }

  updateVelist();

  setup();
}

//==============================================================================
void SaPCollisionManager::clear() {
  for (auto it = AABB_arr.begin(), end = AABB_arr.end(); it != end; ++it) {
    delete (*it)->hi;
    delete (*it)->lo;
    delete *it;
    *it = nullptr;
  }

  AABB_arr.clear();
  overlap_pairs.clear();

  elist[0] = nullptr;
  elist[1] = nullptr;
  elist[2] = nullptr;

  velist[0].clear();
  velist[1].clear();
  velist[2].clear();

  obj_aabb_map.clear();
}

//==============================================================================
void SaPCollisionManager::getObjects(
    std::vector<CollisionObject*>& objs) const {
  objs.resize(AABB_arr.size());
  size_t i = 0;
  for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end;
       ++it, ++i) {
    objs[i] = (*it)->obj;
  }
}

//==============================================================================
bool SaPCollisionManager::collide_(CollisionObject* obj,
                                   CollisionCallBackBase* callback) const {
  int axis = optimal_axis;
  const AABB& obj_aabb = obj->getAABB();

  Scalar min_val = obj_aabb.min_[axis];
  //  Scalar max_val = obj_aabb.max_[axis];

  EndPoint dummy;
  SaPAABB dummy_aabb;
  dummy_aabb.cached = obj_aabb;
  dummy.minmax = 1;
  dummy.aabb = &dummy_aabb;

  // compute stop_pos by binary search, this is cheaper than check it in while
  // iteration linearly
  const auto res_it = std::upper_bound(
      velist[axis].begin(), velist[axis].end(), &dummy,
      std::bind(std::less<Scalar>(),
                std::bind(static_cast<Scalar (EndPoint::*)(int) const>(
                              &EndPoint::getVal),
                          std::placeholders::_1, axis),
                std::bind(static_cast<Scalar (EndPoint::*)(int) const>(
                              &EndPoint::getVal),
                          std::placeholders::_2, axis)));

  EndPoint* end_pos = nullptr;
  if (res_it != velist[axis].end()) end_pos = *res_it;

  EndPoint* pos = elist[axis];

  while (pos != end_pos) {
    if (pos->aabb->obj != obj) {
      if ((pos->minmax == 0) && (pos->aabb->hi->getVal(axis) >= min_val)) {
        if (pos->aabb->cached.overlap(obj->getAABB()))
          if ((*callback)(obj, pos->aabb->obj)) return true;
      }
    }
    pos = pos->next[axis];
  }

  return false;
}

//==============================================================================
void SaPCollisionManager::addToOverlapPairs(const SaPPair& p) {
  bool repeated = false;
  for (auto it = overlap_pairs.begin(), end = overlap_pairs.end(); it != end;
       ++it) {
    if (*it == p) {
      repeated = true;
      break;
    }
  }

  if (!repeated) overlap_pairs.push_back(p);
}

//==============================================================================
void SaPCollisionManager::removeFromOverlapPairs(const SaPPair& p) {
  for (auto it = overlap_pairs.begin(), end = overlap_pairs.end(); it != end;
       ++it) {
    if (*it == p) {
      overlap_pairs.erase(it);
      break;
    }
  }

  // or overlap_pairs.remove_if(isNotValidPair(p));
}

//==============================================================================
void SaPCollisionManager::collide(CollisionObject* obj,
                                  CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SaPCollisionManager::collide(CollisionObject*, "
      "CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  collide_(obj, callback);
}

//==============================================================================
bool SaPCollisionManager::distance_(CollisionObject* obj,
                                    DistanceCallBackBase* callback,
                                    Scalar& min_dist) const {
  Vec3s delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  AABB aabb = obj->getAABB();

  if (min_dist < (std::numeric_limits<Scalar>::max)()) {
    Vec3s min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  int axis = optimal_axis;

  int status = 1;
  Scalar old_min_distance;

  EndPoint* start_pos = elist[axis];

  while (1) {
    old_min_distance = min_dist;
    Scalar min_val = aabb.min_[axis];
    //    Scalar max_val = aabb.max_[axis];

    EndPoint dummy;
    SaPAABB dummy_aabb;
    dummy_aabb.cached = aabb;
    dummy.minmax = 1;
    dummy.aabb = &dummy_aabb;

    const auto res_it = std::upper_bound(
        velist[axis].begin(), velist[axis].end(), &dummy,
        std::bind(std::less<Scalar>(),
                  std::bind(static_cast<Scalar (EndPoint::*)(int) const>(
                                &EndPoint::getVal),
                            std::placeholders::_1, axis),
                  std::bind(static_cast<Scalar (EndPoint::*)(int) const>(
                                &EndPoint::getVal),
                            std::placeholders::_2, axis)));

    EndPoint* end_pos = nullptr;
    if (res_it != velist[axis].end()) end_pos = *res_it;

    EndPoint* pos = start_pos;

    while (pos != end_pos) {
      // can change to pos->aabb->hi->getVal(axis) >= min_val - min_dist, and
      // then update start_pos to end_pos. but this seems slower.
      if ((pos->minmax == 0) && (pos->aabb->hi->getVal(axis) >= min_val)) {
        CollisionObject* curr_obj = pos->aabb->obj;
        if (curr_obj != obj) {
          if (!this->enable_tested_set_) {
            if (pos->aabb->cached.distance(obj->getAABB()) < min_dist) {
              if ((*callback)(curr_obj, obj, min_dist)) return true;
            }
          } else {
            if (!this->inTestedSet(curr_obj, obj)) {
              if (pos->aabb->cached.distance(obj->getAABB()) < min_dist) {
                if ((*callback)(curr_obj, obj, min_dist)) return true;
              }

              this->insertTestedSet(curr_obj, obj);
            }
          }
        }
      }

      pos = pos->next[axis];
    }

    if (status == 1) {
      if (old_min_distance < (std::numeric_limits<Scalar>::max)())
        break;
      else {
        if (min_dist < old_min_distance) {
          Vec3s min_dist_delta(min_dist, min_dist, min_dist);
          aabb = AABB(obj->getAABB(), min_dist_delta);
          status = 0;
        } else {
          if (aabb == obj->getAABB())
            aabb.expand(delta);
          else
            aabb.expand(obj->getAABB(), 2.0);
        }
      }
    } else if (status == 0)
      break;
  }

  return false;
}

//==============================================================================
void SaPCollisionManager::distance(CollisionObject* obj,
                                   DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SaPCollisionManager::distance(CollisionObject*, "
      "DistanceCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();

  distance_(obj, callback, min_dist);
}

//==============================================================================
void SaPCollisionManager::collide(CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SaPCollisionManager::collide(CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  for (auto it = overlap_pairs.cbegin(), end = overlap_pairs.cend(); it != end;
       ++it) {
    CollisionObject* obj1 = it->obj1;
    CollisionObject* obj2 = it->obj2;

    if ((*callback)(obj1, obj2)) return;
  }
}

//==============================================================================
void SaPCollisionManager::distance(DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SaPCollisionManager::distance(DistanceCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  this->enable_tested_set_ = true;
  this->tested_set.clear();

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();

  for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it) {
    if (distance_((*it)->obj, callback, min_dist)) break;
  }

  this->enable_tested_set_ = false;
  this->tested_set.clear();
}

//==============================================================================
void SaPCollisionManager::collide(BroadPhaseCollisionManager* other_manager_,
                                  CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SaPCollisionManager::collide(BroadPhaseCollisionManager*, "
      "CollisionCallBackBase*)");
  callback->init();
  SaPCollisionManager* other_manager =
      static_cast<SaPCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    collide(callback);
    return;
  }

  if (this->size() < other_manager->size()) {
    for (auto it = AABB_arr.cbegin(); it != AABB_arr.cend(); ++it) {
      if (other_manager->collide_((*it)->obj, callback)) return;
    }
  } else {
    for (auto it = other_manager->AABB_arr.cbegin(),
              end = other_manager->AABB_arr.cend();
         it != end; ++it) {
      if (collide_((*it)->obj, callback)) return;
    }
  }
}

//==============================================================================
void SaPCollisionManager::distance(BroadPhaseCollisionManager* other_manager_,
                                   DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SaPCollisionManager::distance(BroadPhaseCollisionManager*, "
      "DistanceCallBackBase*)");
  callback->init();
  SaPCollisionManager* other_manager =
      static_cast<SaPCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    distance(callback);
    return;
  }

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();

  if (this->size() < other_manager->size()) {
    for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it) {
      if (other_manager->distance_((*it)->obj, callback, min_dist)) return;
    }
  } else {
    for (auto it = other_manager->AABB_arr.cbegin(),
              end = other_manager->AABB_arr.cend();
         it != end; ++it) {
      if (distance_((*it)->obj, callback, min_dist)) return;
    }
  }
}

//==============================================================================
bool SaPCollisionManager::empty() const { return AABB_arr.empty(); }

//==============================================================================
size_t SaPCollisionManager::size() const { return AABB_arr.size(); }

//==============================================================================
const Vec3s& SaPCollisionManager::EndPoint::getVal() const {
  if (minmax)
    return aabb->cached.max_;
  else
    return aabb->cached.min_;
}

//==============================================================================
Vec3s& SaPCollisionManager::EndPoint::getVal() {
  if (minmax)
    return aabb->cached.max_;
  else
    return aabb->cached.min_;
}

//==============================================================================
Scalar SaPCollisionManager::EndPoint::getVal(int i) const {
  if (minmax)
    return aabb->cached.max_[i];
  else
    return aabb->cached.min_[i];
}

//==============================================================================
Scalar& SaPCollisionManager::EndPoint::getVal(int i) {
  if (minmax)
    return aabb->cached.max_[i];
  else
    return aabb->cached.min_[i];
}

//==============================================================================
SaPCollisionManager::SaPPair::SaPPair(CollisionObject* a, CollisionObject* b) {
  if (a < b) {
    obj1 = a;
    obj2 = b;
  } else {
    obj1 = b;
    obj2 = a;
  }
}

//==============================================================================
bool SaPCollisionManager::SaPPair::operator==(const SaPPair& other) const {
  return ((obj1 == other.obj1) && (obj2 == other.obj2));
}

//==============================================================================
SaPCollisionManager::isUnregistered::isUnregistered(CollisionObject* obj_)
    : obj(obj_) {}

//==============================================================================
bool SaPCollisionManager::isUnregistered::operator()(
    const SaPPair& pair) const {
  return (pair.obj1 == obj) || (pair.obj2 == obj);
}

//==============================================================================
SaPCollisionManager::isNotValidPair::isNotValidPair(CollisionObject* obj1_,
                                                    CollisionObject* obj2_)
    : obj1(obj1_), obj2(obj2_) {
  // Do nothing
}

//==============================================================================
bool SaPCollisionManager::isNotValidPair::operator()(const SaPPair& pair) {
  return (pair.obj1 == obj1) && (pair.obj2 == obj2);
}

}  // namespace coal
