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

#ifndef COAL_BROADPHASE_BROADPAHSESPATIALHASH_INL_H
#define COAL_BROADPHASE_BROADPAHSESPATIALHASH_INL_H

#include "coal/broadphase/broadphase_spatialhash.h"

namespace coal {

//==============================================================================
template <typename HashTable>
SpatialHashingCollisionManager<HashTable>::SpatialHashingCollisionManager(
    Scalar cell_size, const Vec3s& scene_min, const Vec3s& scene_max,
    unsigned int default_table_size)
    : scene_limit(AABB(scene_min, scene_max)),
      hash_table(new HashTable(detail::SpatialHash(scene_limit, cell_size))) {
  hash_table->init(default_table_size);
}

//==============================================================================
template <typename HashTable>
SpatialHashingCollisionManager<HashTable>::~SpatialHashingCollisionManager() {
  delete hash_table;
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::registerObject(
    CollisionObject* obj) {
  objs.push_back(obj);

  const AABB& obj_aabb = obj->getAABB();
  AABB overlap_aabb;

  if (scene_limit.overlap(obj_aabb, overlap_aabb)) {
    if (!scene_limit.contain(obj_aabb))
      objs_partially_penetrating_scene_limit.push_back(obj);

    hash_table->insert(overlap_aabb, obj);
  } else {
    objs_outside_scene_limit.push_back(obj);
  }

  obj_aabb_map[obj] = obj_aabb;
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::unregisterObject(
    CollisionObject* obj) {
  objs.remove(obj);

  const AABB& obj_aabb = obj->getAABB();
  AABB overlap_aabb;

  if (scene_limit.overlap(obj_aabb, overlap_aabb)) {
    if (!scene_limit.contain(obj_aabb))
      objs_partially_penetrating_scene_limit.remove(obj);

    hash_table->remove(overlap_aabb, obj);
  } else {
    objs_outside_scene_limit.remove(obj);
  }

  obj_aabb_map.erase(obj);
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::setup() {
  // Do nothing
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::update() {
  hash_table->clear();
  objs_partially_penetrating_scene_limit.clear();
  objs_outside_scene_limit.clear();

  for (auto it = objs.cbegin(), end = objs.cend(); it != end; ++it) {
    CollisionObject* obj = *it;
    const AABB& obj_aabb = obj->getAABB();
    AABB overlap_aabb;

    if (scene_limit.overlap(obj_aabb, overlap_aabb)) {
      if (!scene_limit.contain(obj_aabb))
        objs_partially_penetrating_scene_limit.push_back(obj);

      hash_table->insert(overlap_aabb, obj);
    } else {
      objs_outside_scene_limit.push_back(obj);
    }

    obj_aabb_map[obj] = obj_aabb;
  }
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::update(
    CollisionObject* updated_obj) {
  const AABB& new_aabb = updated_obj->getAABB();
  const AABB& old_aabb = obj_aabb_map[updated_obj];

  AABB old_overlap_aabb;
  const auto is_old_aabb_overlapping =
      scene_limit.overlap(old_aabb, old_overlap_aabb);
  if (is_old_aabb_overlapping)
    hash_table->remove(old_overlap_aabb, updated_obj);

  AABB new_overlap_aabb;
  const auto is_new_aabb_overlapping =
      scene_limit.overlap(new_aabb, new_overlap_aabb);
  if (is_new_aabb_overlapping)
    hash_table->insert(new_overlap_aabb, updated_obj);

  ObjectStatus old_status;
  if (is_old_aabb_overlapping) {
    if (scene_limit.contain(old_aabb))
      old_status = Inside;
    else
      old_status = PartiallyPenetrating;
  } else {
    old_status = Outside;
  }

  if (is_new_aabb_overlapping) {
    if (scene_limit.contain(new_aabb)) {
      if (old_status == PartiallyPenetrating) {
        // Status change: PartiallyPenetrating --> Inside
        // Required action(s):
        // - remove object from "objs_partially_penetrating_scene_limit"

        auto find_it = std::find(objs_partially_penetrating_scene_limit.begin(),
                                 objs_partially_penetrating_scene_limit.end(),
                                 updated_obj);
        objs_partially_penetrating_scene_limit.erase(find_it);
      } else if (old_status == Outside) {
        // Status change: Outside --> Inside
        // Required action(s):
        // - remove object from "objs_outside_scene_limit"

        auto find_it = std::find(objs_outside_scene_limit.begin(),
                                 objs_outside_scene_limit.end(), updated_obj);
        objs_outside_scene_limit.erase(find_it);
      }
    } else {
      if (old_status == Inside) {
        // Status change: Inside --> PartiallyPenetrating
        // Required action(s):
        // - add object to "objs_partially_penetrating_scene_limit"

        objs_partially_penetrating_scene_limit.push_back(updated_obj);
      } else if (old_status == Outside) {
        // Status change: Outside --> PartiallyPenetrating
        // Required action(s):
        // - remove object from "objs_outside_scene_limit"
        // - add object to "objs_partially_penetrating_scene_limit"

        auto find_it = std::find(objs_outside_scene_limit.begin(),
                                 objs_outside_scene_limit.end(), updated_obj);
        objs_outside_scene_limit.erase(find_it);

        objs_partially_penetrating_scene_limit.push_back(updated_obj);
      }
    }
  } else {
    if (old_status == Inside) {
      // Status change: Inside --> Outside
      // Required action(s):
      // - add object to "objs_outside_scene_limit"

      objs_outside_scene_limit.push_back(updated_obj);
    } else if (old_status == PartiallyPenetrating) {
      // Status change: PartiallyPenetrating --> Outside
      // Required action(s):
      // - remove object from "objs_partially_penetrating_scene_limit"
      // - add object to "objs_outside_scene_limit"

      auto find_it =
          std::find(objs_partially_penetrating_scene_limit.begin(),
                    objs_partially_penetrating_scene_limit.end(), updated_obj);
      objs_partially_penetrating_scene_limit.erase(find_it);

      objs_outside_scene_limit.push_back(updated_obj);
    }
  }

  obj_aabb_map[updated_obj] = new_aabb;
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::update(
    const std::vector<CollisionObject*>& updated_objs) {
  for (size_t i = 0; i < updated_objs.size(); ++i) update(updated_objs[i]);
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::clear() {
  objs.clear();
  hash_table->clear();
  objs_outside_scene_limit.clear();
  obj_aabb_map.clear();
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::getObjects(
    std::vector<CollisionObject*>& objs_) const {
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::collide(
    CollisionObject* obj, CollisionCallBackBase* callback) const {
  if (size() == 0) return;
  collide_(obj, callback);
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::distance(
    CollisionObject* obj, DistanceCallBackBase* callback) const {
  if (size() == 0) return;
  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  distance_(obj, callback, min_dist);
}

//==============================================================================
template <typename HashTable>
bool SpatialHashingCollisionManager<HashTable>::collide_(
    CollisionObject* obj, CollisionCallBackBase* callback) const {
  const auto& obj_aabb = obj->getAABB();
  AABB overlap_aabb;

  if (scene_limit.overlap(obj_aabb, overlap_aabb)) {
    const auto query_result = hash_table->query(overlap_aabb);
    for (const auto& obj2 : query_result) {
      if (obj == obj2) continue;

      if ((*callback)(obj, obj2)) return true;
    }

    if (!scene_limit.contain(obj_aabb)) {
      for (const auto& obj2 : objs_outside_scene_limit) {
        if (obj == obj2) continue;

        if ((*callback)(obj, obj2)) return true;
      }
    }
  } else {
    for (const auto& obj2 : objs_partially_penetrating_scene_limit) {
      if (obj == obj2) continue;

      if ((*callback)(obj, obj2)) return true;
    }

    for (const auto& obj2 : objs_outside_scene_limit) {
      if (obj == obj2) continue;

      if ((*callback)(obj, obj2)) return true;
    }
  }

  return false;
}

//==============================================================================
template <typename HashTable>
bool SpatialHashingCollisionManager<HashTable>::distance_(
    CollisionObject* obj, DistanceCallBackBase* callback,
    Scalar& min_dist) const {
  auto delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  auto aabb = obj->getAABB();
  if (min_dist < (std::numeric_limits<Scalar>::max)()) {
    Vec3s min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  AABB overlap_aabb;

  auto status = 1;
  Scalar old_min_distance;

  while (1) {
    old_min_distance = min_dist;

    if (scene_limit.overlap(aabb, overlap_aabb)) {
      if (distanceObjectToObjects(obj, hash_table->query(overlap_aabb),
                                  callback, min_dist)) {
        return true;
      }

      if (!scene_limit.contain(aabb)) {
        if (distanceObjectToObjects(obj, objs_outside_scene_limit, callback,
                                    min_dist)) {
          return true;
        }
      }
    } else {
      if (distanceObjectToObjects(obj, objs_partially_penetrating_scene_limit,
                                  callback, min_dist)) {
        return true;
      }

      if (distanceObjectToObjects(obj, objs_outside_scene_limit, callback,
                                  min_dist)) {
        return true;
      }
    }

    if (status == 1) {
      if (old_min_distance < (std::numeric_limits<Scalar>::max)()) {
        break;
      } else {
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
    } else if (status == 0) {
      break;
    }
  }

  return false;
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::collide(
    CollisionCallBackBase* callback) const {
  if (size() == 0) return;

  for (const auto& obj1 : objs) {
    const auto& obj_aabb = obj1->getAABB();
    AABB overlap_aabb;

    if (scene_limit.overlap(obj_aabb, overlap_aabb)) {
      auto query_result = hash_table->query(overlap_aabb);
      for (const auto& obj2 : query_result) {
        if (obj1 < obj2) {
          if ((*callback)(obj1, obj2)) return;
        }
      }

      if (!scene_limit.contain(obj_aabb)) {
        for (const auto& obj2 : objs_outside_scene_limit) {
          if (obj1 < obj2) {
            if ((*callback)(obj1, obj2)) return;
          }
        }
      }
    } else {
      for (const auto& obj2 : objs_partially_penetrating_scene_limit) {
        if (obj1 < obj2) {
          if ((*callback)(obj1, obj2)) return;
        }
      }

      for (const auto& obj2 : objs_outside_scene_limit) {
        if (obj1 < obj2) {
          if ((*callback)(obj1, obj2)) return;
        }
      }
    }
  }
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::distance(
    DistanceCallBackBase* callback) const {
  if (size() == 0) return;

  this->enable_tested_set_ = true;
  this->tested_set.clear();

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();

  for (const auto& obj : objs) {
    if (distance_(obj, callback, min_dist)) break;
  }

  this->enable_tested_set_ = false;
  this->tested_set.clear();
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::collide(
    BroadPhaseCollisionManager* other_manager_,
    CollisionCallBackBase* callback) const {
  auto* other_manager =
      static_cast<SpatialHashingCollisionManager<HashTable>*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    collide(callback);
    return;
  }

  if (this->size() < other_manager->size()) {
    for (const auto& obj : objs) {
      if (other_manager->collide_(obj, callback)) return;
    }
  } else {
    for (const auto& obj : other_manager->objs) {
      if (collide_(obj, callback)) return;
    }
  }
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::distance(
    BroadPhaseCollisionManager* other_manager_,
    DistanceCallBackBase* callback) const {
  auto* other_manager =
      static_cast<SpatialHashingCollisionManager<HashTable>*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    distance(callback);
    return;
  }

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();

  if (this->size() < other_manager->size()) {
    for (const auto& obj : objs)
      if (other_manager->distance_(obj, callback, min_dist)) return;
  } else {
    for (const auto& obj : other_manager->objs)
      if (distance_(obj, callback, min_dist)) return;
  }
}

//==============================================================================
template <typename HashTable>
bool SpatialHashingCollisionManager<HashTable>::empty() const {
  return objs.empty();
}

//==============================================================================
template <typename HashTable>
size_t SpatialHashingCollisionManager<HashTable>::size() const {
  return objs.size();
}

//==============================================================================
template <typename HashTable>
void SpatialHashingCollisionManager<HashTable>::computeBound(
    std::vector<CollisionObject*>& objs, Vec3s& l, Vec3s& u) {
  AABB bound;
  for (unsigned int i = 0; i < objs.size(); ++i) bound += objs[i]->getAABB();

  l = bound.min_;
  u = bound.max_;
}

//==============================================================================
template <typename HashTable>
template <typename Container>
bool SpatialHashingCollisionManager<HashTable>::distanceObjectToObjects(
    CollisionObject* obj, const Container& objs, DistanceCallBackBase* callback,
    Scalar& min_dist) const {
  for (auto& obj2 : objs) {
    if (obj == obj2) continue;

    if (!this->enable_tested_set_) {
      if (obj->getAABB().distance(obj2->getAABB()) < min_dist) {
        if ((*callback)(obj, obj2, min_dist)) return true;
      }
    } else {
      if (!this->inTestedSet(obj, obj2)) {
        if (obj->getAABB().distance(obj2->getAABB()) < min_dist) {
          if ((*callback)(obj, obj2, min_dist)) return true;
        }

        this->insertTestedSet(obj, obj2);
      }
    }
  }

  return false;
}

}  // namespace coal

#endif
