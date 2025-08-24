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

#include "coal/broadphase/broadphase_interval_tree.h"

namespace coal {

//==============================================================================
void IntervalTreeCollisionManager::unregisterObject(CollisionObject* obj) {
  // must sorted before
  setup();

  EndPoint p;
  p.value = obj->getAABB().min_[0];
  auto start1 = std::lower_bound(endpoints[0].begin(), endpoints[0].end(), p);
  p.value = obj->getAABB().max_[0];
  auto end1 = std::upper_bound(start1, endpoints[0].end(), p);

  if (start1 < end1) {
    size_t start_id = (size_t)(start1 - endpoints[0].begin());
    size_t end_id = (size_t)(end1 - endpoints[0].begin());
    size_t cur_id = (size_t)(start_id);

    for (size_t i = start_id; i < end_id; ++i) {
      if (endpoints[0][(size_t)i].obj != obj) {
        if (i == cur_id)
          cur_id++;
        else {
          endpoints[0][(size_t)cur_id] = endpoints[0][(size_t)i];
          cur_id++;
        }
      }
    }
    if (cur_id < end_id) endpoints[0].resize(endpoints[0].size() - 2);
  }

  p.value = obj->getAABB().min_[1];
  auto start2 = std::lower_bound(endpoints[1].begin(), endpoints[1].end(), p);
  p.value = obj->getAABB().max_[1];
  auto end2 = std::upper_bound(start2, endpoints[1].end(), p);

  if (start2 < end2) {
    size_t start_id = (size_t)(start2 - endpoints[1].begin());
    size_t end_id = (size_t)(end2 - endpoints[1].begin());
    size_t cur_id = (size_t)(start_id);

    for (size_t i = start_id; i < end_id; ++i) {
      if (endpoints[1][i].obj != obj) {
        if (i == cur_id)
          cur_id++;
        else {
          endpoints[1][cur_id] = endpoints[1][i];
          cur_id++;
        }
      }
    }
    if (cur_id < end_id) endpoints[1].resize(endpoints[1].size() - 2);
  }

  p.value = obj->getAABB().min_[2];
  auto start3 = std::lower_bound(endpoints[2].begin(), endpoints[2].end(), p);
  p.value = obj->getAABB().max_[2];
  auto end3 = std::upper_bound(start3, endpoints[2].end(), p);

  if (start3 < end3) {
    size_t start_id = (size_t)(start3 - endpoints[2].begin());
    size_t end_id = (size_t)(end3 - endpoints[2].begin());
    size_t cur_id = (size_t)(start_id);

    for (size_t i = start_id; i < end_id; ++i) {
      if (endpoints[2][i].obj != obj) {
        if (i == cur_id)
          cur_id++;
        else {
          endpoints[2][cur_id] = endpoints[2][i];
          cur_id++;
        }
      }
    }
    if (cur_id < end_id) endpoints[2].resize(endpoints[2].size() - 2);
  }

  // update the interval tree
  if (obj_interval_maps[0].find(obj) != obj_interval_maps[0].end()) {
    SAPInterval* ivl1 = obj_interval_maps[0][obj];
    SAPInterval* ivl2 = obj_interval_maps[1][obj];
    SAPInterval* ivl3 = obj_interval_maps[2][obj];

    interval_trees[0]->deleteNode(ivl1);
    interval_trees[1]->deleteNode(ivl2);
    interval_trees[2]->deleteNode(ivl3);

    delete ivl1;
    delete ivl2;
    delete ivl3;

    obj_interval_maps[0].erase(obj);
    obj_interval_maps[1].erase(obj);
    obj_interval_maps[2].erase(obj);
  }
}

//==============================================================================
IntervalTreeCollisionManager::IntervalTreeCollisionManager() : setup_(false) {
  for (int i = 0; i < 3; ++i) interval_trees[i] = nullptr;
}

//==============================================================================
IntervalTreeCollisionManager::~IntervalTreeCollisionManager() { clear(); }

//==============================================================================
void IntervalTreeCollisionManager::registerObject(CollisionObject* obj) {
  EndPoint p, q;

  p.obj = obj;
  q.obj = obj;
  p.minmax = 0;
  q.minmax = 1;
  p.value = obj->getAABB().min_[0];
  q.value = obj->getAABB().max_[0];
  endpoints[0].push_back(p);
  endpoints[0].push_back(q);

  p.value = obj->getAABB().min_[1];
  q.value = obj->getAABB().max_[1];
  endpoints[1].push_back(p);
  endpoints[1].push_back(q);

  p.value = obj->getAABB().min_[2];
  q.value = obj->getAABB().max_[2];
  endpoints[2].push_back(p);
  endpoints[2].push_back(q);
  setup_ = false;
}

//==============================================================================
void IntervalTreeCollisionManager::setup() {
  if (!setup_) {
    std::sort(endpoints[0].begin(), endpoints[0].end());
    std::sort(endpoints[1].begin(), endpoints[1].end());
    std::sort(endpoints[2].begin(), endpoints[2].end());

    for (int i = 0; i < 3; ++i) delete interval_trees[i];

    for (int i = 0; i < 3; ++i) interval_trees[i] = new detail::IntervalTree;

    for (size_t i = 0, size = endpoints[0].size(); i < size; ++i) {
      EndPoint p = endpoints[0][i];
      CollisionObject* obj = p.obj;
      if (p.minmax == 0) {
        SAPInterval* ivl1 = new SAPInterval(obj->getAABB().min_[0],
                                            obj->getAABB().max_[0], obj);
        SAPInterval* ivl2 = new SAPInterval(obj->getAABB().min_[1],
                                            obj->getAABB().max_[1], obj);
        SAPInterval* ivl3 = new SAPInterval(obj->getAABB().min_[2],
                                            obj->getAABB().max_[2], obj);

        interval_trees[0]->insert(ivl1);
        interval_trees[1]->insert(ivl2);
        interval_trees[2]->insert(ivl3);

        obj_interval_maps[0][obj] = ivl1;
        obj_interval_maps[1][obj] = ivl2;
        obj_interval_maps[2][obj] = ivl3;
      }
    }

    setup_ = true;
  }
}

//==============================================================================
void IntervalTreeCollisionManager::update() {
  setup_ = false;

  for (size_t i = 0, size = endpoints[0].size(); i < size; ++i) {
    if (endpoints[0][i].minmax == 0)
      endpoints[0][i].value = endpoints[0][i].obj->getAABB().min_[0];
    else
      endpoints[0][i].value = endpoints[0][i].obj->getAABB().max_[0];
  }

  for (size_t i = 0, size = endpoints[1].size(); i < size; ++i) {
    if (endpoints[1][i].minmax == 0)
      endpoints[1][i].value = endpoints[1][i].obj->getAABB().min_[1];
    else
      endpoints[1][i].value = endpoints[1][i].obj->getAABB().max_[1];
  }

  for (size_t i = 0, size = endpoints[2].size(); i < size; ++i) {
    if (endpoints[2][i].minmax == 0)
      endpoints[2][i].value = endpoints[2][i].obj->getAABB().min_[2];
    else
      endpoints[2][i].value = endpoints[2][i].obj->getAABB().max_[2];
  }

  setup();
}

//==============================================================================
void IntervalTreeCollisionManager::update(CollisionObject* updated_obj) {
  AABB old_aabb;
  const AABB& new_aabb = updated_obj->getAABB();
  for (int i = 0; i < 3; ++i) {
    const auto it = obj_interval_maps[i].find(updated_obj);
    interval_trees[i]->deleteNode(it->second);
    old_aabb.min_[i] = it->second->low;
    old_aabb.max_[i] = it->second->high;
    it->second->low = new_aabb.min_[i];
    it->second->high = new_aabb.max_[i];
    interval_trees[i]->insert(it->second);
  }

  EndPoint dummy;
  typename std::vector<EndPoint>::iterator it;
  for (int i = 0; i < 3; ++i) {
    dummy.value = old_aabb.min_[i];
    it = std::lower_bound(endpoints[i].begin(), endpoints[i].end(), dummy);
    for (; it != endpoints[i].end(); ++it) {
      if (it->obj == updated_obj && it->minmax == 0) {
        it->value = new_aabb.min_[i];
        break;
      }
    }

    dummy.value = old_aabb.max_[i];
    it = std::lower_bound(endpoints[i].begin(), endpoints[i].end(), dummy);
    for (; it != endpoints[i].end(); ++it) {
      if (it->obj == updated_obj && it->minmax == 0) {
        it->value = new_aabb.max_[i];
        break;
      }
    }

    std::sort(endpoints[i].begin(), endpoints[i].end());
  }
}

//==============================================================================
void IntervalTreeCollisionManager::update(
    const std::vector<CollisionObject*>& updated_objs) {
  for (size_t i = 0; i < updated_objs.size(); ++i) update(updated_objs[i]);
}

//==============================================================================
void IntervalTreeCollisionManager::clear() {
  endpoints[0].clear();
  endpoints[1].clear();
  endpoints[2].clear();

  delete interval_trees[0];
  interval_trees[0] = nullptr;
  delete interval_trees[1];
  interval_trees[1] = nullptr;
  delete interval_trees[2];
  interval_trees[2] = nullptr;

  for (int i = 0; i < 3; ++i) {
    for (auto it = obj_interval_maps[i].cbegin(),
              end = obj_interval_maps[i].cend();
         it != end; ++it) {
      delete it->second;
    }
  }

  for (int i = 0; i < 3; ++i) obj_interval_maps[i].clear();

  setup_ = false;
}

//==============================================================================
void IntervalTreeCollisionManager::getObjects(
    std::vector<CollisionObject*>& objs) const {
  objs.resize(endpoints[0].size() / 2);
  size_t j = 0;
  for (size_t i = 0, size = endpoints[0].size(); i < size; ++i) {
    if (endpoints[0][i].minmax == 0) {
      objs[j] = endpoints[0][i].obj;
      j++;
    }
  }
}

//==============================================================================
void IntervalTreeCollisionManager::collide(
    CollisionObject* obj, CollisionCallBackBase* callback) const {
  callback->init();
  if (size() == 0) return;
  collide_(obj, callback);
}

//==============================================================================
bool IntervalTreeCollisionManager::collide_(
    CollisionObject* obj, CollisionCallBackBase* callback) const {
  static const unsigned int CUTOFF = 100;

  std::deque<detail::SimpleInterval*> results0, results1, results2;

  results0 =
      interval_trees[0]->query(obj->getAABB().min_[0], obj->getAABB().max_[0]);
  if (results0.size() > CUTOFF) {
    results1 = interval_trees[1]->query(obj->getAABB().min_[1],
                                        obj->getAABB().max_[1]);
    if (results1.size() > CUTOFF) {
      results2 = interval_trees[2]->query(obj->getAABB().min_[2],
                                          obj->getAABB().max_[2]);
      if (results2.size() > CUTOFF) {
        size_t d1 = results0.size();
        size_t d2 = results1.size();
        size_t d3 = results2.size();

        if (d1 >= d2 && d1 >= d3)
          return checkColl(results0.begin(), results0.end(), obj, callback);
        else if (d2 >= d1 && d2 >= d3)
          return checkColl(results1.begin(), results1.end(), obj, callback);
        else
          return checkColl(results2.begin(), results2.end(), obj, callback);
      } else
        return checkColl(results2.begin(), results2.end(), obj, callback);
    } else
      return checkColl(results1.begin(), results1.end(), obj, callback);
  } else
    return checkColl(results0.begin(), results0.end(), obj, callback);
}

//==============================================================================
void IntervalTreeCollisionManager::distance(
    CollisionObject* obj, DistanceCallBackBase* callback) const {
  callback->init();
  if (size() == 0) return;
  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  distance_(obj, callback, min_dist);
}

//==============================================================================
bool IntervalTreeCollisionManager::distance_(CollisionObject* obj,
                                             DistanceCallBackBase* callback,
                                             Scalar& min_dist) const {
  static const unsigned int CUTOFF = 100;

  Vec3s delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  AABB aabb = obj->getAABB();
  if (min_dist < (std::numeric_limits<Scalar>::max)()) {
    Vec3s min_dist_delta(min_dist, min_dist, min_dist);
    aabb.expand(min_dist_delta);
  }

  int status = 1;
  Scalar old_min_distance;

  while (1) {
    bool dist_res = false;

    old_min_distance = min_dist;

    std::deque<detail::SimpleInterval*> results0, results1, results2;

    results0 = interval_trees[0]->query(aabb.min_[0], aabb.max_[0]);
    if (results0.size() > CUTOFF) {
      results1 = interval_trees[1]->query(aabb.min_[1], aabb.max_[1]);
      if (results1.size() > CUTOFF) {
        results2 = interval_trees[2]->query(aabb.min_[2], aabb.max_[2]);
        if (results2.size() > CUTOFF) {
          size_t d1 = results0.size();
          size_t d2 = results1.size();
          size_t d3 = results2.size();

          if (d1 >= d2 && d1 >= d3)
            dist_res = checkDist(results0.begin(), results0.end(), obj,
                                 callback, min_dist);
          else if (d2 >= d1 && d2 >= d3)
            dist_res = checkDist(results1.begin(), results1.end(), obj,
                                 callback, min_dist);
          else
            dist_res = checkDist(results2.begin(), results2.end(), obj,
                                 callback, min_dist);
        } else
          dist_res = checkDist(results2.begin(), results2.end(), obj, callback,
                               min_dist);
      } else
        dist_res = checkDist(results1.begin(), results1.end(), obj, callback,
                             min_dist);
    } else
      dist_res =
          checkDist(results0.begin(), results0.end(), obj, callback, min_dist);

    if (dist_res) return true;

    results0.clear();
    results1.clear();
    results2.clear();

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
void IntervalTreeCollisionManager::collide(
    CollisionCallBackBase* callback) const {
  callback->init();
  if (size() == 0) return;

  std::set<CollisionObject*> active;
  std::set<std::pair<CollisionObject*, CollisionObject*> > overlap;
  size_t n = endpoints[0].size();
  Scalar diff_x = endpoints[0][0].value - endpoints[0][n - 1].value;
  Scalar diff_y = endpoints[1][0].value - endpoints[1][n - 1].value;
  Scalar diff_z = endpoints[2][0].value - endpoints[2][n - 1].value;

  int axis = 0;
  if (diff_y > diff_x && diff_y > diff_z)
    axis = 1;
  else if (diff_z > diff_y && diff_z > diff_x)
    axis = 2;

  for (unsigned int i = 0; i < n; ++i) {
    const EndPoint& endpoint = endpoints[axis][i];
    CollisionObject* index = endpoint.obj;
    if (endpoint.minmax == 0) {
      auto iter = active.begin();
      auto end = active.end();
      for (; iter != end; ++iter) {
        CollisionObject* active_index = *iter;
        const AABB& b0 = active_index->getAABB();
        const AABB& b1 = index->getAABB();

        int axis2 = (axis + 1) % 3;
        int axis3 = (axis + 2) % 3;

        if (b0.axisOverlap(b1, axis2) && b0.axisOverlap(b1, axis3)) {
          std::pair<typename std::set<std::pair<CollisionObject*,
                                                CollisionObject*> >::iterator,
                    bool>
              insert_res;
          if (active_index < index)
            insert_res = overlap.insert(std::make_pair(active_index, index));
          else
            insert_res = overlap.insert(std::make_pair(index, active_index));

          if (insert_res.second) {
            if ((*callback)(active_index, index)) return;
          }
        }
      }
      active.insert(index);
    } else
      active.erase(index);
  }
}

//==============================================================================
void IntervalTreeCollisionManager::distance(
    DistanceCallBackBase* callback) const {
  callback->init();
  if (size() == 0) return;

  this->enable_tested_set_ = true;
  this->tested_set.clear();
  Scalar min_dist = (std::numeric_limits<Scalar>::max)();

  for (size_t i = 0; i < endpoints[0].size(); ++i)
    if (distance_(endpoints[0][i].obj, callback, min_dist)) break;

  this->enable_tested_set_ = false;
  this->tested_set.clear();
}

//==============================================================================
void IntervalTreeCollisionManager::collide(
    BroadPhaseCollisionManager* other_manager_,
    CollisionCallBackBase* callback) const {
  callback->init();
  IntervalTreeCollisionManager* other_manager =
      static_cast<IntervalTreeCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    collide(callback);
    return;
  }

  if (this->size() < other_manager->size()) {
    for (size_t i = 0, size = endpoints[0].size(); i < size; ++i)
      if (other_manager->collide_(endpoints[0][i].obj, callback)) return;
  } else {
    for (size_t i = 0, size = other_manager->endpoints[0].size(); i < size; ++i)
      if (collide_(other_manager->endpoints[0][i].obj, callback)) return;
  }
}

//==============================================================================
void IntervalTreeCollisionManager::distance(
    BroadPhaseCollisionManager* other_manager_,
    DistanceCallBackBase* callback) const {
  callback->init();
  IntervalTreeCollisionManager* other_manager =
      static_cast<IntervalTreeCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    distance(callback);
    return;
  }

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();

  if (this->size() < other_manager->size()) {
    for (size_t i = 0, size = endpoints[0].size(); i < size; ++i)
      if (other_manager->distance_(endpoints[0][i].obj, callback, min_dist))
        return;
  } else {
    for (size_t i = 0, size = other_manager->endpoints[0].size(); i < size; ++i)
      if (distance_(other_manager->endpoints[0][i].obj, callback, min_dist))
        return;
  }
}

//==============================================================================
bool IntervalTreeCollisionManager::empty() const {
  return endpoints[0].empty();
}

//==============================================================================
size_t IntervalTreeCollisionManager::size() const {
  return endpoints[0].size() / 2;
}

//==============================================================================
bool IntervalTreeCollisionManager::checkColl(
    typename std::deque<detail::SimpleInterval*>::const_iterator pos_start,
    typename std::deque<detail::SimpleInterval*>::const_iterator pos_end,
    CollisionObject* obj, CollisionCallBackBase* callback) const {
  while (pos_start < pos_end) {
    SAPInterval* ivl = static_cast<SAPInterval*>(*pos_start);
    if (ivl->obj != obj) {
      if (ivl->obj->getAABB().overlap(obj->getAABB())) {
        if ((*callback)(ivl->obj, obj)) return true;
      }
    }

    pos_start++;
  }

  return false;
}

//==============================================================================
bool IntervalTreeCollisionManager::checkDist(
    typename std::deque<detail::SimpleInterval*>::const_iterator pos_start,
    typename std::deque<detail::SimpleInterval*>::const_iterator pos_end,
    CollisionObject* obj, DistanceCallBackBase* callback,
    Scalar& min_dist) const {
  while (pos_start < pos_end) {
    SAPInterval* ivl = static_cast<SAPInterval*>(*pos_start);
    if (ivl->obj != obj) {
      if (!this->enable_tested_set_) {
        if (ivl->obj->getAABB().distance(obj->getAABB()) < min_dist) {
          if ((*callback)(ivl->obj, obj, min_dist)) return true;
        }
      } else {
        if (!this->inTestedSet(ivl->obj, obj)) {
          if (ivl->obj->getAABB().distance(obj->getAABB()) < min_dist) {
            if ((*callback)(ivl->obj, obj, min_dist)) return true;
          }

          this->insertTestedSet(ivl->obj, obj);
        }
      }
    }

    pos_start++;
  }

  return false;
}

//==============================================================================
bool IntervalTreeCollisionManager::EndPoint::operator<(
    const typename IntervalTreeCollisionManager::EndPoint& p) const {
  return value < p.value;
}

//==============================================================================
IntervalTreeCollisionManager::SAPInterval::SAPInterval(Scalar low_,
                                                       Scalar high_,
                                                       CollisionObject* obj_)
    : detail::SimpleInterval() {
  this->low = low_;
  this->high = high_;
  obj = obj_;
}

}  // namespace coal
