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

#include "coal/broadphase/broadphase_SSaP.h"
#include "coal/tracy.hh"

namespace coal {

/** @brief Functor sorting objects according to the AABB lower x bound */
struct SortByXLow {
  bool operator()(const CollisionObject* a, const CollisionObject* b) const {
    if (a->getAABB().min_[0] < b->getAABB().min_[0]) return true;
    return false;
  }
};

/** @brief Functor sorting objects according to the AABB lower y bound */
struct SortByYLow {
  bool operator()(const CollisionObject* a, const CollisionObject* b) const {
    if (a->getAABB().min_[1] < b->getAABB().min_[1]) return true;
    return false;
  }
};

/** @brief Functor sorting objects according to the AABB lower z bound */
struct SortByZLow {
  bool operator()(const CollisionObject* a, const CollisionObject* b) const {
    if (a->getAABB().min_[2] < b->getAABB().min_[2]) return true;
    return false;
  }
};

/** @brief Dummy collision object with a point AABB */
class COAL_DLLAPI DummyCollisionObject : public CollisionObject {
 public:
  DummyCollisionObject(const AABB& aabb_)
      : CollisionObject(shared_ptr<CollisionGeometry>()) {
    this->aabb = aabb_;
  }

  void computeLocalAABB() {}
};

//==============================================================================
void SSaPCollisionManager::unregisterObject(CollisionObject* obj) {
  setup();

  DummyCollisionObject dummyHigh(AABB(obj->getAABB().max_));

  auto pos_start1 = objs_x.begin();
  auto pos_end1 =
      std::upper_bound(pos_start1, objs_x.end(), &dummyHigh, SortByXLow());

  while (pos_start1 < pos_end1) {
    if (*pos_start1 == obj) {
      objs_x.erase(pos_start1);
      break;
    }
    ++pos_start1;
  }

  auto pos_start2 = objs_y.begin();
  auto pos_end2 =
      std::upper_bound(pos_start2, objs_y.end(), &dummyHigh, SortByYLow());

  while (pos_start2 < pos_end2) {
    if (*pos_start2 == obj) {
      objs_y.erase(pos_start2);
      break;
    }
    ++pos_start2;
  }

  auto pos_start3 = objs_z.begin();
  auto pos_end3 =
      std::upper_bound(pos_start3, objs_z.end(), &dummyHigh, SortByZLow());

  while (pos_start3 < pos_end3) {
    if (*pos_start3 == obj) {
      objs_z.erase(pos_start3);
      break;
    }
    ++pos_start3;
  }
}

//==============================================================================
SSaPCollisionManager::SSaPCollisionManager() : setup_(false) {
  // Do nothing
}

//==============================================================================
void SSaPCollisionManager::registerObject(CollisionObject* obj) {
  objs_x.push_back(obj);
  objs_y.push_back(obj);
  objs_z.push_back(obj);
  setup_ = false;
}

//==============================================================================
void SSaPCollisionManager::setup() {
  if (!setup_) {
    std::sort(objs_x.begin(), objs_x.end(), SortByXLow());
    std::sort(objs_y.begin(), objs_y.end(), SortByYLow());
    std::sort(objs_z.begin(), objs_z.end(), SortByZLow());
    setup_ = true;
  }
}

//==============================================================================
void SSaPCollisionManager::update() {
  setup_ = false;
  setup();
}

//==============================================================================
void SSaPCollisionManager::clear() {
  objs_x.clear();
  objs_y.clear();
  objs_z.clear();
  setup_ = false;
}

//==============================================================================
void SSaPCollisionManager::getObjects(
    std::vector<CollisionObject*>& objs) const {
  objs.resize(objs_x.size());
  std::copy(objs_x.begin(), objs_x.end(), objs.begin());
}

//==============================================================================
bool SSaPCollisionManager::checkColl(
    std::vector<CollisionObject*>::const_iterator pos_start,
    std::vector<CollisionObject*>::const_iterator pos_end, CollisionObject* obj,
    CollisionCallBackBase* callback) const {
  while (pos_start < pos_end) {
    if (*pos_start != obj)  // no collision between the same object
    {
      if ((*pos_start)->getAABB().overlap(obj->getAABB())) {
        if ((*callback)(*pos_start, obj)) return true;
      }
    }
    pos_start++;
  }
  return false;
}

//==============================================================================
bool SSaPCollisionManager::checkDis(
    typename std::vector<CollisionObject*>::const_iterator pos_start,
    typename std::vector<CollisionObject*>::const_iterator pos_end,
    CollisionObject* obj, DistanceCallBackBase* callback,
    Scalar& min_dist) const {
  while (pos_start < pos_end) {
    if (*pos_start != obj)  // no distance between the same object
    {
      if ((*pos_start)->getAABB().distance(obj->getAABB()) < min_dist) {
        if ((*callback)(*pos_start, obj, min_dist)) return true;
      }
    }
    pos_start++;
  }

  return false;
}

//==============================================================================
void SSaPCollisionManager::collide(CollisionObject* obj,
                                   CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SSaPCollisionManager::collide(CollisionObject*, "
      "CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  collide_(obj, callback);
}

//==============================================================================
bool SSaPCollisionManager::collide_(CollisionObject* obj,
                                    CollisionCallBackBase* callback) const {
  static const unsigned int CUTOFF = 100;

  DummyCollisionObject dummyHigh(AABB(obj->getAABB().max_));
  bool coll_res = false;

  const auto pos_start1 = objs_x.begin();
  const auto pos_end1 =
      std::upper_bound(pos_start1, objs_x.end(), &dummyHigh, SortByXLow());
  long d1 = pos_end1 - pos_start1;

  if (d1 > CUTOFF) {
    const auto pos_start2 = objs_y.begin();
    const auto pos_end2 =
        std::upper_bound(pos_start2, objs_y.end(), &dummyHigh, SortByYLow());
    long d2 = pos_end2 - pos_start2;

    if (d2 > CUTOFF) {
      const auto pos_start3 = objs_z.begin();
      const auto pos_end3 =
          std::upper_bound(pos_start3, objs_z.end(), &dummyHigh, SortByZLow());
      long d3 = pos_end3 - pos_start3;

      if (d3 > CUTOFF) {
        if (d3 <= d2 && d3 <= d1)
          coll_res = checkColl(pos_start3, pos_end3, obj, callback);
        else {
          if (d2 <= d3 && d2 <= d1)
            coll_res = checkColl(pos_start2, pos_end2, obj, callback);
          else
            coll_res = checkColl(pos_start1, pos_end1, obj, callback);
        }
      } else
        coll_res = checkColl(pos_start3, pos_end3, obj, callback);
    } else
      coll_res = checkColl(pos_start2, pos_end2, obj, callback);
  } else
    coll_res = checkColl(pos_start1, pos_end1, obj, callback);

  return coll_res;
}

//==============================================================================
void SSaPCollisionManager::distance(CollisionObject* obj,
                                    DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SSaPCollisionManager::distance(CollisionObject*, "
      "DistanceCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  distance_(obj, callback, min_dist);
}

//==============================================================================
bool SSaPCollisionManager::distance_(CollisionObject* obj,
                                     DistanceCallBackBase* callback,
                                     Scalar& min_dist) const {
  static const unsigned int CUTOFF = 100;
  Vec3s delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
  Vec3s dummy_vector = obj->getAABB().max_;
  if (min_dist < (std::numeric_limits<Scalar>::max)())
    dummy_vector += Vec3s(min_dist, min_dist, min_dist);

  typename std::vector<CollisionObject*>::const_iterator pos_start1 =
      objs_x.begin();
  typename std::vector<CollisionObject*>::const_iterator pos_start2 =
      objs_y.begin();
  typename std::vector<CollisionObject*>::const_iterator pos_start3 =
      objs_z.begin();
  typename std::vector<CollisionObject*>::const_iterator pos_end1 =
      objs_x.end();
  typename std::vector<CollisionObject*>::const_iterator pos_end2 =
      objs_y.end();
  typename std::vector<CollisionObject*>::const_iterator pos_end3 =
      objs_z.end();

  int status = 1;
  Scalar old_min_distance;

  while (1) {
    old_min_distance = min_dist;
    DummyCollisionObject dummyHigh((AABB(dummy_vector)));

    pos_end1 =
        std::upper_bound(pos_start1, objs_x.end(), &dummyHigh, SortByXLow());
    long d1 = pos_end1 - pos_start1;

    bool dist_res = false;

    if (d1 > CUTOFF) {
      pos_end2 =
          std::upper_bound(pos_start2, objs_y.end(), &dummyHigh, SortByYLow());
      long d2 = pos_end2 - pos_start2;

      if (d2 > CUTOFF) {
        pos_end3 = std::upper_bound(pos_start3, objs_z.end(), &dummyHigh,
                                    SortByZLow());
        long d3 = pos_end3 - pos_start3;

        if (d3 > CUTOFF) {
          if (d3 <= d2 && d3 <= d1)
            dist_res = checkDis(pos_start3, pos_end3, obj, callback, min_dist);
          else {
            if (d2 <= d3 && d2 <= d1)
              dist_res =
                  checkDis(pos_start2, pos_end2, obj, callback, min_dist);
            else
              dist_res =
                  checkDis(pos_start1, pos_end1, obj, callback, min_dist);
          }
        } else
          dist_res = checkDis(pos_start3, pos_end3, obj, callback, min_dist);
      } else
        dist_res = checkDis(pos_start2, pos_end2, obj, callback, min_dist);
    } else {
      dist_res = checkDis(pos_start1, pos_end1, obj, callback, min_dist);
    }

    if (dist_res) return true;

    if (status == 1) {
      if (old_min_distance < (std::numeric_limits<Scalar>::max)())
        break;
      else {
        // from infinity to a finite one, only need one additional loop
        // to check the possible missed ones to the right of the objs array
        if (min_dist < old_min_distance) {
          dummy_vector =
              obj->getAABB().max_ + Vec3s(min_dist, min_dist, min_dist);
          status = 0;
        } else  // need more loop
        {
          if (dummy_vector.isApprox(
                  obj->getAABB().max_,
                  std::numeric_limits<Scalar>::epsilon() * 100))
            dummy_vector = dummy_vector + delta;
          else
            dummy_vector = dummy_vector * 2 - obj->getAABB().max_;
        }
      }

      // yes, following is wrong, will result in too large distance.
      // if(pos_end1 != objs_x.end()) pos_start1 = pos_end1;
      // if(pos_end2 != objs_y.end()) pos_start2 = pos_end2;
      // if(pos_end3 != objs_z.end()) pos_start3 = pos_end3;
    } else if (status == 0)
      break;
  }

  return false;
}

//==============================================================================
int SSaPCollisionManager::selectOptimalAxis(
    const std::vector<CollisionObject*>& objs_x,
    const std::vector<CollisionObject*>& objs_y,
    const std::vector<CollisionObject*>& objs_z,
    typename std::vector<CollisionObject*>::const_iterator& it_beg,
    typename std::vector<CollisionObject*>::const_iterator& it_end) {
  /// simple sweep and prune method
  Scalar delta_x = (objs_x[objs_x.size() - 1])->getAABB().min_[0] -
                   (objs_x[0])->getAABB().min_[0];
  Scalar delta_y = (objs_x[objs_y.size() - 1])->getAABB().min_[1] -
                   (objs_y[0])->getAABB().min_[1];
  Scalar delta_z = (objs_z[objs_z.size() - 1])->getAABB().min_[2] -
                   (objs_z[0])->getAABB().min_[2];

  int axis = 0;
  if (delta_y > delta_x && delta_y > delta_z)
    axis = 1;
  else if (delta_z > delta_y && delta_z > delta_x)
    axis = 2;

  switch (axis) {
    case 0:
      it_beg = objs_x.begin();
      it_end = objs_x.end();
      break;
    case 1:
      it_beg = objs_y.begin();
      it_end = objs_y.end();
      break;
    case 2:
      it_beg = objs_z.begin();
      it_end = objs_z.end();
      break;
  }

  return axis;
}

//==============================================================================
void SSaPCollisionManager::collide(CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SSaPCollisionManager::collide(CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  typename std::vector<CollisionObject*>::const_iterator pos, run_pos, pos_end;
  int axis = selectOptimalAxis(objs_x, objs_y, objs_z, pos, pos_end);
  int axis2 = (axis + 1 > 2) ? 0 : (axis + 1);
  int axis3 = (axis2 + 1 > 2) ? 0 : (axis2 + 1);

  run_pos = pos;

  while ((run_pos < pos_end) && (pos < pos_end)) {
    CollisionObject* obj = *(pos++);

    while (1) {
      if ((*run_pos)->getAABB().min_[axis] < obj->getAABB().min_[axis]) {
        run_pos++;
        if (run_pos == pos_end) break;
        continue;
      } else {
        run_pos++;
        break;
      }
    }

    if (run_pos < pos_end) {
      typename std::vector<CollisionObject*>::const_iterator run_pos2 = run_pos;

      while ((*run_pos2)->getAABB().min_[axis] <= obj->getAABB().max_[axis]) {
        CollisionObject* obj2 = *run_pos2;
        run_pos2++;

        if ((obj->getAABB().max_[axis2] >= obj2->getAABB().min_[axis2]) &&
            (obj2->getAABB().max_[axis2] >= obj->getAABB().min_[axis2])) {
          if ((obj->getAABB().max_[axis3] >= obj2->getAABB().min_[axis3]) &&
              (obj2->getAABB().max_[axis3] >= obj->getAABB().min_[axis3])) {
            if ((*callback)(obj, obj2)) return;
          }
        }

        if (run_pos2 == pos_end) break;
      }
    }
  }
}

//==============================================================================
void SSaPCollisionManager::distance(DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SSaPCollisionManager::distance(DistanceCallBackBase*)");
  callback->init();
  if (size() == 0) return;

  typename std::vector<CollisionObject*>::const_iterator it, it_end;
  selectOptimalAxis(objs_x, objs_y, objs_z, it, it_end);

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  for (; it != it_end; ++it) {
    if (distance_(*it, callback, min_dist)) return;
  }
}

//==============================================================================
void SSaPCollisionManager::collide(BroadPhaseCollisionManager* other_manager_,
                                   CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SSaPCollisionManager::collide(BroadPhaseCollisionManager*, "
      "CollisionCallBackBase*)");
  callback->init();
  SSaPCollisionManager* other_manager =
      static_cast<SSaPCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    collide(callback);
    return;
  }

  typename std::vector<CollisionObject*>::const_iterator it, end;
  if (this->size() < other_manager->size()) {
    for (it = objs_x.begin(), end = objs_x.end(); it != end; ++it)
      if (other_manager->collide_(*it, callback)) return;
  } else {
    for (it = other_manager->objs_x.begin(), end = other_manager->objs_x.end();
         it != end; ++it)
      if (collide_(*it, callback)) return;
  }
}

//==============================================================================
void SSaPCollisionManager::distance(BroadPhaseCollisionManager* other_manager_,
                                    DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::SSaPCollisionManager::distance(BroadPhaseCollisionManager*, "
      "DistanceCallBackBase*)");
  callback->init();
  SSaPCollisionManager* other_manager =
      static_cast<SSaPCollisionManager*>(other_manager_);

  if ((size() == 0) || (other_manager->size() == 0)) return;

  if (this == other_manager) {
    distance(callback);
    return;
  }

  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  typename std::vector<CollisionObject*>::const_iterator it, end;
  if (this->size() < other_manager->size()) {
    for (it = objs_x.begin(), end = objs_x.end(); it != end; ++it)
      if (other_manager->distance_(*it, callback, min_dist)) return;
  } else {
    for (it = other_manager->objs_x.begin(), end = other_manager->objs_x.end();
         it != end; ++it)
      if (distance_(*it, callback, min_dist)) return;
  }
}

//==============================================================================
bool SSaPCollisionManager::empty() const { return objs_x.empty(); }

//==============================================================================
size_t SSaPCollisionManager::size() const { return objs_x.size(); }

}  // namespace coal
