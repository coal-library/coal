/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Toyota Research Institute
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
 *   * Neither the name of the copyright holder nor the names of its
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

/** @author Sean Curtis (sean@tri.global) */

#include "coal/broadphase/default_broadphase_callbacks.h"
#include <algorithm>

namespace coal {

bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2,
                              void* data) {
  assert(data != nullptr);
  auto* collision_data = static_cast<CollisionData*>(data);
  const CollisionRequest& request = collision_data->request;
  CollisionResult& result = collision_data->result;

  if (collision_data->done) return true;

  collide(o1, o2, request, result);

  if (result.isCollision() &&
      result.numContacts() >= request.num_max_contacts) {
    collision_data->done = true;
  }

  return collision_data->done;
}

bool CollisionCallBackDefault::collide(CollisionObject* o1,
                                       CollisionObject* o2) {
  return defaultCollisionFunction(o1, o2, &data);
}

bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2,
                             void* data, Scalar& dist) {
  assert(data != nullptr);
  auto* cdata = static_cast<DistanceData*>(data);
  const DistanceRequest& request = cdata->request;
  DistanceResult& result = cdata->result;

  if (cdata->done) {
    dist = result.min_distance;
    return true;
  }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if (dist <= 0) return true;  // in collision or in touch

  return cdata->done;
}

bool DistanceCallBackDefault::distance(CollisionObject* o1, CollisionObject* o2,
                                       Scalar& dist) {
  return defaultDistanceFunction(o1, o2, &data, dist);
}

CollisionCallBackCollect::CollisionCallBackCollect(const size_t max_size)
    : max_size(max_size) {
  collision_pairs.resize(max_size);
}

bool CollisionCallBackCollect::collide(CollisionObject* o1,
                                       CollisionObject* o2) {
  collision_pairs.push_back(std::make_pair(o1, o2));
  return false;
}

size_t CollisionCallBackCollect::numCollisionPairs() const {
  return collision_pairs.size();
}

const std::vector<CollisionCallBackCollect::CollisionPair>&
CollisionCallBackCollect::getCollisionPairs() const {
  return collision_pairs;
}

void CollisionCallBackCollect::init() { collision_pairs.clear(); }

bool CollisionCallBackCollect::exist(CollisionObject* o1,
                                     CollisionObject* o2) const {
  return exist(std::make_pair(o1, o2));
}

bool CollisionCallBackCollect::exist(const CollisionPair& pair) const {
  return std::find(collision_pairs.begin(), collision_pairs.end(), pair) !=
         collision_pairs.end();
}

}  // namespace coal
