/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

/** \author Jia Pan */

#include <../src/collision_node.h>
#include "coal/internal/traversal_recurse.h"

namespace coal {

void checkResultLowerBound(const CollisionResult& result,
                           Scalar sqrDistLowerBound) {
  COAL_UNUSED_VARIABLE(result);
  const Scalar dummy_precision = std::sqrt(Eigen::NumTraits<Scalar>::epsilon());
  COAL_UNUSED_VARIABLE(dummy_precision);
  if (sqrDistLowerBound == 0) {
    COAL_ASSERT(result.distance_lower_bound <= dummy_precision,
                "Distance lower bound should not be positive.",
                std::logic_error);
  } else {
    COAL_ASSERT(result.distance_lower_bound * result.distance_lower_bound -
                        sqrDistLowerBound <
                    dummy_precision,
                "Distance lower bound and sqrDistLowerBound should coincide.",
                std::logic_error);
  }
}

void collide(CollisionTraversalNodeBase* node, const CollisionRequest& request,
             CollisionResult& result, BVHFrontList* front_list,
             bool recursive) {
  if (front_list && front_list->size() > 0) {
    propagateBVHFrontListCollisionRecurse(node, request, result, front_list);
  } else {
    Scalar sqrDistLowerBound = 0;
    if (recursive)
      collisionRecurse(node, 0, 0, front_list, sqrDistLowerBound);
    else
      collisionNonRecurse(node, front_list, sqrDistLowerBound);
    if (!std::isnan(sqrDistLowerBound)) {
      checkResultLowerBound(result, sqrDistLowerBound);
    }
  }
}

void distance(DistanceTraversalNodeBase* node, BVHFrontList* front_list,
              unsigned int qsize) {
  node->preprocess();

  if (qsize <= 2)
    distanceRecurse(node, 0, 0, front_list);
  else
    distanceQueueRecurse(node, 0, 0, front_list, qsize);

  node->postprocess();
}

}  // namespace coal
