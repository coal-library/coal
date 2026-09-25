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

#ifndef COAL_COLLISION_NODE_H
#define COAL_COLLISION_NODE_H

/// @cond INTERNAL

#include <cmath>

#include "coal/BVH/BVH_front.h"
#include "coal/internal/traversal_node_base.h"
#include "coal/internal/traversal_node_bvhs.h"
#include "coal/internal/traversal_recurse.h"

/// @brief collision and distance function on traversal nodes. these functions
/// provide a higher level abstraction for collision functions provided in
/// collision_func_matrix
namespace coal {

inline void checkResultLowerBound(const CollisionResult& result,
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

/// collision on collision traversal node
///
/// @param node node containing both objects to test,
/// @retval squared lower bound to the distance between the objects if they
///         do not collide.
/// @param front_list list of nodes visited by the query, can be used to
///        accelerate computation
/// \todo should be COAL_LOCAL but used in unit test.
template <typename Node>
inline void collide(Node* node, const CollisionRequest& request,
                    CollisionResult& result, BVHFrontList* front_list = nullptr,
                    bool recursive = true) {
  if (front_list && front_list->size() > 0) {
    propagateBVHFrontListCollisionRecurse(node, request, result, front_list);
  } else {
    Scalar sqrDistLowerBound = 0;
    if (recursive)
      collisionRecurse(node, 0, 0, front_list, sqrDistLowerBound);
    else
      collisionNonRecurse(node, front_list, sqrDistLowerBound);
    if (!std::isnan(sqrDistLowerBound))
      checkResultLowerBound(result, sqrDistLowerBound);
  }
}

/// @brief distance computation on distance traversal node; can use front list
/// to accelerate \todo should be COAL_LOCAL but used in unit test.
template <typename Node>
inline void distance(Node* node, BVHFrontList* front_list = nullptr,
                     unsigned int qsize = 2) {
  node->preprocess();
  if (qsize <= 2)
    distanceRecurse(node, 0, 0, front_list);
  else
    distanceQueueRecurse(node, 0, 0, front_list, qsize);
  node->postprocess();
}

}  // namespace coal

/// @endcond

#endif
