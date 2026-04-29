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

#include "coal/BVH/BVH_front.h"
#include "coal/internal/traversal_node_base.h"
#include "coal/internal/traversal_node_bvhs.h"

/// @brief collision and distance function on traversal nodes. these functions
/// provide a higher level abstraction for collision functions provided in
/// collision_func_matrix
namespace coal {

/// collision on collision traversal node
///
/// @param node node containing both objects to test,
/// @retval squared lower bound to the distance between the objects if they
///         do not collide.
/// @param front_list list of nodes visited by the query, can be used to
///        accelerate computation
/// \todo should be COAL_LOCAL but used in unit test.
COAL_DLLAPI void collide(CollisionTraversalNodeBase* node,
                         const CollisionRequest& request,
                         CollisionResult& result,
                         BVHFrontList* front_list = NULL,
                         bool recursive = true);

/// @brief distance computation on distance traversal node; can use front list
/// to accelerate \todo should be COAL_LOCAL but used in unit test.
COAL_DLLAPI void distance(DistanceTraversalNodeBase* node,
                          BVHFrontList* front_list = NULL,
                          unsigned int qsize = 2);

/// @brief Internal helper: validates the squared-lower-bound invariant on
/// the result. Exposed so the templated collide<Node> overload below can
/// reuse the same check as the base-pointer overload.
COAL_DLLAPI void checkResultLowerBound(const CollisionResult& result,
                                       Scalar sqrDistLowerBound);
}  // namespace coal

#include "coal/internal/traversal_recurse.h"

namespace coal {

/// @brief Templated overload of `collide` that takes a concrete traversal-node
/// type and dispatches to `collisionRecurseT`, allowing the compiler to
/// resolve all virtual calls statically. Selected by overload resolution over
/// the non-template `collide(CollisionTraversalNodeBase*, ...)` for any
/// argument that is a strict subclass (because an exact-type template match
/// outranks a derived-to-base pointer conversion).
///
/// Behaviour matches the base-pointer version exactly, including the
/// front-list propagation path and the non-recursive iterative path; both
/// are forwarded to the base-pointer overload so the templated path stays
/// surgical and only optimises the common recursive case.
template <typename Node>
inline void collide(Node* node, const CollisionRequest& request,
                    CollisionResult& result, BVHFrontList* front_list = NULL,
                    bool recursive = true) {
  if (front_list && front_list->size() > 0) {
    return collide(static_cast<CollisionTraversalNodeBase*>(node), request,
                   result, front_list, recursive);
  }
  if (!recursive) {
    return collide(static_cast<CollisionTraversalNodeBase*>(node), request,
                   result, front_list, recursive);
  }
  Scalar sqrDistLowerBound = 0;
  collisionRecurseT(node, 0u, 0u, front_list, sqrDistLowerBound);
  if (!std::isnan(sqrDistLowerBound)) {
    checkResultLowerBound(result, sqrDistLowerBound);
  }
}

/// @brief Templated overload of `distance` analogous to the templated
/// `collide` above. Forwards the queue-search path to the base-pointer
/// version; only the standard recursive case (qsize <= 2) is devirtualised.
template <typename Node>
inline void distance(Node* node, BVHFrontList* front_list = NULL,
                     unsigned int qsize = 2) {
  if (qsize > 2) {
    return distance(static_cast<DistanceTraversalNodeBase*>(node), front_list,
                    qsize);
  }
  node->preprocess();
  distanceRecurseT(node, 0u, 0u, front_list);
  node->postprocess();
}

}  // namespace coal

/// @endcond

#endif
