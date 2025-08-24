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

#ifndef COAL_TRAVERSAL_NODE_BASE_H
#define COAL_TRAVERSAL_NODE_BASE_H

/// @cond INTERNAL

#include "coal/data_types.h"
#include "coal/math/transform.h"
#include "coal/collision_data.h"

namespace coal {

/// @brief Node structure encoding the information required for traversal.

class TraversalNodeBase {
 public:
  TraversalNodeBase() : enable_statistics(false) {}

  virtual ~TraversalNodeBase() {}

  virtual void preprocess() {}

  virtual void postprocess() {}

  /// @brief Whether b is a leaf node in the first BVH tree
  virtual bool isFirstNodeLeaf(unsigned int /*b*/) const { return true; }

  /// @brief Whether b is a leaf node in the second BVH tree
  virtual bool isSecondNodeLeaf(unsigned int /*b*/) const { return true; }

  /// @brief Traverse the subtree of the node in the first tree first
  virtual bool firstOverSecond(unsigned int /*b1*/, unsigned int /*b2*/) const {
    return true;
  }

  /// @brief Get the left child of the node b in the first tree
  virtual int getFirstLeftChild(unsigned int b) const { return (int)b; }

  /// @brief Get the right child of the node b in the first tree
  virtual int getFirstRightChild(unsigned int b) const { return (int)b; }

  /// @brief Get the left child of the node b in the second tree
  virtual int getSecondLeftChild(unsigned int b) const { return (int)b; }

  /// @brief Get the right child of the node b in the second tree
  virtual int getSecondRightChild(unsigned int b) const { return (int)b; }

  /// @brief Whether store some statistics information during traversal
  void enableStatistics(bool enable) { enable_statistics = enable; }

  /// @brief configuation of first object
  Transform3s tf1;

  /// @brief configuration of second object
  Transform3s tf2;

  /// @brief Whether stores statistics
  bool enable_statistics;
};

/// @defgroup Traversal_For_Collision
/// regroup class about traversal for distance.
/// @{

/// @brief Node structure encoding the information required for collision
/// traversal.
class CollisionTraversalNodeBase : public TraversalNodeBase {
 public:
  CollisionTraversalNodeBase(const CollisionRequest& request_)
      : request(request_), result(NULL) {}

  virtual ~CollisionTraversalNodeBase() {}

  /// BV test between b1 and b2
  /// @param b1, b2 Bounding volumes to test,
  /// @retval sqrDistLowerBound square of a lower bound of the minimal
  ///         distance between bounding volumes.
  virtual bool BVDisjoints(unsigned int b1, unsigned int b2,
                           Scalar& sqrDistLowerBound) const = 0;

  /// @brief Leaf test between node b1 and b2, if they are both leafs
  virtual void leafCollides(unsigned int /*b1*/, unsigned int /*b2*/,
                            Scalar& /*sqrDistLowerBound*/) const = 0;

  /// @brief Check whether the traversal can stop
  bool canStop() const { return this->request.isSatisfied(*(this->result)); }

  /// @brief request setting for collision
  const CollisionRequest& request;

  /// @brief collision result kept during the traversal iteration
  CollisionResult* result;
};

/// @}

/// @defgroup Traversal_For_Distance
/// regroup class about traversal for distance.
/// @{

/// @brief Node structure encoding the information required for distance
/// traversal.
class DistanceTraversalNodeBase : public TraversalNodeBase {
 public:
  DistanceTraversalNodeBase() : result(NULL) {}

  virtual ~DistanceTraversalNodeBase() {}

  /// @brief BV test between b1 and b2
  /// @return a lower bound of the distance between the two BV.
  /// @note except for OBB, this method returns the distance.
  virtual Scalar BVDistanceLowerBound(unsigned int /*b1*/,
                                      unsigned int /*b2*/) const {
    return (std::numeric_limits<Scalar>::max)();
  }

  /// @brief Leaf test between node b1 and b2, if they are both leafs
  virtual void leafComputeDistance(unsigned int b1, unsigned int b2) const = 0;

  /// @brief Check whether the traversal can stop
  virtual bool canStop(Scalar /*c*/) const { return false; }

  /// @brief request setting for distance
  DistanceRequest request;

  /// @brief distance result kept during the traversal iteration
  DistanceResult* result;
};

///@}

}  // namespace coal

/// @endcond

#endif
