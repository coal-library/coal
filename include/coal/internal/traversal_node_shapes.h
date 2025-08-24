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

#ifndef COAL_TRAVERSAL_NODE_SHAPES_H
#define COAL_TRAVERSAL_NODE_SHAPES_H

/// @cond INTERNAL

#include "coal/collision_data.h"
#include "coal/BV/BV.h"
#include "coal/shape/geometric_shapes_utility.h"
#include "coal/internal/traversal_node_base.h"
#include "coal/internal/shape_shape_func.h"

namespace coal {

/// @addtogroup Traversal_For_Collision
/// @{

/// @brief Traversal node for collision between two shapes
template <typename S1, typename S2>
class COAL_DLLAPI ShapeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  ShapeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  bool BVDisjoints(int, int, Scalar&) const {
    COAL_THROW_PRETTY("Not implemented", std::runtime_error);
  }

  /// @brief Intersection testing between leaves (two shapes)
  void leafCollides(int, int, Scalar&) const {
    ShapeShapeCollide<S1, S2>(this->model1, this->tf1, this->model2, this->tf2,
                              this->nsolver, this->request, *(this->result));
  }

  const S1* model1;
  const S2* model2;

  const GJKSolver* nsolver;
};

/// @}

/// @addtogroup Traversal_For_Distance
/// @{

/// @brief Traversal node for distance between two shapes
template <typename S1, typename S2>
class COAL_DLLAPI ShapeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  ShapeDistanceTraversalNode() : DistanceTraversalNodeBase() {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  Scalar BVDistanceLowerBound(unsigned int, unsigned int) const {
    return -1;  // should not be used
  }

  /// @brief Distance testing between leaves (two shapes)
  void leafComputeDistance(unsigned int, unsigned int) const {
    ShapeShapeDistance<S1, S2>(this->model1, this->tf1, this->model2, this->tf2,
                               this->nsolver, this->request, *(this->result));
  }

  const S1* model1;
  const S2* model2;

  const GJKSolver* nsolver;
};

/// @}

}  // namespace coal

/// @endcond

#endif
