/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021, INRIA
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

#ifndef COAL_COLLISION_H
#define COAL_COLLISION_H

#include "coal/data_types.h"
#include "coal/collision_object.h"
#include "coal/collision_data.h"
#include "coal/collision_func_matrix.h"
#include "coal/timings.h"

namespace coal {

/// @brief Main collision interface: given two collision objects, and the
/// requirements for contacts, including num of max contacts, whether perform
/// exhaustive collision (i.e., returning returning all the contact points),
/// whether return detailed contact information (i.e., normal, contact point,
/// depth; otherwise only contact primitive id is returned), this function
/// performs the collision between them.
/// Return value is the number of contacts generated between the two objects.
COAL_DLLAPI std::size_t collide(const CollisionObject* o1,
                                const CollisionObject* o2,
                                const CollisionRequest& request,
                                CollisionResult& result);

/// @copydoc collide(const CollisionObject*, const CollisionObject*, const
/// CollisionRequest&, CollisionResult&)
COAL_DLLAPI std::size_t collide(const CollisionGeometry* o1,
                                const Transform3s& tf1,
                                const CollisionGeometry* o2,
                                const Transform3s& tf2,
                                const CollisionRequest& request,
                                CollisionResult& result);

/// @brief This class reduces the cost of identifying the geometry pair.
/// This is mostly useful for repeated shape-shape queries.
///
/// \code
///   ComputeCollision calc_collision (o1, o2);
///   std::size_t ncontacts = calc_collision(tf1, tf2, request, result);
/// \endcode
class COAL_DLLAPI ComputeCollision {
 public:
  /// @brief Default constructor from two Collision Geometries.
  ComputeCollision(const CollisionGeometry* o1, const CollisionGeometry* o2);

  std::size_t operator()(const Transform3s& tf1, const Transform3s& tf2,
                         const CollisionRequest& request,
                         CollisionResult& result) const;

  bool operator==(const ComputeCollision& other) const {
    COAL_EQUAL_OPERATOR_CHECK(((o1 == nullptr && other.o1 == nullptr) ||
                               (o1 != nullptr && other.o1 != nullptr)));
    COAL_EQUAL_OPERATOR_CHECK(((o2 == nullptr && other.o2 == nullptr) ||
                               (o2 != nullptr && other.o2 != nullptr)));
    if ((o1 && other.o1) && (o1 != other.o1))
      COAL_EQUAL_OPERATOR_CHECK(*o1 == *other.o1);
    if ((o2 && other.o2) && (o2 != other.o2))
      COAL_EQUAL_OPERATOR_CHECK(*o2 == *other.o2);
    COAL_EQUAL_OPERATOR_CHECK(solver == other.solver);
    COAL_EQUAL_OPERATOR_CHECK(func == other.func);
    return true;
  }

  bool operator!=(const ComputeCollision& other) const {
    return !(*this == other);
  }

  virtual ~ComputeCollision() {};

 protected:
  // These pointers are made mutable to let the derived classes to update
  // their values when updating the collision geometry (e.g. creating a new
  // one). This feature should be used carefully to avoid any mis usage (e.g,
  // changing the type of the collision geometry should be avoided).
  mutable const CollisionGeometry* o1;
  mutable const CollisionGeometry* o2;

  mutable GJKSolver solver;

  CollisionFunctionMatrix::CollisionFunc func;
  bool swap_geoms;

  virtual std::size_t run(const Transform3s& tf1, const Transform3s& tf2,
                          const CollisionRequest& request,
                          CollisionResult& result) const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace coal

#endif
