/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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
 *   * Neither the name of INRIA nor the names of its
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

/** \author Louis Montaut */

#ifndef COAL_CONTACT_PATCH_H
#define COAL_CONTACT_PATCH_H

#include "coal/data_types.h"
#include "coal/collision_data.h"
#include "coal/contact_patch/contact_patch_solver.h"
#include "coal/contact_patch_func_matrix.h"
#include "coal/contact_patch/contact_patch_simplifier.h"

namespace coal {

/// @brief Main contact patch computation interface.
/// @note Please see @ref ContactPatchRequest and @ref ContactPatchResult for
/// more info on the content of the input/output of this function. Also, please
/// read @ref ContactPatch if you want to fully understand what is meant by
/// "contact patch".
COAL_DLLAPI void computeContactPatch(const CollisionGeometry* o1,
                                     const Transform3s& tf1,
                                     const CollisionGeometry* o2,
                                     const Transform3s& tf2,
                                     const CollisionResult& collision_result,
                                     const ContactPatchRequest& request,
                                     ContactPatchResult& result);

/// @copydoc computeContactPatch(const CollisionGeometry*, const Transform3s&,
// const CollisionGeometry*, const Transform3s&, const CollisionResult&, const
// ContactPatchRequest&, ContactPatchResult&);
COAL_DLLAPI void computeContactPatch(const CollisionObject* o1,
                                     const CollisionObject* o2,
                                     const CollisionResult& collision_result,
                                     const ContactPatchRequest& request,
                                     ContactPatchResult& result);

/// @brief This class reduces the cost of identifying the geometry pair.
/// This is usefull for repeated shape-shape queries.
/// @note This needs to be called after `collide` or after `ComputeCollision`.
///
/// \code
///   ComputeContactPatch calc_patch (o1, o2);
///   calc_patch(tf1, tf2, collision_result, patch_request, patch_result);
/// \endcode
class COAL_DLLAPI ComputeContactPatch {
 public:
  /// @brief Default constructor from two Collision Geometries.
  ComputeContactPatch(const CollisionGeometry* o1, const CollisionGeometry* o2);

  void operator()(const Transform3s& tf1, const Transform3s& tf2,
                  const CollisionResult& collision_result,
                  const ContactPatchRequest& request,
                  ContactPatchResult& result) const;

  bool operator==(const ComputeContactPatch& other) const {
    COAL_EQUAL_OPERATOR_CHECK(((o1 == nullptr && other.o1 == nullptr) ||
                               (o1 != nullptr && other.o1 != nullptr)));
    COAL_EQUAL_OPERATOR_CHECK(((o2 == nullptr && other.o2 == nullptr) ||
                               (o2 != nullptr && other.o2 != nullptr)));
    if ((o1 && other.o1) && (o1 != other.o1))
      COAL_EQUAL_OPERATOR_CHECK(*o1 == *other.o1);
    if ((o2 && other.o2) && (o2 != other.o2))
      COAL_EQUAL_OPERATOR_CHECK(*o2 == *other.o2);
    COAL_EQUAL_OPERATOR_CHECK(csolver == other.csolver);
    COAL_EQUAL_OPERATOR_CHECK(func == other.func);
    COAL_EQUAL_OPERATOR_CHECK(swap_geoms == other.swap_geoms);
    return true;
  }

  bool operator!=(const ComputeContactPatch& other) const {
    return !(*this == other);
  }

  virtual ~ComputeContactPatch() = default;

 protected:
  // These pointers are made mutable to let the derived classes to update
  // their values when updating the collision geometry (e.g. creating a new
  // one). This feature should be used carefully to avoid any mis usage (e.g,
  // changing the type of the collision geometry should be avoided).
  mutable const CollisionGeometry* o1;
  mutable const CollisionGeometry* o2;

  mutable ContactPatchSolver csolver;

  ContactPatchFunctionMatrix::ContactPatchFunc func;
  bool swap_geoms;

  virtual void run(const Transform3s& tf1, const Transform3s& tf2,
                   const CollisionResult& collision_result,
                   const ContactPatchRequest& request,
                   ContactPatchResult& result) const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace coal

#endif
