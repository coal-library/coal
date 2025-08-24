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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef COAL_INTERNAL_SHAPE_SHAPE_CONTACT_PATCH_FUNC_H
#define COAL_INTERNAL_SHAPE_SHAPE_CONTACT_PATCH_FUNC_H

#include "coal/collision_data.h"
#include "coal/collision_utility.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/contact_patch/contact_patch_solver.h"
#include "coal/shape/geometric_shapes_traits.h"

#include "coal/tracy.hh"

namespace coal {

/// @brief Shape-shape contact patch computation.
/// Assumes that `csolver` and the `ContactPatchResult` have already been set up
/// by the `ContactPatchRequest`.
template <typename ShapeType1, typename ShapeType2>
struct ComputeShapeShapeContactPatch {
  static void run(const CollisionGeometry* o1, const Transform3s& tf1,
                  const CollisionGeometry* o2, const Transform3s& tf2,
                  const CollisionResult& collision_result,
                  const ContactPatchSolver* csolver,
                  const ContactPatchRequest& request,
                  ContactPatchResult& result) {
    // Note: see specializations for Plane and Halfspace below.
    if (!collision_result.isCollision()) {
      return;
    }
    COAL_ASSERT(
        result.check(request),
        "The contact patch result and request are incompatible (issue of "
        "contact patch size or maximum number of contact patches). Make sure "
        "result is initialized with request.",
        std::logic_error);

    const ShapeType1& s1 = static_cast<const ShapeType1&>(*o1);
    const ShapeType2& s2 = static_cast<const ShapeType2&>(*o2);
    for (size_t i = 0; i < collision_result.numContacts(); ++i) {
      if (i >= request.max_num_patch) {
        break;
      }
      csolver->setSupportGuess(collision_result.cached_support_func_guess);
      const Contact& contact = collision_result.getContact(i);
      ContactPatch& contact_patch = result.getUnusedContactPatch();
      csolver->computePatch(s1, tf1, s2, tf2, contact, contact_patch);
    }
  }
};

/// @brief Computes the contact patch between a Plane/Halfspace and another
/// shape.
/// @tparam InvertShapes set to true if the first shape of the collision pair
/// is s2 and not s1 (if you had to invert (s1, tf1) and (s2, tf2) when calling
/// this function).
template <bool InvertShapes, typename OtherShapeType, typename PlaneOrHalfspace>
void computePatchPlaneOrHalfspace(const OtherShapeType& s1,
                                  const Transform3s& tf1,
                                  const PlaneOrHalfspace& s2,
                                  const Transform3s& tf2,
                                  const ContactPatchSolver* csolver,
                                  const Contact& contact,
                                  ContactPatch& contact_patch) {
  COAL_UNUSED_VARIABLE(s2);
  COAL_UNUSED_VARIABLE(tf2);
  constructContactPatchFrameFromContact(contact, contact_patch);
  if ((bool)(shape_traits<OtherShapeType>::IsStrictlyConvex)) {
    // Only one point of contact; it has already been computed.
    contact_patch.addPoint(contact.pos);
    return;
  }

  // We only need to compute the support set in the direction of the normal.
  // We need to temporarily express the patch in the local frame of shape1.
  SupportSet& support_set = csolver->support_set_shape1;
  support_set.tf.rotation().noalias() =
      tf1.rotation().transpose() * contact_patch.tf.rotation();
  support_set.tf.translation().noalias() =
      tf1.rotation().transpose() *
      (contact_patch.tf.translation() - tf1.translation());

  // Note: for now, taking into account swept-sphere radius does not change
  // anything to the support set computations. However it will be used in the
  // future if we want to store the offsets to the support plane for each point
  // in a support set.
  using SupportOptions = details::SupportOptions;
  if (InvertShapes) {
    support_set.direction = ContactPatch::PatchDirection::INVERTED;
    details::getShapeSupportSet<SupportOptions::WithSweptSphere>(
        &s1, support_set, csolver->support_guess[1], csolver->supports_data[1],
        csolver->num_samples_curved_shapes, csolver->patch_tolerance);
  } else {
    support_set.direction = ContactPatch::PatchDirection::DEFAULT;
    details::getShapeSupportSet<SupportOptions::WithSweptSphere>(
        &s1, support_set, csolver->support_guess[0], csolver->supports_data[0],
        csolver->num_samples_curved_shapes, csolver->patch_tolerance);
  }
  csolver->getResult(contact, &(support_set.points()), contact_patch);
}

#define PLANE_OR_HSPACE_AND_OTHER_SHAPE_CONTACT_PATCH(PlaneOrHspace)          \
  template <typename OtherShapeType>                                          \
  struct ComputeShapeShapeContactPatch<OtherShapeType, PlaneOrHspace> {       \
    static void run(const CollisionGeometry* o1, const Transform3s& tf1,      \
                    const CollisionGeometry* o2, const Transform3s& tf2,      \
                    const CollisionResult& collision_result,                  \
                    const ContactPatchSolver* csolver,                        \
                    const ContactPatchRequest& request,                       \
                    ContactPatchResult& result) {                             \
      COAL_TRACY_ZONE_SCOPED_N(                                               \
          "coal::ComputeShapeShapeContactPatch<OtherShapeType, "              \
          "PlaneOrHspace>::run");                                             \
      if (!collision_result.isCollision()) {                                  \
        return;                                                               \
      }                                                                       \
      COAL_ASSERT(                                                            \
          result.check(request),                                              \
          "The contact patch result and request are incompatible (issue of "  \
          "contact patch size or maximum number of contact patches). Make "   \
          "sure "                                                             \
          "result is initialized with request.",                              \
          std::logic_error);                                                  \
                                                                              \
      const OtherShapeType& s1 = static_cast<const OtherShapeType&>(*o1);     \
      const PlaneOrHspace& s2 = static_cast<const PlaneOrHspace&>(*o2);       \
      for (size_t i = 0; i < collision_result.numContacts(); ++i) {           \
        if (i >= request.max_num_patch) {                                     \
          break;                                                              \
        }                                                                     \
        csolver->setSupportGuess(collision_result.cached_support_func_guess); \
        const Contact& contact = collision_result.getContact(i);              \
        ContactPatch& contact_patch = result.getUnusedContactPatch();         \
        computePatchPlaneOrHalfspace<false, OtherShapeType, PlaneOrHspace>(   \
            s1, tf1, s2, tf2, csolver, contact, contact_patch);               \
      }                                                                       \
    }                                                                         \
  };                                                                          \
                                                                              \
  template <typename OtherShapeType>                                          \
  struct ComputeShapeShapeContactPatch<PlaneOrHspace, OtherShapeType> {       \
    static void run(const CollisionGeometry* o1, const Transform3s& tf1,      \
                    const CollisionGeometry* o2, const Transform3s& tf2,      \
                    const CollisionResult& collision_result,                  \
                    const ContactPatchSolver* csolver,                        \
                    const ContactPatchRequest& request,                       \
                    ContactPatchResult& result) {                             \
      COAL_TRACY_ZONE_SCOPED_N(                                               \
          "coal::ComputeShapeShapeContactPatch<PlaneOrHspace, "               \
          "OtherShapeType>::run");                                            \
      if (!collision_result.isCollision()) {                                  \
        return;                                                               \
      }                                                                       \
      COAL_ASSERT(                                                            \
          result.check(request),                                              \
          "The contact patch result and request are incompatible (issue of "  \
          "contact patch size or maximum number of contact patches). Make "   \
          "sure "                                                             \
          "result is initialized with request.",                              \
          std::logic_error);                                                  \
                                                                              \
      const PlaneOrHspace& s1 = static_cast<const PlaneOrHspace&>(*o1);       \
      const OtherShapeType& s2 = static_cast<const OtherShapeType&>(*o2);     \
      for (size_t i = 0; i < collision_result.numContacts(); ++i) {           \
        if (i >= request.max_num_patch) {                                     \
          break;                                                              \
        }                                                                     \
        csolver->setSupportGuess(collision_result.cached_support_func_guess); \
        const Contact& contact = collision_result.getContact(i);              \
        ContactPatch& contact_patch = result.getUnusedContactPatch();         \
        computePatchPlaneOrHalfspace<true, OtherShapeType, PlaneOrHspace>(    \
            s2, tf2, s1, tf1, csolver, contact, contact_patch);               \
      }                                                                       \
    }                                                                         \
  };

PLANE_OR_HSPACE_AND_OTHER_SHAPE_CONTACT_PATCH(Plane)
PLANE_OR_HSPACE_AND_OTHER_SHAPE_CONTACT_PATCH(Halfspace)

#define PLANE_HSPACE_CONTACT_PATCH(PlaneOrHspace1, PlaneOrHspace2)           \
  template <>                                                                \
  struct ComputeShapeShapeContactPatch<PlaneOrHspace1, PlaneOrHspace2> {     \
    static void run(const CollisionGeometry* o1, const Transform3s& tf1,     \
                    const CollisionGeometry* o2, const Transform3s& tf2,     \
                    const CollisionResult& collision_result,                 \
                    const ContactPatchSolver* csolver,                       \
                    const ContactPatchRequest& request,                      \
                    ContactPatchResult& result) {                            \
      COAL_UNUSED_VARIABLE(o1);                                              \
      COAL_UNUSED_VARIABLE(tf1);                                             \
      COAL_UNUSED_VARIABLE(o2);                                              \
      COAL_UNUSED_VARIABLE(tf2);                                             \
      COAL_UNUSED_VARIABLE(csolver);                                         \
      if (!collision_result.isCollision()) {                                 \
        return;                                                              \
      }                                                                      \
      COAL_ASSERT(                                                           \
          result.check(request),                                             \
          "The contact patch result and request are incompatible (issue of " \
          "contact patch size or maximum number of contact patches). Make "  \
          "sure "                                                            \
          "result is initialized with request.",                             \
          std::logic_error);                                                 \
                                                                             \
      for (size_t i = 0; i < collision_result.numContacts(); ++i) {          \
        if (i >= request.max_num_patch) {                                    \
          break;                                                             \
        }                                                                    \
        const Contact& contact = collision_result.getContact(i);             \
        ContactPatch& contact_patch = result.getUnusedContactPatch();        \
        constructContactPatchFrameFromContact(contact, contact_patch);       \
        contact_patch.addPoint(contact.pos);                                 \
      }                                                                      \
    }                                                                        \
  };

PLANE_HSPACE_CONTACT_PATCH(Plane, Plane)
PLANE_HSPACE_CONTACT_PATCH(Plane, Halfspace)
PLANE_HSPACE_CONTACT_PATCH(Halfspace, Plane)
PLANE_HSPACE_CONTACT_PATCH(Halfspace, Halfspace)

#undef PLANE_OR_HSPACE_AND_OTHER_SHAPE_CONTACT_PATCH
#undef PLANE_HSPACE_CONTACT_PATCH

template <typename ShapeType1, typename ShapeType2>
void ShapeShapeContactPatch(const CollisionGeometry* o1, const Transform3s& tf1,
                            const CollisionGeometry* o2, const Transform3s& tf2,
                            const CollisionResult& collision_result,
                            const ContactPatchSolver* csolver,
                            const ContactPatchRequest& request,
                            ContactPatchResult& result) {
  COAL_TRACY_ZONE_SCOPED_N("coal::ShapeShapeContactPatch");
  return ComputeShapeShapeContactPatch<ShapeType1, ShapeType2>::run(
      o1, tf1, o2, tf2, collision_result, csolver, request, result);
}

}  // namespace coal

#endif
