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

/** \authors Louis Montaut */

#include "coal/contact_patch/contact_patch_solver.h"

namespace coal {

namespace details {

/// @brief Templated shape support set functions.
template <typename ShapeType,
          int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSetTpl(const ShapeBase* shape, SupportSet& support_set,
                           int& hint, ShapeSupportData& support_data,
                           size_t num_sampled_supports = 6,
                           Scalar tol = Scalar(1e-3)) {
  const ShapeType* shape_ = static_cast<const ShapeType*>(shape);
  getShapeSupportSet<_SupportOptions>(shape_, support_set, hint, support_data,
                                      num_sampled_supports, tol);
}

/// @brief Templated shape support set functions for ConvexBase.
template <typename IndexType,
          int _SupportOptions = SupportOptions::NoSweptSphere>
void getConvexBaseSupportSetTpl(const ShapeBase* shape, SupportSet& support_set,
                                int& hint, ShapeSupportData& support_data,
                                size_t num_sampled_supports = 6,
                                Scalar tol = Scalar(1e-3)) {
  const ConvexBaseTpl<IndexType>* convex =
      static_cast<const ConvexBaseTpl<IndexType>*>(shape);

  if (support_data.polygon.capacity() <
      ::coal::ContactPatchSolver::default_num_preallocated_supports) {
    support_data.polygon.reserve(
        ::coal::ContactPatchSolver::default_num_preallocated_supports);
  }

  if ((size_t)(convex->num_points) >
      ConvexBaseTpl<IndexType>::num_vertices_large_convex_threshold) {
    const LargeConvex<IndexType>* convex_ =
        static_cast<const LargeConvex<IndexType>*>(convex);
    support_data.visited.assign(convex_->num_points, false);
    support_data.last_dir.setZero();
    return getShapeSupportSet<_SupportOptions>(
        convex_, support_set, hint, support_data, num_sampled_supports, tol);
  } else {
    const SmallConvex<IndexType>* convex_ =
        static_cast<const SmallConvex<IndexType>*>(convex);
    return getShapeSupportSet<_SupportOptions>(
        convex_, support_set, hint, support_data, num_sampled_supports, tol);
  }
}

}  // namespace details

// ============================================================================
ContactPatchSolver::SupportSetFunction
ContactPatchSolver::makeSupportSetFunction(const ShapeBase* shape,
                                           ShapeSupportData& support_data) {
  // Note: because the swept-sphere radius was already taken into account when
  // constructing the contact patch frame, there is actually no need to take the
  // swept-sphere radius of shapes into account. The origin of the contact patch
  // frame already encodes this information.
  using Options = details::SupportOptions;
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      return details::getShapeSupportSetTpl<TriangleP, Options::NoSweptSphere>;
    case GEOM_BOX: {
      const size_t num_corners_box = 8;
      support_data.polygon.reserve(num_corners_box);
      return details::getShapeSupportSetTpl<Box, Options::NoSweptSphere>;
    }
    case GEOM_SPHERE:
      return details::getShapeSupportSetTpl<Sphere, Options::NoSweptSphere>;
    case GEOM_ELLIPSOID:
      return details::getShapeSupportSetTpl<Ellipsoid, Options::NoSweptSphere>;
    case GEOM_CAPSULE:
      return details::getShapeSupportSetTpl<Capsule, Options::NoSweptSphere>;
    case GEOM_CONE:
      return details::getShapeSupportSetTpl<Cone, Options::NoSweptSphere>;
    case GEOM_CYLINDER:
      return details::getShapeSupportSetTpl<Cylinder, Options::NoSweptSphere>;
    case GEOM_CONVEX16:
      return details::getConvexBaseSupportSetTpl<Triangle16::IndexType,
                                                 Options::NoSweptSphere>;
    case GEOM_CONVEX32:
      return details::getConvexBaseSupportSetTpl<Triangle32::IndexType,
                                                 Options::NoSweptSphere>;
    default:
      COAL_THROW_PRETTY("Unsupported geometric shape.", std::logic_error);
  }
}

}  // namespace coal
