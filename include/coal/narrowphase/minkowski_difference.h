/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
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

/** \authors Jia Pan, Florent Lamiraux, Josef Mirabel, Louis Montaut */

#ifndef COAL_MINKOWSKI_DIFFERENCE_H
#define COAL_MINKOWSKI_DIFFERENCE_H

#include "coal/shape/geometric_shapes.h"
#include "coal/math/transform.h"
#include "coal/narrowphase/support_functions.h"

namespace coal {

namespace details {

/// @brief Minkowski difference class of two shapes
///
/// @note The Minkowski difference is expressed in the frame of the first shape.
struct COAL_DLLAPI MinkowskiDiff {
  typedef Eigen::Array<Scalar, 1, 2> Array2;

  /// @brief points to two shapes
  const ShapeBase* shapes[2];

  /// @brief Store temporary data for the computation of the support point for
  /// each shape.
  ShapeSupportData data[2];

  /// @brief rotation from shape1 to shape0
  /// such that @f$ p_in_0 = oR1 * p_in_1 + ot1 @f$.
  Matrix3s oR1;

  /// @brief translation from shape1 to shape0
  /// such that @f$ p_in_0 = oR1 * p_in_1 + ot1 @f$.
  Vec3s ot1;

  /// @brief The radii of the sphere swepted around each shape of the Minkowski
  /// difference. The 2 values correspond to the swept-sphere radius of shape 0
  /// and shape 1.
  Array2 swept_sphere_radius;

  /// @brief Wether or not to use the normalize heuristic in the GJK Nesterov
  /// acceleration. This setting is only applied if the Nesterov acceleration in
  /// the GJK class is active.
  bool normalize_support_direction;

  typedef void (*GetSupportFunction)(const MinkowskiDiff& minkowskiDiff,
                                     const Vec3s& dir, Vec3s& support0,
                                     Vec3s& support1,
                                     support_func_guess_t& hint,
                                     ShapeSupportData data[2]);
  GetSupportFunction getSupportFunc;

  MinkowskiDiff() : normalize_support_direction(false), getSupportFunc(NULL) {}

  /// @brief Set the two shapes, assuming the relative transformation between
  /// them is identity.
  /// Consequently, all support points computed with the MinkowskiDiff
  /// will be expressed in the world frame.
  /// @param shape0 the first shape.
  /// @param shape1 the second shape.
  /// @tparam _SupportOptions is a value of the SupportOptions enum. If set to
  /// `WithSweptSphere`, the support computation will take into account the
  /// swept sphere radius of the shapes. If set to `NoSweptSphere`, where
  /// this information is simply stored in the Minkowski's difference
  /// `swept_sphere_radius` array. This array is then used to correct the
  /// solution found when GJK or EPA have converged.
  ///
  /// @note In practice, there is no need to take into account the swept-sphere
  /// radius in the iterations of GJK/EPA. It is in fact detrimental to the
  /// convergence of both algos. This is because it makes corners and edges of
  /// shapes look strictly convex to the algorithms, which leads to poor
  /// convergence. This swept sphere template parameter is only here for
  /// debugging purposes and for specific uses cases where the swept sphere
  /// radius is needed in the support function. The rule is simple. When
  /// interacting with GJK/EPA, the `SupportOptions` template
  /// parameter should **always** be set to `NoSweptSphere` (except for
  /// debugging or testing purposes). When working directly with the shapes
  /// involved in the collision, and not relying on GJK/EPA, the
  /// `SupportOptions` template parameter should be set to `WithSweptSphere`.
  /// This is for example the case for specialized collision/distance functions.
  template <int _SupportOptions = SupportOptions::NoSweptSphere>
  void set(const ShapeBase* shape0, const ShapeBase* shape1);

  /// @brief Set the two shapes, with a relative transformation from shape0 to
  /// shape1. Consequently, all support points computed with the MinkowskiDiff
  /// will be expressed in the frame of shape0.
  /// @param shape0 the first shape.
  /// @param shape1 the second shape.
  /// @param tf0 the transformation of the first shape.
  /// @param tf1 the transformation of the second shape.
  /// @tparam _SupportOptions see `set(const ShapeBase*, const
  /// ShapeBase*)` for more details.
  template <int _SupportOptions = SupportOptions::NoSweptSphere>
  void set(const ShapeBase* shape0, const ShapeBase* shape1,
           const Transform3s& tf0, const Transform3s& tf1);

  /// @brief support function for shape0.
  /// The output vector is expressed in the local frame of shape0.
  /// @return a point which belongs to the set {argmax_{v in shape0}
  /// v.dot(dir)}, i.e a support of shape0 in direction dir.
  /// @param dir support direction.
  /// @param hint used to initialize the search when shape is a ConvexBase
  /// object.
  /// @tparam _SupportOptions see `set(const ShapeBase*, const
  /// ShapeBase*)` for more details.
  template <int _SupportOptions = SupportOptions::NoSweptSphere>
  inline Vec3s support0(const Vec3s& dir, int& hint) const {
    return getSupport<_SupportOptions>(shapes[0], dir, hint);
  }

  /// @brief support function for shape1.
  /// The output vector is expressed in the local frame of shape0.
  /// This is mandatory because in the end we are interested in support points
  /// of the Minkowski difference; hence the supports of shape0 and shape1 must
  /// live in the same frame.
  /// @return a point which belongs to the set {tf * argmax_{v in shape1}
  /// v.dot(R^T * dir)}, i.e. the support of shape1 in direction dir (tf is the
  /// tranform from shape1 to shape0).
  /// @param dir support direction.
  /// @param hint used to initialize the search when shape is a ConvexBase.
  /// @tparam _SupportOptions see `set(const ShapeBase*, const
  /// ShapeBase*)` for more details.
  template <int _SupportOptions = SupportOptions::NoSweptSphere>
  inline Vec3s support1(const Vec3s& dir, int& hint) const {
    // clang-format off
    return oR1 * getSupport<_SupportOptions>(shapes[1], oR1.transpose() * dir, hint) + ot1;
    // clang-format on
  }

  /// @brief Support function for the pair of shapes. This method assumes `set`
  /// has already been called.
  /// @param[in] dir the support direction.
  /// @param[out] supp0 support of shape0 in direction dir, expressed in the
  /// frame of shape0.
  /// @param[out] supp1 support of shape1 in direction -dir, expressed in the
  /// frame of shape0.
  /// @param[in/out] hint used to initialize the search when shape is a
  /// ConvexBase object.
  inline void support(const Vec3s& dir, Vec3s& supp0, Vec3s& supp1,
                      support_func_guess_t& hint) const {
    assert(getSupportFunc != NULL);
    getSupportFunc(*this, dir, supp0, supp1, hint,
                   const_cast<ShapeSupportData*>(data));
  }
};

}  // namespace details

}  // namespace coal

#endif  // COAL_MINKOWSKI_DIFFERENCE_H
