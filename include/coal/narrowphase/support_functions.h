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

#ifndef COAL_SUPPORT_FUNCTIONS_H
#define COAL_SUPPORT_FUNCTIONS_H

#include "coal/shape/geometric_shapes.h"
#include "coal/math/transform.h"
#include "coal/collision_data.h"
#include "coal/narrowphase/support_data.h"

namespace coal {
namespace details {

// ============================================================================
// ============================ SUPPORT FUNCTIONS =============================
// ============================================================================
/// @brief the support function for shape.
/// The output support point is expressed in the local frame of the shape.
/// @return a point which belongs to the set {argmax_{v in shape} v.dot(dir)}.
/// @param shape the shape.
/// @param dir support direction.
/// @param hint used to initialize the search when shape is a ConvexBase object.
/// @tparam _SupportOptions is a value of the SupportOptions enum. If set to
/// `WithSweptSphere`, the support functions take into account the shapes' swept
/// sphere radii. Please see `MinkowskiDiff::set(const ShapeBase*, const
/// ShapeBase*)` for more details.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
Vec3s getSupport(const ShapeBase* shape, const Vec3s& dir, int& hint);

/// @brief Triangle support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const TriangleP* triangle, const Vec3s& dir,
                     Vec3s& support, int& /*unused*/,
                     ShapeSupportData& /*unused*/);

/// @brief Box support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Box* box, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/);

/// @brief Sphere support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Sphere* sphere, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/);

/// @brief Ellipsoid support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Ellipsoid* ellipsoid, const Vec3s& dir,
                     Vec3s& support, int& /*unused*/,
                     ShapeSupportData& /*unused*/);

/// @brief Capsule support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Capsule* capsule, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/);

/// @brief Cone support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Cone* cone, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/);

/// @brief Cylinder support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Cylinder* cylinder, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/);

/// @brief ConvexBase support function.
/// @note See @ref LargeConvex and SmallConvex to see how to optimize
/// ConvexBase's support computation.
template <int _SupportOptions = SupportOptions::NoSweptSphere,
          typename IndexType>
void getShapeSupport(const ConvexBaseTpl<IndexType>* convex, const Vec3s& dir,
                     Vec3s& support, int& hint, ShapeSupportData& /*unused*/);

/// @brief Cast a `ConvexBase` to a `LargeConvex` to use the log version of
/// `getShapeSupport`. This is **much** faster than the linear version of
/// `getShapeSupport` when a `ConvexBase` has more than a few dozen of vertices.
/// @note WARNING: when using a LargeConvex, the neighbors in `ConvexBase` must
/// have been constructed! Otherwise the support function will segfault.
template <typename IndexType>
struct LargeConvex : ConvexBaseTpl<IndexType> {};
typedef LargeConvex<Triangle16::IndexType> LargeConvex16;
typedef LargeConvex<Triangle32::IndexType> LargeConvex32;

/// @brief See @ref LargeConvex.
template <typename IndexType>
struct SmallConvex : ConvexBaseTpl<IndexType> {};
typedef SmallConvex<Triangle16::IndexType> SmallConvex16;
typedef SmallConvex<Triangle32::IndexType> SmallConvex32;

/// @brief Support function for large ConvexBase (>32 vertices).
template <int _SupportOptions = SupportOptions::NoSweptSphere,
          typename IndexType>
void getShapeSupport(const SmallConvex<IndexType>* convex, const Vec3s& dir,
                     Vec3s& support, int& hint, ShapeSupportData& data);

/// @brief Support function for small ConvexBase (<32 vertices).
template <int _SupportOptions = SupportOptions::NoSweptSphere,
          typename IndexType>
void getShapeSupport(const LargeConvex<IndexType>* convex, const Vec3s& dir,
                     Vec3s& support, int& hint, ShapeSupportData& support_data);

// ============================================================================
// ========================== SUPPORT SET FUNCTIONS ===========================
// ============================================================================
/// @brief Computes the support set for shape.
/// This function assumes the frame of the support set has already been
/// computed and that this frame is expressed w.r.t the local frame of the
/// shape (i.e. the local frame of the shape is the WORLD frame of the support
/// set). The support direction used to compute the support set is the positive
/// z-axis if the support set has the DEFAULT direction; negative z-axis if it
/// has the INVERTED direction. (In short, a shape's support set is has the
/// DEFAULT direction if the shape is the first shape in a collision pair. It
/// has the INVERTED direction if the shape is the second one in the collision
/// pair).
/// @return an approximation of the set {argmax_{v in shape} v.dot(dir)}, where
/// dir is the support set's support direction.
/// The support set is a plane passing by the origin of the support set frame
/// and supported by the direction dir. As a consequence, any point added to the
/// set is automatically projected onto this plane.
/// @param[in] shape the shape.
/// @param[in/out] support_set of shape.
/// @param[in/out] hint used to initialize the search when shape is a ConvexBase
/// object.
/// @param[in] num_sampled_supports is only used for shapes with smooth
/// non-strictly convex bases like cones and cylinders (their bases are
/// circles). In such a case, if the support direction points to their base, we
/// have to choose which points we want to add to the set. This is not needed
/// for boxes or ConvexBase for example. Indeed, because their support sets are
/// always polygons, we can characterize the entire support set with the
/// vertices of the polygon.
/// @param[in] tol given a point v on the shape, if
/// `max_{p in shape}(p.dot(dir)) - v.dot(dir) <= tol`, where dir is the set's
/// support direction, then v is added to the support set.
/// Otherwise said, if a point p of the shape is at a distance `tol` from the
/// support plane, it is added to the set. Thus, `tol` can be seen as the
/// "thickness" of the support plane.
/// @tparam _SupportOptions is a value of the SupportOptions enum. If set to
/// `WithSweptSphere`, the support functions take into account the shapes' swept
/// sphere radii.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getSupportSet(const ShapeBase* shape, SupportSet& support_set, int& hint,
                   size_t num_sampled_supports = 6, Scalar tol = Scalar(1e-3));

/// @brief Same as @ref getSupportSet(const ShapeBase*, const Scalar,
/// SupportSet&, const int) but also constructs the support set frame from
/// `dir`.
/// @note The support direction `dir` is expressed in the local frame of the
/// shape.
/// @note This function automatically deals with the `direction` of the
/// SupportSet.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getSupportSet(const ShapeBase* shape, const Vec3s& dir,
                   SupportSet& support_set, int& hint,
                   size_t num_sampled_supports = 6, Scalar tol = Scalar(1e-3)) {
  support_set.tf.rotation() = constructOrthonormalBasisFromVector(dir);
  const Vec3s& support_dir = support_set.getNormal();
  const Vec3s support = getSupport<_SupportOptions>(shape, support_dir, hint);
  getSupportSet<_SupportOptions>(shape, support_set, hint, num_sampled_supports,
                                 tol);
}

/// @brief Triangle support set function.
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSet(const TriangleP* triangle, SupportSet& support_set,
                        int& /*unused*/, ShapeSupportData& /*unused*/,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief Box support set function.
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSet(const Box* box, SupportSet& support_set,
                        int& /*unused*/, ShapeSupportData& support_data,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief Sphere support set function.
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSet(const Sphere* sphere, SupportSet& support_set,
                        int& /*unused*/, ShapeSupportData& /*unused*/,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar /*unused*/ tol = Scalar(1e-3));

/// @brief Ellipsoid support set function.
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSet(const Ellipsoid* ellipsoid, SupportSet& support_set,
                        int& /*unused*/, ShapeSupportData& /*unused*/,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar /*unused*/ tol = Scalar(1e-3));

/// @brief Capsule support set function.
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSet(const Capsule* capsule, SupportSet& support_set,
                        int& /*unused*/, ShapeSupportData& /*unused*/,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief Cone support set function.
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSet(const Cone* cone, SupportSet& support_set,
                        int& /*unused*/, ShapeSupportData& /*unused*/,
                        size_t num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief Cylinder support set function.
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSet(const Cylinder* cylinder, SupportSet& support_set,
                        int& /*unused*/, ShapeSupportData& /*unused*/,
                        size_t num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief ConvexBase support set function.
/// Assumes the support set frame has already been computed.
/// @note See @ref LargeConvex and SmallConvex to see how to optimize
/// ConvexBase's support computation.
template <int _SupportOptions = SupportOptions::NoSweptSphere,
          typename IndexType>
void getShapeSupportSet(const ConvexBaseTpl<IndexType>* convex,
                        SupportSet& support_set, int& hint,
                        ShapeSupportData& support_data,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief Support set function for large ConvexBase (>32 vertices).
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere,
          typename IndexType>
void getShapeSupportSet(const SmallConvex<IndexType>* convex,
                        SupportSet& support_set, int& /*unused*/,
                        ShapeSupportData& /*unused*/,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief Support set function for small ConvexBase (<32 vertices).
/// Assumes the support set frame has already been computed.
template <int _SupportOptions = SupportOptions::NoSweptSphere,
          typename IndexType>
void getShapeSupportSet(const LargeConvex<IndexType>* convex,
                        SupportSet& support_set, int& hint,
                        ShapeSupportData& support_data,
                        size_t /*unused*/ num_sampled_supports = 6,
                        Scalar tol = Scalar(1e-3));

/// @brief Computes the convex-hull of support_set. For now, this function is
/// only needed for Box and ConvexBase.
/// @param[in] cloud data which contains the 2d points of the support set which
/// convex-hull we want to compute.
/// @param[out] 2d points of the the support set's convex-hull.
COAL_DLLAPI void computeSupportSetConvexHull(SupportSet::Polygon& cloud,
                                             SupportSet::Polygon& cvx_hull);

}  // namespace details
}  // namespace coal

#endif  // COAL_SUPPORT_FUNCTIONS_H
