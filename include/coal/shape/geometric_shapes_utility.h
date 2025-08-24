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

#ifndef COAL_GEOMETRIC_SHAPES_UTILITY_H
#define COAL_GEOMETRIC_SHAPES_UTILITY_H

#include <vector>
#include "coal/shape/geometric_shapes.h"
#include "coal/BV/BV.h"
#include "coal/internal/BV_fitter.h"

namespace coal {

/// @cond IGNORE
namespace details {
/// @brief get the vertices of some convex shape which can bound the given shape
/// in a specific configuration
COAL_DLLAPI std::vector<Vec3s> getBoundVertices(const Box& box,
                                                const Transform3s& tf);
COAL_DLLAPI std::vector<Vec3s> getBoundVertices(const Sphere& sphere,
                                                const Transform3s& tf);
COAL_DLLAPI std::vector<Vec3s> getBoundVertices(const Ellipsoid& ellipsoid,
                                                const Transform3s& tf);
COAL_DLLAPI std::vector<Vec3s> getBoundVertices(const Capsule& capsule,
                                                const Transform3s& tf);
COAL_DLLAPI std::vector<Vec3s> getBoundVertices(const Cone& cone,
                                                const Transform3s& tf);
COAL_DLLAPI std::vector<Vec3s> getBoundVertices(const Cylinder& cylinder,
                                                const Transform3s& tf);
COAL_DLLAPI std::vector<Vec3s> getBoundVertices(const TriangleP& triangle,
                                                const Transform3s& tf);
template <typename IndexType>
std::vector<Vec3s> getBoundVertices(const ConvexBaseTpl<IndexType>& convex,
                                    const Transform3s& tf) {
  std::vector<Vec3s> result(convex.num_points);
  const std::vector<Vec3s>& points_ = *(convex.points);
  for (std::size_t i = 0; i < convex.num_points; ++i) {
    result[i] = tf.transform(points_[i]);
  }

  return result;
}
}  // namespace details
/// @endcond

/// @brief calculate a bounding volume for a shape in a specific configuration
template <typename BV, typename S>
inline void computeBV(const S& s, const Transform3s& tf, BV& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  std::vector<Vec3s> convex_bound_vertices = details::getBoundVertices(s, tf);
  fit(&convex_bound_vertices[0], (unsigned int)convex_bound_vertices.size(),
      bv);
}

template <>
COAL_DLLAPI void computeBV<AABB, Box>(const Box& s, const Transform3s& tf,
                                      AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, Sphere>(const Sphere& s, const Transform3s& tf,
                                         AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, Ellipsoid>(const Ellipsoid& e,
                                            const Transform3s& tf, AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, Capsule>(const Capsule& s,
                                          const Transform3s& tf, AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, Cone>(const Cone& s, const Transform3s& tf,
                                       AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, Cylinder>(const Cylinder& s,
                                           const Transform3s& tf, AABB& bv);
template <>
COAL_DLLAPI void computeBV<AABB, ConvexBase32>(const ConvexBase32& s,
                                               const Transform3s& tf, AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, ConvexBase16>(const ConvexBase16& s,
                                               const Transform3s& tf, AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, TriangleP>(const TriangleP& s,
                                            const Transform3s& tf, AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, Halfspace>(const Halfspace& s,
                                            const Transform3s& tf, AABB& bv);

template <>
COAL_DLLAPI void computeBV<AABB, Plane>(const Plane& s, const Transform3s& tf,
                                        AABB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, Box>(const Box& s, const Transform3s& tf,
                                     OBB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, Sphere>(const Sphere& s, const Transform3s& tf,
                                        OBB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, Capsule>(const Capsule& s,
                                         const Transform3s& tf, OBB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, Cone>(const Cone& s, const Transform3s& tf,
                                      OBB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, Cylinder>(const Cylinder& s,
                                          const Transform3s& tf, OBB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, ConvexBase32>(const ConvexBase32& s,
                                              const Transform3s& tf, OBB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, ConvexBase16>(const ConvexBase16& s,
                                              const Transform3s& tf, OBB& bv);

template <>
COAL_DLLAPI void computeBV<OBB, Halfspace>(const Halfspace& s,
                                           const Transform3s& tf, OBB& bv);

template <>
COAL_DLLAPI void computeBV<RSS, Halfspace>(const Halfspace& s,
                                           const Transform3s& tf, RSS& bv);

template <>
COAL_DLLAPI void computeBV<OBBRSS, Halfspace>(const Halfspace& s,
                                              const Transform3s& tf,
                                              OBBRSS& bv);

template <>
COAL_DLLAPI void computeBV<kIOS, Halfspace>(const Halfspace& s,
                                            const Transform3s& tf, kIOS& bv);

template <>
COAL_DLLAPI void computeBV<KDOP<16>, Halfspace>(const Halfspace& s,
                                                const Transform3s& tf,
                                                KDOP<16>& bv);

template <>
COAL_DLLAPI void computeBV<KDOP<18>, Halfspace>(const Halfspace& s,
                                                const Transform3s& tf,
                                                KDOP<18>& bv);

template <>
COAL_DLLAPI void computeBV<KDOP<24>, Halfspace>(const Halfspace& s,
                                                const Transform3s& tf,
                                                KDOP<24>& bv);

template <>
COAL_DLLAPI void computeBV<OBB, Plane>(const Plane& s, const Transform3s& tf,
                                       OBB& bv);

template <>
COAL_DLLAPI void computeBV<RSS, Plane>(const Plane& s, const Transform3s& tf,
                                       RSS& bv);

template <>
COAL_DLLAPI void computeBV<OBBRSS, Plane>(const Plane& s, const Transform3s& tf,
                                          OBBRSS& bv);

template <>
COAL_DLLAPI void computeBV<kIOS, Plane>(const Plane& s, const Transform3s& tf,
                                        kIOS& bv);

template <>
COAL_DLLAPI void computeBV<KDOP<16>, Plane>(const Plane& s,
                                            const Transform3s& tf,
                                            KDOP<16>& bv);

template <>
COAL_DLLAPI void computeBV<KDOP<18>, Plane>(const Plane& s,
                                            const Transform3s& tf,
                                            KDOP<18>& bv);

template <>
COAL_DLLAPI void computeBV<KDOP<24>, Plane>(const Plane& s,
                                            const Transform3s& tf,
                                            KDOP<24>& bv);

/// @brief construct a box shape (with a configuration) from a given bounding
/// volume
COAL_DLLAPI void constructBox(const AABB& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const OBB& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const OBBRSS& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const kIOS& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const RSS& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const KDOP<16>& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const KDOP<18>& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const KDOP<24>& bv, Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const AABB& bv, const Transform3s& tf_bv,
                              Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const OBB& bv, const Transform3s& tf_bv, Box& box,
                              Transform3s& tf);

COAL_DLLAPI void constructBox(const OBBRSS& bv, const Transform3s& tf_bv,
                              Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const kIOS& bv, const Transform3s& tf_bv,
                              Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const RSS& bv, const Transform3s& tf_bv, Box& box,
                              Transform3s& tf);

COAL_DLLAPI void constructBox(const KDOP<16>& bv, const Transform3s& tf_bv,
                              Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const KDOP<18>& bv, const Transform3s& tf_bv,
                              Box& box, Transform3s& tf);

COAL_DLLAPI void constructBox(const KDOP<24>& bv, const Transform3s& tf_bv,
                              Box& box, Transform3s& tf);

COAL_DLLAPI Halfspace transform(const Halfspace& a, const Transform3s& tf);

COAL_DLLAPI Plane transform(const Plane& a, const Transform3s& tf);

COAL_DLLAPI std::array<Halfspace, 2> transformToHalfspaces(
    const Plane& a, const Transform3s& tf);

}  // namespace coal

#endif
