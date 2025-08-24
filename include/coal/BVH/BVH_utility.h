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

#ifndef COAL_BVH_UTILITY_H
#define COAL_BVH_UTILITY_H

#include "coal/BVH/BVH_model.h"

namespace coal {
/// @brief Extract the part of the BVHModel that is inside an AABB.
/// A triangle in collision with the AABB is considered inside.
template <typename BV>
COAL_DLLAPI BVHModel<BV>* BVHExtract(const BVHModel<BV>& model,
                                     const Transform3s& pose, const AABB& aabb);

template <>
COAL_DLLAPI BVHModel<OBB>* BVHExtract(const BVHModel<OBB>& model,
                                      const Transform3s& pose,
                                      const AABB& aabb);
template <>
COAL_DLLAPI BVHModel<AABB>* BVHExtract(const BVHModel<AABB>& model,
                                       const Transform3s& pose,
                                       const AABB& aabb);
template <>
COAL_DLLAPI BVHModel<RSS>* BVHExtract(const BVHModel<RSS>& model,
                                      const Transform3s& pose,
                                      const AABB& aabb);
template <>
COAL_DLLAPI BVHModel<kIOS>* BVHExtract(const BVHModel<kIOS>& model,
                                       const Transform3s& pose,
                                       const AABB& aabb);
template <>
COAL_DLLAPI BVHModel<OBBRSS>* BVHExtract(const BVHModel<OBBRSS>& model,
                                         const Transform3s& pose,
                                         const AABB& aabb);
template <>
COAL_DLLAPI BVHModel<KDOP<16> >* BVHExtract(const BVHModel<KDOP<16> >& model,
                                            const Transform3s& pose,
                                            const AABB& aabb);
template <>
COAL_DLLAPI BVHModel<KDOP<18> >* BVHExtract(const BVHModel<KDOP<18> >& model,
                                            const Transform3s& pose,
                                            const AABB& aabb);
template <>
COAL_DLLAPI BVHModel<KDOP<24> >* BVHExtract(const BVHModel<KDOP<24> >& model,
                                            const Transform3s& pose,
                                            const AABB& aabb);

/// @brief Compute the covariance matrix for a set or subset of points. if ts =
/// null, then indices refer to points directly; otherwise refer to triangles
COAL_DLLAPI void getCovariance(Vec3s* ps, Vec3s* ps2, Triangle32* ts,
                               unsigned int* indices, unsigned int n,
                               Matrix3s& M);

/// @brief Compute the RSS bounding volume parameters: radius, rectangle size
/// and the origin, given the BV axises.
COAL_DLLAPI void getRadiusAndOriginAndRectangleSize(
    Vec3s* ps, Vec3s* ps2, Triangle32* ts, unsigned int* indices,
    unsigned int n, const Matrix3s& axes, Vec3s& origin, Scalar l[2],
    Scalar& r);

/// @brief Compute the bounding volume extent and center for a set or subset of
/// points, given the BV axises.
COAL_DLLAPI void getExtentAndCenter(Vec3s* ps, Vec3s* ps2, Triangle32* ts,
                                    unsigned int* indices, unsigned int n,
                                    Matrix3s& axes, Vec3s& center,
                                    Vec3s& extent);

/// @brief Compute the center and radius for a triangle's circumcircle
COAL_DLLAPI void circumCircleComputation(const Vec3s& a, const Vec3s& b,
                                         const Vec3s& c, Vec3s& center,
                                         Scalar& radius);

/// @brief Compute the maximum distance from a given center point to a point
/// cloud
COAL_DLLAPI Scalar maximumDistance(Vec3s* ps, Vec3s* ps2, Triangle32* ts,
                                   unsigned int* indices, unsigned int n,
                                   const Vec3s& query);

}  // namespace coal

#endif
