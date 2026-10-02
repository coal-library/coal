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

/** \author Louis Montaut */

#include "coal/shape/geometric_shapes.h"

#include "coal/internal/intersect.h"
#include "coal/internal/shape_shape_func.h"
#include "../narrowphase/details.h"

#include "coal/tracy.hh"

namespace coal {

namespace internal {
template <>
Scalar ShapeShapeDistance<TriangleP, TriangleP>(
    const CollisionGeometry* o1, const Transform3s& tf1,
    const CollisionGeometry* o2, const Transform3s& tf2,
    const GJKSolver* /*solver*/, const bool /*compute_signed_distance*/,
    Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::internal::ShapeShapeDistance<TriangleP, TriangleP>");
  const TriangleP& s1 = static_cast<const TriangleP&>(*o1);
  const TriangleP t1(tf1.transform(s1.a), tf1.transform(s1.b),
                     tf1.transform(s1.c));

  const TriangleP& s2 = static_cast<const TriangleP&>(*o2);
  const TriangleP t2(tf2.transform(s2.a), tf2.transform(s2.b),
                     tf2.transform(s2.c));

  const Scalar d2 = TriangleDistance::sqrTriDistance(t1.a, t1.b, t1.c, t2.a,
                                                     t2.b, t2.c, p1, p2);

  if (d2 > 0) {
    const Scalar d = sqrt(d2);
    normal = (p2 - p1) / d;
    return d;
  }

  Scalar signed_distance;
  internal::computeTriangleTriangleContact(t1.a, t1.b, t1.c, t2.a, t2.b, t2.c,
                                           signed_distance, p1, p2, normal);
  return signed_distance;
}
}  // namespace internal

}  // namespace coal
