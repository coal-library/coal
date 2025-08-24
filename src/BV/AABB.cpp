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

#include "coal/BV/AABB.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/collision_data.h"

#include <limits>

namespace coal {

AABB::AABB()
    : min_(Vec3s::Constant((std::numeric_limits<Scalar>::max)())),
      max_(Vec3s::Constant(-(std::numeric_limits<Scalar>::max)())) {}

bool AABB::overlap(const AABB& other, const CollisionRequest& request,
                   Scalar& sqrDistLowerBound) const {
  const Scalar break_distance_squared =
      request.break_distance * request.break_distance;

  sqrDistLowerBound =
      (min_ - other.max_ - Vec3s::Constant(request.security_margin))
          .array()
          .max(Scalar(0))
          .matrix()
          .squaredNorm();
  if (sqrDistLowerBound > break_distance_squared) return false;

  sqrDistLowerBound =
      (other.min_ - max_ - Vec3s::Constant(request.security_margin))
          .array()
          .max(Scalar(0))
          .matrix()
          .squaredNorm();
  if (sqrDistLowerBound > break_distance_squared) return false;

  return true;
}

Scalar AABB::distance(const AABB& other, Vec3s* P, Vec3s* Q) const {
  Scalar result = 0;
  for (Eigen::DenseIndex i = 0; i < 3; ++i) {
    const Scalar& amin = min_[i];
    const Scalar& amax = max_[i];
    const Scalar& bmin = other.min_[i];
    const Scalar& bmax = other.max_[i];

    if (amin > bmax) {
      Scalar delta = bmax - amin;
      result += delta * delta;
      if (P && Q) {
        (*P)[i] = amin;
        (*Q)[i] = bmax;
      }
    } else if (bmin > amax) {
      Scalar delta = amax - bmin;
      result += delta * delta;
      if (P && Q) {
        (*P)[i] = amax;
        (*Q)[i] = bmin;
      }
    } else {
      if (P && Q) {
        if (bmin >= amin) {
          Scalar t = Scalar(0.5) * (amax + bmin);
          (*P)[i] = t;
          (*Q)[i] = t;
        } else {
          Scalar t = Scalar(0.5) * (amin + bmax);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
      }
    }
  }

  return std::sqrt(result);
}

Scalar AABB::distance(const AABB& other) const {
  Scalar result = 0;
  for (Eigen::DenseIndex i = 0; i < 3; ++i) {
    const Scalar& amin = min_[i];
    const Scalar& amax = max_[i];
    const Scalar& bmin = other.min_[i];
    const Scalar& bmax = other.max_[i];

    if (amin > bmax) {
      Scalar delta = bmax - amin;
      result += delta * delta;
    } else if (bmin > amax) {
      Scalar delta = amax - bmin;
      result += delta * delta;
    }
  }

  return std::sqrt(result);
}

bool overlap(const Matrix3s& R0, const Vec3s& T0, const AABB& b1,
             const AABB& b2) {
  AABB bb1(translate(rotate(b1, R0), T0));
  return bb1.overlap(b2);
}

bool overlap(const Matrix3s& R0, const Vec3s& T0, const AABB& b1,
             const AABB& b2, const CollisionRequest& request,
             Scalar& sqrDistLowerBound) {
  AABB bb1(translate(rotate(b1, R0), T0));
  return bb1.overlap(b2, request, sqrDistLowerBound);
}

bool AABB::overlap(const Plane& p) const {
  // Convert AABB to a (box, transform) representation and compute the support
  // points in the directions normal and -normal.
  // If both points lie on different sides of the plane, there is an overlap
  // between the AABB and the plane. Otherwise, there is no overlap.
  const Vec3s halfside = (this->max_ - this->min_) / 2;
  const Vec3s center = (this->max_ + this->min_) / 2;

  const Vec3s support1 = (p.n.array() > 0).select(halfside, -halfside) + center;
  const Vec3s support2 =
      ((-p.n).array() > 0).select(halfside, -halfside) + center;

  const Scalar dist1 = p.n.dot(support1) - p.d;
  const Scalar dist2 = p.n.dot(support2) - p.d;
  const int sign1 = (dist1 > 0) ? 1 : -1;
  const int sign2 = (dist2 > 0) ? 1 : -1;

  if (p.getSweptSphereRadius() > 0) {
    if (sign1 != sign2) {
      // Supports are on different sides of the plane. There is an overlap.
      return true;
    }
    // Both supports are on the same side of the plane.
    // We now need to check if they are on the same side of the plane inflated
    // by the swept-sphere radius.
    const Scalar ssr_dist1 = std::abs(dist1) - p.getSweptSphereRadius();
    const Scalar ssr_dist2 = std::abs(dist2) - p.getSweptSphereRadius();
    const int ssr_sign1 = (ssr_dist1 > 0) ? 1 : -1;
    const int ssr_sign2 = (ssr_dist2 > 0) ? 1 : -1;
    return ssr_sign1 != ssr_sign2;
  }

  return (sign1 != sign2);
}

bool AABB::overlap(const Halfspace& hs) const {
  // Convert AABB to a (box, transform) representation and compute the support
  // points in the direction -normal.
  // If the support is below the plane defined by the halfspace, there is an
  // overlap between the AABB and the halfspace. Otherwise, there is no
  // overlap.
  Vec3s halfside = (this->max_ - this->min_) / 2;
  Vec3s center = (this->max_ + this->min_) / 2;
  Vec3s support = ((-hs.n).array() > 0).select(halfside, -halfside) + center;
  return (hs.signedDistance(support) < 0);
}

}  // namespace coal
