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

#ifndef COAL_AABB_H
#define COAL_AABB_H

#include "coal/data_types.h"

namespace coal {

struct CollisionRequest;
class Plane;
class Halfspace;

/// @defgroup Bounding_Volume Bounding volumes
/// Classes of different types of bounding volume.
/// @{

/// @brief A class describing the AABB collision structure, which is a box in 3D
/// space determined by two diagonal points
class COAL_DLLAPI AABB {
 public:
  /// @brief The min point in the AABB
  Vec3s min_;
  /// @brief The max point in the AABB
  Vec3s max_;

  /// @brief Creating an AABB with zero size (low bound +inf, upper bound -inf)
  AABB();

  /// @brief Creating an AABB at position v with zero size
  AABB(const Vec3s& v) : min_(v), max_(v) {}

  /// @brief Creating an AABB with two endpoints a and b
  AABB(const Vec3s& a, const Vec3s& b)
      : min_(a.cwiseMin(b)), max_(a.cwiseMax(b)) {}

  /// @brief Creating an AABB centered as core and is of half-dimension delta
  AABB(const AABB& core, const Vec3s& delta)
      : min_(core.min_ - delta), max_(core.max_ + delta) {}

  /// @brief Creating an AABB contains three points
  AABB(const Vec3s& a, const Vec3s& b, const Vec3s& c)
      : min_(a.cwiseMin(b).cwiseMin(c)), max_(a.cwiseMax(b).cwiseMax(c)) {}

  AABB(const AABB& other) = default;

  AABB& operator=(const AABB& other) = default;

  AABB& update(const Vec3s& a, const Vec3s& b) {
    min_ = a.cwiseMin(b);
    max_ = a.cwiseMax(b);
    return *this;
  }

  /// @brief Comparison operator
  bool operator==(const AABB& other) const {
    return min_ == other.min_ && max_ == other.max_;
  }

  bool operator!=(const AABB& other) const { return !(*this == other); }

  /// @name Bounding volume API
  /// Common API to BVs.
  /// @{

  /// @brief Check whether the AABB contains a point
  inline bool contain(const Vec3s& p) const {
    if (p[0] < min_[0] || p[0] > max_[0]) return false;
    if (p[1] < min_[1] || p[1] > max_[1]) return false;
    if (p[2] < min_[2] || p[2] > max_[2]) return false;

    return true;
  }

  /// @brief Check whether two AABB are overlap
  inline bool overlap(const AABB& other) const {
    if (min_[0] > other.max_[0]) return false;
    if (min_[1] > other.max_[1]) return false;
    if (min_[2] > other.max_[2]) return false;

    if (max_[0] < other.min_[0]) return false;
    if (max_[1] < other.min_[1]) return false;
    if (max_[2] < other.min_[2]) return false;

    return true;
  }

  /// @brief Check whether AABB overlaps a plane
  bool overlap(const Plane& p) const;

  /// @brief Check whether AABB overlaps a halfspace
  bool overlap(const Halfspace& hs) const;

  /// @brief Check whether two AABB are overlap
  bool overlap(const AABB& other, const CollisionRequest& request,
               Scalar& sqrDistLowerBound) const;

  /// @brief Distance between two AABBs
  Scalar distance(const AABB& other) const;

  /// @brief Distance between two AABBs; P and Q, should not be NULL, return the
  /// nearest points
  Scalar distance(const AABB& other, Vec3s* P, Vec3s* Q) const;

  /// @brief Merge the AABB and a point
  inline AABB& operator+=(const Vec3s& p) {
    min_ = min_.cwiseMin(p);
    max_ = max_.cwiseMax(p);
    return *this;
  }

  /// @brief Merge the AABB and another AABB
  inline AABB& operator+=(const AABB& other) {
    min_ = min_.cwiseMin(other.min_);
    max_ = max_.cwiseMax(other.max_);
    return *this;
  }

  /// @brief Return the merged AABB of current AABB and the other one
  inline AABB operator+(const AABB& other) const {
    AABB res(*this);
    return res += other;
  }

  /// @brief Size of the AABB (used in BV_Splitter to order two AABBs)
  inline Scalar size() const { return (max_ - min_).squaredNorm(); }

  /// @brief Center of the AABB
  inline Vec3s center() const { return (min_ + max_) * 0.5; }

  /// @brief Width of the AABB
  inline Scalar width() const { return max_[0] - min_[0]; }

  /// @brief Height of the AABB
  inline Scalar height() const { return max_[1] - min_[1]; }

  /// @brief Depth of the AABB
  inline Scalar depth() const { return max_[2] - min_[2]; }

  /// @brief Volume of the AABB
  inline Scalar volume() const { return width() * height() * depth(); }

  /// @}

  /// @brief Check whether the AABB contains another AABB
  inline bool contain(const AABB& other) const {
    return (other.min_[0] >= min_[0]) && (other.max_[0] <= max_[0]) &&
           (other.min_[1] >= min_[1]) && (other.max_[1] <= max_[1]) &&
           (other.min_[2] >= min_[2]) && (other.max_[2] <= max_[2]);
  }

  /// @brief Check whether two AABB are overlap and return the overlap part
  inline bool overlap(const AABB& other, AABB& overlap_part) const {
    if (!overlap(other)) {
      return false;
    }

    overlap_part.min_ = min_.cwiseMax(other.min_);
    overlap_part.max_ = max_.cwiseMin(other.max_);
    return true;
  }

  /// @brief Check whether two AABB are overlapped along specific axis
  inline bool axisOverlap(const AABB& other, int axis_id) const {
    if (min_[axis_id] > other.max_[axis_id]) return false;
    if (max_[axis_id] < other.min_[axis_id]) return false;

    return true;
  }

  /// @brief expand the half size of the AABB by delta, and keep the center
  /// unchanged.
  inline AABB& expand(const Vec3s& delta) {
    min_ -= delta;
    max_ += delta;
    return *this;
  }

  /// @brief expand the half size of the AABB by a scalar delta, and keep the
  /// center unchanged.
  inline AABB& expand(const Scalar delta) {
    min_.array() -= delta;
    max_.array() += delta;
    return *this;
  }

  /// @brief expand the aabb by increase the thickness of the plate by a ratio
  inline AABB& expand(const AABB& core, Scalar ratio) {
    min_ = min_ * ratio - core.min_;
    max_ = max_ * ratio - core.max_;
    return *this;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @} */  // end of Bounding_Volume

/// @brief translate the center of AABB by t
static inline AABB translate(const AABB& aabb, const Vec3s& t) {
  AABB res(aabb);
  res.min_ += t;
  res.max_ += t;
  return res;
}

static inline AABB rotate(const AABB& aabb, const Matrix3s& R) {
  AABB res(R * aabb.min_);
  Vec3s corner(aabb.min_);
  const Eigen::DenseIndex bit[3] = {1, 2, 4};
  for (Eigen::DenseIndex ic = 1; ic < 8;
       ++ic) {  // ic = 0 corresponds to aabb.min_. Skip it.
    for (Eigen::DenseIndex i = 0; i < 3; ++i) {
      corner[i] = (ic & bit[i]) ? aabb.max_[i] : aabb.min_[i];
    }
    res += R * corner;
  }
  return res;
}

/// @brief Check collision between two aabbs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
COAL_DLLAPI bool overlap(const Matrix3s& R0, const Vec3s& T0, const AABB& b1,
                         const AABB& b2);

/// @brief Check collision between two aabbs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
COAL_DLLAPI bool overlap(const Matrix3s& R0, const Vec3s& T0, const AABB& b1,
                         const AABB& b2, const CollisionRequest& request,
                         Scalar& sqrDistLowerBound);
}  // namespace coal

#endif
