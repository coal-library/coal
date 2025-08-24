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

#ifndef COAL_KIOS_H
#define COAL_KIOS_H

#include "coal/BV/OBB.h"

namespace coal {

struct CollisionRequest;

/// @addtogroup Bounding_Volume
/// @{

/// @brief A class describing the kIOS collision structure, which is a set of
/// spheres.
class COAL_DLLAPI kIOS {
  /// @brief One sphere in kIOS
  struct COAL_DLLAPI kIOS_Sphere {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3s o;
    Scalar r;

    bool operator==(const kIOS_Sphere& other) const {
      return o == other.o && r == other.r;
    }

    bool operator!=(const kIOS_Sphere& other) const {
      return !(*this == other);
    }
  };

  /// @brief generate one sphere enclosing two spheres
  static kIOS_Sphere encloseSphere(const kIOS_Sphere& s0,
                                   const kIOS_Sphere& s1) {
    Vec3s d = s1.o - s0.o;
    Scalar dist2 = d.squaredNorm();
    Scalar diff_r = s1.r - s0.r;

    /** The sphere with the larger radius encloses the other */
    if (diff_r * diff_r >= dist2) {
      if (s1.r > s0.r)
        return s1;
      else
        return s0;
    } else /** spheres partially overlapping or disjoint */
    {
      float dist = (float)std::sqrt(dist2);
      kIOS_Sphere s;
      s.r = dist + s0.r + s1.r;
      if (dist > 0)
        s.o = s0.o + d * ((s.r - s0.r) / dist);
      else
        s.o = s0.o;
      return s;
    }
  }

 public:
  /// @brief Equality operator
  bool operator==(const kIOS& other) const {
    bool res = obb == other.obb && num_spheres == other.num_spheres;
    if (!res) return false;

    for (size_t k = 0; k < num_spheres; ++k) {
      if (spheres[k] != other.spheres[k]) return false;
    }

    return true;
  }

  /// @brief Difference operator
  bool operator!=(const kIOS& other) const { return !(*this == other); }

  static constexpr size_t max_num_spheres = 5;

  /// @brief The (at most) five spheres for intersection
  kIOS_Sphere spheres[max_num_spheres];

  /// @brief The number of spheres, no larger than 5
  unsigned int num_spheres;

  /// @ OBB related with kIOS
  OBB obb;

  /// @brief Check whether the kIOS contains a point
  bool contain(const Vec3s& p) const;

  /// @brief Check collision between two kIOS
  bool overlap(const kIOS& other) const;

  /// @brief Check collision between two kIOS
  bool overlap(const kIOS& other, const CollisionRequest&,
               Scalar& sqrDistLowerBound) const;

  /// @brief The distance between two kIOS
  Scalar distance(const kIOS& other, Vec3s* P = NULL, Vec3s* Q = NULL) const;

  /// @brief A simple way to merge the kIOS and a point
  kIOS& operator+=(const Vec3s& p);

  /// @brief Merge the kIOS and another kIOS
  kIOS& operator+=(const kIOS& other) {
    *this = *this + other;
    return *this;
  }

  /// @brief Return the merged kIOS of current kIOS and the other one
  kIOS operator+(const kIOS& other) const;

  /// @brief size of the kIOS (used in BV_Splitter to order two kIOSs)
  Scalar size() const;

  /// @brief Center of the kIOS
  const Vec3s& center() const { return spheres[0].o; }

  /// @brief Width of the kIOS
  Scalar width() const;

  /// @brief Height of the kIOS
  Scalar height() const;

  /// @brief Depth of the kIOS
  Scalar depth() const;

  /// @brief Volume of the kIOS
  Scalar volume() const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @} */  // end of Bounding_Volume

/// @brief Translate the kIOS BV
COAL_DLLAPI kIOS translate(const kIOS& bv, const Vec3s& t);

/// @brief Check collision between two kIOSs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
/// @todo Not efficient
COAL_DLLAPI bool overlap(const Matrix3s& R0, const Vec3s& T0, const kIOS& b1,
                         const kIOS& b2);

/// @brief Check collision between two kIOSs, b1 is in configuration (R0, T0)
/// and b2 is in identity.
/// @todo Not efficient
COAL_DLLAPI bool overlap(const Matrix3s& R0, const Vec3s& T0, const kIOS& b1,
                         const kIOS& b2, const CollisionRequest& request,
                         Scalar& sqrDistLowerBound);

/// @brief Approximate distance between two kIOS bounding volumes
/// @todo P and Q is not returned, need implementation
COAL_DLLAPI Scalar distance(const Matrix3s& R0, const Vec3s& T0, const kIOS& b1,
                            const kIOS& b2, Vec3s* P = NULL, Vec3s* Q = NULL);

}  // namespace coal

#endif
