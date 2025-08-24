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

#ifndef COAL_OBBRSS_H
#define COAL_OBBRSS_H

#include "coal/BV/OBB.h"
#include "coal/BV/RSS.h"

namespace coal {

struct CollisionRequest;

/// @addtogroup Bounding_Volume
/// @{

/// @brief Class merging the OBB and RSS, can handle collision and distance
/// simultaneously
struct COAL_DLLAPI OBBRSS {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief OBB member, for rotation
  OBB obb;

  /// @brief RSS member, for distance
  RSS rss;

  /// @brief Equality operator
  bool operator==(const OBBRSS& other) const {
    return obb == other.obb && rss == other.rss;
  }

  /// @brief Difference operator
  bool operator!=(const OBBRSS& other) const { return !(*this == other); }

  /// @brief Check whether the OBBRSS contains a point
  inline bool contain(const Vec3s& p) const { return obb.contain(p); }

  /// @brief Check collision between two OBBRSS
  bool overlap(const OBBRSS& other) const { return obb.overlap(other.obb); }

  /// Check collision between two OBBRSS
  /// @retval sqrDistLowerBound squared lower bound on distance between
  ///         objects if they do not overlap.
  bool overlap(const OBBRSS& other, const CollisionRequest& request,
               Scalar& sqrDistLowerBound) const {
    return obb.overlap(other.obb, request, sqrDistLowerBound);
  }

  /// @brief Distance between two OBBRSS; P and Q , is not NULL, returns the
  /// nearest points
  Scalar distance(const OBBRSS& other, Vec3s* P = NULL, Vec3s* Q = NULL) const {
    return rss.distance(other.rss, P, Q);
  }

  /// @brief Merge the OBBRSS and a point
  OBBRSS& operator+=(const Vec3s& p) {
    obb += p;
    rss += p;
    return *this;
  }

  /// @brief Merge two OBBRSS
  OBBRSS& operator+=(const OBBRSS& other) {
    *this = *this + other;
    return *this;
  }

  /// @brief Merge two OBBRSS
  OBBRSS operator+(const OBBRSS& other) const {
    OBBRSS result;
    result.obb = obb + other.obb;
    result.rss = rss + other.rss;
    return result;
  }

  /// @brief Size of the OBBRSS (used in BV_Splitter to order two OBBRSS)
  inline Scalar size() const { return obb.size(); }

  /// @brief Center of the OBBRSS
  inline const Vec3s& center() const { return obb.center(); }

  /// @brief Width of the OBRSS
  inline Scalar width() const { return obb.width(); }

  /// @brief Height of the OBBRSS
  inline Scalar height() const { return obb.height(); }

  /// @brief Depth of the OBBRSS
  inline Scalar depth() const { return obb.depth(); }

  /// @brief Volume of the OBBRSS
  inline Scalar volume() const { return obb.volume(); }
};

/** @} */  // end of Bounding_Volume

/// @brief Check collision between two OBBRSS, b1 is in configuration (R0, T0)
/// and b2 is in indentity
inline bool overlap(const Matrix3s& R0, const Vec3s& T0, const OBBRSS& b1,
                    const OBBRSS& b2) {
  return overlap(R0, T0, b1.obb, b2.obb);
}

/// Check collision between two OBBRSS
/// @param  R0, T0 configuration of b1
/// @param  b1 first OBBRSS in configuration (R0, T0)
/// @param  b2 second OBBRSS in identity position
/// @retval sqrDistLowerBound squared lower bound on the distance if OBBRSS do
/// not overlap.
inline bool overlap(const Matrix3s& R0, const Vec3s& T0, const OBBRSS& b1,
                    const OBBRSS& b2, const CollisionRequest& request,
                    Scalar& sqrDistLowerBound) {
  return overlap(R0, T0, b1.obb, b2.obb, request, sqrDistLowerBound);
}

/// @brief Computate distance between two OBBRSS, b1 is in configuation (R0, T0)
/// and b2 is in indentity; P and Q, is not NULL, returns the nearest points
inline Scalar distance(const Matrix3s& R0, const Vec3s& T0, const OBBRSS& b1,
                       const OBBRSS& b2, Vec3s* P = NULL, Vec3s* Q = NULL) {
  return distance(R0, T0, b1.rss, b2.rss, P, Q);
}

}  // namespace coal

#endif
