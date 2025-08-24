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

#ifndef COAL_OBB_H
#define COAL_OBB_H

#include "coal/data_types.h"

namespace coal {

struct CollisionRequest;

/// @addtogroup Bounding_Volume
/// @{

/// @brief Oriented bounding box class
struct COAL_DLLAPI OBB {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief Orientation of OBB. axis[i] is the ith column of the orientation
  /// matrix for the box; it is also the i-th principle direction of the box. We
  /// assume that axis[0] corresponds to the axis with the longest box edge,
  /// axis[1] corresponds to the shorter one and axis[2] corresponds to the
  /// shortest one.
  Matrix3s axes;

  /// @brief Center of OBB
  Vec3s To;

  /// @brief Half dimensions of OBB
  Vec3s extent;

  OBB() : axes(Matrix3s::Zero()), To(Vec3s::Zero()), extent(Vec3s::Zero()) {}

  /// @brief Equality operator
  bool operator==(const OBB& other) const {
    return axes == other.axes && To == other.To && extent == other.extent;
  }

  /// @brief Difference operator
  bool operator!=(const OBB& other) const { return !(*this == other); }

  /// @brief Check whether the OBB contains a point.
  bool contain(const Vec3s& p) const;

  /// Check collision between two OBB
  /// @return true if collision happens.
  bool overlap(const OBB& other) const;

  /// Check collision between two OBB
  /// @return true if collision happens.
  /// @retval sqrDistLowerBound squared lower bound on distance between boxes if
  ///         they do not overlap.
  bool overlap(const OBB& other, const CollisionRequest& request,
               Scalar& sqrDistLowerBound) const;

  /// @brief Distance between two OBBs, not implemented.
  Scalar distance(const OBB& other, Vec3s* P = NULL, Vec3s* Q = NULL) const;

  /// @brief A simple way to merge the OBB and a point (the result is not
  /// compact).
  OBB& operator+=(const Vec3s& p);

  /// @brief Merge the OBB and another OBB (the result is not compact).
  OBB& operator+=(const OBB& other) {
    *this = *this + other;
    return *this;
  }

  /// @brief Return the merged OBB of current OBB and the other one (the result
  /// is not compact).
  OBB operator+(const OBB& other) const;

  /// @brief Size of the OBB (used in BV_Splitter to order two OBBs)
  inline Scalar size() const { return extent.squaredNorm(); }

  /// @brief Center of the OBB
  inline const Vec3s& center() const { return To; }

  /// @brief Width of the OBB.
  inline Scalar width() const { return 2 * extent[0]; }

  /// @brief Height of the OBB.
  inline Scalar height() const { return 2 * extent[1]; }

  /// @brief Depth of the OBB
  inline Scalar depth() const { return 2 * extent[2]; }

  /// @brief Volume of the OBB
  inline Scalar volume() const { return width() * height() * depth(); }
};

/** @} */  // end of Bounding_Volume

/// @brief Translate the OBB bv
COAL_DLLAPI OBB translate(const OBB& bv, const Vec3s& t);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
COAL_DLLAPI bool overlap(const Matrix3s& R0, const Vec3s& T0, const OBB& b1,
                         const OBB& b2);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
COAL_DLLAPI bool overlap(const Matrix3s& R0, const Vec3s& T0, const OBB& b1,
                         const OBB& b2, const CollisionRequest& request,
                         Scalar& sqrDistLowerBound);

/// Check collision between two boxes
/// @param B, T orientation and position of first box,
/// @param a half dimensions of first box,
/// @param b half dimensions of second box.
/// The second box is in identity configuration.
COAL_DLLAPI bool obbDisjoint(const Matrix3s& B, const Vec3s& T, const Vec3s& a,
                             const Vec3s& b);
}  // namespace coal

#endif
