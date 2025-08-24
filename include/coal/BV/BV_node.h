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

#ifndef COAL_BV_NODE_H
#define COAL_BV_NODE_H

#include "coal/data_types.h"
#include "coal/BV/BV.h"

namespace coal {

/// @defgroup Construction_Of_BVH Construction of BVHModel
/// Classes which are used to build a BVHModel (Bounding Volume Hierarchy)
/// @{

/// @brief BVNodeBase encodes the tree structure for BVH
struct COAL_DLLAPI BVNodeBase {
  /// @brief An index for first child node or primitive
  /// If the value is positive, it is the index of the first child bv node
  /// If the value is negative, it is -(primitive index + 1)
  /// Zero is not used.
  int first_child;

  /// @brief The start id the primitive belonging to the current node. The index
  /// is referred to the primitive_indices in BVHModel and from that we can
  /// obtain the primitive's index in original data indirectly.
  unsigned int first_primitive;

  /// @brief The number of primitives belonging to the current node
  unsigned int num_primitives;

  /// @brief Default constructor
  BVNodeBase()
      : first_child(0),
        first_primitive(
            (std::numeric_limits<unsigned int>::max)())  // value we should help
                                                         // to raise an issue
        ,
        num_primitives(0) {}

  /// @brief Equality operator
  bool operator==(const BVNodeBase& other) const {
    return first_child == other.first_child &&
           first_primitive == other.first_primitive &&
           num_primitives == other.num_primitives;
  }

  /// @brief Difference operator
  bool operator!=(const BVNodeBase& other) const { return !(*this == other); }

  /// @brief Whether current node is a leaf node (i.e. contains a primitive
  /// index
  inline bool isLeaf() const { return first_child < 0; }

  /// @brief Return the primitive index. The index is referred to the original
  /// data (i.e. vertices or tri_indices) in BVHModel
  inline int primitiveId() const { return -(first_child + 1); }

  /// @brief Return the index of the first child. The index is referred to the
  /// bounding volume array (i.e. bvs) in BVHModel
  inline int leftChild() const { return first_child; }

  /// @brief Return the index of the second child. The index is referred to the
  /// bounding volume array (i.e. bvs) in BVHModel
  inline int rightChild() const { return first_child + 1; }
};

/// @brief A class describing a bounding volume node. It includes the tree
/// structure providing in BVNodeBase and also the geometry data provided in BV
/// template parameter.
template <typename BV>
struct COAL_DLLAPI BVNode : public BVNodeBase {
  typedef BVNodeBase Base;

  /// @brief bounding volume storing the geometry
  BV bv;

  /// @brief Equality operator
  bool operator==(const BVNode& other) const {
    return Base::operator==(other) && bv == other.bv;
  }

  /// @brief Difference operator
  bool operator!=(const BVNode& other) const { return !(*this == other); }

  /// @brief Check whether two BVNode collide
  bool overlap(const BVNode& other) const { return bv.overlap(other.bv); }
  /// @brief Check whether two BVNode collide
  bool overlap(const BVNode& other, const CollisionRequest& request,
               Scalar& sqrDistLowerBound) const {
    return bv.overlap(other.bv, request, sqrDistLowerBound);
  }

  /// @brief Compute the distance between two BVNode. P1 and P2, if not NULL and
  /// the underlying BV supports distance, return the nearest points.
  Scalar distance(const BVNode& other, Vec3s* P1 = NULL,
                  Vec3s* P2 = NULL) const {
    return bv.distance(other.bv, P1, P2);
  }

  /// @brief Access to the center of the BV
  Vec3s getCenter() const { return bv.center(); }

  /// @brief Access to the orientation of the BV
  const Matrix3s& getOrientation() const {
    static const Matrix3s id3 = Matrix3s::Identity();
    return id3;
  }

  /// \cond
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \endcond
};

template <>
inline const Matrix3s& BVNode<OBB>::getOrientation() const {
  return bv.axes;
}

template <>
inline const Matrix3s& BVNode<RSS>::getOrientation() const {
  return bv.axes;
}

template <>
inline const Matrix3s& BVNode<OBBRSS>::getOrientation() const {
  return bv.obb.axes;
}

}  // namespace coal

#endif
