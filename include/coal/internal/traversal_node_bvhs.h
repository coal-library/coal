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

#ifndef COAL_TRAVERSAL_NODE_MESHES_H
#define COAL_TRAVERSAL_NODE_MESHES_H

/// @cond INTERNAL

#include "coal/collision_data.h"
#include "coal/internal/traversal_node_base.h"
#include "coal/BV/BV_node.h"
#include "coal/BV/BV.h"
#include "coal/BVH/BVH_model.h"
#include "coal/internal/intersect.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/internal/traversal.h"
#include "coal/internal/shape_shape_func.h"

#include <cassert>

namespace coal {

/// @addtogroup Traversal_For_Collision
/// @{

/// @brief Traversal node for collision between BVH models
template <typename BV>
class BVHCollisionTraversalNode : public CollisionTraversalNodeBase {
 public:
  BVHCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(unsigned int b) const {
    assert(model1 != NULL && "model1 is NULL");
    return model1->getBV(b).isLeaf();
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(unsigned int b) const {
    assert(model2 != NULL && "model2 is NULL");
    return model2->getBV(b).isLeaf();
  }

  /// @brief Determine the traversal order, is the first BVTT subtree better
  bool firstOverSecond(unsigned int b1, unsigned int b2) const {
    Scalar sz1 = model1->getBV(b1).bv.size();
    Scalar sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if (l2 || (!l1 && (sz1 > sz2))) return true;
    return false;
  }

  /// @brief Obtain the left child of BV node in the first BVH
  int getFirstLeftChild(unsigned int b) const {
    return model1->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the first BVH
  int getFirstRightChild(unsigned int b) const {
    return model1->getBV(b).rightChild();
  }

  /// @brief Obtain the left child of BV node in the second BVH
  int getSecondLeftChild(unsigned int b) const {
    return model2->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the second BVH
  int getSecondRightChild(unsigned int b) const {
    return model2->getBV(b).rightChild();
  }

  /// @brief The first BVH model
  const BVHModel<BV>* model1;
  /// @brief The second BVH model
  const BVHModel<BV>* model2;

  /// @brief statistical information
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable Scalar query_time_seconds;
};

/// @brief Traversal node for collision between two meshes
template <typename BV, int _Options = RelativeTransformationIsIdentity>
class MeshCollisionTraversalNode : public BVHCollisionTraversalNode<BV> {
 public:
  enum {
    Options = _Options,
    RTIsIdentity = _Options & RelativeTransformationIsIdentity
  };

  MeshCollisionTraversalNode(const CollisionRequest& request)
      : BVHCollisionTraversalNode<BV>(request) {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    tri_indices2 = NULL;
  }

  /// BV test between b1 and b2
  /// @param b1, b2 Bounding volumes to test,
  /// @retval sqrDistLowerBound square of a lower bound of the minimal
  ///         distance between bounding volumes.
  bool BVDisjoints(unsigned int b1, unsigned int b2,
                   Scalar& sqrDistLowerBound) const {
    if (this->enable_statistics) this->num_bv_tests++;
    bool disjoint;
    if (RTIsIdentity)
      disjoint = !this->model1->getBV(b1).overlap(
          this->model2->getBV(b2), this->request, sqrDistLowerBound);
    else {
      disjoint = !overlap(RT._R(), RT._T(), this->model2->getBV(b2).bv,
                          this->model1->getBV(b1).bv, this->request,
                          sqrDistLowerBound);
    }
    if (disjoint)
      internal::updateDistanceLowerBoundFromBV(this->request, *this->result,
                                               sqrDistLowerBound);
    return disjoint;
  }

  /// Intersection testing between leaves (two triangles)
  ///
  /// @param b1, b2 id of primitive in bounding volume hierarchy
  /// @retval sqrDistLowerBound squared lower bound of distance between
  ///         primitives if they are not in collision.
  ///
  /// This method supports a security margin. If the distance between
  /// the primitives is less than the security margin, the objects are
  /// considered as in collision. in this case a contact point is
  /// returned in the CollisionResult.
  ///
  /// @note If the distance between objects is less than the security margin,
  ///       and the object are not colliding, the penetration depth is
  ///       negative.
  void leafCollides(unsigned int b1, unsigned int b2,
                    Scalar& sqrDistLowerBound) const {
    if (this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle32& tri_id1 = tri_indices1[primitive_id1];
    const Triangle32& tri_id2 = tri_indices2[primitive_id2];

    const Vec3s& P1 = vertices1[tri_id1[0]];
    const Vec3s& P2 = vertices1[tri_id1[1]];
    const Vec3s& P3 = vertices1[tri_id1[2]];
    const Vec3s& Q1 = vertices2[tri_id2[0]];
    const Vec3s& Q2 = vertices2[tri_id2[1]];
    const Vec3s& Q3 = vertices2[tri_id2[2]];

    TriangleP tri1(P1, P2, P3);
    TriangleP tri2(Q1, Q2, Q3);

    // TODO(louis): MeshCollisionTraversalNode should have its own GJKSolver.
    GJKSolver solver(this->request);

    const bool compute_penetration =
        this->request.enable_contact || (this->request.security_margin < 0);
    Vec3s p1, p2, normal;
    DistanceResult distanceResult;
    Scalar distance = internal::ShapeShapeDistance<TriangleP, TriangleP>(
        &tri1, this->tf1, &tri2, this->tf2, &solver, compute_penetration, p1,
        p2, normal);

    const Scalar distToCollision = distance - this->request.security_margin;

    internal::updateDistanceLowerBoundFromLeaf(this->request, *(this->result),
                                               distToCollision, p1, p2, normal);

    if (distToCollision <=
        this->request.collision_distance_threshold) {  // collision
      sqrDistLowerBound = 0;
      if (this->result->numContacts() < this->request.num_max_contacts) {
        this->result->addContact(Contact(this->model1, this->model2,
                                         primitive_id1, primitive_id2, p1, p2,
                                         normal, distance));
      }
    } else
      sqrDistLowerBound = distToCollision * distToCollision;
  }

  Vec3s* vertices1;
  Vec3s* vertices2;

  Triangle32* tri_indices1;
  Triangle32* tri_indices2;

  details::RelativeTransformation<!bool(RTIsIdentity)> RT;
};

/// @brief Traversal node for collision between two meshes if their underlying
/// BVH node is oriented node (OBB, RSS, OBBRSS, kIOS)
typedef MeshCollisionTraversalNode<OBB, 0> MeshCollisionTraversalNodeOBB;
typedef MeshCollisionTraversalNode<RSS, 0> MeshCollisionTraversalNodeRSS;
typedef MeshCollisionTraversalNode<kIOS, 0> MeshCollisionTraversalNodekIOS;
typedef MeshCollisionTraversalNode<OBBRSS, 0> MeshCollisionTraversalNodeOBBRSS;

/// @}

namespace details {
template <typename BV>
struct DistanceTraversalBVDistanceLowerBound_impl {
  static Scalar run(const BVNode<BV>& b1, const BVNode<BV>& b2) {
    return b1.distance(b2);
  }
  static Scalar run(const Matrix3s& R, const Vec3s& T, const BVNode<BV>& b1,
                    const BVNode<BV>& b2) {
    return distance(R, T, b1.bv, b2.bv);
  }
};

template <>
struct DistanceTraversalBVDistanceLowerBound_impl<OBB> {
  static Scalar run(const BVNode<OBB>& b1, const BVNode<OBB>& b2) {
    Scalar sqrDistLowerBound;
    CollisionRequest request(DISTANCE_LOWER_BOUND, 0);
    // request.break_distance = ?
    if (b1.overlap(b2, request, sqrDistLowerBound)) {
      // TODO A penetration upper bound should be computed.
      return -1;
    }
    return std::sqrt(sqrDistLowerBound);
  }
  static Scalar run(const Matrix3s& R, const Vec3s& T, const BVNode<OBB>& b1,
                    const BVNode<OBB>& b2) {
    Scalar sqrDistLowerBound;
    CollisionRequest request(DISTANCE_LOWER_BOUND, 0);
    // request.break_distance = ?
    if (overlap(R, T, b1.bv, b2.bv, request, sqrDistLowerBound)) {
      // TODO A penetration upper bound should be computed.
      return -1;
    }
    return std::sqrt(sqrDistLowerBound);
  }
};

template <>
struct DistanceTraversalBVDistanceLowerBound_impl<AABB> {
  static Scalar run(const BVNode<AABB>& b1, const BVNode<AABB>& b2) {
    Scalar sqrDistLowerBound;
    CollisionRequest request(DISTANCE_LOWER_BOUND, 0);
    // request.break_distance = ?
    if (b1.overlap(b2, request, sqrDistLowerBound)) {
      // TODO A penetration upper bound should be computed.
      return -1;
    }
    return std::sqrt(sqrDistLowerBound);
  }
  static Scalar run(const Matrix3s& R, const Vec3s& T, const BVNode<AABB>& b1,
                    const BVNode<AABB>& b2) {
    Scalar sqrDistLowerBound;
    CollisionRequest request(DISTANCE_LOWER_BOUND, 0);
    // request.break_distance = ?
    if (overlap(R, T, b1.bv, b2.bv, request, sqrDistLowerBound)) {
      // TODO A penetration upper bound should be computed.
      return -1;
    }
    return std::sqrt(sqrDistLowerBound);
  }
};
}  // namespace details

/// @addtogroup Traversal_For_Distance
/// @{

/// @brief Traversal node for distance computation between BVH models
template <typename BV>
class BVHDistanceTraversalNode : public DistanceTraversalNodeBase {
 public:
  BVHDistanceTraversalNode() : DistanceTraversalNodeBase() {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(unsigned int b) const {
    return model1->getBV(b).isLeaf();
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(unsigned int b) const {
    return model2->getBV(b).isLeaf();
  }

  /// @brief Determine the traversal order, is the first BVTT subtree better
  bool firstOverSecond(unsigned int b1, unsigned int b2) const {
    Scalar sz1 = model1->getBV(b1).bv.size();
    Scalar sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if (l2 || (!l1 && (sz1 > sz2))) return true;
    return false;
  }

  /// @brief Obtain the left child of BV node in the first BVH
  int getFirstLeftChild(unsigned int b) const {
    return model1->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the first BVH
  int getFirstRightChild(unsigned int b) const {
    return model1->getBV(b).rightChild();
  }

  /// @brief Obtain the left child of BV node in the second BVH
  int getSecondLeftChild(unsigned int b) const {
    return model2->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the second BVH
  int getSecondRightChild(unsigned int b) const {
    return model2->getBV(b).rightChild();
  }

  /// @brief The first BVH model
  const BVHModel<BV>* model1;
  /// @brief The second BVH model
  const BVHModel<BV>* model2;

  /// @brief statistical information
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable Scalar query_time_seconds;
};

/// @brief Traversal node for distance computation between two meshes
template <typename BV, int _Options = RelativeTransformationIsIdentity>
class MeshDistanceTraversalNode : public BVHDistanceTraversalNode<BV> {
 public:
  enum {
    Options = _Options,
    RTIsIdentity = _Options & RelativeTransformationIsIdentity
  };

  using BVHDistanceTraversalNode<BV>::enable_statistics;
  using BVHDistanceTraversalNode<BV>::request;
  using BVHDistanceTraversalNode<BV>::result;
  using BVHDistanceTraversalNode<BV>::tf1;
  using BVHDistanceTraversalNode<BV>::model1;
  using BVHDistanceTraversalNode<BV>::model2;
  using BVHDistanceTraversalNode<BV>::num_bv_tests;
  using BVHDistanceTraversalNode<BV>::num_leaf_tests;

  MeshDistanceTraversalNode() : BVHDistanceTraversalNode<BV>() {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    tri_indices2 = NULL;

    rel_err = this->request.rel_err;
    abs_err = this->request.abs_err;
  }

  void preprocess() {
    if (!RTIsIdentity) preprocessOrientedNode();
  }

  void postprocess() {
    if (!RTIsIdentity) postprocessOrientedNode();
  }

  /// @brief BV culling test in one BVTT node
  Scalar BVDistanceLowerBound(unsigned int b1, unsigned int b2) const {
    if (enable_statistics) num_bv_tests++;
    if (RTIsIdentity)
      return details::DistanceTraversalBVDistanceLowerBound_impl<BV>::run(
          model1->getBV(b1), model2->getBV(b2));
    else
      return details::DistanceTraversalBVDistanceLowerBound_impl<BV>::run(
          RT._R(), RT._T(), model1->getBV(b1), model2->getBV(b2));
  }

  /// @brief Distance testing between leaves (two triangles)
  void leafComputeDistance(unsigned int b1, unsigned int b2) const {
    if (this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle32& tri_id1 = tri_indices1[primitive_id1];
    const Triangle32& tri_id2 = tri_indices2[primitive_id2];

    const Vec3s& t11 = vertices1[tri_id1[0]];
    const Vec3s& t12 = vertices1[tri_id1[1]];
    const Vec3s& t13 = vertices1[tri_id1[2]];

    const Vec3s& t21 = vertices2[tri_id2[0]];
    const Vec3s& t22 = vertices2[tri_id2[1]];
    const Vec3s& t23 = vertices2[tri_id2[2]];

    // nearest point pair
    Vec3s P1, P2, normal;

    Scalar d2;
    if (RTIsIdentity)
      d2 = TriangleDistance::sqrTriDistance(t11, t12, t13, t21, t22, t23, P1,
                                            P2);
    else
      d2 = TriangleDistance::sqrTriDistance(t11, t12, t13, t21, t22, t23,
                                            RT._R(), RT._T(), P1, P2);
    Scalar d = sqrt(d2);

    this->result->update(d, this->model1, this->model2, primitive_id1,
                         primitive_id2, P1, P2, normal);
  }

  /// @brief Whether the traversal process can stop early
  bool canStop(Scalar c) const {
    if ((c >= this->result->min_distance - abs_err) &&
        (c * (1 + rel_err) >= this->result->min_distance))
      return true;
    return false;
  }

  Vec3s* vertices1;
  Vec3s* vertices2;

  Triangle32* tri_indices1;
  Triangle32* tri_indices2;

  /// @brief relative and absolute error, default value is 0.01 for both terms
  Scalar rel_err;
  Scalar abs_err;

  details::RelativeTransformation<!bool(RTIsIdentity)> RT;

 private:
  void preprocessOrientedNode() {
    const int init_tri_id1 = 0, init_tri_id2 = 0;
    const Triangle32& init_tri1 = tri_indices1[init_tri_id1];
    const Triangle32& init_tri2 = tri_indices2[init_tri_id2];

    Vec3s init_tri1_points[3];
    Vec3s init_tri2_points[3];

    init_tri1_points[0] = vertices1[init_tri1[0]];
    init_tri1_points[1] = vertices1[init_tri1[1]];
    init_tri1_points[2] = vertices1[init_tri1[2]];

    init_tri2_points[0] = vertices2[init_tri2[0]];
    init_tri2_points[1] = vertices2[init_tri2[1]];
    init_tri2_points[2] = vertices2[init_tri2[2]];

    Vec3s p1, p2, normal;
    Scalar distance = sqrt(TriangleDistance::sqrTriDistance(
        init_tri1_points[0], init_tri1_points[1], init_tri1_points[2],
        init_tri2_points[0], init_tri2_points[1], init_tri2_points[2], RT._R(),
        RT._T(), p1, p2));

    result->update(distance, model1, model2, init_tri_id1, init_tri_id2, p1, p2,
                   normal);
  }
  void postprocessOrientedNode() {
    /// the points obtained by triDistance are not in world space: both are in
    /// object1's local coordinate system, so we need to convert them into the
    /// world space.
    if (request.enable_nearest_points && (result->o1 == model1) &&
        (result->o2 == model2)) {
      result->nearest_points[0] = tf1.transform(result->nearest_points[0]);
      result->nearest_points[1] = tf1.transform(result->nearest_points[1]);
    }
  }
};

/// @brief Traversal node for distance computation between two meshes if their
/// underlying BVH node is oriented node (RSS, OBBRSS, kIOS)
typedef MeshDistanceTraversalNode<RSS, 0> MeshDistanceTraversalNodeRSS;
typedef MeshDistanceTraversalNode<kIOS, 0> MeshDistanceTraversalNodekIOS;
typedef MeshDistanceTraversalNode<OBBRSS, 0> MeshDistanceTraversalNodeOBBRSS;

/// @}

/// @brief for OBB and RSS, there is local coordinate of BV, so normal need to
/// be transformed
namespace details {

template <typename BV>
inline const Matrix3s& getBVAxes(const BV& bv) {
  return bv.axes;
}

template <>
inline const Matrix3s& getBVAxes<OBBRSS>(const OBBRSS& bv) {
  return bv.obb.axes;
}

}  // namespace details

}  // namespace coal

/// @endcond

#endif
