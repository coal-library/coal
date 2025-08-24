/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2022-2023, INRIA
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

#ifndef COAL_TRAVERSAL_NODE_OCTREE_H
#define COAL_TRAVERSAL_NODE_OCTREE_H

/// @cond INTERNAL

#include "coal/collision_data.h"
#include "coal/internal/traversal_node_base.h"
#include "coal/internal/traversal_node_hfield_shape.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/hfield.h"
#include "coal/octree.h"
#include "coal/BVH/BVH_model.h"
#include "coal/shape/geometric_shapes_utility.h"
#include "coal/internal/shape_shape_func.h"

namespace coal {

/// @brief Algorithms for collision related with octree
class COAL_DLLAPI OcTreeSolver {
 private:
  const GJKSolver* solver;

  mutable const CollisionRequest* crequest;
  mutable const DistanceRequest* drequest;

  mutable CollisionResult* cresult;
  mutable DistanceResult* dresult;

 public:
  OcTreeSolver(const GJKSolver* solver_)
      : solver(solver_),
        crequest(NULL),
        drequest(NULL),
        cresult(NULL),
        dresult(NULL) {}

  /// @brief collision between two octrees
  void OcTreeIntersect(const OcTree* tree1, const OcTree* tree2,
                       const Transform3s& tf1, const Transform3s& tf2,
                       const CollisionRequest& request_,
                       CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    OcTreeIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(), tree2,
                           tree2->getRoot(), tree2->getRootBV(), tf1, tf2);
  }

  /// @brief distance between two octrees
  void OcTreeDistance(const OcTree* tree1, const OcTree* tree2,
                      const Transform3s& tf1, const Transform3s& tf2,
                      const DistanceRequest& request_,
                      DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    OcTreeDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(), tree2,
                          tree2->getRoot(), tree2->getRootBV(), tf1, tf2);
  }

  /// @brief collision between octree and mesh
  template <typename BV>
  void OcTreeMeshIntersect(const OcTree* tree1, const BVHModel<BV>* tree2,
                           const Transform3s& tf1, const Transform3s& tf2,
                           const CollisionRequest& request_,
                           CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    OcTreeMeshIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                               tree2, 0, tf1, tf2);
  }

  /// @brief distance between octree and mesh
  template <typename BV>
  void OcTreeMeshDistance(const OcTree* tree1, const BVHModel<BV>* tree2,
                          const Transform3s& tf1, const Transform3s& tf2,
                          const DistanceRequest& request_,
                          DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    OcTreeMeshDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                              tree2, 0, tf1, tf2);
  }

  /// @brief collision between mesh and octree
  template <typename BV>
  void MeshOcTreeIntersect(const BVHModel<BV>* tree1, const OcTree* tree2,
                           const Transform3s& tf1, const Transform3s& tf2,
                           const CollisionRequest& request_,
                           CollisionResult& result_) const

  {
    crequest = &request_;
    cresult = &result_;

    OcTreeMeshIntersectRecurse(tree2, tree2->getRoot(), tree2->getRootBV(),
                               tree1, 0, tf2, tf1);
  }

  /// @brief distance between mesh and octree
  template <typename BV>
  void MeshOcTreeDistance(const BVHModel<BV>* tree1, const OcTree* tree2,
                          const Transform3s& tf1, const Transform3s& tf2,
                          const DistanceRequest& request_,
                          DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    OcTreeMeshDistanceRecurse(tree1, 0, tree2, tree2->getRoot(),
                              tree2->getRootBV(), tf1, tf2);
  }

  template <typename BV>
  void OcTreeHeightFieldIntersect(
      const OcTree* tree1, const HeightField<BV>* tree2, const Transform3s& tf1,
      const Transform3s& tf2, const CollisionRequest& request_,
      CollisionResult& result_, Scalar& sqrDistLowerBound) const {
    crequest = &request_;
    cresult = &result_;

    OcTreeHeightFieldIntersectRecurse(tree1, tree1->getRoot(),
                                      tree1->getRootBV(), tree2, 0, tf1, tf2,
                                      sqrDistLowerBound);
  }

  template <typename BV>
  void HeightFieldOcTreeIntersect(const HeightField<BV>* tree1,
                                  const OcTree* tree2, const Transform3s& tf1,
                                  const Transform3s& tf2,
                                  const CollisionRequest& request_,
                                  CollisionResult& result_,
                                  Scalar& sqrDistLowerBound) const {
    crequest = &request_;
    cresult = &result_;

    HeightFieldOcTreeIntersectRecurse(tree1, 0, tree2, tree2->getRoot(),
                                      tree2->getRootBV(), tf1, tf2,
                                      sqrDistLowerBound);
  }

  /// @brief collision between octree and shape
  template <typename S>
  void OcTreeShapeIntersect(const OcTree* tree, const S& s,
                            const Transform3s& tf1, const Transform3s& tf2,
                            const CollisionRequest& request_,
                            CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    AABB bv2;
    computeBV<AABB>(s, Transform3s(), bv2);
    OBB obb2;
    convertBV(bv2, tf2, obb2);
    OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                                obb2, tf1, tf2);
  }

  /// @brief collision between shape and octree
  template <typename S>
  void ShapeOcTreeIntersect(const S& s, const OcTree* tree,
                            const Transform3s& tf1, const Transform3s& tf2,
                            const CollisionRequest& request_,
                            CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    AABB bv1;
    computeBV<AABB>(s, Transform3s(), bv1);
    OBB obb1;
    convertBV(bv1, tf1, obb1);
    OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                                obb1, tf2, tf1);
  }

  /// @brief distance between octree and shape
  template <typename S>
  void OcTreeShapeDistance(const OcTree* tree, const S& s,
                           const Transform3s& tf1, const Transform3s& tf2,
                           const DistanceRequest& request_,
                           DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    AABB aabb2;
    computeBV<AABB>(s, tf2, aabb2);
    OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                               aabb2, tf1, tf2);
  }

  /// @brief distance between shape and octree
  template <typename S>
  void ShapeOcTreeDistance(const S& s, const OcTree* tree,
                           const Transform3s& tf1, const Transform3s& tf2,
                           const DistanceRequest& request_,
                           DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    AABB aabb1;
    computeBV<AABB>(s, tf1, aabb1);
    OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                               aabb1, tf2, tf1);
  }

 private:
  template <typename S>
  bool OcTreeShapeDistanceRecurse(const OcTree* tree1,
                                  const OcTree::OcTreeNode* root1,
                                  const AABB& bv1, const S& s,
                                  const AABB& aabb2, const Transform3s& tf1,
                                  const Transform3s& tf2) const {
    if (!tree1->nodeHasChildren(root1)) {
      if (tree1->isNodeOccupied(root1)) {
        Box box;
        Transform3s box_tf;
        constructBox(bv1, tf1, box, box_tf);

        if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
          box.computeLocalAABB();
        }

        Vec3s p1, p2, normal;
        const Scalar distance = internal::ShapeShapeDistance<Box, S>(
            &box, box_tf, &s, tf2, this->solver,
            this->drequest->enable_signed_distance, p1, p2, normal);

        this->dresult->update(distance, tree1, &s,
                              (int)(root1 - tree1->getRoot()),
                              DistanceResult::NONE, p1, p2, normal);

        return drequest->isSatisfied(*dresult);
      } else
        return false;
    }

    if (!tree1->isNodeOccupied(root1)) return false;

    for (unsigned int i = 0; i < 8; ++i) {
      if (tree1->nodeChildExists(root1, i)) {
        const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB child_bv;
        computeChildBV(bv1, i, child_bv);

        AABB aabb1;
        convertBV(child_bv, tf1, aabb1);
        Scalar d = aabb1.distance(aabb2);
        if (d < dresult->min_distance) {
          if (OcTreeShapeDistanceRecurse(tree1, child, child_bv, s, aabb2, tf1,
                                         tf2))
            return true;
        }
      }
    }

    return false;
  }

  template <typename S>
  bool OcTreeShapeIntersectRecurse(const OcTree* tree1,
                                   const OcTree::OcTreeNode* root1,
                                   const AABB& bv1, const S& s, const OBB& obb2,
                                   const Transform3s& tf1,
                                   const Transform3s& tf2) const {
    // Empty OcTree is considered free.
    if (!root1) return false;

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least of one the nodes is free; OR
    ///           2) (two uncertain nodes or one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree1->isNodeFree(root1))
      return false;
    else if ((tree1->isNodeUncertain(root1) || s.isUncertain()))
      return false;
    else {
      OBB obb1;
      convertBV(bv1, tf1, obb1);
      Scalar sqrDistLowerBound;
      if (!obb1.overlap(obb2, *crequest, sqrDistLowerBound)) {
        internal::updateDistanceLowerBoundFromBV(*crequest, *cresult,
                                                 sqrDistLowerBound);
        return false;
      }
    }

    if (!tree1->nodeHasChildren(root1)) {
      assert(tree1->isNodeOccupied(root1));  // it isn't free nor uncertain.

      Box box;
      Transform3s box_tf;
      constructBox(bv1, tf1, box, box_tf);
      if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
        box.computeLocalAABB();
      }

      bool contactNotAdded =
          (cresult->numContacts() >= crequest->num_max_contacts);
      std::size_t ncontact = ShapeShapeCollide<Box, S>(
          &box, box_tf, &s, tf2, solver, *crequest, *cresult);
      assert(ncontact == 0 || ncontact == 1);
      if (!contactNotAdded && ncontact == 1) {
        // Update contact information.
        const Contact& c = cresult->getContact(cresult->numContacts() - 1);
        cresult->setContact(
            cresult->numContacts() - 1,
            Contact(tree1, c.o2, static_cast<int>(root1 - tree1->getRoot()),
                    c.b2, c.pos, c.normal, c.penetration_depth));
      }

      // no need to call `internal::updateDistanceLowerBoundFromLeaf` here
      // as it is already done internally in `ShapeShapeCollide` above.
      return crequest->isSatisfied(*cresult);
    }

    for (unsigned int i = 0; i < 8; ++i) {
      if (tree1->nodeChildExists(root1, i)) {
        const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB child_bv;
        computeChildBV(bv1, i, child_bv);

        if (OcTreeShapeIntersectRecurse(tree1, child, child_bv, s, obb2, tf1,
                                        tf2))
          return true;
      }
    }

    return false;
  }

  template <typename BV>
  bool OcTreeMeshDistanceRecurse(const OcTree* tree1,
                                 const OcTree::OcTreeNode* root1,
                                 const AABB& bv1, const BVHModel<BV>* tree2,
                                 unsigned int root2, const Transform3s& tf1,
                                 const Transform3s& tf2) const {
    if (!tree1->nodeHasChildren(root1) && tree2->getBV(root2).isLeaf()) {
      if (tree1->isNodeOccupied(root1)) {
        Box box;
        Transform3s box_tf;
        constructBox(bv1, tf1, box, box_tf);

        if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
          box.computeLocalAABB();
        }

        size_t primitive_id =
            static_cast<size_t>(tree2->getBV(root2).primitiveId());
        const Triangle32& tri_id = (*(tree2->tri_indices))[primitive_id];
        const TriangleP tri((*(tree2->vertices))[tri_id[0]],
                            (*(tree2->vertices))[tri_id[1]],
                            (*(tree2->vertices))[tri_id[2]]);

        Vec3s p1, p2, normal;
        const Scalar distance = internal::ShapeShapeDistance<Box, TriangleP>(
            &box, box_tf, &tri, tf2, this->solver,
            this->drequest->enable_signed_distance, p1, p2, normal);

        this->dresult->update(distance, tree1, tree2,
                              (int)(root1 - tree1->getRoot()),
                              static_cast<int>(primitive_id), p1, p2, normal);

        return this->drequest->isSatisfied(*dresult);
      } else
        return false;
    }

    if (!tree1->isNodeOccupied(root1)) return false;

    if (tree2->getBV(root2).isLeaf() ||
        (tree1->nodeHasChildren(root1) &&
         (bv1.size() > tree2->getBV(root2).bv.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          Scalar d;
          AABB aabb1, aabb2;
          convertBV(child_bv, tf1, aabb1);
          convertBV(tree2->getBV(root2).bv, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if (d < dresult->min_distance) {
            if (OcTreeMeshDistanceRecurse(tree1, child, child_bv, tree2, root2,
                                          tf1, tf2))
              return true;
          }
        }
      }
    } else {
      Scalar d;
      AABB aabb1, aabb2;
      convertBV(bv1, tf1, aabb1);
      unsigned int child = (unsigned int)tree2->getBV(root2).leftChild();
      convertBV(tree2->getBV(child).bv, tf2, aabb2);
      d = aabb1.distance(aabb2);

      if (d < dresult->min_distance) {
        if (OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, child, tf1,
                                      tf2))
          return true;
      }

      child = (unsigned int)tree2->getBV(root2).rightChild();
      convertBV(tree2->getBV(child).bv, tf2, aabb2);
      d = aabb1.distance(aabb2);

      if (d < dresult->min_distance) {
        if (OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, child, tf1,
                                      tf2))
          return true;
      }
    }

    return false;
  }

  /// \return True if the request is satisfied.
  template <typename BV>
  bool OcTreeMeshIntersectRecurse(const OcTree* tree1,
                                  const OcTree::OcTreeNode* root1,
                                  const AABB& bv1, const BVHModel<BV>* tree2,
                                  unsigned int root2, const Transform3s& tf1,
                                  const Transform3s& tf2) const {
    // FIXME(jmirabel) I do not understand why the BVHModel was traversed. The
    // code in this if(!root1) did not output anything so the empty OcTree is
    // considered free. Should an empty OcTree be considered free ?

    // Empty OcTree is considered free.
    if (!root1) return false;
    BVNode<BV> const& bvn2 = tree2->getBV(root2);

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree1->isNodeFree(root1))
      return false;
    else if ((tree1->isNodeUncertain(root1) || tree2->isUncertain()))
      return false;
    else {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bvn2.bv, tf2, obb2);
      Scalar sqrDistLowerBound;
      if (!obb1.overlap(obb2, *crequest, sqrDistLowerBound)) {
        internal::updateDistanceLowerBoundFromBV(*crequest, *cresult,
                                                 sqrDistLowerBound);
        return false;
      }
    }

    // Check if leaf collides.
    if (!tree1->nodeHasChildren(root1) && bvn2.isLeaf()) {
      assert(tree1->isNodeOccupied(root1));  // it isn't free nor uncertain.
      Box box;
      Transform3s box_tf;
      constructBox(bv1, tf1, box, box_tf);
      if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
        box.computeLocalAABB();
      }

      size_t primitive_id = static_cast<size_t>(bvn2.primitiveId());
      const Triangle32& tri_id = (*(tree2->tri_indices))[primitive_id];
      const TriangleP tri((*(tree2->vertices))[tri_id[0]],
                          (*(tree2->vertices))[tri_id[1]],
                          (*(tree2->vertices))[tri_id[2]]);

      // When reaching this point, `this->solver` has already been set up
      // by the CollisionRequest `this->crequest`.
      // The only thing we need to (and can) pass to `ShapeShapeDistance` is
      // whether or not penetration information is should be computed in case of
      // collision.
      const bool compute_penetration = this->crequest->enable_contact ||
                                       (this->crequest->security_margin < 0);
      Vec3s c1, c2, normal;
      const Scalar distance = internal::ShapeShapeDistance<Box, TriangleP>(
          &box, box_tf, &tri, tf2, this->solver, compute_penetration, c1, c2,
          normal);
      const Scalar distToCollision = distance - this->crequest->security_margin;

      internal::updateDistanceLowerBoundFromLeaf(
          *(this->crequest), *(this->cresult), distToCollision, c1, c2, normal);

      if (cresult->numContacts() < crequest->num_max_contacts) {
        if (distToCollision <= crequest->collision_distance_threshold) {
          cresult->addContact(Contact(
              tree1, tree2, (int)(root1 - tree1->getRoot()),
              static_cast<int>(primitive_id), c1, c2, normal, distance));
        }
      }
      return crequest->isSatisfied(*cresult);
    }

    // Determine which tree to traverse first.
    if (bvn2.isLeaf() ||
        (tree1->nodeHasChildren(root1) && (bv1.size() > bvn2.bv.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          if (OcTreeMeshIntersectRecurse(tree1, child, child_bv, tree2, root2,
                                         tf1, tf2))
            return true;
        }
      }
    } else {
      if (OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2,
                                     (unsigned int)bvn2.leftChild(), tf1, tf2))
        return true;

      if (OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2,
                                     (unsigned int)bvn2.rightChild(), tf1, tf2))
        return true;
    }

    return false;
  }

  /// \return True if the request is satisfied.
  template <typename BV>
  bool OcTreeHeightFieldIntersectRecurse(
      const OcTree* tree1, const OcTree::OcTreeNode* root1, const AABB& bv1,
      const HeightField<BV>* tree2, unsigned int root2, const Transform3s& tf1,
      const Transform3s& tf2, Scalar& sqrDistLowerBound) const {
    // FIXME(jmirabel) I do not understand why the BVHModel was traversed. The
    // code in this if(!root1) did not output anything so the empty OcTree is
    // considered free. Should an empty OcTree be considered free ?

    // Empty OcTree is considered free.
    if (!root1) return false;
    HFNode<BV> const& bvn2 = tree2->getBV(root2);

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree1->isNodeFree(root1))
      return false;
    else if ((tree1->isNodeUncertain(root1) || tree2->isUncertain()))
      return false;
    else {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bvn2.bv, tf2, obb2);
      Scalar sqrDistLowerBound_;
      if (!obb1.overlap(obb2, *crequest, sqrDistLowerBound_)) {
        if (sqrDistLowerBound_ < sqrDistLowerBound)
          sqrDistLowerBound = sqrDistLowerBound_;
        internal::updateDistanceLowerBoundFromBV(*crequest, *cresult,
                                                 sqrDistLowerBound);
        return false;
      }
    }

    // Check if leaf collides.
    if (!tree1->nodeHasChildren(root1) && bvn2.isLeaf()) {
      assert(tree1->isNodeOccupied(root1));  // it isn't free nor uncertain.
      Box box;
      Transform3s box_tf;
      constructBox(bv1, tf1, box, box_tf);
      if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
        box.computeLocalAABB();
      }

      typedef ConvexTpl<Triangle32> ConvexTriangle;
      ConvexTriangle convex1, convex2;
      int convex1_active_faces, convex2_active_faces;
      details::buildConvexTriangles(bvn2, *tree2, convex1, convex1_active_faces,
                                    convex2, convex2_active_faces);
      if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
        convex1.computeLocalAABB();
        convex2.computeLocalAABB();
      }

      Vec3s c1, c2, normal, normal_top;
      Scalar distance;
      bool hfield_witness_is_on_bin_side;

      bool collision = details::shapeDistance<Triangle32, Box, 0>(
          solver, *crequest, convex1, convex1_active_faces, convex2,
          convex2_active_faces, tf2, box, box_tf, distance, c2, c1, normal,
          normal_top, hfield_witness_is_on_bin_side);

      Scalar distToCollision =
          distance - crequest->security_margin * (normal_top.dot(normal));

      if (distToCollision <= crequest->collision_distance_threshold) {
        sqrDistLowerBound = 0;
        if (crequest->num_max_contacts > cresult->numContacts()) {
          if (normal_top.isApprox(normal) &&
              (collision || !hfield_witness_is_on_bin_side)) {
            cresult->addContact(
                Contact(tree1, tree2, (int)(root1 - tree1->getRoot()),
                        (int)Contact::NONE, c1, c2, -normal, distance));
          }
        }
      } else
        sqrDistLowerBound = distToCollision * distToCollision;

      //    const Vec3s c1 = contact_point - distance * 0.5 * normal;
      //    const Vec3s c2 = contact_point + distance * 0.5 * normal;
      internal::updateDistanceLowerBoundFromLeaf(
          *crequest, *cresult, distToCollision, c1, c2, -normal);

      assert(cresult->isCollision() || sqrDistLowerBound > 0);
      return crequest->isSatisfied(*cresult);
    }

    // Determine which tree to traverse first.
    if (bvn2.isLeaf() ||
        (tree1->nodeHasChildren(root1) && (bv1.size() > bvn2.bv.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          if (OcTreeHeightFieldIntersectRecurse(tree1, child, child_bv, tree2,
                                                root2, tf1, tf2,
                                                sqrDistLowerBound))
            return true;
        }
      }
    } else {
      if (OcTreeHeightFieldIntersectRecurse(tree1, root1, bv1, tree2,
                                            (unsigned int)bvn2.leftChild(), tf1,
                                            tf2, sqrDistLowerBound))
        return true;

      if (OcTreeHeightFieldIntersectRecurse(tree1, root1, bv1, tree2,
                                            (unsigned int)bvn2.rightChild(),
                                            tf1, tf2, sqrDistLowerBound))
        return true;
    }

    return false;
  }

  /// \return True if the request is satisfied.
  template <typename BV>
  bool HeightFieldOcTreeIntersectRecurse(
      const HeightField<BV>* tree1, unsigned int root1, const OcTree* tree2,
      const OcTree::OcTreeNode* root2, const AABB& bv2, const Transform3s& tf1,
      const Transform3s& tf2, Scalar& sqrDistLowerBound) const {
    // FIXME(jmirabel) I do not understand why the BVHModel was traversed. The
    // code in this if(!root1) did not output anything so the empty OcTree is
    // considered free. Should an empty OcTree be considered free ?

    // Empty OcTree is considered free.
    if (!root2) return false;
    HFNode<BV> const& bvn1 = tree1->getBV(root1);

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree2->isNodeFree(root2))
      return false;
    else if ((tree2->isNodeUncertain(root2) || tree1->isUncertain()))
      return false;
    else {
      OBB obb1, obb2;
      convertBV(bvn1.bv, tf1, obb1);
      convertBV(bv2, tf2, obb2);
      Scalar sqrDistLowerBound_;
      if (!obb2.overlap(obb1, *crequest, sqrDistLowerBound_)) {
        if (sqrDistLowerBound_ < sqrDistLowerBound)
          sqrDistLowerBound = sqrDistLowerBound_;
        internal::updateDistanceLowerBoundFromBV(*crequest, *cresult,
                                                 sqrDistLowerBound);
        return false;
      }
    }

    // Check if leaf collides.
    if (!tree2->nodeHasChildren(root2) && bvn1.isLeaf()) {
      assert(tree2->isNodeOccupied(root2));  // it isn't free nor uncertain.
      Box box;
      Transform3s box_tf;
      constructBox(bv2, tf2, box, box_tf);
      if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
        box.computeLocalAABB();
      }

      typedef ConvexTpl<Triangle32> ConvexTriangle;
      ConvexTriangle convex1, convex2;
      int convex1_active_faces, convex2_active_faces;
      details::buildConvexTriangles(bvn1, *tree1, convex1, convex1_active_faces,
                                    convex2, convex2_active_faces);
      if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
        convex1.computeLocalAABB();
        convex2.computeLocalAABB();
      }

      Vec3s c1, c2, normal, normal_top;
      Scalar distance;
      bool hfield_witness_is_on_bin_side;

      bool collision = details::shapeDistance<Triangle32, Box, 0>(
          solver, *crequest, convex1, convex1_active_faces, convex2,
          convex2_active_faces, tf1, box, box_tf, distance, c1, c2, normal,
          normal_top, hfield_witness_is_on_bin_side);

      Scalar distToCollision =
          distance - crequest->security_margin * (normal_top.dot(normal));

      if (distToCollision <= crequest->collision_distance_threshold) {
        sqrDistLowerBound = 0;
        if (crequest->num_max_contacts > cresult->numContacts()) {
          if (normal_top.isApprox(normal) &&
              (collision || !hfield_witness_is_on_bin_side)) {
            cresult->addContact(Contact(tree1, tree2, (int)Contact::NONE,
                                        (int)(root2 - tree2->getRoot()), c1, c2,
                                        normal, distance));
          }
        }
      } else
        sqrDistLowerBound = distToCollision * distToCollision;

      //    const Vec3s c1 = contact_point - distance * 0.5 * normal;
      //    const Vec3s c2 = contact_point + distance * 0.5 * normal;
      internal::updateDistanceLowerBoundFromLeaf(
          *crequest, *cresult, distToCollision, c1, c2, normal);

      assert(cresult->isCollision() || sqrDistLowerBound > 0);
      return crequest->isSatisfied(*cresult);
    }

    // Determine which tree to traverse first.
    if (bvn1.isLeaf() ||
        (tree2->nodeHasChildren(root2) && (bv2.size() > bvn1.bv.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree2->nodeChildExists(root2, i)) {
          const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);

          if (HeightFieldOcTreeIntersectRecurse(tree1, root1, tree2, child,
                                                child_bv, tf1, tf2,
                                                sqrDistLowerBound))
            return true;
        }
      }
    } else {
      if (HeightFieldOcTreeIntersectRecurse(
              tree1, (unsigned int)bvn1.leftChild(), tree2, root2, bv2, tf1,
              tf2, sqrDistLowerBound))
        return true;

      if (HeightFieldOcTreeIntersectRecurse(
              tree1, (unsigned int)bvn1.rightChild(), tree2, root2, bv2, tf1,
              tf2, sqrDistLowerBound))
        return true;
    }

    return false;
  }

  bool OcTreeDistanceRecurse(const OcTree* tree1,
                             const OcTree::OcTreeNode* root1, const AABB& bv1,
                             const OcTree* tree2,
                             const OcTree::OcTreeNode* root2, const AABB& bv2,
                             const Transform3s& tf1,
                             const Transform3s& tf2) const {
    if (!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2)) {
      if (tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2)) {
        Box box1, box2;
        Transform3s box1_tf, box2_tf;
        constructBox(bv1, tf1, box1, box1_tf);
        constructBox(bv2, tf2, box2, box2_tf);

        if (solver->gjk_initial_guess == GJKInitialGuess::BoundingVolumeGuess) {
          box1.computeLocalAABB();
          box2.computeLocalAABB();
        }

        Vec3s p1, p2, normal;
        const Scalar distance = internal::ShapeShapeDistance<Box, Box>(
            &box1, box1_tf, &box2, box2_tf, this->solver,
            this->drequest->enable_signed_distance, p1, p2, normal);

        this->dresult->update(distance, tree1, tree2,
                              (int)(root1 - tree1->getRoot()),
                              (int)(root2 - tree2->getRoot()), p1, p2, normal);

        return drequest->isSatisfied(*dresult);
      } else
        return false;
    }

    if (!tree1->isNodeOccupied(root1) || !tree2->isNodeOccupied(root2))
      return false;

    if (!tree2->nodeHasChildren(root2) ||
        (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          Scalar d;
          AABB aabb1, aabb2;
          convertBV(bv1, tf1, aabb1);
          convertBV(bv2, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if (d < dresult->min_distance) {
            if (OcTreeDistanceRecurse(tree1, child, child_bv, tree2, root2, bv2,
                                      tf1, tf2))
              return true;
          }
        }
      }
    } else {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree2->nodeChildExists(root2, i)) {
          const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);

          Scalar d;
          AABB aabb1, aabb2;
          convertBV(bv1, tf1, aabb1);
          convertBV(bv2, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if (d < dresult->min_distance) {
            if (OcTreeDistanceRecurse(tree1, root1, bv1, tree2, child, child_bv,
                                      tf1, tf2))
              return true;
          }
        }
      }
    }

    return false;
  }

  bool OcTreeIntersectRecurse(const OcTree* tree1,
                              const OcTree::OcTreeNode* root1, const AABB& bv1,
                              const OcTree* tree2,
                              const OcTree::OcTreeNode* root2, const AABB& bv2,
                              const Transform3s& tf1,
                              const Transform3s& tf2) const {
    // Empty OcTree is considered free.
    if (!root1) return false;
    if (!root2) return false;

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree1->isNodeFree(root1) || tree2->isNodeFree(root2))
      return false;
    else if ((tree1->isNodeUncertain(root1) || tree2->isNodeUncertain(root2)))
      return false;

    bool bothAreLeaves =
        (!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2));
    if (!bothAreLeaves || !crequest->enable_contact) {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bv2, tf2, obb2);
      Scalar sqrDistLowerBound;
      if (!obb1.overlap(obb2, *crequest, sqrDistLowerBound)) {
        if (cresult->distance_lower_bound > 0 &&
            sqrDistLowerBound <
                cresult->distance_lower_bound * cresult->distance_lower_bound)
          cresult->distance_lower_bound =
              sqrt(sqrDistLowerBound) - crequest->security_margin;
        return false;
      }
      if (crequest->enable_contact) {  // Overlap
        if (cresult->numContacts() < crequest->num_max_contacts)
          cresult->addContact(
              Contact(tree1, tree2, static_cast<int>(root1 - tree1->getRoot()),
                      static_cast<int>(root2 - tree2->getRoot())));
        return crequest->isSatisfied(*cresult);
      }
    }

    // Both node are leaves
    if (bothAreLeaves) {
      assert(tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2));

      Box box1, box2;
      Transform3s box1_tf, box2_tf;
      constructBox(bv1, tf1, box1, box1_tf);
      constructBox(bv2, tf2, box2, box2_tf);

      if (this->solver->gjk_initial_guess ==
          GJKInitialGuess::BoundingVolumeGuess) {
        box1.computeLocalAABB();
        box2.computeLocalAABB();
      }

      // When reaching this point, `this->solver` has already been set up
      // by the CollisionRequest `this->request`.
      // The only thing we need to (and can) pass to `ShapeShapeDistance` is
      // whether or not penetration information is should be computed in case of
      // collision.
      const bool compute_penetration = (this->crequest->enable_contact ||
                                        (this->crequest->security_margin < 0));
      Vec3s c1, c2, normal;
      Scalar distance = internal::ShapeShapeDistance<Box, Box>(
          &box1, box1_tf, &box2, box2_tf, this->solver, compute_penetration, c1,
          c2, normal);

      const Scalar distToCollision = distance - this->crequest->security_margin;

      internal::updateDistanceLowerBoundFromLeaf(
          *(this->crequest), *(this->cresult), distToCollision, c1, c2, normal);

      if (this->cresult->numContacts() < this->crequest->num_max_contacts) {
        if (distToCollision <= this->crequest->collision_distance_threshold)
          this->cresult->addContact(
              Contact(tree1, tree2, static_cast<int>(root1 - tree1->getRoot()),
                      static_cast<int>(root2 - tree2->getRoot()), c1, c2,
                      normal, distance));
      }

      return crequest->isSatisfied(*cresult);
    }

    // Determine which tree to traverse first.
    if (!tree2->nodeHasChildren(root2) ||
        (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          if (OcTreeIntersectRecurse(tree1, child, child_bv, tree2, root2, bv2,
                                     tf1, tf2))
            return true;
        }
      }
    } else {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree2->nodeChildExists(root2, i)) {
          const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);

          if (OcTreeIntersectRecurse(tree1, root1, bv1, tree2, child, child_bv,
                                     tf1, tf2))
            return true;
        }
      }
    }

    return false;
  }
};

/// @addtogroup Traversal_For_Collision
/// @{

/// @brief Traversal node for octree collision
class COAL_DLLAPI OcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  OcTreeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned, unsigned, Scalar&) const { return false; }

  void leafCollides(unsigned, unsigned, Scalar& sqrDistLowerBound) const {
    otsolver->OcTreeIntersect(model1, model2, tf1, tf2, request, *result);
    sqrDistLowerBound = std::max((Scalar)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const OcTree* model1;
  const OcTree* model2;

  Transform3s tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for shape-octree collision
template <typename S>
class COAL_DLLAPI ShapeOcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  ShapeOcTreeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, Scalar&) const { return false; }

  void leafCollides(unsigned int, unsigned int,
                    Scalar& sqrDistLowerBound) const {
    otsolver->OcTreeShapeIntersect(model2, *model1, tf2, tf1, request, *result);
    sqrDistLowerBound = std::max((Scalar)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const S* model1;
  const OcTree* model2;

  Transform3s tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-shape collision

template <typename S>
class COAL_DLLAPI OcTreeShapeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  OcTreeShapeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, coal::Scalar&) const {
    return false;
  }

  void leafCollides(unsigned int, unsigned int,
                    Scalar& sqrDistLowerBound) const {
    otsolver->OcTreeShapeIntersect(model1, *model2, tf1, tf2, request, *result);
    sqrDistLowerBound = std::max((Scalar)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const OcTree* model1;
  const S* model2;

  Transform3s tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for mesh-octree collision
template <typename BV>
class COAL_DLLAPI MeshOcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  MeshOcTreeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, Scalar&) const { return false; }

  void leafCollides(unsigned int, unsigned int,
                    Scalar& sqrDistLowerBound) const {
    otsolver->OcTreeMeshIntersect(model2, model1, tf2, tf1, request, *result);
    sqrDistLowerBound = std::max((Scalar)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const BVHModel<BV>* model1;
  const OcTree* model2;

  Transform3s tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-mesh collision
template <typename BV>
class COAL_DLLAPI OcTreeMeshCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  OcTreeMeshCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, Scalar&) const { return false; }

  void leafCollides(unsigned int, unsigned int,
                    Scalar& sqrDistLowerBound) const {
    otsolver->OcTreeMeshIntersect(model1, model2, tf1, tf2, request, *result);
    sqrDistLowerBound = std::max((Scalar)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const OcTree* model1;
  const BVHModel<BV>* model2;

  Transform3s tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-height-field collision
template <typename BV>
class COAL_DLLAPI OcTreeHeightFieldCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  OcTreeHeightFieldCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, Scalar&) const { return false; }

  void leafCollides(unsigned int, unsigned int,
                    Scalar& sqrDistLowerBound) const {
    otsolver->OcTreeHeightFieldIntersect(model1, model2, tf1, tf2, request,
                                         *result, sqrDistLowerBound);
  }

  const OcTree* model1;
  const HeightField<BV>* model2;

  Transform3s tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-height-field collision
template <typename BV>
class COAL_DLLAPI HeightFieldOcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  HeightFieldOcTreeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, Scalar&) const { return false; }

  void leafCollides(unsigned int, unsigned int,
                    Scalar& sqrDistLowerBound) const {
    otsolver->HeightFieldOcTreeIntersect(model1, model2, tf1, tf2, request,
                                         *result, sqrDistLowerBound);
  }

  const HeightField<BV>* model1;
  const OcTree* model2;

  Transform3s tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @}

/// @addtogroup Traversal_For_Distance
/// @{

/// @brief Traversal node for octree distance
class COAL_DLLAPI OcTreeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  OcTreeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  Scalar BVDistanceLowerBound(unsigned, unsigned) const { return -1; }

  bool BVDistanceLowerBound(unsigned, unsigned, Scalar&) const { return false; }

  void leafComputeDistance(unsigned, unsigned int) const {
    otsolver->OcTreeDistance(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const OcTree* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for shape-octree distance
template <typename Shape>
class COAL_DLLAPI ShapeOcTreeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  ShapeOcTreeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  Scalar BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeShapeDistance(model2, *model1, tf2, tf1, request, *result);
  }

  const Shape* model1;
  const OcTree* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-shape distance
template <typename Shape>
class COAL_DLLAPI OcTreeShapeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  OcTreeShapeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  Scalar BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeShapeDistance(model1, *model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const Shape* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for mesh-octree distance
template <typename BV>
class COAL_DLLAPI MeshOcTreeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  MeshOcTreeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  Scalar BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeMeshDistance(model2, model1, tf2, tf1, request, *result);
  }

  const BVHModel<BV>* model1;
  const OcTree* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-mesh distance
template <typename BV>
class COAL_DLLAPI OcTreeMeshDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  OcTreeMeshDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  Scalar BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeMeshDistance(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const BVHModel<BV>* model2;

  const OcTreeSolver* otsolver;
};

/// @}

}  // namespace coal

/// @endcond

#endif
