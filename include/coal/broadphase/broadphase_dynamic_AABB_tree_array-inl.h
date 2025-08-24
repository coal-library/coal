/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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

/** @author Jia Pan */

#ifndef COAL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_INL_H
#define COAL_BROAD_PHASE_DYNAMIC_AABB_TREE_ARRAY_INL_H

#include "coal/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "coal/shape/geometric_shapes_utility.h"

#if COAL_HAVE_OCTOMAP
#include "coal/octree.h"
#endif

namespace coal {
namespace detail {
namespace dynamic_AABB_tree_array {

#if COAL_HAVE_OCTOMAP

//==============================================================================
template <typename Derived>
bool collisionRecurse_(
    DynamicAABBTreeArrayCollisionManager::DynamicAABBNode* nodes1,
    size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2,
    const AABB& root2_bv, const Eigen::MatrixBase<Derived>& translation2,
    CollisionCallBackBase* callback) {
  DynamicAABBTreeArrayCollisionManager::DynamicAABBNode* root1 =
      nodes1 + root1_id;
  if (!root2) {
    if (root1->isLeaf()) {
      CollisionObject* obj1 = static_cast<CollisionObject*>(root1->data);

      if (!obj1->collisionGeometry()->isFree()) {
        const AABB& root_bv_t = translate(root2_bv, translation2);
        if (root1->bv.overlap(root_bv_t)) {
          Box* box = new Box();
          Transform3s box_tf;
          Transform3s tf2 = Transform3s::Identity();
          tf2.translation() = translation2;
          constructBox(root2_bv, tf2, *box, box_tf);

          box->cost_density = tree2->getDefaultOccupancy();

          CollisionObject obj2(shared_ptr<CollisionGeometry>(box), box_tf);
          return (*callback)(obj1, &obj2);
        }
      }
    } else {
      if (collisionRecurse_(nodes1, root1->children[0], tree2, nullptr,
                            root2_bv, translation2, callback))
        return true;
      if (collisionRecurse_(nodes1, root1->children[1], tree2, nullptr,
                            root2_bv, translation2, callback))
        return true;
    }

    return false;
  } else if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
    CollisionObject* obj1 = static_cast<CollisionObject*>(root1->data);
    if (!tree2->isNodeFree(root2) && !obj1->collisionGeometry()->isFree()) {
      const AABB& root_bv_t = translate(root2_bv, translation2);
      if (root1->bv.overlap(root_bv_t)) {
        Box* box = new Box();
        Transform3s box_tf;
        Transform3s tf2 = Transform3s::Identity();
        tf2.translation() = translation2;
        constructBox(root2_bv, tf2, *box, box_tf);

        box->cost_density = Scalar(root2->getOccupancy());
        box->threshold_occupied = tree2->getOccupancyThres();

        CollisionObject obj2(shared_ptr<CollisionGeometry>(box), box_tf);
        return (*callback)(obj1, &obj2);
      } else
        return false;
    } else
      return false;
  }

  const AABB& root_bv_t = translate(root2_bv, translation2);
  if (tree2->isNodeFree(root2) || !root1->bv.overlap(root_bv_t)) return false;

  if (!tree2->nodeHasChildren(root2) ||
      (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
    if (collisionRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv,
                          translation2, callback))
      return true;
    if (collisionRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv,
                          translation2, callback))
      return true;
  } else {
    for (unsigned int i = 0; i < 8; ++i) {
      if (tree2->nodeChildExists(root2, i)) {
        const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if (collisionRecurse_(nodes1, root1_id, tree2, child, child_bv,
                              translation2, callback))
          return true;
      } else {
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);
        if (collisionRecurse_(nodes1, root1_id, tree2, nullptr, child_bv,
                              translation2, callback))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
template <typename Derived>
bool distanceRecurse_(
    DynamicAABBTreeArrayCollisionManager::DynamicAABBNode* nodes1,
    size_t root1_id, const OcTree* tree2, const OcTree::OcTreeNode* root2,
    const AABB& root2_bv, const Eigen::MatrixBase<Derived>& translation2,
    DistanceCallBackBase* callback, Scalar& min_dist) {
  DynamicAABBTreeArrayCollisionManager::DynamicAABBNode* root1 =
      nodes1 + root1_id;
  if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
    if (tree2->isNodeOccupied(root2)) {
      Box* box = new Box();
      Transform3s box_tf;
      Transform3s tf2 = Transform3s::Identity();
      tf2.translation() = translation2;
      constructBox(root2_bv, tf2, *box, box_tf);
      CollisionObject obj(shared_ptr<CollisionGeometry>(box), box_tf);
      return (*callback)(static_cast<CollisionObject*>(root1->data), &obj,
                         min_dist);
    } else
      return false;
  }

  if (!tree2->isNodeOccupied(root2)) return false;

  if (!tree2->nodeHasChildren(root2) ||
      (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
    const AABB& aabb2 = translate(root2_bv, translation2);

    Scalar d1 = aabb2.distance((nodes1 + root1->children[0])->bv);
    Scalar d2 = aabb2.distance((nodes1 + root1->children[1])->bv);

    if (d2 < d1) {
      if (d2 < min_dist) {
        if (distanceRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv,
                             translation2, callback, min_dist))
          return true;
      }

      if (d1 < min_dist) {
        if (distanceRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv,
                             translation2, callback, min_dist))
          return true;
      }
    } else {
      if (d1 < min_dist) {
        if (distanceRecurse_(nodes1, root1->children[0], tree2, root2, root2_bv,
                             translation2, callback, min_dist))
          return true;
      }

      if (d2 < min_dist) {
        if (distanceRecurse_(nodes1, root1->children[1], tree2, root2, root2_bv,
                             translation2, callback, min_dist))
          return true;
      }
    }
  } else {
    for (unsigned int i = 0; i < 8; ++i) {
      if (tree2->nodeChildExists(root2, i)) {
        const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        const AABB& aabb2 = translate(child_bv, translation2);
        Scalar d = root1->bv.distance(aabb2);

        if (d < min_dist) {
          if (distanceRecurse_(nodes1, root1_id, tree2, child, child_bv,
                               translation2, callback, min_dist))
            return true;
        }
      }
    }
  }

  return false;
}

#endif

}  // namespace dynamic_AABB_tree_array
}  // namespace detail
}  // namespace coal

#endif
