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

#include "coal/broadphase/broadphase_dynamic_AABB_tree.h"
#include "coal/tracy.hh"

#ifdef COAL_HAVE_OCTOMAP
#include "coal/octree.h"
#endif

#include "coal/BV/BV.h"
#include "coal/shape/geometric_shapes_utility.h"

#include <limits>

namespace coal {
namespace detail {

namespace dynamic_AABB_tree {

#if COAL_HAVE_OCTOMAP
//==============================================================================
bool collisionRecurse_(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1,
                       const OcTree* tree2, const OcTree::OcTreeNode* root2,
                       const AABB& root2_bv, const Transform3s& tf2,
                       CollisionCallBackBase* callback) {
  if (!root2) {
    if (root1->isLeaf()) {
      CollisionObject* obj1 = static_cast<CollisionObject*>(root1->data);

      if (!obj1->collisionGeometry()->isFree()) {
        OBB obb1, obb2;
        convertBV(root1->bv, Transform3s::Identity(), obb1);
        convertBV(root2_bv, tf2, obb2);

        if (obb1.overlap(obb2)) {
          Box* box = new Box();
          Transform3s box_tf;
          constructBox(root2_bv, tf2, *box, box_tf);

          box->cost_density = tree2->getDefaultOccupancy();

          CollisionObject obj2(shared_ptr<CollisionGeometry>(box), box_tf);
          return (*callback)(obj1, &obj2);
        }
      }
    } else {
      if (collisionRecurse_(root1->children[0], tree2, nullptr, root2_bv, tf2,
                            callback))
        return true;
      if (collisionRecurse_(root1->children[1], tree2, nullptr, root2_bv, tf2,
                            callback))
        return true;
    }

    return false;
  } else if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
    CollisionObject* obj1 = static_cast<CollisionObject*>(root1->data);

    if (!tree2->isNodeFree(root2) && !obj1->collisionGeometry()->isFree()) {
      OBB obb1, obb2;
      convertBV(root1->bv, Transform3s::Identity(), obb1);
      convertBV(root2_bv, tf2, obb2);

      if (obb1.overlap(obb2)) {
        Box* box = new Box();
        Transform3s box_tf;
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

  OBB obb1, obb2;
  convertBV(root1->bv, Transform3s::Identity(), obb1);
  convertBV(root2_bv, tf2, obb2);

  if (tree2->isNodeFree(root2) || !obb1.overlap(obb2)) return false;

  if (!tree2->nodeHasChildren(root2) ||
      (!root1->isLeaf() && (root1->bv.size() > root2_bv.size()))) {
    if (collisionRecurse_(root1->children[0], tree2, root2, root2_bv, tf2,
                          callback))
      return true;
    if (collisionRecurse_(root1->children[1], tree2, root2, root2_bv, tf2,
                          callback))
      return true;
  } else {
    for (unsigned int i = 0; i < 8; ++i) {
      if (tree2->nodeChildExists(root2, i)) {
        const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        if (collisionRecurse_(root1, tree2, child, child_bv, tf2, callback))
          return true;
      } else {
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);
        if (collisionRecurse_(root1, tree2, nullptr, child_bv, tf2, callback))
          return true;
      }
    }
  }
  return false;
}

//==============================================================================
bool distanceRecurse_(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1,
                      const OcTree* tree2, const OcTree::OcTreeNode* root2,
                      const AABB& root2_bv, const Transform3s& tf2,
                      DistanceCallBackBase* callback, Scalar& min_dist) {
  if (root1->isLeaf() && !tree2->nodeHasChildren(root2)) {
    if (tree2->isNodeOccupied(root2)) {
      Box* box = new Box();
      Transform3s box_tf;
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
    AABB aabb2;
    convertBV(root2_bv, tf2, aabb2);

    Scalar d1 = aabb2.distance(root1->children[0]->bv);
    Scalar d2 = aabb2.distance(root1->children[1]->bv);

    if (d2 < d1) {
      if (d2 < min_dist) {
        if (distanceRecurse_(root1->children[1], tree2, root2, root2_bv, tf2,
                             callback, min_dist))
          return true;
      }

      if (d1 < min_dist) {
        if (distanceRecurse_(root1->children[0], tree2, root2, root2_bv, tf2,
                             callback, min_dist))
          return true;
      }
    } else {
      if (d1 < min_dist) {
        if (distanceRecurse_(root1->children[0], tree2, root2, root2_bv, tf2,
                             callback, min_dist))
          return true;
      }

      if (d2 < min_dist) {
        if (distanceRecurse_(root1->children[1], tree2, root2, root2_bv, tf2,
                             callback, min_dist))
          return true;
      }
    }
  } else {
    for (unsigned int i = 0; i < 8; ++i) {
      if (tree2->nodeChildExists(root2, i)) {
        const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB child_bv;
        computeChildBV(root2_bv, i, child_bv);

        AABB aabb2;
        convertBV(child_bv, tf2, aabb2);
        Scalar d = root1->bv.distance(aabb2);

        if (d < min_dist) {
          if (distanceRecurse_(root1, tree2, child, child_bv, tf2, callback,
                               min_dist))
            return true;
        }
      }
    }
  }

  return false;
}

//==============================================================================
bool collisionRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1,
                      const OcTree* tree2, const OcTree::OcTreeNode* root2,
                      const AABB& root2_bv, const Transform3s& tf2,
                      CollisionCallBackBase* callback) {
  if (tf2.rotation().isIdentity())
    return collisionRecurse_(root1, tree2, root2, root2_bv, tf2.translation(),
                             callback);
  else  // has rotation
    return collisionRecurse_(root1, tree2, root2, root2_bv, tf2, callback);
}

//==============================================================================
bool distanceRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1,
                     const OcTree* tree2, const OcTree::OcTreeNode* root2,
                     const AABB& root2_bv, const Transform3s& tf2,
                     DistanceCallBackBase* callback, Scalar& min_dist) {
  if (tf2.rotation().isIdentity())
    return distanceRecurse_(root1, tree2, root2, root2_bv, tf2.translation(),
                            callback, min_dist);
  else
    return distanceRecurse_(root1, tree2, root2, root2_bv, tf2, callback,
                            min_dist);
}

#endif

//==============================================================================
bool leafCollide(CollisionObject* o1, CollisionObject* o2,
                 CollisionCallBackBase* callback) {
  if ((o1->getNodeType() == GEOM_HALFSPACE ||
       o1->getNodeType() == GEOM_PLANE) &&
      (o2->getNodeType() == GEOM_HALFSPACE ||
       o2->getNodeType() == GEOM_PLANE)) {
    // Halfspace-plane / Halfspace-plane collision, there is no need to check
    // AABBs. We can directly use the callback.
    return (*callback)(o1, o2);
  }

  bool overlap = false;
  if (o1->getNodeType() == GEOM_HALFSPACE || o1->getNodeType() == GEOM_PLANE) {
    if (o1->getNodeType() == GEOM_HALFSPACE) {
      const auto& halfspace =
          static_cast<const Halfspace&>(*(o1->collisionGeometryPtr()));
      overlap = o2->getAABB().overlap(transform(halfspace, o1->getTransform()));
    } else {
      const auto& plane =
          static_cast<const Plane&>(*(o1->collisionGeometryPtr()));
      overlap = o2->getAABB().overlap(transform(plane, o1->getTransform()));
    }
  }  //
  else if (o2->getNodeType() == GEOM_HALFSPACE ||
           o2->getNodeType() == GEOM_PLANE) {
    if (o2->getNodeType() == GEOM_HALFSPACE) {
      const auto& halfspace =
          static_cast<const Halfspace&>(*(o2->collisionGeometryPtr()));
      overlap = o1->getAABB().overlap(transform(halfspace, o2->getTransform()));
    } else {
      const auto& plane =
          static_cast<const Plane&>(*(o2->collisionGeometryPtr()));
      overlap = o1->getAABB().overlap(transform(plane, o2->getTransform()));
    }
  }  //
  else {
    overlap = o1->getAABB().overlap(o2->getAABB());
  }

  if (overlap) {
    return (*callback)(o1, o2);
  }
  return false;
}

//==============================================================================
bool nodeCollide(DynamicAABBTreeCollisionManager::DynamicAABBNode* node1,
                 DynamicAABBTreeCollisionManager::DynamicAABBNode* node2) {
  // This function assumes that at least node1 or node2 is not a leaf of the
  // tree.
  if (node1->isLeaf()) {
    CollisionObject* o1 = static_cast<CollisionObject*>(node1->data);
    if (o1->getNodeType() == GEOM_HALFSPACE ||
        o1->getNodeType() == GEOM_PLANE) {
      if (o1->getNodeType() == GEOM_HALFSPACE) {
        const auto& halfspace =
            static_cast<const Halfspace&>(*(o1->collisionGeometryPtr()));
        return node2->bv.overlap(transform(halfspace, o1->getTransform()));
      }
      const auto& plane =
          static_cast<const Plane&>(*(o1->collisionGeometryPtr()));
      return node2->bv.overlap(transform(plane, o1->getTransform()));
    }
  }

  if (node2->isLeaf()) {
    CollisionObject* o2 = static_cast<CollisionObject*>(node2->data);
    if (o2->getNodeType() == GEOM_HALFSPACE ||
        o2->getNodeType() == GEOM_PLANE) {
      if (o2->getNodeType() == GEOM_HALFSPACE) {
        const auto& halfspace =
            static_cast<const Halfspace&>(*(o2->collisionGeometryPtr()));
        return node1->bv.overlap(transform(halfspace, o2->getTransform()));
      }
      const auto& plane =
          static_cast<const Plane&>(*(o2->collisionGeometryPtr()));
      return node1->bv.overlap(transform(plane, o2->getTransform()));
    }
  }

  return node1->bv.overlap(node2->bv);
}

//==============================================================================
bool collisionRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1,
                      DynamicAABBTreeCollisionManager::DynamicAABBNode* root2,
                      CollisionCallBackBase* callback) {
  if (root1->isLeaf() && root2->isLeaf()) {
    CollisionObject* o1 = static_cast<CollisionObject*>(root1->data);
    CollisionObject* o2 = static_cast<CollisionObject*>(root2->data);
    return leafCollide(o1, o2, callback);
  }

  if (!nodeCollide(root1, root2)) {
    return false;
  }

  if (root2->isLeaf() ||
      (!root1->isLeaf() && (root1->bv.size() > root2->bv.size()))) {
    if (collisionRecurse(root1->children[0], root2, callback)) return true;
    if (collisionRecurse(root1->children[1], root2, callback)) return true;
  } else {
    if (collisionRecurse(root1, root2->children[0], callback)) return true;
    if (collisionRecurse(root1, root2->children[1], callback)) return true;
  }
  return false;
}

//==============================================================================
bool collisionRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root,
                      CollisionObject* query, CollisionCallBackBase* callback) {
  if (root->isLeaf()) {
    CollisionObject* leaf = static_cast<CollisionObject*>(root->data);
    return leafCollide(leaf, query, callback);
  }

  // Create a temporary node, attached to no tree.
  // This allows to reuse the `nodeCollide` function, which only checks for
  // overlap between the AABBs of two nodes.
  DynamicAABBTreeCollisionManager::DynamicAABBNode query_node;
  query_node.data = query;
  query_node.bv = query->getAABB();
  query_node.parent = nullptr;
  query_node.children[1] = nullptr;
  if (!nodeCollide(root, &query_node)) {
    return false;
  }

  size_t select_res =
      select(query->getAABB(), *(root->children[0]), *(root->children[1]));

  if (collisionRecurse(root->children[select_res], query, callback))
    return true;

  if (collisionRecurse(root->children[1 - select_res], query, callback))
    return true;

  return false;
}

//==============================================================================
bool selfCollisionRecurse(
    DynamicAABBTreeCollisionManager::DynamicAABBNode* root,
    CollisionCallBackBase* callback) {
  if (root->isLeaf()) return false;

  if (selfCollisionRecurse(root->children[0], callback)) return true;

  if (selfCollisionRecurse(root->children[1], callback)) return true;

  if (collisionRecurse(root->children[0], root->children[1], callback))
    return true;

  return false;
}

//==============================================================================
bool distanceRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root1,
                     DynamicAABBTreeCollisionManager::DynamicAABBNode* root2,
                     DistanceCallBackBase* callback, Scalar& min_dist) {
  if (root1->isLeaf() && root2->isLeaf()) {
    CollisionObject* root1_obj = static_cast<CollisionObject*>(root1->data);
    CollisionObject* root2_obj = static_cast<CollisionObject*>(root2->data);
    return (*callback)(root1_obj, root2_obj, min_dist);
  }

  if (root2->isLeaf() ||
      (!root1->isLeaf() && (root1->bv.size() > root2->bv.size()))) {
    Scalar d1 = root2->bv.distance(root1->children[0]->bv);
    Scalar d2 = root2->bv.distance(root1->children[1]->bv);

    if (d2 < d1) {
      if (d2 < min_dist) {
        if (distanceRecurse(root1->children[1], root2, callback, min_dist))
          return true;
      }

      if (d1 < min_dist) {
        if (distanceRecurse(root1->children[0], root2, callback, min_dist))
          return true;
      }
    } else {
      if (d1 < min_dist) {
        if (distanceRecurse(root1->children[0], root2, callback, min_dist))
          return true;
      }

      if (d2 < min_dist) {
        if (distanceRecurse(root1->children[1], root2, callback, min_dist))
          return true;
      }
    }
  } else {
    Scalar d1 = root1->bv.distance(root2->children[0]->bv);
    Scalar d2 = root1->bv.distance(root2->children[1]->bv);

    if (d2 < d1) {
      if (d2 < min_dist) {
        if (distanceRecurse(root1, root2->children[1], callback, min_dist))
          return true;
      }

      if (d1 < min_dist) {
        if (distanceRecurse(root1, root2->children[0], callback, min_dist))
          return true;
      }
    } else {
      if (d1 < min_dist) {
        if (distanceRecurse(root1, root2->children[0], callback, min_dist))
          return true;
      }

      if (d2 < min_dist) {
        if (distanceRecurse(root1, root2->children[1], callback, min_dist))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool distanceRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root,
                     CollisionObject* query, DistanceCallBackBase* callback,
                     Scalar& min_dist) {
  if (root->isLeaf()) {
    CollisionObject* root_obj = static_cast<CollisionObject*>(root->data);
    return (*callback)(root_obj, query, min_dist);
  }

  Scalar d1 = query->getAABB().distance(root->children[0]->bv);
  Scalar d2 = query->getAABB().distance(root->children[1]->bv);

  if (d2 < d1) {
    if (d2 < min_dist) {
      if (distanceRecurse(root->children[1], query, callback, min_dist))
        return true;
    }

    if (d1 < min_dist) {
      if (distanceRecurse(root->children[0], query, callback, min_dist))
        return true;
    }
  } else {
    if (d1 < min_dist) {
      if (distanceRecurse(root->children[0], query, callback, min_dist))
        return true;
    }

    if (d2 < min_dist) {
      if (distanceRecurse(root->children[1], query, callback, min_dist))
        return true;
    }
  }

  return false;
}

//==============================================================================
bool selfDistanceRecurse(DynamicAABBTreeCollisionManager::DynamicAABBNode* root,
                         DistanceCallBackBase* callback, Scalar& min_dist) {
  if (root->isLeaf()) return false;

  if (selfDistanceRecurse(root->children[0], callback, min_dist)) return true;

  if (selfDistanceRecurse(root->children[1], callback, min_dist)) return true;

  if (distanceRecurse(root->children[0], root->children[1], callback, min_dist))
    return true;

  return false;
}

}  // namespace dynamic_AABB_tree

}  // namespace detail

//==============================================================================
DynamicAABBTreeCollisionManager::DynamicAABBTreeCollisionManager() {
  tree_topdown_balance_threshold = &dtree.bu_threshold;
  tree_topdown_level = &dtree.topdown_level;
  max_tree_nonbalanced_level = 10;
  tree_incremental_balance_pass = 10;
  *tree_topdown_balance_threshold = 2;
  *tree_topdown_level = 0;
  tree_init_level = 0;
  setup_ = false;

  // from experiment, this is the optimal setting
  octree_as_geometry_collide = true;
  octree_as_geometry_distance = false;
}

//==============================================================================
void DynamicAABBTreeCollisionManager::registerObjects(
    const std::vector<CollisionObject*>& other_objs) {
  if (other_objs.empty()) return;

  if (size() > 0) {
    BroadPhaseCollisionManager::registerObjects(other_objs);
  } else {
    std::vector<DynamicAABBNode*> leaves(other_objs.size());
    table.rehash(other_objs.size());
    for (size_t i = 0, size = other_objs.size(); i < size; ++i) {
      DynamicAABBNode* node =
          new DynamicAABBNode;  // node will be managed by the dtree
      node->bv = other_objs[i]->getAABB();
      node->parent = nullptr;
      node->children[1] = nullptr;
      node->data = other_objs[i];
      table[other_objs[i]] = node;
      leaves[i] = node;
    }

    dtree.init(leaves, tree_init_level);

    setup_ = true;
  }
}

//==============================================================================
void DynamicAABBTreeCollisionManager::registerObject(CollisionObject* obj) {
  DynamicAABBNode* node = dtree.insert(obj->getAABB(), obj);
  table[obj] = node;
}

//==============================================================================
void DynamicAABBTreeCollisionManager::unregisterObject(CollisionObject* obj) {
  DynamicAABBNode* node = table[obj];
  table.erase(obj);
  dtree.remove(node);
}

//==============================================================================
void DynamicAABBTreeCollisionManager::setup() {
  if (!setup_) {
    size_t num = dtree.size();
    if (num == 0) {
      setup_ = true;
      return;
    }

    size_t height = dtree.getMaxHeight();

    if (((Scalar)height - std::log((Scalar)num) / std::log(2.0)) <
        max_tree_nonbalanced_level)
      dtree.balanceIncremental(tree_incremental_balance_pass);
    else
      dtree.balanceTopdown();

    setup_ = true;
  }
}

//==============================================================================
void DynamicAABBTreeCollisionManager::update() {
  for (auto it = table.cbegin(); it != table.cend(); ++it) {
    CollisionObject* obj = it->first;
    DynamicAABBNode* node = it->second;
    node->bv = obj->getAABB();
    if (node->bv.volume() <= 0.)
      COAL_THROW_PRETTY("The bounding volume has a negative volume.",
                        std::invalid_argument)
  }

  dtree.refit();
  setup_ = false;

  setup();
}

//==============================================================================
void DynamicAABBTreeCollisionManager::update_(CollisionObject* updated_obj) {
  const auto it = table.find(updated_obj);
  if (it != table.end()) {
    DynamicAABBNode* node = it->second;
    if (!(node->bv == updated_obj->getAABB()))
      dtree.update(node, updated_obj->getAABB());
  }
  setup_ = false;
}

//==============================================================================
void DynamicAABBTreeCollisionManager::update(CollisionObject* updated_obj) {
  update_(updated_obj);
  setup();
}

//==============================================================================
void DynamicAABBTreeCollisionManager::update(
    const std::vector<CollisionObject*>& updated_objs) {
  for (size_t i = 0, size = updated_objs.size(); i < size; ++i)
    update_(updated_objs[i]);
  setup();
}

//==============================================================================
void DynamicAABBTreeCollisionManager::clear() {
  dtree.clear();
  table.clear();
}

//==============================================================================
void DynamicAABBTreeCollisionManager::getObjects(
    std::vector<CollisionObject*>& objs) const {
  objs.resize(this->size());
  std::transform(
      table.begin(), table.end(), objs.begin(),
      std::bind(&DynamicAABBTable::value_type::first, std::placeholders::_1));
}

//==============================================================================
void DynamicAABBTreeCollisionManager::collide(
    CollisionObject* obj, CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::DynamicAABBTreeCollisionManager::collide(CollisionObject*, "
      "CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;
  switch (obj->collisionGeometry()->getNodeType()) {
#if COAL_HAVE_OCTOMAP
    case GEOM_OCTREE: {
      if (!octree_as_geometry_collide) {
        const OcTree* octree =
            static_cast<const OcTree*>(obj->collisionGeometryPtr());
        detail::dynamic_AABB_tree::collisionRecurse(
            dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(),
            obj->getTransform(), callback);
      } else
        detail::dynamic_AABB_tree::collisionRecurse(dtree.getRoot(), obj,
                                                    callback);
    } break;
#endif
    default:
      detail::dynamic_AABB_tree::collisionRecurse(dtree.getRoot(), obj,
                                                  callback);
  }
}

//==============================================================================
void DynamicAABBTreeCollisionManager::distance(
    CollisionObject* obj, DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::DynamicAABBTreeCollisionManager::distance(CollisionObject*, "
      "DistanceCallBackBase*)");
  callback->init();
  if (size() == 0) return;
  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  switch (obj->collisionGeometry()->getNodeType()) {
#if COAL_HAVE_OCTOMAP
    case GEOM_OCTREE: {
      if (!octree_as_geometry_distance) {
        const OcTree* octree =
            static_cast<const OcTree*>(obj->collisionGeometryPtr());
        detail::dynamic_AABB_tree::distanceRecurse(
            dtree.getRoot(), octree, octree->getRoot(), octree->getRootBV(),
            obj->getTransform(), callback, min_dist);
      } else
        detail::dynamic_AABB_tree::distanceRecurse(dtree.getRoot(), obj,
                                                   callback, min_dist);
    } break;
#endif
    default:
      detail::dynamic_AABB_tree::distanceRecurse(dtree.getRoot(), obj, callback,
                                                 min_dist);
  }
}

//==============================================================================
void DynamicAABBTreeCollisionManager::collide(
    CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::DynamicAABBTreeCollisionManager::collide(CollisionCallBackBase*)");
  callback->init();
  if (size() == 0) return;
  detail::dynamic_AABB_tree::selfCollisionRecurse(dtree.getRoot(), callback);
}

//==============================================================================
void DynamicAABBTreeCollisionManager::distance(
    DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::DynamicAABBTreeCollisionManager::distance(DistanceCallBackBase*)");
  callback->init();
  if (size() == 0) return;
  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  detail::dynamic_AABB_tree::selfDistanceRecurse(dtree.getRoot(), callback,
                                                 min_dist);
}

//==============================================================================
void DynamicAABBTreeCollisionManager::collide(
    BroadPhaseCollisionManager* other_manager_,
    CollisionCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::DynamicAABBTreeCollisionManager::collide("
      "BroadPhaseCollisionManager*, CollisionCallBackBase*)");
  callback->init();
  DynamicAABBTreeCollisionManager* other_manager =
      static_cast<DynamicAABBTreeCollisionManager*>(other_manager_);
  if ((size() == 0) || (other_manager->size() == 0)) return;
  detail::dynamic_AABB_tree::collisionRecurse(
      dtree.getRoot(), other_manager->dtree.getRoot(), callback);
}

//==============================================================================
void DynamicAABBTreeCollisionManager::distance(
    BroadPhaseCollisionManager* other_manager_,
    DistanceCallBackBase* callback) const {
  COAL_TRACY_ZONE_SCOPED_N(
      "coal::DynamicAABBTreeCollisionManager::distance("
      "BroadPhaseCollisionManager*, DistanceCallBackBase*)");
  callback->init();
  DynamicAABBTreeCollisionManager* other_manager =
      static_cast<DynamicAABBTreeCollisionManager*>(other_manager_);
  if ((size() == 0) || (other_manager->size() == 0)) return;
  Scalar min_dist = (std::numeric_limits<Scalar>::max)();
  detail::dynamic_AABB_tree::distanceRecurse(
      dtree.getRoot(), other_manager->dtree.getRoot(), callback, min_dist);
}

//==============================================================================
bool DynamicAABBTreeCollisionManager::empty() const { return dtree.empty(); }

//==============================================================================
size_t DynamicAABBTreeCollisionManager::size() const { return dtree.size(); }

//==============================================================================
const detail::HierarchyTree<AABB>& DynamicAABBTreeCollisionManager::getTree()
    const {
  return dtree;
}

detail::HierarchyTree<AABB>& DynamicAABBTreeCollisionManager::getTree() {
  return dtree;
}

}  // namespace coal
