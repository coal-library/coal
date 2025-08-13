/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2022-2024, Inria
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

#ifndef COAL_OCTREE_H
#define COAL_OCTREE_H

#include <algorithm>
#include <limits>

#include <octomap/octomap.h>
#include "coal/fwd.hh"
#include "coal/BV/AABB.h"
#include "coal/collision_object.h"

namespace coal {

/// @brief Octree is one type of collision geometry which can encode uncertainty
/// information in the sensor data.
class COAL_DLLAPI OcTree : public CollisionGeometry {
 protected:
  shared_ptr<const octomap::OcTree> tree;

  Scalar default_occupancy;

  Scalar occupancy_threshold;
  Scalar free_threshold;

 public:
  typedef octomap::OcTreeNode OcTreeNode;

  /// @brief construct octree with a given resolution
  explicit OcTree(Scalar resolution)
      : tree(shared_ptr<const octomap::OcTree>(
            new octomap::OcTree(resolution))) {
    default_occupancy = Scalar(tree->getOccupancyThres());

    // default occupancy/free threshold is consistent with default setting from
    // octomap
    occupancy_threshold = Scalar(tree->getOccupancyThres());
    free_threshold = 0;
  }

  /// @brief construct octree from octomap
  explicit OcTree(const shared_ptr<const octomap::OcTree>& tree_)
      : tree(tree_) {
    default_occupancy = Scalar(tree->getOccupancyThres());

    // default occupancy/free threshold is consistent with default setting from
    // octomap
    occupancy_threshold = Scalar(tree->getOccupancyThres());
    free_threshold = 0;
  }

  ///  \brief Copy constructor
  OcTree(const OcTree& other)
      : CollisionGeometry(other),
        tree(other.tree),
        default_occupancy(other.default_occupancy),
        occupancy_threshold(other.occupancy_threshold),
        free_threshold(other.free_threshold) {}

  /// \brief Clone *this into a new Octree
  OcTree* clone() const { return new OcTree(*this); }

  /// \brief Returns the tree associated to the underlying octomap OcTree.
  shared_ptr<const octomap::OcTree> getTree() const { return tree; }

  void exportAsObjFile(const std::string& filename) const;

  /// @brief compute the AABB for the octree in its local coordinate system
  void computeLocalAABB() {
    typedef Eigen::Matrix<float, 3, 1> Vec3float;
    Vec3float max_extent, min_extent;

    octomap::OcTree::iterator it =
        tree->begin((unsigned char)tree->getTreeDepth());
    octomap::OcTree::iterator end = tree->end();

    if (it == end) return;

    {
      max_extent.fill(std::numeric_limits<float>::lowest());
      min_extent.fill(std::numeric_limits<float>::max());

      for (; it != end; ++it) {
        const octomap::point3d& coord = it.getCoordinate();
        const Vec3float pos = Eigen::Map<const Vec3float>(&coord.x());
        // Account for the size of the box.
        const Vec3float max_pos =
            pos + Vec3float::Constant(float(it.getSize() / 2.));
        const Vec3float min_pos =
            pos - Vec3float::Constant(float(it.getSize() / 2.));
        max_extent.array() = max_extent.array().max(max_pos.array());
        min_extent.array() = min_extent.array().min(min_pos.array());
      }
    }

    aabb_local = AABB(min_extent.cast<Scalar>(), max_extent.cast<Scalar>());
    aabb_center = aabb_local.center();
    aabb_radius = (aabb_local.min_ - aabb_center).norm();
  }

  /// @brief get the bounding volume for the root
  AABB getRootBV() const {
    Scalar delta =
        Scalar((1 << tree->getTreeDepth()) * tree->getResolution() / 2);

    // std::cout << "octree size " << delta << std::endl;
    return AABB(Vec3s(-delta, -delta, -delta), Vec3s(delta, delta, delta));
  }

  /// @brief Returns the depth of the octree
  unsigned int getTreeDepth() const { return tree->getTreeDepth(); }

  /// @brief Returns the size of the octree
  unsigned long size() const { return tree->size(); }

  /// @brief Returns the resolution of the octree
  Scalar getResolution() const { return Scalar(tree->getResolution()); }

  /// @brief get the root node of the octree
  OcTreeNode* getRoot() const { return tree->getRoot(); }

  /// @brief whether one node is completely occupied
  bool isNodeOccupied(const OcTreeNode* node) const {
    // return tree->isNodeOccupied(node);
    return node->getOccupancy() >= occupancy_threshold;
  }

  /// @brief whether one node is completely free
  bool isNodeFree(const OcTreeNode* node) const {
    // return false; // default no definitely free node
    return node->getOccupancy() <= free_threshold;
  }

  /// @brief whether one node is uncertain
  bool isNodeUncertain(const OcTreeNode* node) const {
    return (!isNodeOccupied(node)) && (!isNodeFree(node));
  }

  /// @brief transform the octree into a bunch of boxes; uncertainty information
  /// is kept in the boxes. However, we only keep the occupied boxes (i.e., the
  /// boxes whose occupied probability is higher enough).
  std::vector<Vec6s> toBoxes() const {
    std::vector<Vec6s> boxes;
    boxes.reserve(tree->size() / 2);
    for (octomap::OcTree::iterator
             it = tree->begin((unsigned char)tree->getTreeDepth()),
             end = tree->end();
         it != end; ++it) {
      // if(tree->isNodeOccupied(*it))
      if (isNodeOccupied(&*it)) {
        Scalar x = Scalar(it.getX());
        Scalar y = Scalar(it.getY());
        Scalar z = Scalar(it.getZ());
        Scalar size = Scalar(it.getSize());
        Scalar c = Scalar((*it).getOccupancy());
        Scalar t = Scalar(tree->getOccupancyThres());

        Vec6s box;
        box << x, y, z, size, c, t;
        boxes.push_back(box);
      }
    }
    return boxes;
  }

  /// \brief Returns a byte description of *this
  std::vector<uint8_t> tobytes() const {
    typedef Eigen::Matrix<float, 3, 1> Vec3sloat;
    const size_t total_size = (tree->size() * sizeof(Scalar) * 3) / 2;
    std::vector<uint8_t> bytes;
    bytes.reserve(total_size);

    for (octomap::OcTree::iterator
             it = tree->begin((unsigned char)tree->getTreeDepth()),
             end = tree->end();
         it != end; ++it) {
      const Vec3s box_pos =
          Eigen::Map<Vec3sloat>(&it.getCoordinate().x()).cast<Scalar>();
      if (isNodeOccupied(&*it))
        std::copy(box_pos.data(), box_pos.data() + sizeof(Scalar) * 3,
                  std::back_inserter(bytes));
    }

    return bytes;
  }

  /// @brief the threshold used to decide whether one node is occupied, this is
  /// NOT the octree occupied_thresold
  Scalar getOccupancyThres() const { return occupancy_threshold; }

  /// @brief the threshold used to decide whether one node is free, this is NOT
  /// the octree free_threshold
  Scalar getFreeThres() const { return free_threshold; }

  Scalar getDefaultOccupancy() const { return default_occupancy; }

  void setCellDefaultOccupancy(Scalar d) { default_occupancy = d; }

  void setOccupancyThres(Scalar d) { occupancy_threshold = d; }

  void setFreeThres(Scalar d) { free_threshold = d; }

  /// @return ptr to child number childIdx of node
  OcTreeNode* getNodeChild(OcTreeNode* node, unsigned int childIdx) {
    return tree->getNodeChild(node, childIdx);
  }

  /// @return const ptr to child number childIdx of node
  const OcTreeNode* getNodeChild(const OcTreeNode* node,
                                 unsigned int childIdx) const {
    return tree->getNodeChild(node, childIdx);
  }

  /// @brief return true if the child at childIdx exists
  bool nodeChildExists(const OcTreeNode* node, unsigned int childIdx) const {
    return tree->nodeChildExists(node, childIdx);
  }

  /// @brief return true if node has at least one child
  bool nodeHasChildren(const OcTreeNode* node) const {
    return tree->nodeHasChildren(node);
  }

  /// @brief return object type, it is an octree
  OBJECT_TYPE getObjectType() const { return OT_OCTREE; }

  /// @brief return node type, it is an octree
  NODE_TYPE getNodeType() const { return GEOM_OCTREE; }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const OcTree* other_ptr = dynamic_cast<const OcTree*>(&_other);
    if (other_ptr == nullptr) return false;
    const OcTree& other = *other_ptr;

    return (tree.get() == other.tree.get() || toBoxes() == other.toBoxes()) &&
           default_occupancy == other.default_occupancy &&
           occupancy_threshold == other.occupancy_threshold &&
           free_threshold == other.free_threshold;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief compute the bounding volume of an octree node's i-th child
static inline void computeChildBV(const AABB& root_bv, unsigned int i,
                                  AABB& child_bv) {
  const Scalar half = Scalar(0.5);
  if (i & 1) {
    child_bv.min_[0] = (root_bv.min_[0] + root_bv.max_[0]) * half;
    child_bv.max_[0] = root_bv.max_[0];
  } else {
    child_bv.min_[0] = root_bv.min_[0];
    child_bv.max_[0] = (root_bv.min_[0] + root_bv.max_[0]) * half;
  }

  if (i & 2) {
    child_bv.min_[1] = (root_bv.min_[1] + root_bv.max_[1]) * half;
    child_bv.max_[1] = root_bv.max_[1];
  } else {
    child_bv.min_[1] = root_bv.min_[1];
    child_bv.max_[1] = (root_bv.min_[1] + root_bv.max_[1]) * half;
  }

  if (i & 4) {
    child_bv.min_[2] = (root_bv.min_[2] + root_bv.max_[2]) * half;
    child_bv.max_[2] = root_bv.max_[2];
  } else {
    child_bv.min_[2] = root_bv.min_[2];
    child_bv.max_[2] = (root_bv.min_[2] + root_bv.max_[2]) * half;
  }
}

///
/// \brief Build an OcTree from a point cloud and a given resolution
///
/// \param[in] point_cloud The input points to insert in the OcTree
/// \param[in] resolution of the octree.
///
/// \returns An OcTree that can be used for collision checking and more.
///
COAL_DLLAPI OcTreePtr_t
makeOctree(const Eigen::Matrix<Scalar, Eigen::Dynamic, 3>& point_cloud,
           const Scalar resolution);

}  // namespace coal

#endif
