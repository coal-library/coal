/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2020-2022, INRIA
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

#ifndef COAL_BVH_MODEL_H
#define COAL_BVH_MODEL_H

#include "coal/fwd.hh"
#include "coal/collision_object.h"
#include "coal/BVH/BVH_internal.h"
#include "coal/BV/BV_node.h"

#include <vector>
#include <memory>
#include <iostream>

namespace coal {

/// @addtogroup Construction_Of_BVH
/// @{

template <typename _IndexType>
class ConvexBaseTpl;

template <typename BV>
class BVFitter;
template <typename BV>
class BVSplitter;

/// @brief A base class describing the bounding hierarchy of a mesh model or a
/// point cloud model (which is viewed as a degraded version of mesh)
class COAL_DLLAPI BVHModelBase : public CollisionGeometry {
 public:
  typedef typename Triangle32::IndexType IndexType;
  typedef ConvexBaseTpl<IndexType> ConvexType;

  /// @brief Geometry point data
  std::shared_ptr<std::vector<Vec3s>> vertices;

  /// @brief Geometry triangle index data, will be NULL for point clouds
  std::shared_ptr<std::vector<Triangle32>> tri_indices;

  /// @brief Geometry point data in previous frame
  std::shared_ptr<std::vector<Vec3s>> prev_vertices;

  /// @brief Number of triangles
  unsigned int num_tris;

  /// @brief Number of points
  unsigned int num_vertices;

  /// @brief The state of BVH building process
  BVHBuildState build_state;

  /// @brief Convex<Triangle> representation of this object
  // TODO: deprecated
  shared_ptr<ConvexType> convex;

  /// @brief Model type described by the instance
  BVHModelType getModelType() const {
    if (num_tris && num_vertices)
      return BVH_MODEL_TRIANGLES;
    else if (num_vertices)
      return BVH_MODEL_POINTCLOUD;
    else
      return BVH_MODEL_UNKNOWN;
  }

  /// @brief Constructing an empty BVH
  BVHModelBase();

  /// @brief copy from another BVH
  BVHModelBase(const BVHModelBase& other);

  /// @brief deconstruction, delete mesh data related.
  virtual ~BVHModelBase() {}

  /// @brief Get the object type: it is a BVH
  OBJECT_TYPE getObjectType() const { return OT_BVH; }

  /// @brief Compute the AABB for the BVH, used for broad-phase collision
  void computeLocalAABB();

  /// @brief Begin a new BVH model
  int beginModel(unsigned int num_tris = 0, unsigned int num_vertices = 0);

  /// @brief Add one point in the new BVH model
  int addVertex(const Vec3s& p);

  /// @brief Add points in the new BVH model
  int addVertices(const MatrixX3s& points);

  /// @brief Add triangles in the new BVH model
  int addTriangles(const Matrixx3i& triangles);

  /// @brief Add one triangle in the new BVH model
  int addTriangle(const Vec3s& p1, const Vec3s& p2, const Vec3s& p3);

  /// @brief Add a set of triangles in the new BVH model
  int addSubModel(const std::vector<Vec3s>& ps,
                  const std::vector<Triangle32>& ts);

  /// @brief Add a set of points in the new BVH model
  int addSubModel(const std::vector<Vec3s>& ps);

  /// @brief End BVH model construction, will build the bounding volume
  /// hierarchy
  int endModel();

  /// @brief Replace the geometry information of current frame (i.e. should have
  /// the same mesh topology with the previous frame)
  int beginReplaceModel();

  /// @brief Replace one point in the old BVH model
  int replaceVertex(const Vec3s& p);

  /// @brief Replace one triangle in the old BVH model
  int replaceTriangle(const Vec3s& p1, const Vec3s& p2, const Vec3s& p3);

  /// @brief Replace a set of points in the old BVH model
  int replaceSubModel(const std::vector<Vec3s>& ps);

  /// @brief End BVH model replacement, will also refit or rebuild the bounding
  /// volume hierarchy
  int endReplaceModel(bool refit = true, bool bottomup = true);

  /// @brief Replace the geometry information of current frame (i.e. should have
  /// the same mesh topology with the previous frame). The current frame will be
  /// saved as the previous frame in prev_vertices.
  int beginUpdateModel();

  /// @brief Update one point in the old BVH model
  int updateVertex(const Vec3s& p);

  /// @brief Update one triangle in the old BVH model
  int updateTriangle(const Vec3s& p1, const Vec3s& p2, const Vec3s& p3);

  /// @brief Update a set of points in the old BVH model
  int updateSubModel(const std::vector<Vec3s>& ps);

  /// @brief End BVH model update, will also refit or rebuild the bounding
  /// volume hierarchy
  int endUpdateModel(bool refit = true, bool bottomup = true);

  /// @brief Build this \ref Convex "Convex<Triangle>" representation of this
  /// model. The result is stored in attribute \ref convex. \note this only
  /// takes the points of this model. It does not check that the
  ///       object is convex. It does not compute a convex hull.
  void buildConvexRepresentation(bool share_memory);

  /// @brief Build a convex hull
  /// and store it in attribute \ref convex.
  /// \param keepTriangle whether the convex should be triangulated.
  /// \param qhullCommand see \ref ConvexBase::convexHull.
  /// \return \c true if this object is convex, hence the convex hull represents
  ///         the same object.
  /// \sa ConvexBase::convexHull
  /// \warning At the moment, the return value only checks whether there are as
  ///          many points in the convex hull as in the original object. This is
  ///          neither necessary (duplicated vertices get merged) nor sufficient
  ///          (think of a U with 4 vertices and 3 edges).
  bool buildConvexHull(bool keepTriangle, const char* qhullCommand = NULL);

  virtual int memUsage(const bool msg = false) const = 0;

  /// @brief This is a special acceleration: BVH_model default stores the BV's
  /// transform in world coordinate. However, we can also store each BV's
  /// transform related to its parent BV node. When traversing the BVH, this can
  /// save one matrix transformation.
  virtual void makeParentRelative() = 0;

  Vec3s computeCOM() const {
    Scalar vol = 0;
    Vec3s com(0, 0, 0);
    if (!(vertices.get())) {
      std::cerr << "BVH Error in `computeCOM`! The BVHModel does not contain "
                   "vertices."
                << std::endl;
      return com;
    }
    const std::vector<Vec3s>& vertices_ = *vertices;
    if (!(tri_indices.get())) {
      std::cerr << "BVH Error in `computeCOM`! The BVHModel does not contain "
                   "triangles."
                << std::endl;
      return com;
    }
    const std::vector<Triangle32>& tri_indices_ = *tri_indices;

    for (unsigned int i = 0; i < num_tris; ++i) {
      const Triangle32& tri = tri_indices_[i];
      Scalar d_six_vol =
          (vertices_[tri[0]].cross(vertices_[tri[1]])).dot(vertices_[tri[2]]);
      vol += d_six_vol;
      com += (vertices_[tri[0]] + vertices_[tri[1]] + vertices_[tri[2]]) *
             d_six_vol;
    }

    return com / (vol * 4);
  }

  Scalar computeVolume() const {
    Scalar vol = 0;
    if (!(vertices.get())) {
      std::cerr << "BVH Error in `computeCOM`! The BVHModel does not contain "
                   "vertices."
                << std::endl;
      return vol;
    }
    const std::vector<Vec3s>& vertices_ = *vertices;
    if (!(tri_indices.get())) {
      std::cerr << "BVH Error in `computeCOM`! The BVHModel does not contain "
                   "triangles."
                << std::endl;
      return vol;
    }
    const std::vector<Triangle32>& tri_indices_ = *tri_indices;
    for (unsigned int i = 0; i < num_tris; ++i) {
      const Triangle32& tri = tri_indices_[i];
      Scalar d_six_vol =
          (vertices_[tri[0]].cross(vertices_[tri[1]])).dot(vertices_[tri[2]]);
      vol += d_six_vol;
    }

    return vol / 6;
  }

  Matrix3s computeMomentofInertia() const {
    Matrix3s C = Matrix3s::Zero();

    Matrix3s C_canonical;
    C_canonical << Scalar(1 / 60.0),  //
        Scalar(1 / 120.0),            //
        Scalar(1 / 120.0),            //
        Scalar(1 / 120.0),            //
        Scalar(1 / 60.0),             //
        Scalar(1 / 120.0),            //
        Scalar(1 / 120.0),            //
        Scalar(1 / 120.0),            //
        Scalar(1 / 60.0);

    if (!(vertices.get())) {
      std::cerr << "BVH Error in `computeMomentofInertia`! The BVHModel does "
                   "not contain vertices."
                << std::endl;
      return C;
    }
    const std::vector<Vec3s>& vertices_ = *vertices;
    if (!(vertices.get())) {
      std::cerr << "BVH Error in `computeMomentofInertia`! The BVHModel does "
                   "not contain vertices."
                << std::endl;
      return C;
    }
    const std::vector<Triangle32>& tri_indices_ = *tri_indices;
    for (unsigned int i = 0; i < num_tris; ++i) {
      const Triangle32& tri = tri_indices_[i];
      const Vec3s& v1 = vertices_[tri[0]];
      const Vec3s& v2 = vertices_[tri[1]];
      const Vec3s& v3 = vertices_[tri[2]];
      Matrix3s A;
      A << v1.transpose(), v2.transpose(), v3.transpose();
      C += A.derived().transpose() * C_canonical * A * (v1.cross(v2)).dot(v3);
    }

    return C.trace() * Matrix3s::Identity() - C;
  }

 protected:
  virtual void deleteBVs() = 0;
  virtual bool allocateBVs() = 0;

  /// @brief Build the bounding volume hierarchy
  virtual int buildTree() = 0;

  /// @brief Refit the bounding volume hierarchy
  virtual int refitTree(bool bottomup) = 0;

  unsigned int num_tris_allocated;
  unsigned int num_vertices_allocated;
  unsigned int num_vertex_updated;  /// for ccd vertex update

 protected:
  /// \brief Comparison operators
  virtual bool isEqual(const CollisionGeometry& other) const;
};

/// @brief A class describing the bounding hierarchy of a mesh model or a point
/// cloud model (which is viewed as a degraded version of mesh) \tparam BV one
/// of the bounding volume class in \ref Bounding_Volume.
template <typename BV>
class COAL_DLLAPI BVHModel : public BVHModelBase {
  typedef BVHModelBase Base;

 public:
  using bv_node_vector_t =
      std::vector<BVNode<BV>, Eigen::aligned_allocator<BVNode<BV>>>;

  /// @brief Split rule to split one BV node into two children
  shared_ptr<BVSplitter<BV>> bv_splitter;

  /// @brief Fitting rule to fit a BV node to a set of geometry primitives
  shared_ptr<BVFitter<BV>> bv_fitter;

  /// @brief Default constructor to build an empty BVH
  BVHModel();

  /// @brief Copy constructor from another BVH
  ///
  /// \param[in] other BVHModel to copy.
  ///
  BVHModel(const BVHModel& other);

  /// @brief Clone *this into a new BVHModel
  virtual BVHModel<BV>* clone() const { return new BVHModel(*this); }

  /// @brief deconstruction, delete mesh data related.
  ~BVHModel() {}

  /// @brief We provide getBV() and getNumBVs() because BVH may be compressed
  /// (in future), so we must provide some flexibility here

  /// @brief Access the bv giving the its index
  const BVNode<BV>& getBV(unsigned int i) const {
    assert(i < num_bvs);
    return (*bvs)[i];
  }

  /// @brief Access the bv giving the its index
  BVNode<BV>& getBV(unsigned int i) {
    assert(i < num_bvs);
    return (*bvs)[i];
  }

  /// @brief Get the number of bv in the BVH
  unsigned int getNumBVs() const { return num_bvs; }

  /// @brief Get the BV type: default is unknown
  NODE_TYPE getNodeType() const { return BV_UNKNOWN; }

  /// @brief Check the number of memory used
  int memUsage(const bool msg) const;

  /// @brief This is a special acceleration: BVH_model default stores the BV's
  /// transform in world coordinate. However, we can also store each BV's
  /// transform related to its parent BV node. When traversing the BVH, this can
  /// save one matrix transformation.
  void makeParentRelative() {
    Matrix3s I(Matrix3s::Identity());
    makeParentRelativeRecurse(0, I, Vec3s::Zero());
  }

 protected:
  void deleteBVs();
  bool allocateBVs();

  unsigned int num_bvs_allocated;
  std::shared_ptr<std::vector<unsigned int>> primitive_indices;

  /// @brief Bounding volume hierarchy
  std::shared_ptr<bv_node_vector_t> bvs;

  /// @brief Number of BV nodes in bounding volume hierarchy
  unsigned int num_bvs;

  /// @brief Build the bounding volume hierarchy
  int buildTree();

  /// @brief Refit the bounding volume hierarchy
  int refitTree(bool bottomup);

  /// @brief Refit the bounding volume hierarchy in a top-down way (slow but
  /// more compact)
  int refitTree_topdown();

  /// @brief Refit the bounding volume hierarchy in a bottom-up way (fast but
  /// less compact)
  int refitTree_bottomup();

  /// @brief Recursive kernel for hierarchy construction
  int recursiveBuildTree(int bv_id, unsigned int first_primitive,
                         unsigned int num_primitives);

  /// @brief Recursive kernel for bottomup refitting
  int recursiveRefitTree_bottomup(int bv_id);

  /// @ recursively compute each bv's transform related to its parent. For
  /// default BV, only the translation works. For oriented BV (OBB, RSS,
  /// OBBRSS), special implementation is provided.
  void makeParentRelativeRecurse(int bv_id, Matrix3s& parent_axes,
                                 const Vec3s& parent_c) {
    bv_node_vector_t& bvs_ = *bvs;
    if (!bvs_[static_cast<size_t>(bv_id)].isLeaf()) {
      makeParentRelativeRecurse(bvs_[static_cast<size_t>(bv_id)].first_child,
                                parent_axes,
                                bvs_[static_cast<size_t>(bv_id)].getCenter());

      makeParentRelativeRecurse(
          bvs_[static_cast<size_t>(bv_id)].first_child + 1, parent_axes,
          bvs_[static_cast<size_t>(bv_id)].getCenter());
    }

    bvs_[static_cast<size_t>(bv_id)].bv =
        translate(bvs_[static_cast<size_t>(bv_id)].bv, -parent_c);
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const BVHModel* other_ptr = dynamic_cast<const BVHModel*>(&_other);
    if (other_ptr == nullptr) return false;
    const BVHModel& other = *other_ptr;

    bool res = Base::isEqual(other);
    if (!res) return false;

    // unsigned int other_num_primitives = 0;
    // if(other.primitive_indices)
    // {

    //   switch(other.getModelType())
    //   {
    //     case BVH_MODEL_TRIANGLES:
    //       other_num_primitives = num_tris;
    //       break;
    //     case BVH_MODEL_POINTCLOUD:
    //       other_num_primitives = num_vertices;
    //       break;
    //     default:
    //       ;
    //   }
    // }

    //    unsigned int num_primitives = 0;
    //    if(primitive_indices)
    //    {
    //
    //      switch(other.getModelType())
    //      {
    //        case BVH_MODEL_TRIANGLES:
    //          num_primitives = num_tris;
    //          break;
    //        case BVH_MODEL_POINTCLOUD:
    //          num_primitives = num_vertices;
    //          break;
    //        default:
    //          ;
    //      }
    //    }
    //
    //    if(num_primitives != other_num_primitives)
    //      return false;
    //
    //    for(int k = 0; k < num_primitives; ++k)
    //    {
    //      if(primitive_indices[k] != other.primitive_indices[k])
    //        return false;
    //    }

    if (num_bvs != other.num_bvs) return false;

    if ((!(bvs.get()) && other.bvs.get()) || (bvs.get() && !(other.bvs.get())))
      return false;
    if (bvs.get() && other.bvs.get()) {
      const bv_node_vector_t& bvs_ = *bvs;
      const bv_node_vector_t& other_bvs_ = *(other.bvs);
      for (unsigned int k = 0; k < num_bvs; ++k) {
        if (bvs_[k] != other_bvs_[k]) return false;
      }
    }

    return true;
  }
};

/// @}

template <>
void BVHModel<OBB>::makeParentRelativeRecurse(int bv_id, Matrix3s& parent_axes,
                                              const Vec3s& parent_c);

template <>
void BVHModel<RSS>::makeParentRelativeRecurse(int bv_id, Matrix3s& parent_axes,
                                              const Vec3s& parent_c);

template <>
void BVHModel<OBBRSS>::makeParentRelativeRecurse(int bv_id,
                                                 Matrix3s& parent_axes,
                                                 const Vec3s& parent_c);

/// @brief Specialization of getNodeType() for BVHModel with different BV types
template <>
NODE_TYPE BVHModel<AABB>::getNodeType() const;

template <>
NODE_TYPE BVHModel<OBB>::getNodeType() const;

template <>
NODE_TYPE BVHModel<RSS>::getNodeType() const;

template <>
NODE_TYPE BVHModel<kIOS>::getNodeType() const;

template <>
NODE_TYPE BVHModel<OBBRSS>::getNodeType() const;

template <>
NODE_TYPE BVHModel<KDOP<16>>::getNodeType() const;

template <>
NODE_TYPE BVHModel<KDOP<18>>::getNodeType() const;

template <>
NODE_TYPE BVHModel<KDOP<24>>::getNodeType() const;

}  // namespace coal

#endif
