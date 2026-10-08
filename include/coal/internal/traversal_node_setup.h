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

#ifndef COAL_TRAVERSAL_NODE_SETUP_H
#define COAL_TRAVERSAL_NODE_SETUP_H

/// @cond INTERNAL

#include "coal/internal/tools.h"
#include "coal/internal/traversal_node_shapes.h"

#include "coal/internal/traversal_node_bvhs.h"
#include "coal/internal/traversal_node_bvh_shape.h"

// #include <hpp/fcl/internal/traversal_node_hfields.h>
#include "coal/internal/traversal_node_hfield_shape.h"

#ifdef COAL_HAS_OCTOMAP
#include "coal/internal/traversal_node_octree.h"
#endif

#include "coal/BVH/BVH_utility.h"

namespace coal {

#ifdef COAL_HAS_OCTOMAP
/// @brief Initialize traversal node for collision between two octrees, given
/// current object transform
inline bool initialize(OcTreeCollisionTraversalNode& node, const OcTree& model1,
                       const Transform3s& tf1, const OcTree& model2,
                       const Transform3s& tf2, const OcTreeSolver* otsolver,
                       CollisionResult& result) {
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for distance between two octrees, given
/// current object transform
inline bool initialize(OcTreeDistanceTraversalNode& node, const OcTree& model1,
                       const Transform3s& tf1, const OcTree& model2,
                       const Transform3s& tf2, const OcTreeSolver* otsolver,
                       const DistanceRequest& request, DistanceResult& result)

{
  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for collision between one shape and one
/// octree, given current object transform
template <typename S>
bool initialize(ShapeOcTreeCollisionTraversalNode<S>& node, const S& model1,
                const Transform3s& tf1, const OcTree& model2,
                const Transform3s& tf2, const OcTreeSolver* otsolver,
                CollisionResult& result) {
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for collision between one octree and one
/// shape, given current object transform
template <typename S>
bool initialize(OcTreeShapeCollisionTraversalNode<S>& node,
                const OcTree& model1, const Transform3s& tf1, const S& model2,
                const Transform3s& tf2, const OcTreeSolver* otsolver,
                CollisionResult& result) {
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for distance between one shape and one
/// octree, given current object transform
template <typename S>
bool initialize(ShapeOcTreeDistanceTraversalNode<S>& node, const S& model1,
                const Transform3s& tf1, const OcTree& model2,
                const Transform3s& tf2, const OcTreeSolver* otsolver,
                const DistanceRequest& request, DistanceResult& result) {
  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for distance between one octree and one
/// shape, given current object transform
template <typename S>
bool initialize(OcTreeShapeDistanceTraversalNode<S>& node, const OcTree& model1,
                const Transform3s& tf1, const S& model2, const Transform3s& tf2,
                const OcTreeSolver* otsolver, const DistanceRequest& request,
                DistanceResult& result) {
  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for collision between one mesh and one
/// octree, given current object transform
template <typename BV>
bool initialize(MeshOcTreeCollisionTraversalNode<BV>& node,
                const BVHModel<BV>& model1, const Transform3s& tf1,
                const OcTree& model2, const Transform3s& tf2,
                const OcTreeSolver* otsolver, CollisionResult& result) {
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for collision between one octree and one
/// mesh, given current object transform
template <typename BV>
bool initialize(OcTreeMeshCollisionTraversalNode<BV>& node,
                const OcTree& model1, const Transform3s& tf1,
                const BVHModel<BV>& model2, const Transform3s& tf2,
                const OcTreeSolver* otsolver, CollisionResult& result) {
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for collision between one octree and one
/// height field, given current object transform
template <typename BV>
bool initialize(OcTreeHeightFieldCollisionTraversalNode<BV>& node,
                const OcTree& model1, const Transform3s& tf1,
                const HeightField<BV>& model2, const Transform3s& tf2,
                const OcTreeSolver* otsolver, CollisionResult& result) {
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for collision between one height field and
/// one octree, given current object transform
template <typename BV>
bool initialize(HeightFieldOcTreeCollisionTraversalNode<BV>& node,
                const HeightField<BV>& model1, const Transform3s& tf1,
                const OcTree& model2, const Transform3s& tf2,
                const OcTreeSolver* otsolver, CollisionResult& result) {
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for distance between one mesh and one
/// octree, given current object transform
template <typename BV>
bool initialize(MeshOcTreeDistanceTraversalNode<BV>& node,
                const BVHModel<BV>& model1, const Transform3s& tf1,
                const OcTree& model2, const Transform3s& tf2,
                const OcTreeSolver* otsolver, const DistanceRequest& request,
                DistanceResult& result) {
  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

/// @brief Initialize traversal node for collision between one octree and one
/// mesh, given current object transform
template <typename BV>
bool initialize(OcTreeMeshDistanceTraversalNode<BV>& node, const OcTree& model1,
                const Transform3s& tf1, const BVHModel<BV>& model2,
                const Transform3s& tf2, const OcTreeSolver* otsolver,
                const DistanceRequest& request, DistanceResult& result) {
  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.model2 = &model2;

  node.otsolver = otsolver;

  node.tf1 = tf1;
  node.tf2 = tf2;

  return true;
}

#endif

/// @brief Initialize traversal node for collision between two geometric shapes,
/// given current object transform
template <typename S1, typename S2>
bool initialize(ShapeCollisionTraversalNode<S1, S2>& node, const S1& shape1,
                const Transform3s& tf1, const S2& shape2,
                const Transform3s& tf2, const GJKSolver* nsolver,
                CollisionResult& result) {
  node.model1 = &shape1;
  node.tf1 = tf1;
  node.model2 = &shape2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.result = &result;

  return true;
}

/// @brief Initialize traversal node for collision between one mesh and one
/// shape, given current object transform
template <typename BV, typename S>
bool initialize(MeshShapeCollisionTraversalNode<BV, S>& node,
                BVHModel<BV>& model1, Transform3s& tf1, const S& model2,
                const Transform3s& tf2, const GJKSolver* nsolver,
                CollisionResult& result, bool use_refit = false,
                bool refit_bottomup = false) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  if (!tf1.isIdentity() &&
      model1.vertices.get())  // TODO(jcarpent): vectorized version
  {
    std::vector<Vec3s> vertices_transformed(model1.num_vertices);
    const std::vector<Vec3s>& model1_vertices_ = *(model1.vertices);
    for (unsigned int i = 0; i < model1.num_vertices; ++i) {
      const Vec3s& p = model1_vertices_[i];
      Vec3s new_v = tf1.transform(p);
      vertices_transformed[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model2, tf2, node.model2_bv);

  node.vertices = model1.vertices.get() ? model1.vertices->data() : NULL;
  node.tri_indices =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;

  node.result = &result;

  return true;
}

/// @brief Initialize traversal node for collision between one mesh and one
/// shape
template <typename BV, typename S>
bool initialize(MeshShapeCollisionTraversalNode<BV, S, 0>& node,
                const BVHModel<BV>& model1, const Transform3s& tf1,
                const S& model2, const Transform3s& tf2,
                const GJKSolver* nsolver, CollisionResult& result) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model2, tf2, node.model2_bv);

  node.vertices = model1.vertices.get() ? model1.vertices->data() : NULL;
  node.tri_indices =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;

  node.result = &result;

  return true;
}

/// @brief Initialize traversal node for collision between one mesh and one
/// shape, given current object transform
template <typename BV, typename S>
bool initialize(HeightFieldShapeCollisionTraversalNode<BV, S>& node,
                HeightField<BV>& model1, Transform3s& tf1, const S& model2,
                const Transform3s& tf2, const GJKSolver* nsolver,
                CollisionResult& result, bool use_refit = false,
                bool refit_bottomup = false);

/// @brief Initialize traversal node for collision between one mesh and one
/// shape
template <typename BV, typename S>
bool initialize(HeightFieldShapeCollisionTraversalNode<BV, S, 0>& node,
                const HeightField<BV>& model1, const Transform3s& tf1,
                const S& model2, const Transform3s& tf2,
                const GJKSolver* nsolver, CollisionResult& result) {
  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model2, tf2, node.model2_bv);

  node.result = &result;

  return true;
}

/// @cond IGNORE
namespace details {
template <typename S, typename BV, template <typename> class OrientedNode>
static inline bool setupShapeMeshCollisionOrientedNode(
    OrientedNode<S>& node, const S& model1, const Transform3s& tf1,
    const BVHModel<BV>& model2, const Transform3s& tf2,
    const GJKSolver* nsolver, CollisionResult& result) {
  if (model2.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model2 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model1, tf1, node.model1_bv);

  node.vertices = model2.vertices.get() ? model2.vertices->data() : NULL;
  node.tri_indices =
      model2.tri_indices.get() ? model2.tri_indices->data() : NULL;

  node.result = &result;

  return true;
}
}  // namespace details
/// @endcond

/// @brief Initialize traversal node for collision between two meshes, given the
/// current transforms
template <typename BV>
bool initialize(
    MeshCollisionTraversalNode<BV, RelativeTransformationIsIdentity>& node,
    BVHModel<BV>& model1, Transform3s& tf1, BVHModel<BV>& model2,
    Transform3s& tf2, CollisionResult& result, bool use_refit = false,
    bool refit_bottomup = false) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)
  if (model2.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model2 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  if (!tf1.isIdentity() && model1.vertices.get()) {
    std::vector<Vec3s> vertices_transformed1(model1.num_vertices);
    const std::vector<Vec3s>& model1_vertices_ = *(model1.vertices);
    for (unsigned int i = 0; i < model1.num_vertices; ++i) {
      const Vec3s& p = model1_vertices_[i];
      Vec3s new_v = tf1.transform(p);
      vertices_transformed1[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed1);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  if (!tf2.isIdentity() && model2.vertices.get()) {
    std::vector<Vec3s> vertices_transformed2(model2.num_vertices);
    const std::vector<Vec3s>& model2_vertices_ = *(model2.vertices);
    for (unsigned int i = 0; i < model2.num_vertices; ++i) {
      const Vec3s& p = model2_vertices_[i];
      Vec3s new_v = tf2.transform(p);
      vertices_transformed2[i] = new_v;
    }

    model2.beginReplaceModel();
    model2.replaceSubModel(vertices_transformed2);
    model2.endReplaceModel(use_refit, refit_bottomup);

    tf2.setIdentity();
  }

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices.get() ? model1.vertices->data() : NULL;
  node.vertices2 = model2.vertices.get() ? model2.vertices->data() : NULL;

  node.tri_indices1 =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;
  node.tri_indices2 =
      model2.tri_indices.get() ? model2.tri_indices->data() : NULL;

  node.result = &result;

  return true;
}

template <typename BV>
bool initialize(MeshCollisionTraversalNode<BV, 0>& node,
                const BVHModel<BV>& model1, const Transform3s& tf1,
                const BVHModel<BV>& model2, const Transform3s& tf2,
                CollisionResult& result) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)
  if (model2.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model2 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  node.vertices1 = model1.vertices ? model1.vertices->data() : NULL;
  node.vertices2 = model2.vertices ? model2.vertices->data() : NULL;

  node.tri_indices1 =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;
  node.tri_indices2 =
      model2.tri_indices.get() ? model2.tri_indices->data() : NULL;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.result = &result;

  node.RT.R.noalias() = tf1.getRotation().transpose() * tf2.getRotation();
  node.RT.T.noalias() = tf1.getRotation().transpose() *
                        (tf2.getTranslation() - tf1.getTranslation());
  node.RT.R_transpose.noalias() = node.RT.R.transpose();
  node.RT.inv_T.noalias() = -(node.RT.R_transpose * node.RT.T);

  return true;
}

/// @brief Initialize traversal node for distance between two geometric shapes
template <typename S1, typename S2>
bool initialize(ShapeDistanceTraversalNode<S1, S2>& node, const S1& shape1,
                const Transform3s& tf1, const S2& shape2,
                const Transform3s& tf2, const GJKSolver* nsolver,
                const DistanceRequest& request, DistanceResult& result) {
  node.request = request;
  node.result = &result;

  node.model1 = &shape1;
  node.tf1 = tf1;
  node.model2 = &shape2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  return true;
}

/// @brief Initialize traversal node for distance computation between two
/// meshes, given the current transforms
template <typename BV>
bool initialize(
    MeshDistanceTraversalNode<BV, RelativeTransformationIsIdentity>& node,
    BVHModel<BV>& model1, Transform3s& tf1, BVHModel<BV>& model2,
    Transform3s& tf2, const DistanceRequest& request, DistanceResult& result,
    bool use_refit = false, bool refit_bottomup = false) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)
  if (model2.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model2 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  if (!tf1.isIdentity() && model1.vertices.get()) {
    std::vector<Vec3s> vertices_transformed1(model1.num_vertices);
    const std::vector<Vec3s>& model1_vertices_ = *(model1.vertices);
    for (unsigned int i = 0; i < model1.num_vertices; ++i) {
      const Vec3s& p = model1_vertices_[i];
      Vec3s new_v = tf1.transform(p);
      vertices_transformed1[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed1);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  if (!tf2.isIdentity() && model2.vertices.get()) {
    std::vector<Vec3s> vertices_transformed2(model2.num_vertices);
    const std::vector<Vec3s>& model2_vertices_ = *(model2.vertices);
    for (unsigned int i = 0; i < model2.num_vertices; ++i) {
      const Vec3s& p = model2_vertices_[i];
      Vec3s new_v = tf2.transform(p);
      vertices_transformed2[i] = new_v;
    }

    model2.beginReplaceModel();
    model2.replaceSubModel(vertices_transformed2);
    model2.endReplaceModel(use_refit, refit_bottomup);

    tf2.setIdentity();
  }

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices.get() ? model1.vertices->data() : NULL;
  node.vertices2 = model2.vertices.get() ? model2.vertices->data() : NULL;

  node.tri_indices1 =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;
  node.tri_indices2 =
      model2.tri_indices.get() ? model2.tri_indices->data() : NULL;

  return true;
}

/// @brief Initialize traversal node for distance computation between two meshes
template <typename BV>
bool initialize(MeshDistanceTraversalNode<BV, 0>& node,
                const BVHModel<BV>& model1, const Transform3s& tf1,
                const BVHModel<BV>& model2, const Transform3s& tf2,
                const DistanceRequest& request, DistanceResult& result) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)
  if (model2.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model2 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;

  node.vertices1 = model1.vertices.get() ? model1.vertices->data() : NULL;
  node.vertices2 = model2.vertices.get() ? model2.vertices->data() : NULL;

  node.tri_indices1 =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;
  node.tri_indices2 =
      model2.tri_indices.get() ? model2.tri_indices->data() : NULL;

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED

  relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(),
                    tf2.getTranslation(), node.RT.R, node.RT.T);
  node.RT.R_transpose.noalias() = node.RT.R.transpose();
  node.RT.inv_T.noalias() = -(node.RT.R_transpose * node.RT.T);

  COAL_COMPILER_DIAGNOSTIC_POP

  return true;
}

/// @brief Initialize traversal node for distance computation between one mesh
/// and one shape, given the current transforms
template <typename BV, typename S>
bool initialize(MeshShapeDistanceTraversalNode<BV, S>& node,
                BVHModel<BV>& model1, Transform3s& tf1, const S& model2,
                const Transform3s& tf2, const GJKSolver* nsolver,
                const DistanceRequest& request, DistanceResult& result,
                bool use_refit = false, bool refit_bottomup = false) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  if (!tf1.isIdentity() && model1.vertices.get()) {
    const std::vector<Vec3s>& model1_vertices_ = *(model1.vertices);
    std::vector<Vec3s> vertices_transformed1(model1.num_vertices);
    for (unsigned int i = 0; i < model1.num_vertices; ++i) {
      const Vec3s& p = model1_vertices_[i];
      Vec3s new_v = tf1.transform(p);
      vertices_transformed1[i] = new_v;
    }

    model1.beginReplaceModel();
    model1.replaceSubModel(vertices_transformed1);
    model1.endReplaceModel(use_refit, refit_bottomup);

    tf1.setIdentity();
  }

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.vertices = model1.vertices.get() ? model1.vertices->data() : NULL;
  node.tri_indices =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;

  computeBV(model2, tf2, node.model2_bv);

  return true;
}

/// @cond IGNORE
namespace details {

template <typename BV, typename S, template <typename> class OrientedNode>
static inline bool setupMeshShapeDistanceOrientedNode(
    OrientedNode<S>& node, const BVHModel<BV>& model1, const Transform3s& tf1,
    const S& model2, const Transform3s& tf2, const GJKSolver* nsolver,
    const DistanceRequest& request, DistanceResult& result) {
  if (model1.getModelType() != BVH_MODEL_TRIANGLES)
    COAL_THROW_PRETTY(
        "model1 should be of type BVHModelType::BVH_MODEL_TRIANGLES.",
        std::invalid_argument)

  node.request = request;
  node.result = &result;

  node.model1 = &model1;
  node.tf1 = tf1;
  node.model2 = &model2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  computeBV(model2, tf2, node.model2_bv);

  node.vertices = model1.vertices.get() ? model1.vertices->data() : NULL;
  node.tri_indices =
      model1.tri_indices.get() ? model1.tri_indices->data() : NULL;

  return true;
}
}  // namespace details
/// @endcond

/// @brief Initialize traversal node for distance computation between one mesh
/// and one shape, specialized for RSS type
template <typename S>
bool initialize(MeshShapeDistanceTraversalNodeRSS<S>& node,
                const BVHModel<RSS>& model1, const Transform3s& tf1,
                const S& model2, const Transform3s& tf2,
                const GJKSolver* nsolver, const DistanceRequest& request,
                DistanceResult& result) {
  return details::setupMeshShapeDistanceOrientedNode(
      node, model1, tf1, model2, tf2, nsolver, request, result);
}

/// @brief Initialize traversal node for distance computation between one mesh
/// and one shape, specialized for kIOS type
template <typename S>
bool initialize(MeshShapeDistanceTraversalNodekIOS<S>& node,
                const BVHModel<kIOS>& model1, const Transform3s& tf1,
                const S& model2, const Transform3s& tf2,
                const GJKSolver* nsolver, const DistanceRequest& request,
                DistanceResult& result) {
  return details::setupMeshShapeDistanceOrientedNode(
      node, model1, tf1, model2, tf2, nsolver, request, result);
}

/// @brief Initialize traversal node for distance computation between one mesh
/// and one shape, specialized for OBBRSS type
template <typename S>
bool initialize(MeshShapeDistanceTraversalNodeOBBRSS<S>& node,
                const BVHModel<OBBRSS>& model1, const Transform3s& tf1,
                const S& model2, const Transform3s& tf2,
                const GJKSolver* nsolver, const DistanceRequest& request,
                DistanceResult& result) {
  return details::setupMeshShapeDistanceOrientedNode(
      node, model1, tf1, model2, tf2, nsolver, request, result);
}

}  // namespace coal

/// @endcond

#endif
