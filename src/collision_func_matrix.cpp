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

#include "coal/collision_func_matrix.h"

#include "coal/internal/traversal_node_setup.h"
#include <../src/collision_node.h>
#include "coal/narrowphase/narrowphase.h"
#include "coal/internal/shape_shape_func.h"
#include "coal/shape/geometric_shapes_traits.h"
#include <../src/traits_traversal.h>

namespace coal {

#ifdef COAL_HAS_OCTOMAP

template <typename TypeA, typename TypeB>
std::size_t OctreeCollide(const CollisionGeometry* o1, const Transform3s& tf1,
                          const CollisionGeometry* o2, const Transform3s& tf2,
                          const GJKSolver* nsolver,
                          const CollisionRequest& request,
                          CollisionResult& result) {
  if (request.isSatisfied(result)) return result.numContacts();

  if (request.security_margin < 0)
    COAL_THROW_PRETTY("Negative security margin are not handled yet for Octree",
                      std::invalid_argument);

  typename TraversalTraitsCollision<TypeA, TypeB>::CollisionTraversal_t node(
      request);
  const TypeA* obj1 = dynamic_cast<const TypeA*>(o1);
  const TypeB* obj2 = dynamic_cast<const TypeB*>(o2);
  OcTreeSolver otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, result);
  collide(&node, request, result);

  return result.numContacts();
}

#endif

namespace details {
template <typename T_BVH, typename T_SH>
struct bvh_shape_traits {
  enum { Options = RelativeTransformationIsIdentity };
};
#define BVH_SHAPE_DEFAULT_TO_ORIENTED(bv) \
  template <typename T_SH>                \
  struct bvh_shape_traits<bv, T_SH> {     \
    enum { Options = 0 };                 \
  }
BVH_SHAPE_DEFAULT_TO_ORIENTED(OBB);
BVH_SHAPE_DEFAULT_TO_ORIENTED(RSS);
BVH_SHAPE_DEFAULT_TO_ORIENTED(kIOS);
BVH_SHAPE_DEFAULT_TO_ORIENTED(OBBRSS);
#undef BVH_SHAPE_DEFAULT_TO_ORIENTED
}  // namespace details

/// \tparam _Options takes two values.
///         - RelativeTransformationIsIdentity if object 1 should be moved
///           into the frame of object 2 before computing collisions.
///         - 0 if the query should be made with non-aligned object frames.
template <typename T_BVH, typename T_SH,
          int _Options = details::bvh_shape_traits<T_BVH, T_SH>::Options>
struct COAL_LOCAL BVHShapeCollider {
  static std::size_t collide(const CollisionGeometry* o1,
                             const Transform3s& tf1,
                             const CollisionGeometry* o2,
                             const Transform3s& tf2, const GJKSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result) {
    if (request.isSatisfied(result)) return result.numContacts();

    if (request.security_margin < 0)
      COAL_THROW_PRETTY(
          "Negative security margin are not handled yet for BVHModel",
          std::invalid_argument);

    if (_Options & RelativeTransformationIsIdentity)
      return aligned(o1, tf1, o2, tf2, nsolver, request, result);
    else
      return oriented(o1, tf1, o2, tf2, nsolver, request, result);
  }  // namespace coal

  static std::size_t aligned(const CollisionGeometry* o1,
                             const Transform3s& tf1,
                             const CollisionGeometry* o2,
                             const Transform3s& tf2, const GJKSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result) {
    if (request.isSatisfied(result)) return result.numContacts();

    MeshShapeCollisionTraversalNode<T_BVH, T_SH,
                                    RelativeTransformationIsIdentity>
        node(request);
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3s tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, result);
    coal::collide(&node, request, result);

    delete obj1_tmp;
    return result.numContacts();
  }

  static std::size_t oriented(const CollisionGeometry* o1,
                              const Transform3s& tf1,
                              const CollisionGeometry* o2,
                              const Transform3s& tf2, const GJKSolver* nsolver,
                              const CollisionRequest& request,
                              CollisionResult& result) {
    if (request.isSatisfied(result)) return result.numContacts();

    MeshShapeCollisionTraversalNode<T_BVH, T_SH, 0> node(request);
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, result);
    coal::collide(&node, request, result);
    return result.numContacts();
  }
};

/// @brief Collider functor for HeightField data structure
/// \tparam _Options takes two values.
///         - RelativeTransformationIsIdentity if object 1 should be moved
///           into the frame of object 2 before computing collisions.
///         - 0 if the query should be made with non-aligned object frames.
template <typename BV, typename Shape>
struct COAL_LOCAL HeightFieldShapeCollider {
  typedef HeightField<BV> HF;

  static std::size_t collide(const CollisionGeometry* o1,
                             const Transform3s& tf1,
                             const CollisionGeometry* o2,
                             const Transform3s& tf2, const GJKSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result) {
    if (request.isSatisfied(result)) return result.numContacts();

    const HF& height_field = static_cast<const HF&>(*o1);
    const Shape& shape = static_cast<const Shape&>(*o2);

    HeightFieldShapeCollisionTraversalNode<BV, Shape, 0> node(request);

    initialize(node, height_field, tf1, shape, tf2, nsolver, result);
    coal::collide(&node, request, result);
    return result.numContacts();
  }
};

namespace details {
template <typename OrientedMeshCollisionTraversalNode, typename T_BVH>
std::size_t orientedMeshCollide(const CollisionGeometry* o1,
                                const Transform3s& tf1,
                                const CollisionGeometry* o2,
                                const Transform3s& tf2,
                                const CollisionRequest& request,
                                CollisionResult& result) {
  if (request.isSatisfied(result)) return result.numContacts();

  OrientedMeshCollisionTraversalNode node(request);
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, result);
  collide(&node, request, result);

  return result.numContacts();
}

}  // namespace details

template <typename T_BVH>
std::size_t BVHCollide(const CollisionGeometry* o1, const Transform3s& tf1,
                       const CollisionGeometry* o2, const Transform3s& tf2,
                       const CollisionRequest& request,
                       CollisionResult& result) {
  if (request.isSatisfied(result)) return result.numContacts();

  // TODO(louis): each time we call collide on BVH-BVH, we:
  //   - Allocate 2 new BVHs
  //   - Copy and transform the vertices in both BVHs so that they are in the
  //     same frame
  //   - Recompute BVs of the BVH structure
  // -> all that just to avoid doing (a few) rotations/translations of AABBs.
  // Is it really worth it?
  MeshCollisionTraversalNode<T_BVH> node(request);
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3s tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3s tf2_tmp = tf2;

  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, result);
  coal::collide(&node, request, result);

  delete obj1_tmp;
  delete obj2_tmp;

  return result.numContacts();
}

template <>
std::size_t BVHCollide<OBB>(const CollisionGeometry* o1, const Transform3s& tf1,
                            const CollisionGeometry* o2, const Transform3s& tf2,
                            const CollisionRequest& request,
                            CollisionResult& result) {
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBB, OBB>(
      o1, tf1, o2, tf2, request, result);
}

template <>
std::size_t BVHCollide<OBBRSS>(const CollisionGeometry* o1,
                               const Transform3s& tf1,
                               const CollisionGeometry* o2,
                               const Transform3s& tf2,
                               const CollisionRequest& request,
                               CollisionResult& result) {
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBBRSS, OBBRSS>(
      o1, tf1, o2, tf2, request, result);
}

template <>
std::size_t BVHCollide<kIOS>(const CollisionGeometry* o1,
                             const Transform3s& tf1,
                             const CollisionGeometry* o2,
                             const Transform3s& tf2,
                             const CollisionRequest& request,
                             CollisionResult& result) {
  return details::orientedMeshCollide<MeshCollisionTraversalNodekIOS, kIOS>(
      o1, tf1, o2, tf2, request, result);
}

template <typename T_BVH>
std::size_t BVHCollide(const CollisionGeometry* o1, const Transform3s& tf1,
                       const CollisionGeometry* o2, const Transform3s& tf2,
                       const GJKSolver* /*nsolver*/,
                       const CollisionRequest& request,
                       CollisionResult& result) {
  return BVHCollide<T_BVH>(o1, tf1, o2, tf2, request, result);
}

CollisionFunctionMatrix::CollisionFunctionMatrix() {
  for (int i = 0; i < NODE_COUNT; ++i) {
    for (int j = 0; j < NODE_COUNT; ++j) collision_matrix[i][j] = NULL;
  }

  // clang-format off
  collision_matrix[GEOM_BOX][GEOM_BOX]                    = &ShapeShapeCollide<Box, Box>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE]                 = &ShapeShapeCollide<Box, Sphere>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE]                = &ShapeShapeCollide<Box, Capsule>;
  collision_matrix[GEOM_BOX][GEOM_CONE]                   = &ShapeShapeCollide<Box, Cone>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER]               = &ShapeShapeCollide<Box, Cylinder>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX16]               = &ShapeShapeCollide<Box, ConvexBase16>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX32]               = &ShapeShapeCollide<Box, ConvexBase32>;
  collision_matrix[GEOM_BOX][GEOM_PLANE]                  = &ShapeShapeCollide<Box, Plane>;
  collision_matrix[GEOM_BOX][GEOM_HALFSPACE]              = &ShapeShapeCollide<Box, Halfspace>;
  collision_matrix[GEOM_BOX][GEOM_ELLIPSOID]              = &ShapeShapeCollide<Box, Ellipsoid>;
  collision_matrix[GEOM_BOX][GEOM_TRIANGLE]               = &ShapeShapeCollide<Box, TriangleP>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX]                 = &ShapeShapeCollide<Sphere, Box>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE]              = &ShapeShapeCollide<Sphere, Sphere>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE]             = &ShapeShapeCollide<Sphere, Capsule>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE]                = &ShapeShapeCollide<Sphere, Cone>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER]            = &ShapeShapeCollide<Sphere, Cylinder>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX16]            = &ShapeShapeCollide<Sphere, ConvexBase16>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX32]            = &ShapeShapeCollide<Sphere, ConvexBase32>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE]               = &ShapeShapeCollide<Sphere, Plane>;
  collision_matrix[GEOM_SPHERE][GEOM_HALFSPACE]           = &ShapeShapeCollide<Sphere, Halfspace>;
  collision_matrix[GEOM_SPHERE][GEOM_ELLIPSOID]           = &ShapeShapeCollide<Sphere, Ellipsoid>;
  collision_matrix[GEOM_SPHERE][GEOM_TRIANGLE]            = &ShapeShapeCollide<Sphere, TriangleP>;

  collision_matrix[GEOM_ELLIPSOID][GEOM_BOX]              = &ShapeShapeCollide<Ellipsoid, Box>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_SPHERE]           = &ShapeShapeCollide<Ellipsoid, Sphere>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE]          = &ShapeShapeCollide<Ellipsoid, Capsule>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONE]             = &ShapeShapeCollide<Ellipsoid, Cone>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER]         = &ShapeShapeCollide<Ellipsoid, Cylinder>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONVEX16]         = &ShapeShapeCollide<Ellipsoid, ConvexBase16>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_CONVEX32]         = &ShapeShapeCollide<Ellipsoid, ConvexBase32>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_PLANE]            = &ShapeShapeCollide<Ellipsoid, Plane>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE]        = &ShapeShapeCollide<Ellipsoid, Halfspace>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID]        = &ShapeShapeCollide<Ellipsoid, Ellipsoid>;
  collision_matrix[GEOM_ELLIPSOID][GEOM_TRIANGLE]         = &ShapeShapeCollide<Ellipsoid, TriangleP>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX]                = &ShapeShapeCollide<Capsule, Box>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE]             = &ShapeShapeCollide<Capsule, Sphere>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE]            = &ShapeShapeCollide<Capsule, Capsule>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE]               = &ShapeShapeCollide<Capsule, Cone>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER]           = &ShapeShapeCollide<Capsule, Cylinder>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX16]           = &ShapeShapeCollide<Capsule, ConvexBase16>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX32]           = &ShapeShapeCollide<Capsule, ConvexBase32>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE]              = &ShapeShapeCollide<Capsule, Plane>;
  collision_matrix[GEOM_CAPSULE][GEOM_HALFSPACE]          = &ShapeShapeCollide<Capsule, Halfspace>;
  collision_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID]          = &ShapeShapeCollide<Capsule, Ellipsoid>;
  collision_matrix[GEOM_CAPSULE][GEOM_TRIANGLE]           = &ShapeShapeCollide<Capsule, TriangleP>;

  collision_matrix[GEOM_CONE][GEOM_BOX]                   = &ShapeShapeCollide<Cone, Box>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE]                = &ShapeShapeCollide<Cone, Sphere>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE]               = &ShapeShapeCollide<Cone, Capsule>;
  collision_matrix[GEOM_CONE][GEOM_CONE]                  = &ShapeShapeCollide<Cone, Cone>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER]              = &ShapeShapeCollide<Cone, Cylinder>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX16]              = &ShapeShapeCollide<Cone, ConvexBase16>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX32]              = &ShapeShapeCollide<Cone, ConvexBase32>;
  collision_matrix[GEOM_CONE][GEOM_PLANE]                 = &ShapeShapeCollide<Cone, Plane>;
  collision_matrix[GEOM_CONE][GEOM_HALFSPACE]             = &ShapeShapeCollide<Cone, Halfspace>;
  collision_matrix[GEOM_CONE][GEOM_ELLIPSOID]             = &ShapeShapeCollide<Cone, Ellipsoid>;
  collision_matrix[GEOM_CONE][GEOM_TRIANGLE]              = &ShapeShapeCollide<Cone, TriangleP>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX]               = &ShapeShapeCollide<Cylinder, Box>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE]            = &ShapeShapeCollide<Cylinder, Sphere>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE]           = &ShapeShapeCollide<Cylinder, Capsule>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE]              = &ShapeShapeCollide<Cylinder, Cone>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER]          = &ShapeShapeCollide<Cylinder, Cylinder>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX16]          = &ShapeShapeCollide<Cylinder, ConvexBase16>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX32]          = &ShapeShapeCollide<Cylinder, ConvexBase32>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE]             = &ShapeShapeCollide<Cylinder, Plane>;
  collision_matrix[GEOM_CYLINDER][GEOM_HALFSPACE]         = &ShapeShapeCollide<Cylinder, Halfspace>;
  collision_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID]         = &ShapeShapeCollide<Cylinder, Ellipsoid>;
  collision_matrix[GEOM_CYLINDER][GEOM_TRIANGLE]          = &ShapeShapeCollide<Cylinder, TriangleP>;

  collision_matrix[GEOM_CONVEX16][GEOM_BOX]               = &ShapeShapeCollide<ConvexBase16, Box>;
  collision_matrix[GEOM_CONVEX16][GEOM_SPHERE]            = &ShapeShapeCollide<ConvexBase16, Sphere>;
  collision_matrix[GEOM_CONVEX16][GEOM_CAPSULE]           = &ShapeShapeCollide<ConvexBase16, Capsule>;
  collision_matrix[GEOM_CONVEX16][GEOM_CONE]              = &ShapeShapeCollide<ConvexBase16, Cone>;
  collision_matrix[GEOM_CONVEX16][GEOM_CYLINDER]          = &ShapeShapeCollide<ConvexBase16, Cylinder>;
  collision_matrix[GEOM_CONVEX16][GEOM_CONVEX16]          = &ShapeShapeCollide<ConvexBase16, ConvexBase16>;
  collision_matrix[GEOM_CONVEX16][GEOM_CONVEX32]          = &ShapeShapeCollide<ConvexBase16, ConvexBase32>;
  collision_matrix[GEOM_CONVEX16][GEOM_PLANE]             = &ShapeShapeCollide<ConvexBase16, Plane>;
  collision_matrix[GEOM_CONVEX16][GEOM_HALFSPACE]         = &ShapeShapeCollide<ConvexBase16, Halfspace>;
  collision_matrix[GEOM_CONVEX16][GEOM_ELLIPSOID]         = &ShapeShapeCollide<ConvexBase16, Ellipsoid>;
  collision_matrix[GEOM_CONVEX16][GEOM_TRIANGLE]          = &ShapeShapeCollide<ConvexBase16, TriangleP>;

  collision_matrix[GEOM_CONVEX32][GEOM_BOX]               = &ShapeShapeCollide<ConvexBase32, Box>;
  collision_matrix[GEOM_CONVEX32][GEOM_SPHERE]            = &ShapeShapeCollide<ConvexBase32, Sphere>;
  collision_matrix[GEOM_CONVEX32][GEOM_CAPSULE]           = &ShapeShapeCollide<ConvexBase32, Capsule>;
  collision_matrix[GEOM_CONVEX32][GEOM_CONE]              = &ShapeShapeCollide<ConvexBase32, Cone>;
  collision_matrix[GEOM_CONVEX32][GEOM_CYLINDER]          = &ShapeShapeCollide<ConvexBase32, Cylinder>;
  collision_matrix[GEOM_CONVEX32][GEOM_CONVEX16]          = &ShapeShapeCollide<ConvexBase32, ConvexBase16>;
  collision_matrix[GEOM_CONVEX32][GEOM_CONVEX32]          = &ShapeShapeCollide<ConvexBase32, ConvexBase32>;
  collision_matrix[GEOM_CONVEX32][GEOM_PLANE]             = &ShapeShapeCollide<ConvexBase32, Plane>;
  collision_matrix[GEOM_CONVEX32][GEOM_HALFSPACE]         = &ShapeShapeCollide<ConvexBase32, Halfspace>;
  collision_matrix[GEOM_CONVEX32][GEOM_ELLIPSOID]         = &ShapeShapeCollide<ConvexBase32, Ellipsoid>;
  collision_matrix[GEOM_CONVEX32][GEOM_TRIANGLE]          = &ShapeShapeCollide<ConvexBase32, TriangleP>;

  collision_matrix[GEOM_PLANE][GEOM_BOX]                  = &ShapeShapeCollide<Plane, Box>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE]               = &ShapeShapeCollide<Plane, Sphere>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE]              = &ShapeShapeCollide<Plane, Capsule>;
  collision_matrix[GEOM_PLANE][GEOM_CONE]                 = &ShapeShapeCollide<Plane, Cone>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER]             = &ShapeShapeCollide<Plane, Cylinder>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX16]             = &ShapeShapeCollide<Plane, ConvexBase16>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX32]             = &ShapeShapeCollide<Plane, ConvexBase32>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE]                = &ShapeShapeCollide<Plane, Plane>;
  collision_matrix[GEOM_PLANE][GEOM_HALFSPACE]            = &ShapeShapeCollide<Plane, Halfspace>;
  collision_matrix[GEOM_PLANE][GEOM_ELLIPSOID]            = &ShapeShapeCollide<Plane, Ellipsoid>;
  collision_matrix[GEOM_PLANE][GEOM_TRIANGLE]             = &ShapeShapeCollide<Plane, TriangleP>;

  collision_matrix[GEOM_HALFSPACE][GEOM_BOX]              = &ShapeShapeCollide<Halfspace, Box>;
  collision_matrix[GEOM_HALFSPACE][GEOM_SPHERE]           = &ShapeShapeCollide<Halfspace, Sphere>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CAPSULE]          = &ShapeShapeCollide<Halfspace, Capsule>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONE]             = &ShapeShapeCollide<Halfspace, Cone>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CYLINDER]         = &ShapeShapeCollide<Halfspace, Cylinder>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONVEX16]         = &ShapeShapeCollide<Halfspace, ConvexBase16>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONVEX32]         = &ShapeShapeCollide<Halfspace, ConvexBase32>;
  collision_matrix[GEOM_HALFSPACE][GEOM_PLANE]            = &ShapeShapeCollide<Halfspace, Plane>;
  collision_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE]        = &ShapeShapeCollide<Halfspace, Halfspace>;
  collision_matrix[GEOM_HALFSPACE][GEOM_ELLIPSOID]        = &ShapeShapeCollide<Halfspace, Ellipsoid>;
  collision_matrix[GEOM_HALFSPACE][GEOM_TRIANGLE]         = &ShapeShapeCollide<Halfspace, TriangleP>;

  collision_matrix[GEOM_TRIANGLE][GEOM_BOX]               = &ShapeShapeCollide<TriangleP, Box>;
  collision_matrix[GEOM_TRIANGLE][GEOM_SPHERE]            = &ShapeShapeCollide<TriangleP, Sphere>;
  collision_matrix[GEOM_TRIANGLE][GEOM_CAPSULE]           = &ShapeShapeCollide<TriangleP, Capsule>;
  collision_matrix[GEOM_TRIANGLE][GEOM_CONE]              = &ShapeShapeCollide<TriangleP, Cone>;
  collision_matrix[GEOM_TRIANGLE][GEOM_CYLINDER]          = &ShapeShapeCollide<TriangleP, Cylinder>;
  collision_matrix[GEOM_TRIANGLE][GEOM_CONVEX16]          = &ShapeShapeCollide<TriangleP, ConvexBase16>;
  collision_matrix[GEOM_TRIANGLE][GEOM_CONVEX32]          = &ShapeShapeCollide<TriangleP, ConvexBase32>;
  collision_matrix[GEOM_TRIANGLE][GEOM_PLANE]             = &ShapeShapeCollide<TriangleP, Plane>;
  collision_matrix[GEOM_TRIANGLE][GEOM_HALFSPACE]         = &ShapeShapeCollide<TriangleP, Halfspace>;
  collision_matrix[GEOM_TRIANGLE][GEOM_ELLIPSOID]         = &ShapeShapeCollide<TriangleP, Ellipsoid>;
  collision_matrix[GEOM_TRIANGLE][GEOM_TRIANGLE]          = &ShapeShapeCollide<TriangleP, TriangleP>;

  collision_matrix[BV_AABB][GEOM_BOX]                     = &BVHShapeCollider<AABB, Box>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE]                  = &BVHShapeCollider<AABB, Sphere>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE]                 = &BVHShapeCollider<AABB, Capsule>::collide;
  collision_matrix[BV_AABB][GEOM_CONE]                    = &BVHShapeCollider<AABB, Cone>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER]                = &BVHShapeCollider<AABB, Cylinder>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX16]                = &BVHShapeCollider<AABB, ConvexBase16>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX32]                = &BVHShapeCollider<AABB, ConvexBase32>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE]                   = &BVHShapeCollider<AABB, Plane>::collide;
  collision_matrix[BV_AABB][GEOM_HALFSPACE]               = &BVHShapeCollider<AABB, Halfspace>::collide;
  collision_matrix[BV_AABB][GEOM_ELLIPSOID]               = &BVHShapeCollider<AABB, Ellipsoid>::collide;

  collision_matrix[BV_OBB][GEOM_BOX]                      = &BVHShapeCollider<OBB, Box>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE]                   = &BVHShapeCollider<OBB, Sphere>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE]                  = &BVHShapeCollider<OBB, Capsule>::collide;
  collision_matrix[BV_OBB][GEOM_CONE]                     = &BVHShapeCollider<OBB, Cone>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER]                 = &BVHShapeCollider<OBB, Cylinder>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX16]                 = &BVHShapeCollider<OBB, ConvexBase16>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX32]                 = &BVHShapeCollider<OBB, ConvexBase32>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE]                    = &BVHShapeCollider<OBB, Plane>::collide;
  collision_matrix[BV_OBB][GEOM_HALFSPACE]                = &BVHShapeCollider<OBB, Halfspace>::collide;
  collision_matrix[BV_OBB][GEOM_ELLIPSOID]                = &BVHShapeCollider<OBB, Ellipsoid>::collide;

  collision_matrix[BV_RSS][GEOM_BOX]                      = &BVHShapeCollider<RSS, Box>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE]                   = &BVHShapeCollider<RSS, Sphere>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE]                  = &BVHShapeCollider<RSS, Capsule>::collide;
  collision_matrix[BV_RSS][GEOM_CONE]                     = &BVHShapeCollider<RSS, Cone>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER]                 = &BVHShapeCollider<RSS, Cylinder>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX16]                 = &BVHShapeCollider<RSS, ConvexBase16>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX32]                 = &BVHShapeCollider<RSS, ConvexBase32>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE]                    = &BVHShapeCollider<RSS, Plane>::collide;
  collision_matrix[BV_RSS][GEOM_HALFSPACE]                = &BVHShapeCollider<RSS, Halfspace>::collide;
  collision_matrix[BV_RSS][GEOM_ELLIPSOID]                = &BVHShapeCollider<RSS, Ellipsoid>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX]                   = &BVHShapeCollider<KDOP<16>, Box>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE]                = &BVHShapeCollider<KDOP<16>, Sphere>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE]               = &BVHShapeCollider<KDOP<16>, Capsule>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE]                  = &BVHShapeCollider<KDOP<16>, Cone>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER]              = &BVHShapeCollider<KDOP<16>, Cylinder>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX16]              = &BVHShapeCollider<KDOP<16>, ConvexBase16>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX32]              = &BVHShapeCollider<KDOP<16>, ConvexBase32>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE]                 = &BVHShapeCollider<KDOP<16>, Plane>::collide;
  collision_matrix[BV_KDOP16][GEOM_HALFSPACE]             = &BVHShapeCollider<KDOP<16>, Halfspace>::collide;
  collision_matrix[BV_KDOP16][GEOM_ELLIPSOID]             = &BVHShapeCollider<KDOP<16>, Ellipsoid>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX]                   = &BVHShapeCollider<KDOP<18>, Box>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE]                = &BVHShapeCollider<KDOP<18>, Sphere>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE]               = &BVHShapeCollider<KDOP<18>, Capsule>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE]                  = &BVHShapeCollider<KDOP<18>, Cone>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER]              = &BVHShapeCollider<KDOP<18>, Cylinder>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX16]              = &BVHShapeCollider<KDOP<18>, ConvexBase16>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX32]              = &BVHShapeCollider<KDOP<18>, ConvexBase32>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE]                 = &BVHShapeCollider<KDOP<18>, Plane>::collide;
  collision_matrix[BV_KDOP18][GEOM_HALFSPACE]             = &BVHShapeCollider<KDOP<18>, Halfspace>::collide;
  collision_matrix[BV_KDOP18][GEOM_ELLIPSOID]             = &BVHShapeCollider<KDOP<18>, Ellipsoid>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX]                   = &BVHShapeCollider<KDOP<24>, Box>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE]                = &BVHShapeCollider<KDOP<24>, Sphere>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE]               = &BVHShapeCollider<KDOP<24>, Capsule>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE]                  = &BVHShapeCollider<KDOP<24>, Cone>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER]              = &BVHShapeCollider<KDOP<24>, Cylinder>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX16]              = &BVHShapeCollider<KDOP<24>, ConvexBase16>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX32]              = &BVHShapeCollider<KDOP<24>, ConvexBase32>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE]                 = &BVHShapeCollider<KDOP<24>, Plane>::collide;
  collision_matrix[BV_KDOP24][GEOM_HALFSPACE]             = &BVHShapeCollider<KDOP<24>, Halfspace>::collide;
  collision_matrix[BV_KDOP24][GEOM_ELLIPSOID]             = &BVHShapeCollider<KDOP<24>, Ellipsoid>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX]                     = &BVHShapeCollider<kIOS, Box>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE]                  = &BVHShapeCollider<kIOS, Sphere>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE]                 = &BVHShapeCollider<kIOS, Capsule>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE]                    = &BVHShapeCollider<kIOS, Cone>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER]                = &BVHShapeCollider<kIOS, Cylinder>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX16]                = &BVHShapeCollider<kIOS, ConvexBase16>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX32]                = &BVHShapeCollider<kIOS, ConvexBase32>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE]                   = &BVHShapeCollider<kIOS, Plane>::collide;
  collision_matrix[BV_kIOS][GEOM_HALFSPACE]               = &BVHShapeCollider<kIOS, Halfspace>::collide;
  collision_matrix[BV_kIOS][GEOM_ELLIPSOID]               = &BVHShapeCollider<kIOS, Ellipsoid>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX]                   = &BVHShapeCollider<OBBRSS, Box>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE]                = &BVHShapeCollider<OBBRSS, Sphere>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE]               = &BVHShapeCollider<OBBRSS, Capsule>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE]                  = &BVHShapeCollider<OBBRSS, Cone>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER]              = &BVHShapeCollider<OBBRSS, Cylinder>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX16]              = &BVHShapeCollider<OBBRSS, ConvexBase16>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX32]              = &BVHShapeCollider<OBBRSS, ConvexBase32>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE]                 = &BVHShapeCollider<OBBRSS, Plane>::collide;
  collision_matrix[BV_OBBRSS][GEOM_HALFSPACE]             = &BVHShapeCollider<OBBRSS, Halfspace>::collide;
  collision_matrix[BV_OBBRSS][GEOM_ELLIPSOID]             = &BVHShapeCollider<OBBRSS, Ellipsoid>::collide;

  collision_matrix[HF_AABB][GEOM_BOX]                     = &HeightFieldShapeCollider<AABB, Box>::collide;
  collision_matrix[HF_AABB][GEOM_SPHERE]                  = &HeightFieldShapeCollider<AABB, Sphere>::collide;
  collision_matrix[HF_AABB][GEOM_CAPSULE]                 = &HeightFieldShapeCollider<AABB, Capsule>::collide;
  collision_matrix[HF_AABB][GEOM_CONE]                    = &HeightFieldShapeCollider<AABB, Cone>::collide;
  collision_matrix[HF_AABB][GEOM_CYLINDER]                = &HeightFieldShapeCollider<AABB, Cylinder>::collide;
  collision_matrix[HF_AABB][GEOM_CONVEX16]                = &HeightFieldShapeCollider<AABB, ConvexBase16>::collide;
  collision_matrix[HF_AABB][GEOM_CONVEX32]                = &HeightFieldShapeCollider<AABB, ConvexBase32>::collide;
  collision_matrix[HF_AABB][GEOM_PLANE]                   = &HeightFieldShapeCollider<AABB, Plane>::collide;
  collision_matrix[HF_AABB][GEOM_HALFSPACE]               = &HeightFieldShapeCollider<AABB, Halfspace>::collide;
  collision_matrix[HF_AABB][GEOM_ELLIPSOID]               = &HeightFieldShapeCollider<AABB, Ellipsoid>::collide;

  collision_matrix[HF_OBBRSS][GEOM_BOX]                   = &HeightFieldShapeCollider<OBBRSS, Box>::collide;
  collision_matrix[HF_OBBRSS][GEOM_SPHERE]                = &HeightFieldShapeCollider<OBBRSS, Sphere>::collide;
  collision_matrix[HF_OBBRSS][GEOM_CAPSULE]               = &HeightFieldShapeCollider<OBBRSS, Capsule>::collide;
  collision_matrix[HF_OBBRSS][GEOM_CONE]                  = &HeightFieldShapeCollider<OBBRSS, Cone>::collide;
  collision_matrix[HF_OBBRSS][GEOM_CYLINDER]              = &HeightFieldShapeCollider<OBBRSS, Cylinder>::collide;
  collision_matrix[HF_OBBRSS][GEOM_CONVEX16]              = &HeightFieldShapeCollider<OBBRSS, ConvexBase16>::collide;
  collision_matrix[HF_OBBRSS][GEOM_CONVEX32]              = &HeightFieldShapeCollider<OBBRSS, ConvexBase32>::collide;
  collision_matrix[HF_OBBRSS][GEOM_PLANE]                 = &HeightFieldShapeCollider<OBBRSS, Plane>::collide;
  collision_matrix[HF_OBBRSS][GEOM_HALFSPACE]             = &HeightFieldShapeCollider<OBBRSS, Halfspace>::collide;
  collision_matrix[HF_OBBRSS][GEOM_ELLIPSOID]             = &HeightFieldShapeCollider<OBBRSS, Ellipsoid>::collide;

  collision_matrix[BV_AABB][BV_AABB]                      = &BVHCollide<AABB>;
  collision_matrix[BV_OBB][BV_OBB]                        = &BVHCollide<OBB>;
  collision_matrix[BV_RSS][BV_RSS]                        = &BVHCollide<RSS>;
  collision_matrix[BV_KDOP16][BV_KDOP16]                  = &BVHCollide<KDOP<16> >;
  collision_matrix[BV_KDOP18][BV_KDOP18]                  = &BVHCollide<KDOP<18> >;
  collision_matrix[BV_KDOP24][BV_KDOP24]                  = &BVHCollide<KDOP<24> >;
  collision_matrix[BV_kIOS][BV_kIOS]                      = &BVHCollide<kIOS>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS]                  = &BVHCollide<OBBRSS>;

#ifdef COAL_HAS_OCTOMAP
  collision_matrix[GEOM_OCTREE][GEOM_BOX]                 = &OctreeCollide<OcTree, Box>;
  collision_matrix[GEOM_OCTREE][GEOM_SPHERE]              = &OctreeCollide<OcTree, Sphere>;
  collision_matrix[GEOM_OCTREE][GEOM_CAPSULE]             = &OctreeCollide<OcTree, Capsule>;
  collision_matrix[GEOM_OCTREE][GEOM_CONE]                = &OctreeCollide<OcTree, Cone>;
  collision_matrix[GEOM_OCTREE][GEOM_CYLINDER]            = &OctreeCollide<OcTree, Cylinder>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX16]            = &OctreeCollide<OcTree, ConvexBase16>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX32]            = &OctreeCollide<OcTree, ConvexBase32>;
  collision_matrix[GEOM_OCTREE][GEOM_PLANE]               = &OctreeCollide<OcTree, Plane>;
  collision_matrix[GEOM_OCTREE][GEOM_HALFSPACE]           = &OctreeCollide<OcTree, Halfspace>;
  collision_matrix[GEOM_OCTREE][GEOM_ELLIPSOID]           = &OctreeCollide<OcTree, Ellipsoid>;

  collision_matrix[GEOM_BOX][GEOM_OCTREE]                 = &OctreeCollide<Box, OcTree>;
  collision_matrix[GEOM_SPHERE][GEOM_OCTREE]              = &OctreeCollide<Sphere, OcTree>;
  collision_matrix[GEOM_CAPSULE][GEOM_OCTREE]             = &OctreeCollide<Capsule, OcTree>;
  collision_matrix[GEOM_CONE][GEOM_OCTREE]                = &OctreeCollide<Cone, OcTree>;
  collision_matrix[GEOM_CYLINDER][GEOM_OCTREE]            = &OctreeCollide<Cylinder, OcTree>;
  collision_matrix[GEOM_CONVEX16][GEOM_OCTREE]            = &OctreeCollide<ConvexBase16, OcTree>;
  collision_matrix[GEOM_CONVEX32][GEOM_OCTREE]            = &OctreeCollide<ConvexBase32, OcTree>;
  collision_matrix[GEOM_PLANE][GEOM_OCTREE]               = &OctreeCollide<Plane, OcTree>;
  collision_matrix[GEOM_HALFSPACE][GEOM_OCTREE]           = &OctreeCollide<Halfspace, OcTree>;

  collision_matrix[GEOM_OCTREE][GEOM_OCTREE]              = &OctreeCollide<OcTree, OcTree>;

  collision_matrix[GEOM_OCTREE][BV_AABB]                  = &OctreeCollide<OcTree, BVHModel<AABB> >;
  collision_matrix[GEOM_OCTREE][BV_OBB]                   = &OctreeCollide<OcTree, BVHModel<OBB> >;
  collision_matrix[GEOM_OCTREE][BV_RSS]                   = &OctreeCollide<OcTree, BVHModel<RSS> >;
  collision_matrix[GEOM_OCTREE][BV_OBBRSS]                = &OctreeCollide<OcTree, BVHModel<OBBRSS> >;
  collision_matrix[GEOM_OCTREE][BV_kIOS]                  = &OctreeCollide<OcTree, BVHModel<kIOS> >;
  collision_matrix[GEOM_OCTREE][BV_KDOP16]                = &OctreeCollide<OcTree, BVHModel<KDOP<16> > >;
  collision_matrix[GEOM_OCTREE][BV_KDOP18]                = &OctreeCollide<OcTree, BVHModel<KDOP<18> > >;
  collision_matrix[GEOM_OCTREE][BV_KDOP24]                = &OctreeCollide<OcTree, BVHModel<KDOP<24> > >;
  collision_matrix[GEOM_OCTREE][HF_AABB]                  = &OctreeCollide<OcTree, HeightField<AABB> >;
  collision_matrix[GEOM_OCTREE][HF_OBBRSS]                = &OctreeCollide<OcTree, HeightField<OBBRSS> >;

  collision_matrix[BV_AABB][GEOM_OCTREE]                  = &OctreeCollide<BVHModel<AABB>, OcTree>;
  collision_matrix[BV_OBB][GEOM_OCTREE]                   = &OctreeCollide<BVHModel<OBB>, OcTree>;
  collision_matrix[BV_RSS][GEOM_OCTREE]                   = &OctreeCollide<BVHModel<RSS>, OcTree>;
  collision_matrix[BV_OBBRSS][GEOM_OCTREE]                = &OctreeCollide<BVHModel<OBBRSS>, OcTree>;
  collision_matrix[BV_kIOS][GEOM_OCTREE]                  = &OctreeCollide<BVHModel<kIOS>, OcTree>;
  collision_matrix[BV_KDOP16][GEOM_OCTREE]                = &OctreeCollide<BVHModel<KDOP<16> >, OcTree>;
  collision_matrix[BV_KDOP18][GEOM_OCTREE]                = &OctreeCollide<BVHModel<KDOP<18> >, OcTree>;
  collision_matrix[BV_KDOP24][GEOM_OCTREE]                = &OctreeCollide<BVHModel<KDOP<24> >, OcTree>;
  collision_matrix[HF_AABB][GEOM_OCTREE]                  = &OctreeCollide<HeightField<AABB>, OcTree>;
  collision_matrix[HF_OBBRSS][GEOM_OCTREE]                = &OctreeCollide<HeightField<OBBRSS>, OcTree>;
// clang-format on
#endif
}
// template struct CollisionFunctionMatrix;
}  // namespace coal
