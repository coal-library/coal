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

#include "coal/distance_func_matrix.h"

#include <../src/collision_node.h>
#include "coal/internal/shape_shape_func.h"
#include "coal/internal/traversal_node_setup.h"
#include "coal/internal/shape_shape_func.h"
#include <../src/traits_traversal.h>

namespace coal {

#ifdef COAL_HAS_OCTOMAP

template <typename TypeA, typename TypeB>
Scalar Distance(const CollisionGeometry* o1, const Transform3s& tf1,
                const CollisionGeometry* o2, const Transform3s& tf2,
                const GJKSolver* nsolver, const DistanceRequest& request,
                DistanceResult& result) {
  if (request.isSatisfied(result)) return result.min_distance;
  typename TraversalTraitsDistance<TypeA, TypeB>::CollisionTraversal_t node;
  const TypeA* obj1 = static_cast<const TypeA*>(o1);
  const TypeB* obj2 = static_cast<const TypeB*>(o2);
  OcTreeSolver otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

#endif

COAL_LOCAL Scalar distance_function_not_implemented(
    const CollisionGeometry* o1, const Transform3s& /*tf1*/,
    const CollisionGeometry* o2, const Transform3s& /*tf2*/,
    const GJKSolver* /*nsolver*/, const DistanceRequest& /*request*/,
    DistanceResult& /*result*/) {
  NODE_TYPE node_type1 = o1->getNodeType();
  NODE_TYPE node_type2 = o2->getNodeType();

  COAL_THROW_PRETTY("Distance function between node type "
                        << std::string(get_node_type_name(node_type1))
                        << " and node type "
                        << std::string(get_node_type_name(node_type2))
                        << " is not yet supported.",
                    std::invalid_argument);
}

template <typename T_BVH, typename T_SH>
struct COAL_LOCAL BVHShapeDistancer {
  static Scalar distance(const CollisionGeometry* o1, const Transform3s& tf1,
                         const CollisionGeometry* o2, const Transform3s& tf2,
                         const GJKSolver* nsolver,
                         const DistanceRequest& request,
                         DistanceResult& result) {
    if (request.isSatisfied(result)) return result.min_distance;
    MeshShapeDistanceTraversalNode<T_BVH, T_SH> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3s tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
    ::coal::distance(&node);

    delete obj1_tmp;
    return result.min_distance;
  }  // namespace coal
};

namespace details {

template <typename OrientedMeshShapeDistanceTraversalNode, typename T_BVH,
          typename T_SH>
Scalar orientedBVHShapeDistance(const CollisionGeometry* o1,
                                const Transform3s& tf1,
                                const CollisionGeometry* o2,
                                const Transform3s& tf2,
                                const GJKSolver* nsolver,
                                const DistanceRequest& request,
                                DistanceResult& result) {
  if (request.isSatisfied(result)) return result.min_distance;
  OrientedMeshShapeDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  ::coal::distance(&node);

  return result.min_distance;
}

}  // namespace details

template <typename T_SH>
struct COAL_LOCAL BVHShapeDistancer<RSS, T_SH> {
  static Scalar distance(const CollisionGeometry* o1, const Transform3s& tf1,
                         const CollisionGeometry* o2, const Transform3s& tf2,
                         const GJKSolver* nsolver,
                         const DistanceRequest& request,
                         DistanceResult& result) {
    return details::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodeRSS<T_SH>, RSS, T_SH>(
        o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename T_SH>
struct COAL_LOCAL BVHShapeDistancer<kIOS, T_SH> {
  static Scalar distance(const CollisionGeometry* o1, const Transform3s& tf1,
                         const CollisionGeometry* o2, const Transform3s& tf2,
                         const GJKSolver* nsolver,
                         const DistanceRequest& request,
                         DistanceResult& result) {
    return details::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodekIOS<T_SH>, kIOS, T_SH>(
        o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename T_SH>
struct COAL_LOCAL BVHShapeDistancer<OBBRSS, T_SH> {
  static Scalar distance(const CollisionGeometry* o1, const Transform3s& tf1,
                         const CollisionGeometry* o2, const Transform3s& tf2,
                         const GJKSolver* nsolver,
                         const DistanceRequest& request,
                         DistanceResult& result) {
    return details::orientedBVHShapeDistance<
        MeshShapeDistanceTraversalNodeOBBRSS<T_SH>, OBBRSS, T_SH>(
        o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template <typename T_HF, typename T_SH>
struct COAL_LOCAL HeightFieldShapeDistancer {
  static Scalar distance(const CollisionGeometry* o1, const Transform3s& tf1,
                         const CollisionGeometry* o2, const Transform3s& tf2,
                         const GJKSolver* nsolver,
                         const DistanceRequest& request,
                         DistanceResult& result) {
    COAL_UNUSED_VARIABLE(o1);
    COAL_UNUSED_VARIABLE(tf1);
    COAL_UNUSED_VARIABLE(o2);
    COAL_UNUSED_VARIABLE(tf2);
    COAL_UNUSED_VARIABLE(nsolver);
    COAL_UNUSED_VARIABLE(request);
    // TODO(jcarpent)
    COAL_THROW_PRETTY(
        "Distance between a height field and a shape is not implemented",
        std::invalid_argument);
    //    if(request.isSatisfied(result)) return result.min_distance;
    //    HeightFieldShapeDistanceTraversalNode<T_HF, T_SH> node;
    //
    //    const HeightField<T_HF>* obj1 = static_cast<const HeightField<T_HF>*
    //    >(o1); const T_SH* obj2 = static_cast<const T_SH*>(o2);
    //
    //    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
    //    fcl::distance(&node);

    return result.min_distance;
  }
};

template <typename T_BVH>
Scalar BVHDistance(const CollisionGeometry* o1, const Transform3s& tf1,
                   const CollisionGeometry* o2, const Transform3s& tf2,
                   const DistanceRequest& request, DistanceResult& result) {
  if (request.isSatisfied(result)) return result.min_distance;
  MeshDistanceTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3s tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3s tf2_tmp = tf2;

  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
  distance(&node);
  delete obj1_tmp;
  delete obj2_tmp;

  return result.min_distance;
}

namespace details {
template <typename OrientedMeshDistanceTraversalNode, typename T_BVH>
Scalar orientedMeshDistance(const CollisionGeometry* o1, const Transform3s& tf1,
                            const CollisionGeometry* o2, const Transform3s& tf2,
                            const DistanceRequest& request,
                            DistanceResult& result) {
  if (request.isSatisfied(result)) return result.min_distance;
  OrientedMeshDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  distance(&node);

  return result.min_distance;
}

}  // namespace details

template <>
Scalar BVHDistance<RSS>(const CollisionGeometry* o1, const Transform3s& tf1,
                        const CollisionGeometry* o2, const Transform3s& tf2,
                        const DistanceRequest& request,
                        DistanceResult& result) {
  return details::orientedMeshDistance<MeshDistanceTraversalNodeRSS, RSS>(
      o1, tf1, o2, tf2, request, result);
}

template <>
Scalar BVHDistance<kIOS>(const CollisionGeometry* o1, const Transform3s& tf1,
                         const CollisionGeometry* o2, const Transform3s& tf2,
                         const DistanceRequest& request,
                         DistanceResult& result) {
  return details::orientedMeshDistance<MeshDistanceTraversalNodekIOS, kIOS>(
      o1, tf1, o2, tf2, request, result);
}

template <>
Scalar BVHDistance<OBBRSS>(const CollisionGeometry* o1, const Transform3s& tf1,
                           const CollisionGeometry* o2, const Transform3s& tf2,
                           const DistanceRequest& request,
                           DistanceResult& result) {
  return details::orientedMeshDistance<MeshDistanceTraversalNodeOBBRSS, OBBRSS>(
      o1, tf1, o2, tf2, request, result);
}

template <typename T_BVH>
Scalar BVHDistance(const CollisionGeometry* o1, const Transform3s& tf1,
                   const CollisionGeometry* o2, const Transform3s& tf2,
                   const GJKSolver* /*nsolver*/, const DistanceRequest& request,
                   DistanceResult& result) {
  return BVHDistance<T_BVH>(o1, tf1, o2, tf2, request, result);
}

DistanceFunctionMatrix::DistanceFunctionMatrix() {
  for (int i = 0; i < NODE_COUNT; ++i) {
    for (int j = 0; j < NODE_COUNT; ++j) distance_matrix[i][j] = NULL;
  }

  // clang-format off
  distance_matrix[GEOM_BOX][GEOM_BOX]             = &ShapeShapeDistance<Box, Box>;
  distance_matrix[GEOM_BOX][GEOM_SPHERE]          = &ShapeShapeDistance<Box, Sphere>;
  distance_matrix[GEOM_BOX][GEOM_CAPSULE]         = &ShapeShapeDistance<Box, Capsule>;
  distance_matrix[GEOM_BOX][GEOM_CONE]            = &ShapeShapeDistance<Box, Cone>;
  distance_matrix[GEOM_BOX][GEOM_CYLINDER]        = &ShapeShapeDistance<Box, Cylinder>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX16]        = &ShapeShapeDistance<Box, ConvexBase16>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX32]        = &ShapeShapeDistance<Box, ConvexBase32>;
  distance_matrix[GEOM_BOX][GEOM_PLANE]           = &ShapeShapeDistance<Box, Plane>;
  distance_matrix[GEOM_BOX][GEOM_HALFSPACE]       = &ShapeShapeDistance<Box, Halfspace>;
  distance_matrix[GEOM_BOX][GEOM_ELLIPSOID]       = &ShapeShapeDistance<Box, Ellipsoid>;

  distance_matrix[GEOM_SPHERE][GEOM_BOX]          = &ShapeShapeDistance<Sphere, Box>;
  distance_matrix[GEOM_SPHERE][GEOM_SPHERE]       = &ShapeShapeDistance<Sphere, Sphere>;
  distance_matrix[GEOM_SPHERE][GEOM_CAPSULE]      = &ShapeShapeDistance<Sphere, Capsule>;
  distance_matrix[GEOM_SPHERE][GEOM_CONE]         = &ShapeShapeDistance<Sphere, Cone>;
  distance_matrix[GEOM_SPHERE][GEOM_CYLINDER]     = &ShapeShapeDistance<Sphere, Cylinder>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX16]     = &ShapeShapeDistance<Sphere, ConvexBase16>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX32]     = &ShapeShapeDistance<Sphere, ConvexBase32>;
  distance_matrix[GEOM_SPHERE][GEOM_PLANE]        = &ShapeShapeDistance<Sphere, Plane>;
  distance_matrix[GEOM_SPHERE][GEOM_HALFSPACE]    = &ShapeShapeDistance<Sphere, Halfspace>;
  distance_matrix[GEOM_SPHERE][GEOM_ELLIPSOID]    = &ShapeShapeDistance<Sphere, Ellipsoid>;

  distance_matrix[GEOM_ELLIPSOID][GEOM_BOX]       = &ShapeShapeDistance<Ellipsoid, Box>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_SPHERE]    = &ShapeShapeDistance<Ellipsoid, Sphere>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE]   = &ShapeShapeDistance<Ellipsoid, Capsule>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONE]      = &ShapeShapeDistance<Ellipsoid, Cone>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER]  = &ShapeShapeDistance<Ellipsoid, Cylinder>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONVEX16]  = &ShapeShapeDistance<Ellipsoid, ConvexBase16>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_CONVEX32]  = &ShapeShapeDistance<Ellipsoid, ConvexBase32>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_PLANE]     = &ShapeShapeDistance<Ellipsoid, Plane>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeDistance<Ellipsoid, Halfspace>;
  distance_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeDistance<Ellipsoid, Ellipsoid>;

  distance_matrix[GEOM_CAPSULE][GEOM_BOX]         = &ShapeShapeDistance<Capsule, Box>;
  distance_matrix[GEOM_CAPSULE][GEOM_SPHERE]      = &ShapeShapeDistance<Capsule, Sphere>;
  distance_matrix[GEOM_CAPSULE][GEOM_CAPSULE]     = &ShapeShapeDistance<Capsule, Capsule>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONE]        = &ShapeShapeDistance<Capsule, Cone>;
  distance_matrix[GEOM_CAPSULE][GEOM_CYLINDER]    = &ShapeShapeDistance<Capsule, Cylinder>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX16]    = &ShapeShapeDistance<Capsule, ConvexBase16>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX32]    = &ShapeShapeDistance<Capsule, ConvexBase32>;
  distance_matrix[GEOM_CAPSULE][GEOM_PLANE]       = &ShapeShapeDistance<Capsule, Plane>;
  distance_matrix[GEOM_CAPSULE][GEOM_HALFSPACE]   = &ShapeShapeDistance<Capsule, Halfspace>;
  distance_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID]   = &ShapeShapeDistance<Capsule, Ellipsoid>;

  distance_matrix[GEOM_CONE][GEOM_BOX]            = &ShapeShapeDistance<Cone, Box>;
  distance_matrix[GEOM_CONE][GEOM_SPHERE]         = &ShapeShapeDistance<Cone, Sphere>;
  distance_matrix[GEOM_CONE][GEOM_CAPSULE]        = &ShapeShapeDistance<Cone, Capsule>;
  distance_matrix[GEOM_CONE][GEOM_CONE]           = &ShapeShapeDistance<Cone, Cone>;
  distance_matrix[GEOM_CONE][GEOM_CYLINDER]       = &ShapeShapeDistance<Cone, Cylinder>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX16]       = &ShapeShapeDistance<Cone, ConvexBase16>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX32]       = &ShapeShapeDistance<Cone, ConvexBase32>;
  distance_matrix[GEOM_CONE][GEOM_PLANE]          = &ShapeShapeDistance<Cone, Plane>;
  distance_matrix[GEOM_CONE][GEOM_HALFSPACE]      = &ShapeShapeDistance<Cone, Halfspace>;
  distance_matrix[GEOM_CONE][GEOM_ELLIPSOID]      = &ShapeShapeDistance<Cone, Ellipsoid>;

  distance_matrix[GEOM_CYLINDER][GEOM_BOX]        = &ShapeShapeDistance<Cylinder, Box>;
  distance_matrix[GEOM_CYLINDER][GEOM_SPHERE]     = &ShapeShapeDistance<Cylinder, Sphere>;
  distance_matrix[GEOM_CYLINDER][GEOM_CAPSULE]    = &ShapeShapeDistance<Cylinder, Capsule>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONE]       = &ShapeShapeDistance<Cylinder, Cone>;
  distance_matrix[GEOM_CYLINDER][GEOM_CYLINDER]   = &ShapeShapeDistance<Cylinder, Cylinder>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX16]   = &ShapeShapeDistance<Cylinder, ConvexBase16>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX32]   = &ShapeShapeDistance<Cylinder, ConvexBase32>;
  distance_matrix[GEOM_CYLINDER][GEOM_PLANE]      = &ShapeShapeDistance<Cylinder, Plane>;
  distance_matrix[GEOM_CYLINDER][GEOM_HALFSPACE]  = &ShapeShapeDistance<Cylinder, Halfspace>;
  distance_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID]  = &ShapeShapeDistance<Cylinder, Ellipsoid>;

  distance_matrix[GEOM_CONVEX16][GEOM_BOX]          = &ShapeShapeDistance<ConvexBase16, Box>;
  distance_matrix[GEOM_CONVEX16][GEOM_SPHERE]       = &ShapeShapeDistance<ConvexBase16, Sphere>;
  distance_matrix[GEOM_CONVEX16][GEOM_CAPSULE]      = &ShapeShapeDistance<ConvexBase16, Capsule>;
  distance_matrix[GEOM_CONVEX16][GEOM_CONE]         = &ShapeShapeDistance<ConvexBase16, Cone>;
  distance_matrix[GEOM_CONVEX16][GEOM_CYLINDER]     = &ShapeShapeDistance<ConvexBase16, Cylinder>;
  distance_matrix[GEOM_CONVEX16][GEOM_CONVEX16]     = &ShapeShapeDistance<ConvexBase16, ConvexBase16>;
  distance_matrix[GEOM_CONVEX16][GEOM_CONVEX32]     = &ShapeShapeDistance<ConvexBase16, ConvexBase32>;
  distance_matrix[GEOM_CONVEX16][GEOM_PLANE]        = &ShapeShapeDistance<ConvexBase16, Plane>;
  distance_matrix[GEOM_CONVEX16][GEOM_HALFSPACE]    = &ShapeShapeDistance<ConvexBase16, Halfspace>;
  distance_matrix[GEOM_CONVEX16][GEOM_ELLIPSOID]    = &ShapeShapeDistance<ConvexBase16, Ellipsoid>;

  distance_matrix[GEOM_CONVEX32][GEOM_BOX]          = &ShapeShapeDistance<ConvexBase32, Box>;
  distance_matrix[GEOM_CONVEX32][GEOM_SPHERE]       = &ShapeShapeDistance<ConvexBase32, Sphere>;
  distance_matrix[GEOM_CONVEX32][GEOM_CAPSULE]      = &ShapeShapeDistance<ConvexBase32, Capsule>;
  distance_matrix[GEOM_CONVEX32][GEOM_CONE]         = &ShapeShapeDistance<ConvexBase32, Cone>;
  distance_matrix[GEOM_CONVEX32][GEOM_CYLINDER]     = &ShapeShapeDistance<ConvexBase32, Cylinder>;
  distance_matrix[GEOM_CONVEX32][GEOM_CONVEX16]     = &ShapeShapeDistance<ConvexBase32, ConvexBase16>;
  distance_matrix[GEOM_CONVEX32][GEOM_CONVEX32]     = &ShapeShapeDistance<ConvexBase32, ConvexBase32>;
  distance_matrix[GEOM_CONVEX32][GEOM_PLANE]        = &ShapeShapeDistance<ConvexBase32, Plane>;
  distance_matrix[GEOM_CONVEX32][GEOM_HALFSPACE]    = &ShapeShapeDistance<ConvexBase32, Halfspace>;
  distance_matrix[GEOM_CONVEX32][GEOM_ELLIPSOID]    = &ShapeShapeDistance<ConvexBase32, Ellipsoid>;

  distance_matrix[GEOM_PLANE][GEOM_BOX]           = &ShapeShapeDistance<Plane, Box>;
  distance_matrix[GEOM_PLANE][GEOM_SPHERE]        = &ShapeShapeDistance<Plane, Sphere>;
  distance_matrix[GEOM_PLANE][GEOM_CAPSULE]       = &ShapeShapeDistance<Plane, Capsule>;
  distance_matrix[GEOM_PLANE][GEOM_CONE]          = &ShapeShapeDistance<Plane, Cone>;
  distance_matrix[GEOM_PLANE][GEOM_CYLINDER]      = &ShapeShapeDistance<Plane, Cylinder>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX16]      = &ShapeShapeDistance<Plane, ConvexBase16>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX32]      = &ShapeShapeDistance<Plane, ConvexBase32>;
  distance_matrix[GEOM_PLANE][GEOM_PLANE]         = &ShapeShapeDistance<Plane, Plane>;
  distance_matrix[GEOM_PLANE][GEOM_HALFSPACE]     = &ShapeShapeDistance<Plane, Halfspace>;
  distance_matrix[GEOM_PLANE][GEOM_ELLIPSOID]     = &ShapeShapeDistance<Plane, Ellipsoid>;

  distance_matrix[GEOM_HALFSPACE][GEOM_BOX]       = &ShapeShapeDistance<Halfspace, Box>;
  distance_matrix[GEOM_HALFSPACE][GEOM_SPHERE]    = &ShapeShapeDistance<Halfspace, Sphere>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CAPSULE]   = &ShapeShapeDistance<Halfspace, Capsule>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONE]      = &ShapeShapeDistance<Halfspace, Cone>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CYLINDER]  = &ShapeShapeDistance<Halfspace, Cylinder>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONVEX16]  = &ShapeShapeDistance<Halfspace, ConvexBase16>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONVEX32]  = &ShapeShapeDistance<Halfspace, ConvexBase32>;
  distance_matrix[GEOM_HALFSPACE][GEOM_PLANE]     = &ShapeShapeDistance<Halfspace, Plane>;
  distance_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeDistance<Halfspace, Halfspace>;
  distance_matrix[GEOM_HALFSPACE][GEOM_ELLIPSOID] = &ShapeShapeDistance<Halfspace, Ellipsoid>;
  // clang-format on

  /* AABB distance not implemented */
  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABB, Box>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABB,
  Sphere>::distance; distance_matrix[BV_AABB][GEOM_CAPSULE] =
  &BVHShapeDistancer<AABB, Capsule>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABB,
  Cone>::distance; distance_matrix[BV_AABB][GEOM_CYLINDER] =
  &BVHShapeDistancer<AABB, Cylinder>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABB,
  ConvexBase>::distance; distance_matrix[BV_AABB][GEOM_PLANE] =
  &BVHShapeDistancer<AABB, Plane>::distance;
  distance_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeDistancer<AABB,
  Halfspace>::distance;
  */

  // clang-format off
  distance_matrix[BV_OBB][GEOM_BOX]              = &BVHShapeDistancer<OBB, Box>::distance;
  distance_matrix[BV_OBB][GEOM_SPHERE]           = &BVHShapeDistancer<OBB, Sphere>::distance;
  distance_matrix[BV_OBB][GEOM_CAPSULE]          = &BVHShapeDistancer<OBB, Capsule>::distance;
  distance_matrix[BV_OBB][GEOM_CONE]             = &BVHShapeDistancer<OBB, Cone>::distance;
  distance_matrix[BV_OBB][GEOM_CYLINDER]         = &BVHShapeDistancer<OBB, Cylinder>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX16]         = &BVHShapeDistancer<OBB, ConvexBase16>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX32]         = &BVHShapeDistancer<OBB, ConvexBase32>::distance;
  distance_matrix[BV_OBB][GEOM_PLANE]            = &BVHShapeDistancer<OBB, Plane>::distance;
  distance_matrix[BV_OBB][GEOM_HALFSPACE]        = &BVHShapeDistancer<OBB, Halfspace>::distance;
  distance_matrix[BV_OBB][GEOM_ELLIPSOID]        = &BVHShapeDistancer<OBB, Ellipsoid>::distance;

  distance_matrix[BV_RSS][GEOM_BOX]              = &BVHShapeDistancer<RSS, Box>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE]           = &BVHShapeDistancer<RSS, Sphere>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE]          = &BVHShapeDistancer<RSS, Capsule>::distance;
  distance_matrix[BV_RSS][GEOM_CONE]             = &BVHShapeDistancer<RSS, Cone>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER]         = &BVHShapeDistancer<RSS, Cylinder>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX16]         = &BVHShapeDistancer<RSS, ConvexBase16>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX32]         = &BVHShapeDistancer<RSS, ConvexBase32>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE]            = &BVHShapeDistancer<RSS, Plane>::distance;
  distance_matrix[BV_RSS][GEOM_HALFSPACE]        = &BVHShapeDistancer<RSS, Halfspace>::distance;
  distance_matrix[BV_RSS][GEOM_ELLIPSOID]        = &BVHShapeDistancer<RSS, Ellipsoid>::distance;
  // clang-format on

  /* KDOP distance not implemented */
  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOP<16>,
  Box>::distance; distance_matrix[BV_KDOP16][GEOM_SPHERE] =
  &BVHShapeDistancer<KDOP<16>, Sphere>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<16>,
  Capsule>::distance; distance_matrix[BV_KDOP16][GEOM_CONE] =
  &BVHShapeDistancer<KDOP<16>, Cone>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<16>,
  Cylinder>::distance; distance_matrix[BV_KDOP16][GEOM_CONVEX] =
  &BVHShapeDistancer<KDOP<16>, ConvexBase>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOP<16>,
  Plane>::distance; distance_matrix[BV_KDOP16][GEOM_HALFSPACE] =
  &BVHShapeDistancer<KDOP<16>, Halfspace>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOP<18>,
  Box>::distance; distance_matrix[BV_KDOP18][GEOM_SPHERE] =
  &BVHShapeDistancer<KDOP<18>, Sphere>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<18>,
  Capsule>::distance; distance_matrix[BV_KDOP18][GEOM_CONE] =
  &BVHShapeDistancer<KDOP<18>, Cone>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<18>,
  Cylinder>::distance; distance_matrix[BV_KDOP18][GEOM_CONVEX] =
  &BVHShapeDistancer<KDOP<18>, ConvexBase>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOP<18>,
  Plane>::distance; distance_matrix[BV_KDOP18][GEOM_HALFSPACE] =
  &BVHShapeDistancer<KDOP<18>, Halfspace>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOP<24>,
  Box>::distance; distance_matrix[BV_KDOP24][GEOM_SPHERE] =
  &BVHShapeDistancer<KDOP<24>, Sphere>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<24>,
  Capsule>::distance; distance_matrix[BV_KDOP24][GEOM_CONE] =
  &BVHShapeDistancer<KDOP<24>, Cone>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<24>,
  Cylinder>::distance; distance_matrix[BV_KDOP24][GEOM_CONVEX] =
  &BVHShapeDistancer<KDOP<24>, ConvexBase>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOP<24>,
  Plane>::distance; distance_matrix[BV_KDOP24][GEOM_HALFSPACE] =
  &BVHShapeDistancer<KDOP<24>, Halfspace>::distance;
  */

  // clang-format off
  distance_matrix[BV_kIOS][GEOM_BOX]                        = &BVHShapeDistancer<kIOS, Box>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE]                     = &BVHShapeDistancer<kIOS, Sphere>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE]                    = &BVHShapeDistancer<kIOS, Capsule>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE]                       = &BVHShapeDistancer<kIOS, Cone>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER]                   = &BVHShapeDistancer<kIOS, Cylinder>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX16]                   = &BVHShapeDistancer<kIOS, ConvexBase16>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX32]                   = &BVHShapeDistancer<kIOS, ConvexBase32>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE]                      = &BVHShapeDistancer<kIOS, Plane>::distance;
  distance_matrix[BV_kIOS][GEOM_HALFSPACE]                  = &BVHShapeDistancer<kIOS, Halfspace>::distance;
  distance_matrix[BV_kIOS][GEOM_ELLIPSOID]                  = &BVHShapeDistancer<kIOS, Ellipsoid>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX]                      = &BVHShapeDistancer<OBBRSS, Box>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE]                   = &BVHShapeDistancer<OBBRSS, Sphere>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE]                  = &BVHShapeDistancer<OBBRSS, Capsule>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE]                     = &BVHShapeDistancer<OBBRSS, Cone>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER]                 = &BVHShapeDistancer<OBBRSS, Cylinder>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX16]                 = &BVHShapeDistancer<OBBRSS, ConvexBase16>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX32]                 = &BVHShapeDistancer<OBBRSS, ConvexBase32>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE]                    = &BVHShapeDistancer<OBBRSS, Plane>::distance;
  distance_matrix[BV_OBBRSS][GEOM_HALFSPACE]                = &BVHShapeDistancer<OBBRSS, Halfspace>::distance;
  distance_matrix[BV_OBBRSS][GEOM_ELLIPSOID]                = &BVHShapeDistancer<OBBRSS, Ellipsoid>::distance;

  distance_matrix[HF_AABB][GEOM_BOX]                        = &HeightFieldShapeDistancer<AABB, Box>::distance;
  distance_matrix[HF_AABB][GEOM_SPHERE]                     = &HeightFieldShapeDistancer<AABB, Sphere>::distance;
  distance_matrix[HF_AABB][GEOM_CAPSULE]                    = &HeightFieldShapeDistancer<AABB, Capsule>::distance;
  distance_matrix[HF_AABB][GEOM_CONE]                       = &HeightFieldShapeDistancer<AABB, Cone>::distance;
  distance_matrix[HF_AABB][GEOM_CYLINDER]                   = &HeightFieldShapeDistancer<AABB, Cylinder>::distance;
  distance_matrix[HF_AABB][GEOM_CONVEX16]                   = &HeightFieldShapeDistancer<AABB, ConvexBase16>::distance;
  distance_matrix[HF_AABB][GEOM_CONVEX32]                   = &HeightFieldShapeDistancer<AABB, ConvexBase32>::distance;
  distance_matrix[HF_AABB][GEOM_PLANE]                      = &HeightFieldShapeDistancer<AABB, Plane>::distance;
  distance_matrix[HF_AABB][GEOM_HALFSPACE]                  = &HeightFieldShapeDistancer<AABB, Halfspace>::distance;
  distance_matrix[HF_AABB][GEOM_ELLIPSOID]                  = &HeightFieldShapeDistancer<AABB, Ellipsoid>::distance;

  distance_matrix[HF_OBBRSS][GEOM_BOX]                      = &HeightFieldShapeDistancer<OBBRSS, Box>::distance;
  distance_matrix[HF_OBBRSS][GEOM_SPHERE]                   = &HeightFieldShapeDistancer<OBBRSS, Sphere>::distance;
  distance_matrix[HF_OBBRSS][GEOM_CAPSULE]                  = &HeightFieldShapeDistancer<OBBRSS, Capsule>::distance;
  distance_matrix[HF_OBBRSS][GEOM_CONE]                     = &HeightFieldShapeDistancer<OBBRSS, Cone>::distance;
  distance_matrix[HF_OBBRSS][GEOM_CYLINDER]                 = &HeightFieldShapeDistancer<OBBRSS, Cylinder>::distance;
  distance_matrix[HF_OBBRSS][GEOM_CONVEX16]                 = &HeightFieldShapeDistancer<OBBRSS, ConvexBase16>::distance;
  distance_matrix[HF_OBBRSS][GEOM_CONVEX32]                 = &HeightFieldShapeDistancer<OBBRSS, ConvexBase32>::distance;
  distance_matrix[HF_OBBRSS][GEOM_PLANE]                    = &HeightFieldShapeDistancer<OBBRSS, Plane>::distance;
  distance_matrix[HF_OBBRSS][GEOM_HALFSPACE]                = &HeightFieldShapeDistancer<OBBRSS, Halfspace>::distance;
  distance_matrix[HF_OBBRSS][GEOM_ELLIPSOID]                = &HeightFieldShapeDistancer<OBBRSS, Ellipsoid>::distance;

  distance_matrix[BV_AABB][BV_AABB]                         = &BVHDistance<AABB>;
  distance_matrix[BV_OBB][BV_OBB]                           = &BVHDistance<OBB>;
  distance_matrix[BV_RSS][BV_RSS]                           = &BVHDistance<RSS>;
  distance_matrix[BV_kIOS][BV_kIOS]                         = &BVHDistance<kIOS>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS]                     = &BVHDistance<OBBRSS>;

#ifdef COAL_HAS_OCTOMAP
  distance_matrix[GEOM_OCTREE][GEOM_BOX]                    = &Distance<OcTree, Box>;
  distance_matrix[GEOM_OCTREE][GEOM_SPHERE]                 = &Distance<OcTree, Sphere>;
  distance_matrix[GEOM_OCTREE][GEOM_CAPSULE]                = &Distance<OcTree, Capsule>;
  distance_matrix[GEOM_OCTREE][GEOM_CONE]                   = &Distance<OcTree, Cone>;
  distance_matrix[GEOM_OCTREE][GEOM_CYLINDER]               = &Distance<OcTree, Cylinder>;
  distance_matrix[GEOM_OCTREE][GEOM_CONVEX16]               = &Distance<OcTree, ConvexBase16>;
  distance_matrix[GEOM_OCTREE][GEOM_CONVEX32]               = &Distance<OcTree, ConvexBase32>;
  distance_matrix[GEOM_OCTREE][GEOM_PLANE]                  = &Distance<OcTree, Plane>;
  distance_matrix[GEOM_OCTREE][GEOM_HALFSPACE]              = &Distance<OcTree, Halfspace>;
  distance_matrix[GEOM_OCTREE][GEOM_ELLIPSOID]              = &Distance<OcTree, Ellipsoid>;

  distance_matrix[GEOM_BOX][GEOM_OCTREE]                    = &Distance<Box, OcTree>;
  distance_matrix[GEOM_SPHERE][GEOM_OCTREE]                 = &Distance<Sphere, OcTree>;
  distance_matrix[GEOM_CAPSULE][GEOM_OCTREE]                = &Distance<Capsule, OcTree>;
  distance_matrix[GEOM_CONE][GEOM_OCTREE]                   = &Distance<Cone, OcTree>;
  distance_matrix[GEOM_CYLINDER][GEOM_OCTREE]               = &Distance<Cylinder, OcTree>;
  distance_matrix[GEOM_CONVEX16][GEOM_OCTREE]               = &Distance<ConvexBase16, OcTree>;
  distance_matrix[GEOM_CONVEX32][GEOM_OCTREE]               = &Distance<ConvexBase32, OcTree>;
  distance_matrix[GEOM_PLANE][GEOM_OCTREE]                  = &Distance<Plane, OcTree>;
  distance_matrix[GEOM_HALFSPACE][GEOM_OCTREE]              = &Distance<Halfspace, OcTree>;

  distance_matrix[GEOM_OCTREE][GEOM_OCTREE]                 = &Distance<OcTree, OcTree>;

  distance_matrix[GEOM_OCTREE][BV_AABB]                     = &Distance<OcTree, BVHModel<AABB> >;
  distance_matrix[GEOM_OCTREE][BV_OBB]                      = &Distance<OcTree, BVHModel<OBB> >;
  distance_matrix[GEOM_OCTREE][BV_RSS]                      = &Distance<OcTree, BVHModel<RSS> >;
  distance_matrix[GEOM_OCTREE][BV_OBBRSS]                   = &Distance<OcTree, BVHModel<OBBRSS> >;
  distance_matrix[GEOM_OCTREE][BV_kIOS]                     = &Distance<OcTree, BVHModel<kIOS> >;
  distance_matrix[GEOM_OCTREE][BV_KDOP16]                   = &Distance<OcTree, BVHModel<KDOP<16> > >;
  distance_matrix[GEOM_OCTREE][BV_KDOP18]                   = &Distance<OcTree, BVHModel<KDOP<18> > >;
  distance_matrix[GEOM_OCTREE][BV_KDOP24]                   = &Distance<OcTree, BVHModel<KDOP<24> > >;

  distance_matrix[BV_AABB][GEOM_OCTREE]                     = &Distance<BVHModel<AABB>, OcTree>;
  distance_matrix[BV_OBB][GEOM_OCTREE]                      = &Distance<BVHModel<OBB>, OcTree>;
  distance_matrix[BV_RSS][GEOM_OCTREE]                      = &Distance<BVHModel<RSS>, OcTree>;
  distance_matrix[BV_OBBRSS][GEOM_OCTREE]                   = &Distance<BVHModel<OBBRSS>, OcTree>;
  distance_matrix[BV_kIOS][GEOM_OCTREE]                     = &Distance<BVHModel<kIOS>, OcTree>;
  distance_matrix[BV_KDOP16][GEOM_OCTREE]                   = &Distance<BVHModel<KDOP<16> >, OcTree>;
  distance_matrix[BV_KDOP18][GEOM_OCTREE]                   = &Distance<BVHModel<KDOP<18> >, OcTree>;
  distance_matrix[BV_KDOP24][GEOM_OCTREE]                   = &Distance<BVHModel<KDOP<24> >, OcTree>;
  distance_matrix[GEOM_OCTREE][HF_AABB]                     = &distance_function_not_implemented;
  distance_matrix[GEOM_OCTREE][HF_OBBRSS]                   = &distance_function_not_implemented;
  distance_matrix[HF_AABB][GEOM_OCTREE]                     = &distance_function_not_implemented;
  distance_matrix[HF_OBBRSS][GEOM_OCTREE]                   = &distance_function_not_implemented;
#endif
  // clang-format on
}
// template struct DistanceFunctionMatrix;
}  // namespace coal
