/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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
 *   * Neither the name of INRIA nor the names of its
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

/** \author Louis Montaut */

#include "coal/contact_patch_func_matrix.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/shape_shape_contact_patch_func.h"
#include "coal/BV/BV.h"

namespace coal {

template <typename T_BVH, typename T_SH>
struct BVHShapeComputeContactPatch {
  static void run(const CollisionGeometry* o1, const Transform3s& tf1,
                  const CollisionGeometry* o2, const Transform3s& tf2,
                  const CollisionResult& collision_result,
                  const ContactPatchSolver* csolver,
                  const ContactPatchRequest& request,
                  ContactPatchResult& result) {
    COAL_UNUSED_VARIABLE(o1);
    COAL_UNUSED_VARIABLE(tf1);
    COAL_UNUSED_VARIABLE(o2);
    COAL_UNUSED_VARIABLE(tf2);
    COAL_UNUSED_VARIABLE(csolver);
    for (size_t i = 0; i < collision_result.numContacts(); ++i) {
      if (i >= request.max_num_patch) {
        break;
      }
      const Contact& contact = collision_result.getContact(i);
      ContactPatch& contact_patch = result.getUnusedContactPatch();
      constructContactPatchFrameFromContact(contact, contact_patch);
      contact_patch.addPoint(contact.pos);
    }
  }
};

template <typename BV, typename Shape>
struct HeightFieldShapeComputeContactPatch {
  static void run(const CollisionGeometry* o1, const Transform3s& tf1,
                  const CollisionGeometry* o2, const Transform3s& tf2,
                  const CollisionResult& collision_result,
                  const ContactPatchSolver* csolver,
                  const ContactPatchRequest& request,
                  ContactPatchResult& result) {
    COAL_UNUSED_VARIABLE(o1);
    COAL_UNUSED_VARIABLE(tf1);
    COAL_UNUSED_VARIABLE(o2);
    COAL_UNUSED_VARIABLE(tf2);
    COAL_UNUSED_VARIABLE(csolver);
    for (size_t i = 0; i < collision_result.numContacts(); ++i) {
      if (i >= request.max_num_patch) {
        break;
      }
      const Contact& contact = collision_result.getContact(i);
      ContactPatch& contact_patch = result.getUnusedContactPatch();
      constructContactPatchFrameFromContact(contact, contact_patch);
      contact_patch.addPoint(contact.pos);
    }
  }
};

template <typename BV>
struct BVHComputeContactPatch {
  static void run(const CollisionGeometry* o1, const Transform3s& tf1,
                  const CollisionGeometry* o2, const Transform3s& tf2,
                  const CollisionResult& collision_result,
                  const ContactPatchSolver* csolver,
                  const ContactPatchRequest& request,
                  ContactPatchResult& result) {
    COAL_UNUSED_VARIABLE(o1);
    COAL_UNUSED_VARIABLE(tf1);
    COAL_UNUSED_VARIABLE(o2);
    COAL_UNUSED_VARIABLE(tf2);
    COAL_UNUSED_VARIABLE(csolver);
    for (size_t i = 0; i < collision_result.numContacts(); ++i) {
      if (i >= request.max_num_patch) {
        break;
      }
      const Contact& contact = collision_result.getContact(i);
      ContactPatch& contact_patch = result.getUnusedContactPatch();
      constructContactPatchFrameFromContact(contact, contact_patch);
      contact_patch.addPoint(contact.pos);
    }
  }
};

COAL_LOCAL void contact_patch_function_not_implemented(
    const CollisionGeometry* o1, const Transform3s& /*tf1*/,
    const CollisionGeometry* o2, const Transform3s& /*tf2*/,
    const CollisionResult& /*collision_result*/,
    const ContactPatchSolver* /*csolver*/,
    const ContactPatchRequest& /*request*/, ContactPatchResult& /*result*/) {
  NODE_TYPE node_type1 = o1->getNodeType();
  NODE_TYPE node_type2 = o2->getNodeType();

  COAL_THROW_PRETTY("Contact patch function between node type "
                        << std::string(get_node_type_name(node_type1))
                        << " and node type "
                        << std::string(get_node_type_name(node_type2))
                        << " is not yet supported.",
                    std::invalid_argument);
}

ContactPatchFunctionMatrix::ContactPatchFunctionMatrix() {
  for (int i = 0; i < NODE_COUNT; ++i) {
    for (int j = 0; j < NODE_COUNT; ++j) contact_patch_matrix[i][j] = nullptr;
  }

  // clang-format off
  contact_patch_matrix[GEOM_BOX][GEOM_BOX]             = &ShapeShapeContactPatch<Box, Box>;
  contact_patch_matrix[GEOM_BOX][GEOM_SPHERE]          = &ShapeShapeContactPatch<Box, Sphere>;
  contact_patch_matrix[GEOM_BOX][GEOM_CAPSULE]         = &ShapeShapeContactPatch<Box, Capsule>;
  contact_patch_matrix[GEOM_BOX][GEOM_CONE]            = &ShapeShapeContactPatch<Box, Cone>;
  contact_patch_matrix[GEOM_BOX][GEOM_CYLINDER]        = &ShapeShapeContactPatch<Box, Cylinder>;
  contact_patch_matrix[GEOM_BOX][GEOM_CONVEX16]        = &ShapeShapeContactPatch<Box, ConvexBase16>;
  contact_patch_matrix[GEOM_BOX][GEOM_CONVEX32]        = &ShapeShapeContactPatch<Box, ConvexBase32>;
  contact_patch_matrix[GEOM_BOX][GEOM_PLANE]           = &ShapeShapeContactPatch<Box, Plane>;
  contact_patch_matrix[GEOM_BOX][GEOM_HALFSPACE]       = &ShapeShapeContactPatch<Box, Halfspace>;
  contact_patch_matrix[GEOM_BOX][GEOM_ELLIPSOID]       = &ShapeShapeContactPatch<Box, Ellipsoid>;
  contact_patch_matrix[GEOM_BOX][GEOM_TRIANGLE]        = &ShapeShapeContactPatch<Box, TriangleP>;

  contact_patch_matrix[GEOM_SPHERE][GEOM_BOX]          = &ShapeShapeContactPatch<Sphere, Box>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_SPHERE]       = &ShapeShapeContactPatch<Sphere, Sphere>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CAPSULE]      = &ShapeShapeContactPatch<Sphere, Capsule>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CONE]         = &ShapeShapeContactPatch<Sphere, Cone>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CYLINDER]     = &ShapeShapeContactPatch<Sphere, Cylinder>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CONVEX16]     = &ShapeShapeContactPatch<Sphere, ConvexBase16>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CONVEX32]     = &ShapeShapeContactPatch<Sphere, ConvexBase32>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_PLANE]        = &ShapeShapeContactPatch<Sphere, Plane>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_HALFSPACE]    = &ShapeShapeContactPatch<Sphere, Halfspace>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_ELLIPSOID]    = &ShapeShapeContactPatch<Sphere, Ellipsoid>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_TRIANGLE]     = &ShapeShapeContactPatch<Sphere, TriangleP>;

  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_BOX]       = &ShapeShapeContactPatch<Ellipsoid, Box>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_SPHERE]    = &ShapeShapeContactPatch<Ellipsoid, Sphere>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE]   = &ShapeShapeContactPatch<Ellipsoid, Capsule>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CONE]      = &ShapeShapeContactPatch<Ellipsoid, Cone>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER]  = &ShapeShapeContactPatch<Ellipsoid, Cylinder>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CONVEX16]  = &ShapeShapeContactPatch<Ellipsoid, ConvexBase16>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CONVEX32]  = &ShapeShapeContactPatch<Ellipsoid, ConvexBase32>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_PLANE]     = &ShapeShapeContactPatch<Ellipsoid, Plane>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeContactPatch<Ellipsoid, Halfspace>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeContactPatch<Ellipsoid, Ellipsoid>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_TRIANGLE]  = &ShapeShapeContactPatch<Ellipsoid, TriangleP>;

  contact_patch_matrix[GEOM_CAPSULE][GEOM_BOX]         = &ShapeShapeContactPatch<Capsule, Box>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_SPHERE]      = &ShapeShapeContactPatch<Capsule, Sphere>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CAPSULE]     = &ShapeShapeContactPatch<Capsule, Capsule>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CONE]        = &ShapeShapeContactPatch<Capsule, Cone>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CYLINDER]    = &ShapeShapeContactPatch<Capsule, Cylinder>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CONVEX16]    = &ShapeShapeContactPatch<Capsule, ConvexBase16>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CONVEX32]    = &ShapeShapeContactPatch<Capsule, ConvexBase32>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_PLANE]       = &ShapeShapeContactPatch<Capsule, Plane>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_HALFSPACE]   = &ShapeShapeContactPatch<Capsule, Halfspace>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID]   = &ShapeShapeContactPatch<Capsule, Ellipsoid>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_TRIANGLE]    = &ShapeShapeContactPatch<Capsule, TriangleP>;

  contact_patch_matrix[GEOM_CONE][GEOM_BOX]            = &ShapeShapeContactPatch<Cone, Box>;
  contact_patch_matrix[GEOM_CONE][GEOM_SPHERE]         = &ShapeShapeContactPatch<Cone, Sphere>;
  contact_patch_matrix[GEOM_CONE][GEOM_CAPSULE]        = &ShapeShapeContactPatch<Cone, Capsule>;
  contact_patch_matrix[GEOM_CONE][GEOM_CONE]           = &ShapeShapeContactPatch<Cone, Cone>;
  contact_patch_matrix[GEOM_CONE][GEOM_CYLINDER]       = &ShapeShapeContactPatch<Cone, Cylinder>;
  contact_patch_matrix[GEOM_CONE][GEOM_CONVEX16]       = &ShapeShapeContactPatch<Cone, ConvexBase16>;
  contact_patch_matrix[GEOM_CONE][GEOM_CONVEX32]       = &ShapeShapeContactPatch<Cone, ConvexBase32>;
  contact_patch_matrix[GEOM_CONE][GEOM_PLANE]          = &ShapeShapeContactPatch<Cone, Plane>;
  contact_patch_matrix[GEOM_CONE][GEOM_HALFSPACE]      = &ShapeShapeContactPatch<Cone, Halfspace>;
  contact_patch_matrix[GEOM_CONE][GEOM_ELLIPSOID]      = &ShapeShapeContactPatch<Cone, Ellipsoid>;
  contact_patch_matrix[GEOM_CONE][GEOM_TRIANGLE]       = &ShapeShapeContactPatch<Cone, TriangleP>;

  contact_patch_matrix[GEOM_CYLINDER][GEOM_BOX]        = &ShapeShapeContactPatch<Cylinder, Box>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_SPHERE]     = &ShapeShapeContactPatch<Cylinder, Sphere>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CAPSULE]    = &ShapeShapeContactPatch<Cylinder, Capsule>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CONE]       = &ShapeShapeContactPatch<Cylinder, Cone>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CYLINDER]   = &ShapeShapeContactPatch<Cylinder, Cylinder>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CONVEX16]   = &ShapeShapeContactPatch<Cylinder, ConvexBase16>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CONVEX32]   = &ShapeShapeContactPatch<Cylinder, ConvexBase32>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_PLANE]      = &ShapeShapeContactPatch<Cylinder, Plane>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_HALFSPACE]  = &ShapeShapeContactPatch<Cylinder, Halfspace>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID]  = &ShapeShapeContactPatch<Cylinder, Ellipsoid>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_TRIANGLE]   = &ShapeShapeContactPatch<Cylinder, TriangleP>;

  contact_patch_matrix[GEOM_CONVEX16][GEOM_BOX]          = &ShapeShapeContactPatch<ConvexBase16, Box>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_SPHERE]       = &ShapeShapeContactPatch<ConvexBase16, Sphere>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_CAPSULE]      = &ShapeShapeContactPatch<ConvexBase16, Capsule>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_CONE]         = &ShapeShapeContactPatch<ConvexBase16, Cone>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_CYLINDER]     = &ShapeShapeContactPatch<ConvexBase16, Cylinder>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_CONVEX16]     = &ShapeShapeContactPatch<ConvexBase16, ConvexBase16>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_CONVEX32]     = &ShapeShapeContactPatch<ConvexBase16, ConvexBase32>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_PLANE]        = &ShapeShapeContactPatch<ConvexBase16, Plane>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_HALFSPACE]    = &ShapeShapeContactPatch<ConvexBase16, Halfspace>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_ELLIPSOID]    = &ShapeShapeContactPatch<ConvexBase16, Ellipsoid>;
  contact_patch_matrix[GEOM_CONVEX16][GEOM_TRIANGLE]     = &ShapeShapeContactPatch<ConvexBase16, TriangleP>;

  contact_patch_matrix[GEOM_CONVEX32][GEOM_BOX]          = &ShapeShapeContactPatch<ConvexBase32, Box>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_SPHERE]       = &ShapeShapeContactPatch<ConvexBase32, Sphere>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_CAPSULE]      = &ShapeShapeContactPatch<ConvexBase32, Capsule>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_CONE]         = &ShapeShapeContactPatch<ConvexBase32, Cone>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_CYLINDER]     = &ShapeShapeContactPatch<ConvexBase32, Cylinder>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_CONVEX16]     = &ShapeShapeContactPatch<ConvexBase32, ConvexBase16>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_CONVEX32]     = &ShapeShapeContactPatch<ConvexBase32, ConvexBase32>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_PLANE]        = &ShapeShapeContactPatch<ConvexBase32, Plane>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_HALFSPACE]    = &ShapeShapeContactPatch<ConvexBase32, Halfspace>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_ELLIPSOID]    = &ShapeShapeContactPatch<ConvexBase32, Ellipsoid>;
  contact_patch_matrix[GEOM_CONVEX32][GEOM_TRIANGLE]     = &ShapeShapeContactPatch<ConvexBase32, TriangleP>;

  contact_patch_matrix[GEOM_PLANE][GEOM_BOX]           = &ShapeShapeContactPatch<Plane, Box>;
  contact_patch_matrix[GEOM_PLANE][GEOM_SPHERE]        = &ShapeShapeContactPatch<Plane, Sphere>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CAPSULE]       = &ShapeShapeContactPatch<Plane, Capsule>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CONE]          = &ShapeShapeContactPatch<Plane, Cone>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CYLINDER]      = &ShapeShapeContactPatch<Plane, Cylinder>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CONVEX16]      = &ShapeShapeContactPatch<Plane, ConvexBase16>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CONVEX32]      = &ShapeShapeContactPatch<Plane, ConvexBase32>;
  contact_patch_matrix[GEOM_PLANE][GEOM_PLANE]         = &ShapeShapeContactPatch<Plane, Plane>;
  contact_patch_matrix[GEOM_PLANE][GEOM_HALFSPACE]     = &ShapeShapeContactPatch<Plane, Halfspace>;
  contact_patch_matrix[GEOM_PLANE][GEOM_ELLIPSOID]     = &ShapeShapeContactPatch<Plane, Ellipsoid>;
  contact_patch_matrix[GEOM_PLANE][GEOM_TRIANGLE]      = &ShapeShapeContactPatch<Plane, TriangleP>;

  contact_patch_matrix[GEOM_HALFSPACE][GEOM_BOX]       = &ShapeShapeContactPatch<Halfspace, Box>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_SPHERE]    = &ShapeShapeContactPatch<Halfspace, Sphere>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CAPSULE]   = &ShapeShapeContactPatch<Halfspace, Capsule>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CONE]      = &ShapeShapeContactPatch<Halfspace, Cone>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CYLINDER]  = &ShapeShapeContactPatch<Halfspace, Cylinder>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CONVEX16]  = &ShapeShapeContactPatch<Halfspace, ConvexBase16>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CONVEX32]  = &ShapeShapeContactPatch<Halfspace, ConvexBase32>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_PLANE]     = &ShapeShapeContactPatch<Halfspace, Plane>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeContactPatch<Halfspace, Halfspace>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_ELLIPSOID] = &ShapeShapeContactPatch<Halfspace, Ellipsoid>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_TRIANGLE]  = &ShapeShapeContactPatch<Halfspace, TriangleP>;

  contact_patch_matrix[GEOM_TRIANGLE][GEOM_BOX]        = &ShapeShapeContactPatch<TriangleP, Box>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_SPHERE]     = &ShapeShapeContactPatch<TriangleP, Sphere>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CAPSULE]    = &ShapeShapeContactPatch<TriangleP, Capsule>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CONE]       = &ShapeShapeContactPatch<TriangleP, Cone>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CYLINDER]   = &ShapeShapeContactPatch<TriangleP, Cylinder>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CONVEX16]   = &ShapeShapeContactPatch<TriangleP, ConvexBase16>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CONVEX32]   = &ShapeShapeContactPatch<TriangleP, ConvexBase32>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_PLANE]      = &ShapeShapeContactPatch<TriangleP, Plane>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_HALFSPACE]  = &ShapeShapeContactPatch<TriangleP, Halfspace>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_ELLIPSOID]  = &ShapeShapeContactPatch<TriangleP, Ellipsoid>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_TRIANGLE]   = &ShapeShapeContactPatch<TriangleP, TriangleP>;

  // TODO(louis): properly handle non-convex shapes like BVH, Octrees and Hfields.
  // The following functions work. However apart from the contact frame, these functions don't
  // compute more information than a call to `collide`.
  contact_patch_matrix[BV_AABB][GEOM_BOX]         = &BVHShapeComputeContactPatch<AABB, Box>::run;
  contact_patch_matrix[BV_AABB][GEOM_SPHERE]      = &BVHShapeComputeContactPatch<AABB, Sphere>::run;
  contact_patch_matrix[BV_AABB][GEOM_CAPSULE]     = &BVHShapeComputeContactPatch<AABB, Capsule>::run;
  contact_patch_matrix[BV_AABB][GEOM_CONE]        = &BVHShapeComputeContactPatch<AABB, Cone>::run;
  contact_patch_matrix[BV_AABB][GEOM_CYLINDER]    = &BVHShapeComputeContactPatch<AABB, Cylinder>::run;
  contact_patch_matrix[BV_AABB][GEOM_CONVEX16]    = &BVHShapeComputeContactPatch<AABB, ConvexBase16>::run;
  contact_patch_matrix[BV_AABB][GEOM_CONVEX32]    = &BVHShapeComputeContactPatch<AABB, ConvexBase32>::run;
  contact_patch_matrix[BV_AABB][GEOM_PLANE]       = &BVHShapeComputeContactPatch<AABB, Plane>::run;
  contact_patch_matrix[BV_AABB][GEOM_HALFSPACE]   = &BVHShapeComputeContactPatch<AABB, Halfspace>::run;
  contact_patch_matrix[BV_AABB][GEOM_ELLIPSOID]   = &BVHShapeComputeContactPatch<AABB, Ellipsoid>::run;

  contact_patch_matrix[BV_OBB][GEOM_BOX]          = &BVHShapeComputeContactPatch<OBB, Box>::run;
  contact_patch_matrix[BV_OBB][GEOM_SPHERE]       = &BVHShapeComputeContactPatch<OBB, Sphere>::run;
  contact_patch_matrix[BV_OBB][GEOM_CAPSULE]      = &BVHShapeComputeContactPatch<OBB, Capsule>::run;
  contact_patch_matrix[BV_OBB][GEOM_CONE]         = &BVHShapeComputeContactPatch<OBB, Cone>::run;
  contact_patch_matrix[BV_OBB][GEOM_CYLINDER]     = &BVHShapeComputeContactPatch<OBB, Cylinder>::run;
  contact_patch_matrix[BV_OBB][GEOM_CONVEX16]     = &BVHShapeComputeContactPatch<OBB, ConvexBase16>::run;
  contact_patch_matrix[BV_OBB][GEOM_CONVEX32]     = &BVHShapeComputeContactPatch<OBB, ConvexBase32>::run;
  contact_patch_matrix[BV_OBB][GEOM_PLANE]        = &BVHShapeComputeContactPatch<OBB, Plane>::run;
  contact_patch_matrix[BV_OBB][GEOM_HALFSPACE]    = &BVHShapeComputeContactPatch<OBB, Halfspace>::run;
  contact_patch_matrix[BV_OBB][GEOM_ELLIPSOID]    = &BVHShapeComputeContactPatch<OBB, Ellipsoid>::run;

  contact_patch_matrix[BV_RSS][GEOM_BOX]          = &BVHShapeComputeContactPatch<RSS, Box>::run;
  contact_patch_matrix[BV_RSS][GEOM_SPHERE]       = &BVHShapeComputeContactPatch<RSS, Sphere>::run;
  contact_patch_matrix[BV_RSS][GEOM_CAPSULE]      = &BVHShapeComputeContactPatch<RSS, Capsule>::run;
  contact_patch_matrix[BV_RSS][GEOM_CONE]         = &BVHShapeComputeContactPatch<RSS, Cone>::run;
  contact_patch_matrix[BV_RSS][GEOM_CYLINDER]     = &BVHShapeComputeContactPatch<RSS, Cylinder>::run;
  contact_patch_matrix[BV_RSS][GEOM_CONVEX16]     = &BVHShapeComputeContactPatch<RSS, ConvexBase16>::run;
  contact_patch_matrix[BV_RSS][GEOM_CONVEX32]     = &BVHShapeComputeContactPatch<RSS, ConvexBase32>::run;
  contact_patch_matrix[BV_RSS][GEOM_PLANE]        = &BVHShapeComputeContactPatch<RSS, Plane>::run;
  contact_patch_matrix[BV_RSS][GEOM_HALFSPACE]    = &BVHShapeComputeContactPatch<RSS, Halfspace>::run;
  contact_patch_matrix[BV_RSS][GEOM_ELLIPSOID]    = &BVHShapeComputeContactPatch<RSS, Ellipsoid>::run;

  contact_patch_matrix[BV_KDOP16][GEOM_BOX]       = &BVHShapeComputeContactPatch<KDOP<16>, Box>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_SPHERE]    = &BVHShapeComputeContactPatch<KDOP<16>, Sphere>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_CAPSULE]   = &BVHShapeComputeContactPatch<KDOP<16>, Capsule>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_CONE]      = &BVHShapeComputeContactPatch<KDOP<16>, Cone>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_CYLINDER]  = &BVHShapeComputeContactPatch<KDOP<16>, Cylinder>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_CONVEX16]  = &BVHShapeComputeContactPatch<KDOP<16>, ConvexBase16>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_CONVEX32]  = &BVHShapeComputeContactPatch<KDOP<16>, ConvexBase32>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_PLANE]     = &BVHShapeComputeContactPatch<KDOP<16>, Plane>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeComputeContactPatch<KDOP<16>, Halfspace>::run;
  contact_patch_matrix[BV_KDOP16][GEOM_ELLIPSOID] = &BVHShapeComputeContactPatch<KDOP<16>, Ellipsoid>::run;

  contact_patch_matrix[BV_KDOP18][GEOM_BOX]       = &BVHShapeComputeContactPatch<KDOP<18>, Box>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_SPHERE]    = &BVHShapeComputeContactPatch<KDOP<18>, Sphere>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_CAPSULE]   = &BVHShapeComputeContactPatch<KDOP<18>, Capsule>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_CONE]      = &BVHShapeComputeContactPatch<KDOP<18>, Cone>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_CYLINDER]  = &BVHShapeComputeContactPatch<KDOP<18>, Cylinder>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_CONVEX16]  = &BVHShapeComputeContactPatch<KDOP<18>, ConvexBase16>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_CONVEX32]  = &BVHShapeComputeContactPatch<KDOP<18>, ConvexBase32>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_PLANE]     = &BVHShapeComputeContactPatch<KDOP<18>, Plane>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeComputeContactPatch<KDOP<18>, Halfspace>::run;
  contact_patch_matrix[BV_KDOP18][GEOM_ELLIPSOID] = &BVHShapeComputeContactPatch<KDOP<18>, Ellipsoid>::run;

  contact_patch_matrix[BV_KDOP24][GEOM_BOX]       = &BVHShapeComputeContactPatch<KDOP<24>, Box>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_SPHERE]    = &BVHShapeComputeContactPatch<KDOP<24>, Sphere>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_CAPSULE]   = &BVHShapeComputeContactPatch<KDOP<24>, Capsule>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_CONE]      = &BVHShapeComputeContactPatch<KDOP<24>, Cone>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_CYLINDER]  = &BVHShapeComputeContactPatch<KDOP<24>, Cylinder>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_CONVEX16]  = &BVHShapeComputeContactPatch<KDOP<24>, ConvexBase16>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_CONVEX32]  = &BVHShapeComputeContactPatch<KDOP<24>, ConvexBase32>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_PLANE]     = &BVHShapeComputeContactPatch<KDOP<24>, Plane>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeComputeContactPatch<KDOP<24>, Halfspace>::run;
  contact_patch_matrix[BV_KDOP24][GEOM_ELLIPSOID] = &BVHShapeComputeContactPatch<KDOP<24>, Ellipsoid>::run;

  contact_patch_matrix[BV_kIOS][GEOM_BOX]         = &BVHShapeComputeContactPatch<kIOS, Box>::run;
  contact_patch_matrix[BV_kIOS][GEOM_SPHERE]      = &BVHShapeComputeContactPatch<kIOS, Sphere>::run;
  contact_patch_matrix[BV_kIOS][GEOM_CAPSULE]     = &BVHShapeComputeContactPatch<kIOS, Capsule>::run;
  contact_patch_matrix[BV_kIOS][GEOM_CONE]        = &BVHShapeComputeContactPatch<kIOS, Cone>::run;
  contact_patch_matrix[BV_kIOS][GEOM_CYLINDER]    = &BVHShapeComputeContactPatch<kIOS, Cylinder>::run;
  contact_patch_matrix[BV_kIOS][GEOM_CONVEX16]    = &BVHShapeComputeContactPatch<kIOS, ConvexBase16>::run;
  contact_patch_matrix[BV_kIOS][GEOM_CONVEX32]    = &BVHShapeComputeContactPatch<kIOS, ConvexBase32>::run;
  contact_patch_matrix[BV_kIOS][GEOM_PLANE]       = &BVHShapeComputeContactPatch<kIOS, Plane>::run;
  contact_patch_matrix[BV_kIOS][GEOM_HALFSPACE]   = &BVHShapeComputeContactPatch<kIOS, Halfspace>::run;
  contact_patch_matrix[BV_kIOS][GEOM_ELLIPSOID]   = &BVHShapeComputeContactPatch<kIOS, Ellipsoid>::run;

  contact_patch_matrix[BV_OBBRSS][GEOM_BOX]       = &BVHShapeComputeContactPatch<OBBRSS, Box>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_SPHERE]    = &BVHShapeComputeContactPatch<OBBRSS, Sphere>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_CAPSULE]   = &BVHShapeComputeContactPatch<OBBRSS, Capsule>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_CONE]      = &BVHShapeComputeContactPatch<OBBRSS, Cone>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_CYLINDER]  = &BVHShapeComputeContactPatch<OBBRSS, Cylinder>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_CONVEX16]  = &BVHShapeComputeContactPatch<OBBRSS, ConvexBase16>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_CONVEX32]  = &BVHShapeComputeContactPatch<OBBRSS, ConvexBase32>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_PLANE]     = &BVHShapeComputeContactPatch<OBBRSS, Plane>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeComputeContactPatch<OBBRSS, Halfspace>::run;
  contact_patch_matrix[BV_OBBRSS][GEOM_ELLIPSOID] = &BVHShapeComputeContactPatch<OBBRSS, Ellipsoid>::run;

  contact_patch_matrix[HF_AABB][GEOM_BOX]         = &HeightFieldShapeComputeContactPatch<AABB, Box>::run;
  contact_patch_matrix[HF_AABB][GEOM_SPHERE]      = &HeightFieldShapeComputeContactPatch<AABB, Sphere>::run;
  contact_patch_matrix[HF_AABB][GEOM_CAPSULE]     = &HeightFieldShapeComputeContactPatch<AABB, Capsule>::run;
  contact_patch_matrix[HF_AABB][GEOM_CONE]        = &HeightFieldShapeComputeContactPatch<AABB, Cone>::run;
  contact_patch_matrix[HF_AABB][GEOM_CYLINDER]    = &HeightFieldShapeComputeContactPatch<AABB, Cylinder>::run;
  contact_patch_matrix[HF_AABB][GEOM_CONVEX16]    = &HeightFieldShapeComputeContactPatch<AABB, ConvexBase16>::run;
  contact_patch_matrix[HF_AABB][GEOM_CONVEX32]    = &HeightFieldShapeComputeContactPatch<AABB, ConvexBase32>::run;
  contact_patch_matrix[HF_AABB][GEOM_PLANE]       = &HeightFieldShapeComputeContactPatch<AABB, Plane>::run;
  contact_patch_matrix[HF_AABB][GEOM_HALFSPACE]   = &HeightFieldShapeComputeContactPatch<AABB, Halfspace>::run;
  contact_patch_matrix[HF_AABB][GEOM_ELLIPSOID]   = &HeightFieldShapeComputeContactPatch<AABB, Ellipsoid>::run;

  contact_patch_matrix[HF_OBBRSS][GEOM_BOX]       = &HeightFieldShapeComputeContactPatch<OBBRSS, Box>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_SPHERE]    = &HeightFieldShapeComputeContactPatch<OBBRSS, Sphere>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_CAPSULE]   = &HeightFieldShapeComputeContactPatch<OBBRSS, Capsule>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_CONE]      = &HeightFieldShapeComputeContactPatch<OBBRSS, Cone>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_CYLINDER]  = &HeightFieldShapeComputeContactPatch<OBBRSS, Cylinder>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_CONVEX16]  = &HeightFieldShapeComputeContactPatch<OBBRSS, ConvexBase16>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_CONVEX32]  = &HeightFieldShapeComputeContactPatch<OBBRSS, ConvexBase32>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_PLANE]     = &HeightFieldShapeComputeContactPatch<OBBRSS, Plane>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_HALFSPACE] = &HeightFieldShapeComputeContactPatch<OBBRSS, Halfspace>::run;
  contact_patch_matrix[HF_OBBRSS][GEOM_ELLIPSOID] = &HeightFieldShapeComputeContactPatch<OBBRSS, Ellipsoid>::run;

  contact_patch_matrix[BV_AABB][BV_AABB]          = &BVHComputeContactPatch<AABB>::run;
  contact_patch_matrix[BV_OBB][BV_OBB]            = &BVHComputeContactPatch<OBB>::run;
  contact_patch_matrix[BV_RSS][BV_RSS]            = &BVHComputeContactPatch<RSS>::run;
  contact_patch_matrix[BV_KDOP16][BV_KDOP16]      = &BVHComputeContactPatch<KDOP<16>>::run;
  contact_patch_matrix[BV_KDOP18][BV_KDOP18]      = &BVHComputeContactPatch<KDOP<18>>::run;
  contact_patch_matrix[BV_KDOP24][BV_KDOP24]      = &BVHComputeContactPatch<KDOP<24>>::run;
  contact_patch_matrix[BV_kIOS][BV_kIOS]          = &BVHComputeContactPatch<kIOS>::run;
  contact_patch_matrix[BV_OBBRSS][BV_OBBRSS]      = &BVHComputeContactPatch<OBBRSS>::run;

  // TODO(louis): octrees
#ifdef COAL_HAS_OCTOMAP
  contact_patch_matrix[GEOM_OCTREE][GEOM_OCTREE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_BOX] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_SPHERE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_CONE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_CONVEX] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_PLANE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_ELLIPSOID] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][GEOM_TRIANGLE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_AABB] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_OBB] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_RSS] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_OBBRSS] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_kIOS] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_KDOP16] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_KDOP18] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][BV_KDOP24] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][HF_AABB] = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_OCTREE][HF_OBBRSS] = &contact_patch_function_not_implemented;

  contact_patch_matrix[GEOM_BOX][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_SPHERE][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_CONE][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_CONVEX][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_PLANE][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_AABB][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_OBB][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_RSS][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_OBBRSS][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_kIOS][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_KDOP16][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_KDOP18][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[BV_KDOP24][GEOM_OCTREE]  = &contact_patch_function_not_implemented;
  contact_patch_matrix[HF_AABB][GEOM_OCTREE] = &contact_patch_function_not_implemented;
  contact_patch_matrix[HF_OBBRSS][GEOM_OCTREE] = &contact_patch_function_not_implemented;
#endif
  // clang-format on
}

}  // namespace coal
