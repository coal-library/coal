/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
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

/** \authors Jia Pan, Florent Lamiraux, Josef Mirabel, Louis Montaut */

#include "coal/narrowphase/minkowski_difference.h"
#include "coal/shape/geometric_shapes_traits.h"

namespace coal {
namespace details {

// ============================================================================
template <typename Shape0, typename Shape1, bool TransformIsIdentity,
          int _SupportOptions>
void getSupportTpl(const Shape0* s0, const Shape1* s1, const Matrix3s& oR1,
                   const Vec3s& ot1, const Vec3s& dir, Vec3s& support0,
                   Vec3s& support1, support_func_guess_t& hint,
                   ShapeSupportData data[2]) {
  assert(dir.norm() > Eigen::NumTraits<Scalar>::epsilon());
  getShapeSupport<_SupportOptions>(s0, dir, support0, hint[0], data[0]);

  if (TransformIsIdentity) {
    getShapeSupport<_SupportOptions>(s1, -dir, support1, hint[1], data[1]);
  } else {
    getShapeSupport<_SupportOptions>(s1, -oR1.transpose() * dir, support1,
                                     hint[1], data[1]);
    support1 = oR1 * support1 + ot1;
  }
}

// ============================================================================
template <typename Shape0, typename Shape1, bool TransformIsIdentity,
          int _SupportOptions>
void getSupportFuncTpl(const MinkowskiDiff& md, const Vec3s& dir,
                       Vec3s& support0, Vec3s& support1,
                       support_func_guess_t& hint, ShapeSupportData data[2]) {
  getSupportTpl<Shape0, Shape1, TransformIsIdentity, _SupportOptions>(
      static_cast<const Shape0*>(md.shapes[0]),
      static_cast<const Shape1*>(md.shapes[1]), md.oR1, md.ot1, dir, support0,
      support1, hint, data);
}

// ============================================================================
template <typename Shape0, int _SupportOptions>
MinkowskiDiff::GetSupportFunction makeGetSupportFunction1(
    const ShapeBase* s1, bool identity,
    Eigen::Array<Scalar, 1, 2>& swept_sphere_radius, ShapeSupportData data[2]) {
  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    // No need to store the information of swept sphere radius
    swept_sphere_radius[1] = 0;
  } else {
    // We store the information of swept sphere radius.
    // GJK and EPA will use this information to correct the solution they find.
    swept_sphere_radius[1] = s1->getSweptSphereRadius();
  }

  switch (s1->getNodeType()) {
    case GEOM_TRIANGLE:
      if (identity)
        return getSupportFuncTpl<Shape0, TriangleP, true, _SupportOptions>;
      else
        return getSupportFuncTpl<Shape0, TriangleP, false, _SupportOptions>;
    case GEOM_BOX:
      if (identity)
        return getSupportFuncTpl<Shape0, Box, true, _SupportOptions>;
      else
        return getSupportFuncTpl<Shape0, Box, false, _SupportOptions>;
    case GEOM_SPHERE:
      if (_SupportOptions == SupportOptions::NoSweptSphere) {
        // Sphere can be considered a swept-sphere point.
        swept_sphere_radius[1] += static_cast<const Sphere*>(s1)->radius;
      }
      if (identity)
        return getSupportFuncTpl<Shape0, Sphere, true, _SupportOptions>;
      else
        return getSupportFuncTpl<Shape0, Sphere, false, _SupportOptions>;
    case GEOM_ELLIPSOID:
      if (identity)
        return getSupportFuncTpl<Shape0, Ellipsoid, true, _SupportOptions>;
      else
        return getSupportFuncTpl<Shape0, Ellipsoid, false, _SupportOptions>;
    case GEOM_CAPSULE:
      if (_SupportOptions == SupportOptions::NoSweptSphere) {
        // Sphere can be considered as a swept-sphere segment.
        swept_sphere_radius[1] += static_cast<const Capsule*>(s1)->radius;
      }
      if (identity)
        return getSupportFuncTpl<Shape0, Capsule, true, _SupportOptions>;
      else
        return getSupportFuncTpl<Shape0, Capsule, false, _SupportOptions>;
    case GEOM_CONE:
      if (identity)
        return getSupportFuncTpl<Shape0, Cone, true, _SupportOptions>;
      else
        return getSupportFuncTpl<Shape0, Cone, false, _SupportOptions>;
    case GEOM_CYLINDER:
      if (identity)
        return getSupportFuncTpl<Shape0, Cylinder, true, _SupportOptions>;
      else
        return getSupportFuncTpl<Shape0, Cylinder, false, _SupportOptions>;
    case GEOM_CONVEX16: {
      const ConvexBase16* convex1 = static_cast<const ConvexBase16*>(s1);
      if (static_cast<size_t>(convex1->num_points) >
          ConvexBase16::num_vertices_large_convex_threshold) {
        data[1].visited.assign(convex1->num_points, false);
        data[1].last_dir.setZero();
        if (identity)
          return getSupportFuncTpl<Shape0, LargeConvex16, true,
                                   _SupportOptions>;
        else
          return getSupportFuncTpl<Shape0, LargeConvex16, false,
                                   _SupportOptions>;
      } else {
        if (identity)
          return getSupportFuncTpl<Shape0, SmallConvex16, true,
                                   _SupportOptions>;
        else
          return getSupportFuncTpl<Shape0, SmallConvex16, false,
                                   _SupportOptions>;
      }
    }
    case GEOM_CONVEX32: {
      const ConvexBase32* convex1 = static_cast<const ConvexBase32*>(s1);
      if (static_cast<size_t>(convex1->num_points) >
          ConvexBase32::num_vertices_large_convex_threshold) {
        data[1].visited.assign(convex1->num_points, false);
        data[1].last_dir.setZero();
        if (identity)
          return getSupportFuncTpl<Shape0, LargeConvex32, true,
                                   _SupportOptions>;
        else
          return getSupportFuncTpl<Shape0, LargeConvex32, false,
                                   _SupportOptions>;
      } else {
        if (identity)
          return getSupportFuncTpl<Shape0, SmallConvex32, true,
                                   _SupportOptions>;
        else
          return getSupportFuncTpl<Shape0, SmallConvex32, false,
                                   _SupportOptions>;
      }
    }
    default:
      COAL_THROW_PRETTY("Unsupported geometric shape.", std::logic_error);
  }
}

// ============================================================================
template <int _SupportOptions>
MinkowskiDiff::GetSupportFunction makeGetSupportFunction0(
    const ShapeBase* s0, const ShapeBase* s1, bool identity,
    Eigen::Array<Scalar, 1, 2>& swept_sphere_radius, ShapeSupportData data[2]) {
  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    // No need to store the information of swept sphere radius
    swept_sphere_radius[0] = 0;
  } else {
    // We store the information of swept sphere radius.
    // GJK and EPA will use this information to correct the solution they find.
    swept_sphere_radius[0] = s0->getSweptSphereRadius();
  }

  switch (s0->getNodeType()) {
    case GEOM_TRIANGLE:
      return makeGetSupportFunction1<TriangleP, _SupportOptions>(
          s1, identity, swept_sphere_radius, data);
      break;
    case GEOM_BOX:
      return makeGetSupportFunction1<Box, _SupportOptions>(
          s1, identity, swept_sphere_radius, data);
      break;
    case GEOM_SPHERE:
      if (_SupportOptions == SupportOptions::NoSweptSphere) {
        // Sphere can always be considered as a swept-sphere point.
        swept_sphere_radius[0] += static_cast<const Sphere*>(s0)->radius;
      }
      return makeGetSupportFunction1<Sphere, _SupportOptions>(
          s1, identity, swept_sphere_radius, data);
      break;
    case GEOM_ELLIPSOID:
      return makeGetSupportFunction1<Ellipsoid, _SupportOptions>(
          s1, identity, swept_sphere_radius, data);
      break;
    case GEOM_CAPSULE:
      if (_SupportOptions == SupportOptions::NoSweptSphere) {
        // Capsule can always be considered as a swept-sphere segment.
        swept_sphere_radius[0] += static_cast<const Capsule*>(s0)->radius;
      }
      return makeGetSupportFunction1<Capsule, _SupportOptions>(
          s1, identity, swept_sphere_radius, data);
      break;
    case GEOM_CONE:
      return makeGetSupportFunction1<Cone, _SupportOptions>(
          s1, identity, swept_sphere_radius, data);
      break;
    case GEOM_CYLINDER:
      return makeGetSupportFunction1<Cylinder, _SupportOptions>(
          s1, identity, swept_sphere_radius, data);
      break;
    case GEOM_CONVEX16: {
      const ConvexBase16* convex0 = static_cast<const ConvexBase16*>(s0);
      if (static_cast<size_t>(convex0->num_points) >
          ConvexBase16::num_vertices_large_convex_threshold) {
        data[0].visited.assign(convex0->num_points, false);
        data[0].last_dir.setZero();
        return makeGetSupportFunction1<LargeConvex16, _SupportOptions>(
            s1, identity, swept_sphere_radius, data);
      } else
        return makeGetSupportFunction1<SmallConvex16, _SupportOptions>(
            s1, identity, swept_sphere_radius, data);
      break;
    }
    case GEOM_CONVEX32: {
      const ConvexBase32* convex0 = static_cast<const ConvexBase32*>(s0);
      if (static_cast<size_t>(convex0->num_points) >
          ConvexBase32::num_vertices_large_convex_threshold) {
        data[0].visited.assign(convex0->num_points, false);
        data[0].last_dir.setZero();
        return makeGetSupportFunction1<LargeConvex32, _SupportOptions>(
            s1, identity, swept_sphere_radius, data);
      } else
        return makeGetSupportFunction1<SmallConvex32, _SupportOptions>(
            s1, identity, swept_sphere_radius, data);
      break;
    }
    default:
      COAL_THROW_PRETTY("Unsupported geometric shape", std::logic_error);
  }
}

// ============================================================================
bool getNormalizeSupportDirection(const ShapeBase* shape) {
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      return (bool)shape_traits<TriangleP>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_BOX:
      return (bool)shape_traits<Box>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_SPHERE:
      return (bool)shape_traits<Sphere>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_ELLIPSOID:
      return (bool)shape_traits<Ellipsoid>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CAPSULE:
      return (bool)shape_traits<Capsule>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CONE:
      return (bool)shape_traits<Cone>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CYLINDER:
      return (bool)shape_traits<Cylinder>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CONVEX16:
      return (bool)shape_traits<ConvexBase16>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CONVEX32:
      return (bool)shape_traits<ConvexBase32>::NeedNesterovNormalizeHeuristic;
      break;
    default:
      COAL_THROW_PRETTY("Unsupported geometric shape", std::logic_error);
  }
}

// ============================================================================
void getNormalizeSupportDirectionFromShapes(const ShapeBase* shape0,
                                            const ShapeBase* shape1,
                                            bool& normalize_support_direction) {
  normalize_support_direction = getNormalizeSupportDirection(shape0) &&
                                getNormalizeSupportDirection(shape1);
}

// ============================================================================
template <int _SupportOptions>
void MinkowskiDiff::set(const ShapeBase* shape0, const ShapeBase* shape1,
                        const Transform3s& tf0, const Transform3s& tf1) {
  shapes[0] = shape0;
  shapes[1] = shape1;
  getNormalizeSupportDirectionFromShapes(shape0, shape1,
                                         normalize_support_direction);

  oR1.noalias() = tf0.getRotation().transpose() * tf1.getRotation();
  ot1.noalias() = tf0.getRotation().transpose() *
                  (tf1.getTranslation() - tf0.getTranslation());

  bool identity = (oR1.isIdentity() && ot1.isZero());

  getSupportFunc = makeGetSupportFunction0<_SupportOptions>(
      shape0, shape1, identity, swept_sphere_radius, data);
}
// clang-format off
template void COAL_DLLAPI MinkowskiDiff::set<SupportOptions::NoSweptSphere>(const ShapeBase*, const ShapeBase*);

template void COAL_DLLAPI MinkowskiDiff::set<SupportOptions::WithSweptSphere>(const ShapeBase*, const ShapeBase*);
// clang-format on

// ============================================================================
template <int _SupportOptions>
void MinkowskiDiff::set(const ShapeBase* shape0, const ShapeBase* shape1) {
  shapes[0] = shape0;
  shapes[1] = shape1;
  getNormalizeSupportDirectionFromShapes(shape0, shape1,
                                         normalize_support_direction);

  oR1.setIdentity();
  ot1.setZero();

  getSupportFunc = makeGetSupportFunction0<_SupportOptions>(
      shape0, shape1, true, swept_sphere_radius, data);
}
// clang-format off
template void COAL_DLLAPI MinkowskiDiff::set<SupportOptions::NoSweptSphere>(const ShapeBase*, const ShapeBase*, const Transform3s&, const Transform3s&);

template void COAL_DLLAPI MinkowskiDiff::set<SupportOptions::WithSweptSphere>(const ShapeBase*, const ShapeBase*, const Transform3s&, const Transform3s&);
// clang-format on

// ============================================================================
// clang-format off
template Vec3s COAL_DLLAPI MinkowskiDiff::support0<SupportOptions::NoSweptSphere>(const Vec3s&, int&) const;

template Vec3s COAL_DLLAPI MinkowskiDiff::support0<SupportOptions::WithSweptSphere>(const Vec3s&, int&) const;

template Vec3s COAL_DLLAPI MinkowskiDiff::support1<SupportOptions::NoSweptSphere>(const Vec3s&, int&) const;

template Vec3s COAL_DLLAPI MinkowskiDiff::support1<SupportOptions::WithSweptSphere>(const Vec3s&, int&) const;
// clang-format on

}  // namespace details
}  // namespace coal
