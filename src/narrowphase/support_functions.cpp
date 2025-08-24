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

#include "coal/narrowphase/support_functions.h"

#include <algorithm>

namespace coal {
namespace details {

// ============================================================================
#define CALL_GET_SHAPE_SUPPORT(ShapeType)                                     \
  getShapeSupport<_SupportOptions>(static_cast<const ShapeType*>(shape), dir, \
                                   support, hint, support_data)

template <int _SupportOptions>
Vec3s getSupport(const ShapeBase* shape, const Vec3s& dir, int& hint) {
  Vec3s support;
  ShapeSupportData support_data;
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      CALL_GET_SHAPE_SUPPORT(TriangleP);
      break;
    case GEOM_BOX:
      CALL_GET_SHAPE_SUPPORT(Box);
      break;
    case GEOM_SPHERE:
      CALL_GET_SHAPE_SUPPORT(Sphere);
      break;
    case GEOM_ELLIPSOID:
      CALL_GET_SHAPE_SUPPORT(Ellipsoid);
      break;
    case GEOM_CAPSULE:
      CALL_GET_SHAPE_SUPPORT(Capsule);
      break;
    case GEOM_CONE:
      CALL_GET_SHAPE_SUPPORT(Cone);
      break;
    case GEOM_CYLINDER:
      CALL_GET_SHAPE_SUPPORT(Cylinder);
      break;
    case GEOM_CONVEX16:
      CALL_GET_SHAPE_SUPPORT(ConvexBaseTpl<Triangle16::IndexType>);
      break;
    case GEOM_CONVEX32:
      CALL_GET_SHAPE_SUPPORT(ConvexBaseTpl<Triangle32::IndexType>);
      break;
    case GEOM_PLANE:
    case GEOM_HALFSPACE:
    default:
      support.setZero();
      ;  // nothing
  }

  return support;
}
#undef CALL_GET_SHAPE_SUPPORT

// Explicit instantiation
// clang-format off
template COAL_DLLAPI Vec3s getSupport<SupportOptions::NoSweptSphere>(const ShapeBase*, const Vec3s&, int&);

template COAL_DLLAPI Vec3s getSupport<SupportOptions::WithSweptSphere>(const ShapeBase*, const Vec3s&, int&);
// clang-format on

// ============================================================================
#define getShapeSupportTplInstantiation(ShapeType)                            \
  template COAL_DLLAPI void getShapeSupport<SupportOptions::NoSweptSphere>(   \
      const ShapeType* shape_, const Vec3s& dir, Vec3s& support, int& hint,   \
      ShapeSupportData& support_data);                                        \
                                                                              \
  template COAL_DLLAPI void getShapeSupport<SupportOptions::WithSweptSphere>( \
      const ShapeType* shape_, const Vec3s& dir, Vec3s& support, int& hint,   \
      ShapeSupportData& support_data);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const TriangleP* triangle, const Vec3s& dir,
                     Vec3s& support, int& /*unused*/,
                     ShapeSupportData& /*unused*/) {
  Scalar dota = dir.dot(triangle->a);
  Scalar dotb = dir.dot(triangle->b);
  Scalar dotc = dir.dot(triangle->c);
  if (dota > dotb) {
    if (dotc > dota) {
      support = triangle->c;
    } else {
      support = triangle->a;
    }
  } else {
    if (dotc > dotb) {
      support = triangle->c;
    } else {
      support = triangle->b;
    }
  }

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += triangle->getSweptSphereRadius() * dir.normalized();
  }
}
getShapeSupportTplInstantiation(TriangleP);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Box* box, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/) {
  // The inflate value is simply to make the specialized functions with box
  // have a preferred side for edge cases.
  static const Scalar inflate =
      (dir.array() == 0).any() ? 1 + Scalar(1e-10) : 1;
  static const Scalar dummy_precision =
      Eigen::NumTraits<Scalar>::dummy_precision();
  Vec3s support1 = (dir.array() > dummy_precision).select(box->halfSide, 0);
  Vec3s support2 =
      (dir.array() < -dummy_precision).select(-inflate * box->halfSide, 0);
  support.noalias() = support1 + support2;

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += box->getSweptSphereRadius() * dir.normalized();
  }
}
getShapeSupportTplInstantiation(Box);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Sphere* sphere, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/) {
  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support.noalias() =
        (sphere->radius + sphere->getSweptSphereRadius()) * dir.normalized();
  } else {
    support.setZero();
  }

  COAL_UNUSED_VARIABLE(sphere);
  COAL_UNUSED_VARIABLE(dir);
}
getShapeSupportTplInstantiation(Sphere);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Ellipsoid* ellipsoid, const Vec3s& dir,
                     Vec3s& support, int& /*unused*/,
                     ShapeSupportData& /*unused*/) {
  Scalar a2 = ellipsoid->radii[0] * ellipsoid->radii[0];
  Scalar b2 = ellipsoid->radii[1] * ellipsoid->radii[1];
  Scalar c2 = ellipsoid->radii[2] * ellipsoid->radii[2];

  Vec3s v(a2 * dir[0], b2 * dir[1], c2 * dir[2]);

  Scalar d = std::sqrt(v.dot(dir));

  support = v / d;

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += ellipsoid->getSweptSphereRadius() * dir.normalized();
  }
}
getShapeSupportTplInstantiation(Ellipsoid);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Capsule* capsule, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/) {
  static const Scalar dummy_precision =
      Eigen::NumTraits<Scalar>::dummy_precision();
  support.setZero();
  if (dir[2] > dummy_precision) {
    support[2] = capsule->halfLength;
  } else if (dir[2] < -dummy_precision) {
    support[2] = -capsule->halfLength;
  }

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support +=
        (capsule->radius + capsule->getSweptSphereRadius()) * dir.normalized();
  }
}
getShapeSupportTplInstantiation(Capsule);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Cone* cone, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/) {
  static const Scalar dummy_precision =
      Eigen::NumTraits<Scalar>::dummy_precision();

  // The cone radius is, for -h < z < h, (h - z) * r / (2*h)
  // The inflate value is simply to make the specialized functions with cone
  // have a preferred side for edge cases.
  static const Scalar inflate = 1 + Scalar(1e-10);
  Scalar h = cone->halfLength;
  Scalar r = cone->radius;

  if (dir.head<2>().isZero(dummy_precision)) {
    support.head<2>().setZero();
    if (dir[2] > dummy_precision) {
      support[2] = h;
    } else {
      support[2] = -inflate * h;
    }
  } else {
    Scalar zdist = dir[0] * dir[0] + dir[1] * dir[1];
    Scalar len = zdist + dir[2] * dir[2];
    zdist = std::sqrt(zdist);

    if (dir[2] <= 0) {
      Scalar rad = r / zdist;
      support.head<2>() = rad * dir.head<2>();
      support[2] = -h;
    } else {
      len = std::sqrt(len);
      Scalar sin_a = r / std::sqrt(r * r + 4 * h * h);

      if (dir[2] > len * sin_a)
        support << 0, 0, h;
      else {
        Scalar rad = r / zdist;
        support.head<2>() = rad * dir.head<2>();
        support[2] = -h;
      }
    }
  }

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += cone->getSweptSphereRadius() * dir.normalized();
  }
}
getShapeSupportTplInstantiation(Cone);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Cylinder* cylinder, const Vec3s& dir, Vec3s& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/) {
  static const Scalar dummy_precision =
      Eigen::NumTraits<Scalar>::dummy_precision();

  // The inflate value is simply to make the specialized functions with cylinder
  // have a preferred side for edge cases.
  static const Scalar inflate = 1 + Scalar(1e-10);
  Scalar half_h = cylinder->halfLength;
  Scalar r = cylinder->radius;

  const bool dir_is_aligned_with_z = dir.head<2>().isZero(dummy_precision);
  if (dir_is_aligned_with_z) half_h *= inflate;

  if (dir[2] > dummy_precision) {
    support[2] = half_h;
  } else if (dir[2] < -dummy_precision) {
    support[2] = -half_h;
  } else {
    support[2] = 0;
    r *= inflate;
  }

  if (dir_is_aligned_with_z) {
    support.head<2>().setZero();
  } else {
    support.head<2>() = dir.head<2>().normalized() * r;
  }

  assert(fabs(support[0] * dir[1] - support[1] * dir[0]) <
         sqrt(std::numeric_limits<Scalar>::epsilon()));

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += cylinder->getSweptSphereRadius() * dir.normalized();
  }
}
getShapeSupportTplInstantiation(Cylinder);

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupportLog(const ConvexBaseTpl<IndexType>* convex,
                        const Vec3s& dir, Vec3s& support, int& hint,
                        ShapeSupportData& support_data) {
  assert(convex->neighbors != nullptr && "Convex has no neighbors.");

  // Use warm start if current support direction is distant from last support
  // direction.
  const Scalar use_warm_start_threshold = Scalar(0.9);
  Vec3s dir_normalized = dir.normalized();
  if (!support_data.last_dir.isZero() &&
      !convex->support_warm_starts.points.empty() &&
      support_data.last_dir.dot(dir_normalized) < use_warm_start_threshold) {
    // Change hint if last dir is too far from current dir.
    Scalar maxdot = convex->support_warm_starts.points[0].dot(dir);
    hint = int(convex->support_warm_starts.indices[0]);
    for (size_t i = 1; i < convex->support_warm_starts.points.size(); ++i) {
      Scalar dot = convex->support_warm_starts.points[i].dot(dir);
      if (dot > maxdot) {
        maxdot = dot;
        hint = int(convex->support_warm_starts.indices[i]);
      }
    }
  }
  support_data.last_dir = dir_normalized;

  const std::vector<Vec3s>& pts = *(convex->points);
  typedef typename ConvexBaseTpl<IndexType>::Neighbors Neighbors;
  const std::vector<Neighbors>& nn = *(convex->neighbors);

  if (hint < 0 || hint >= (int)convex->num_points) {
    hint = 0;
  }
  Scalar maxdot = pts[static_cast<size_t>(hint)].dot(dir);
  std::vector<int8_t>& visited = support_data.visited;
  if (support_data.visited.size() == convex->num_points) {
    std::fill(visited.begin(), visited.end(), false);
  } else {
    // std::vector::assign not only assigns the values of the vector but also
    // resizes the vector. So if `visited` has not been set up yet, this makes
    // sure the size convex's points and visited are identical.
    support_data.visited.assign(convex->num_points, false);
  }
  visited[static_cast<std::size_t>(hint)] = true;
  // When the first face is orthogonal to dir, all the dot products will be
  // equal. Yet, the neighbors must be visited.
  bool found = true;
  bool loose_check = true;
  while (found) {
    const Neighbors& n = nn[static_cast<size_t>(hint)];
    found = false;
    IndexType current_vertex_idx = IndexType(hint);
    for (IndexType in = 0; in < n.count; ++in) {
      const IndexType ip = convex->neighbor(current_vertex_idx, in);
      if (visited[ip]) continue;
      visited[ip] = true;
      const Scalar dot = pts[ip].dot(dir);
      bool better = false;
      if (dot > maxdot) {
        better = true;
        loose_check = false;
      } else if (loose_check && dot == maxdot)
        better = true;
      if (better) {
        maxdot = dot;
        hint = static_cast<int>(ip);
        found = true;
      }
    }
  }

  support = pts[static_cast<size_t>(hint)];

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += convex->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupportLinear(const ConvexBaseTpl<IndexType>* convex,
                           const Vec3s& dir, Vec3s& support, int& hint,
                           ShapeSupportData& /*unused*/) {
  const std::vector<Vec3s>& pts = *(convex->points);

  hint = 0;
  Scalar maxdot = pts[0].dot(dir);
  for (int i = 1; i < (int)convex->num_points; ++i) {
    Scalar dot = pts[static_cast<size_t>(i)].dot(dir);
    if (dot > maxdot) {
      maxdot = dot;
      hint = i;
    }
  }

  support = pts[static_cast<size_t>(hint)];

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += convex->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupport(const ConvexBaseTpl<IndexType>* convex, const Vec3s& dir,
                     Vec3s& support, int& hint,
                     ShapeSupportData& support_data) {
  // TODO add benchmark to set a proper value for switching between linear and
  // logarithmic.
  if (convex->num_points >
          ConvexBaseTpl<IndexType>::num_vertices_large_convex_threshold &&
      convex->neighbors != nullptr) {
    getShapeSupportLog<_SupportOptions>(convex, dir, support, hint,
                                        support_data);
  } else {
    getShapeSupportLinear<_SupportOptions>(convex, dir, support, hint,
                                           support_data);
  }
}
getShapeSupportTplInstantiation(ConvexBaseTpl<Triangle16::IndexType>);
getShapeSupportTplInstantiation(ConvexBaseTpl<Triangle32::IndexType>);

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupport(const SmallConvex<IndexType>* convex, const Vec3s& dir,
                     Vec3s& support, int& hint,
                     ShapeSupportData& support_data) {
  getShapeSupportLinear<_SupportOptions>(
      reinterpret_cast<const ConvexBaseTpl<IndexType>*>(convex), dir, support,
      hint, support_data);
}
getShapeSupportTplInstantiation(SmallConvex<Triangle16::IndexType>);
getShapeSupportTplInstantiation(SmallConvex<Triangle32::IndexType>);

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupport(const LargeConvex<IndexType>* convex, const Vec3s& dir,
                     Vec3s& support, int& hint,
                     ShapeSupportData& support_data) {
  getShapeSupportLog<_SupportOptions>(
      reinterpret_cast<const ConvexBaseTpl<IndexType>*>(convex), dir, support,
      hint, support_data);
}
getShapeSupportTplInstantiation(LargeConvex<Triangle16::IndexType>);
getShapeSupportTplInstantiation(LargeConvex<Triangle32::IndexType>);

// ============================================================================
#define CALL_GET_SHAPE_SUPPORT_SET(ShapeType)                               \
  getShapeSupportSet<_SupportOptions>(static_cast<const ShapeType*>(shape), \
                                      support_set, hint, support_data,      \
                                      max_num_supports, tol)

template <int _SupportOptions>
void getSupportSet(const ShapeBase* shape, SupportSet& support_set, int& hint,
                   size_t max_num_supports, Scalar tol) {
  ShapeSupportData support_data;
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      CALL_GET_SHAPE_SUPPORT_SET(TriangleP);
      break;
    case GEOM_BOX:
      CALL_GET_SHAPE_SUPPORT_SET(Box);
      break;
    case GEOM_SPHERE:
      CALL_GET_SHAPE_SUPPORT_SET(Sphere);
      break;
    case GEOM_ELLIPSOID:
      CALL_GET_SHAPE_SUPPORT_SET(Ellipsoid);
      break;
    case GEOM_CAPSULE:
      CALL_GET_SHAPE_SUPPORT_SET(Capsule);
      break;
    case GEOM_CONE:
      CALL_GET_SHAPE_SUPPORT_SET(Cone);
      break;
    case GEOM_CYLINDER:
      CALL_GET_SHAPE_SUPPORT_SET(Cylinder);
      break;
    case GEOM_CONVEX16:
      CALL_GET_SHAPE_SUPPORT_SET(ConvexBaseTpl<Triangle16::IndexType>);
      break;
    case GEOM_CONVEX32:
      CALL_GET_SHAPE_SUPPORT_SET(ConvexBaseTpl<Triangle32::IndexType>);
      break;
    case GEOM_PLANE:
    case GEOM_HALFSPACE:
    default:;  // nothing
  }
}
#undef CALL_GET_SHAPE_SUPPORT

// Explicit instantiation
// clang-format off
template COAL_DLLAPI void getSupportSet<SupportOptions::NoSweptSphere>(const ShapeBase*, SupportSet&, int&, size_t, Scalar);

template COAL_DLLAPI void getSupportSet<SupportOptions::WithSweptSphere>(const ShapeBase*, SupportSet&, int&, size_t, Scalar);
// clang-format on

// ============================================================================
#define getShapeSupportSetTplInstantiation(ShapeType)                          \
  template COAL_DLLAPI void getShapeSupportSet<SupportOptions::NoSweptSphere>( \
      const ShapeType* shape_, SupportSet& support_set, int& hint,             \
      ShapeSupportData& data, size_t num_sampled_supports, Scalar tol);        \
                                                                               \
  template COAL_DLLAPI void                                                    \
  getShapeSupportSet<SupportOptions::WithSweptSphere>(                         \
      const ShapeType* shape_, SupportSet& support_set, int& hint,             \
      ShapeSupportData& data, size_t num_sampled_supports, Scalar tol);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const TriangleP* triangle, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, Scalar tol) {
  assert(tol > 0);
  support_set.clear();

  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  // We simply want to compute the support value, no need to take the
  // swept-sphere radius into account.
  getShapeSupport<SupportOptions::NoSweptSphere>(triangle, support_dir, support,
                                                 hint, support_data);
  const Scalar support_value = support.dot(support_dir);

  if (support_value - support_dir.dot(triangle->a) < tol) {
    // Note: at the moment, it's useless to take into account the
    // swept-sphere radius, but in the future we might want to store the
    // offsets to the plane in `SupportSet`.
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      support_set.addPoint(triangle->a +
                           triangle->getSweptSphereRadius() * support_dir);
    } else {
      support_set.addPoint(triangle->a);
    }
  }
  if (support_value - support_dir.dot(triangle->b) < tol) {
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      support_set.addPoint(triangle->b +
                           triangle->getSweptSphereRadius() * support_dir);
    } else {
      support_set.addPoint(triangle->b);
    }
  }
  if (support_value - support_dir.dot(triangle->c) < tol) {
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      support_set.addPoint(triangle->c +
                           triangle->getSweptSphereRadius() * support_dir);
    } else {
      support_set.addPoint(triangle->c);
    }
  }
}
getShapeSupportSetTplInstantiation(TriangleP);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Box* box, SupportSet& support_set,
                        int& hint /*unused*/, ShapeSupportData& support_data,
                        size_t /*unused*/, Scalar tol) {
  assert(tol > 0);
  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(box, support_dir, support,
                                                 hint, support_data);
  const Scalar support_value = support.dot(support_dir);

  const Scalar x = box->halfSide[0];
  const Scalar y = box->halfSide[1];
  const Scalar z = box->halfSide[2];
  const std::array<Vec3s, 8> corners = {
      Vec3s(x, y, z),  Vec3s(-x, y, z),  Vec3s(-x, -y, z),  Vec3s(x, -y, z),
      Vec3s(x, y, -z), Vec3s(-x, y, -z), Vec3s(-x, -y, -z), Vec3s(x, -y, -z),
  };

  SupportSet::Polygon& polygon = support_data.polygon;
  polygon.clear();
  const Transform3s& tf = support_set.tf;
  for (const Vec3s& corner : corners) {
    const Scalar val = corner.dot(support_dir);
    if (support_value - val < tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        const Vec2s p =
            tf.inverseTransform(corner +
                                box->getSweptSphereRadius() * support_dir)
                .template head<2>();
        polygon.emplace_back(p);
      } else {
        const Vec2s p = tf.inverseTransform(corner).template head<2>();
        polygon.emplace_back(p);
      }
    }
  }
  computeSupportSetConvexHull(polygon, support_set.points());
}
getShapeSupportSetTplInstantiation(Box);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Sphere* sphere, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, Scalar /*unused*/) {
  support_set.points().clear();

  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupport<_SupportOptions>(sphere, support_dir, support, hint,
                                   support_data);
  support_set.addPoint(support);
}
getShapeSupportSetTplInstantiation(Sphere);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Ellipsoid* ellipsoid, SupportSet& support_set,
                        int& hint, ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, Scalar /*unused*/) {
  support_set.points().clear();

  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupport<_SupportOptions>(ellipsoid, support_dir, support, hint,
                                   support_data);
  support_set.addPoint(support);
}
getShapeSupportSetTplInstantiation(Ellipsoid);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Capsule* capsule, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, Scalar tol) {
  // clang-format on
  assert(tol > 0);
  support_set.points().clear();

  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(capsule, support_dir, support,
                                                 hint, support_data);
  const Scalar support_value =
      support_dir.dot(support + capsule->radius * support_dir);
  // The support set of a capsule has either 2 points or 1 point.
  // The two candidate points lie at the frontier between the cylinder and
  // sphere parts of the capsule.
  const Scalar h = capsule->halfLength;
  const Scalar r = capsule->radius;
  const Vec3s p1(r * support_dir[0], r * support_dir[1], h);
  const Vec3s p2(r * support_dir[0], r * support_dir[1], -h);
  if ((support_value - support_dir.dot(p1) <= tol) &&
      (support_value - support_dir.dot(p2) <= tol)) {
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      const Vec3s ssr_vec = support_dir * capsule->getSweptSphereRadius();
      support_set.addPoint(p1 + ssr_vec);
      support_set.addPoint(p2 + ssr_vec);
    } else {
      support_set.addPoint(p1);
      support_set.addPoint(p2);
    }
  } else {
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      const Vec3s ssr_vec = support_dir * capsule->getSweptSphereRadius();
      support_set.addPoint(support + ssr_vec);
    } else {
      support_set.addPoint(support);
    }
  }
}
getShapeSupportSetTplInstantiation(Capsule);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Cone* cone, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t num_sampled_supports, Scalar tol) {
  assert(tol > 0);
  support_set.points().clear();

  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(cone, support_dir, support,
                                                 hint, support_data);
  const Scalar support_value = support.dot(support_dir);

  // If the support direction is perpendicular to the cone's basis, there is an
  // infinite amount of support points; otherwise there are up to two support
  // points (two if direction is perpendicular to the side of the cone and one
  // otherwise).
  //
  // To check this condition, we look at two points on the cone's basis;
  // these two points are symmetrical w.r.t the center of the circle. If
  // both these points are tol away from the support plane, then all the
  // points of the circle are tol away from the support plane.
  const Scalar r = cone->radius;
  const Scalar z = -cone->halfLength;
  const Vec3s p1(r * support_dir[0], r * support_dir[1], z);
  const Vec3s p2(-r * support_dir[0], -r * support_dir[1], z);

  if ((support_value - support_dir.dot(p1) <= tol) &&
      (support_value - support_dir.dot(p2) <= tol)) {
    // If this check passed, support direction is considered perpendicular to
    // the basis of the cone. We sample `num_sampled_supports` points on the
    // base of the cone. We are guaranteed that these points like at a distance
    // tol of the support plane.
    const Scalar angle_increment =
        Scalar(2 * EIGEN_PI) / (Scalar(num_sampled_supports));
    for (size_t i = 0; i < num_sampled_supports; ++i) {
      const Scalar theta = (Scalar)(i)*angle_increment;
      Vec3s point_on_cone_base(r * std::cos(theta), r * std::sin(theta), z);
      assert(std::abs(support_dir.dot(support - point_on_cone_base)) <= tol);
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_cone_base += cone->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_cone_base);
    }
  } else {
    // There are two potential supports to add: the tip of the cone and a point
    // on the basis of the cone. We compare each of these points to the support
    // value.
    Vec3s cone_tip(0, 0, cone->halfLength);
    if (support_value - support_dir.dot(cone_tip) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        cone_tip += cone->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(cone_tip);
    }

    Vec3s point_on_cone_base = Vec3s(cone->radius * support_dir[0],  //
                                     cone->radius * support_dir[1],  //
                                     z);
    if (support_value - support_dir.dot(point_on_cone_base) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_cone_base += cone->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_cone_base);
    }
  }
}
getShapeSupportSetTplInstantiation(Cone);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Cylinder* cylinder, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t num_sampled_supports, Scalar tol) {
  assert(tol > 0);
  support_set.points().clear();

  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(cylinder, support_dir, support,
                                                 hint, support_data);
  const Scalar support_value = support.dot(support_dir);

  // The following is very similar to what is done for Cone's support set
  // computation.
  const Scalar r = cylinder->radius;
  const Scalar z =
      support_dir[2] <= 0 ? -cylinder->halfLength : cylinder->halfLength;
  const Vec3s p1(r * support_dir[0], r * support_dir[1], z);
  const Vec3s p2(-r * support_dir[0], -r * support_dir[1], z);

  if ((support_value - support_dir.dot(p1) <= tol) &&
      (support_value - support_dir.dot(p2) <= tol)) {
    const Scalar angle_increment =
        Scalar(2 * EIGEN_PI) / (Scalar(num_sampled_supports));
    for (size_t i = 0; i < num_sampled_supports; ++i) {
      const Scalar theta = (Scalar)(i)*angle_increment;
      Vec3s point_on_cone_base(r * std::cos(theta), r * std::sin(theta), z);
      assert(std::abs(support_dir.dot(support - point_on_cone_base)) <= tol);
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_cone_base += cylinder->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_cone_base);
    }
  } else {
    // There are two potential supports to add: one on each circle bases of the
    // cylinder.
    Vec3s point_on_lower_circle = Vec3s(cylinder->radius * support_dir[0],  //
                                        cylinder->radius * support_dir[1],  //
                                        -cylinder->halfLength);
    if (support_value - support_dir.dot(point_on_lower_circle) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_lower_circle += cylinder->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_lower_circle);
    }

    Vec3s point_on_upper_circle = Vec3s(cylinder->radius * support_dir[0],  //
                                        cylinder->radius * support_dir[1],  //
                                        cylinder->halfLength);
    if (support_value - support_dir.dot(point_on_upper_circle) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_upper_circle += cylinder->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_upper_circle);
    }
  }
}
getShapeSupportSetTplInstantiation(Cylinder);

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupportSetLinear(const ConvexBaseTpl<IndexType>* convex,
                              SupportSet& support_set, int& hint /*unused*/,
                              ShapeSupportData& support_data, size_t /*unused*/,
                              Scalar tol) {
  assert(tol > 0);
  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(convex, support_dir, support,
                                                 hint, support_data);
  const Scalar support_value = support_dir.dot(support);

  const std::vector<Vec3s>& points = *(convex->points);
  SupportSet::Polygon& polygon = support_data.polygon;
  polygon.clear();
  const Transform3s& tf = support_set.tf;
  for (const Vec3s& point : points) {
    const Scalar dot = support_dir.dot(point);
    if (support_value - dot <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        const Vec2s p =
            tf.inverseTransform(point +
                                convex->getSweptSphereRadius() * support_dir)
                .template head<2>();
        polygon.emplace_back(p);
      } else {
        const Vec2s p = tf.inverseTransform(point).template head<2>();
        polygon.emplace_back(p);
      }
    }
  }

  computeSupportSetConvexHull(polygon, support_set.points());
}

// ============================================================================
template <int _SupportOptions, typename IndexType>
void convexSupportSetRecurse(const ConvexBaseTpl<IndexType>* convex,
                             const size_t vertex_idx, const Vec3s& support_dir,
                             const Scalar support_value, const Transform3s& tf,
                             std::vector<int8_t>& visited,
                             SupportSet::Polygon& polygon, Scalar tol) {
  if (visited[vertex_idx]) {
    return;
  }

  visited[vertex_idx] = true;
  const std::vector<Vec3s>& points = *(convex->points);
  const Vec3s& point = points[vertex_idx];
  const Scalar val = point.dot(support_dir);
  Scalar swept_sphere_radius = convex->getSweptSphereRadius();
  if (support_value - val <= tol) {
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      const Vec2s p =
          tf.inverseTransform(point + swept_sphere_radius * support_dir)
              .template head<2>();
      polygon.emplace_back(p);

    } else {
      const Vec2s p = tf.inverseTransform(point).template head<2>();
      polygon.emplace_back(p);
    }

    typedef typename ConvexBaseTpl<IndexType>::Neighbors Neighbors;
    const std::vector<Neighbors>& neighbors = *(convex->neighbors);
    const Neighbors& point_neighbors = neighbors[vertex_idx];
    for (IndexType i = 0; i < point_neighbors.count; ++i) {
      const IndexType neighbor_index =
          convex->neighbor(IndexType(vertex_idx), i);
      convexSupportSetRecurse<_SupportOptions, IndexType>(
          convex, neighbor_index, support_dir, support_value, tf, visited,
          polygon, tol);
    }
  }
}

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupportSetLog(const ConvexBaseTpl<IndexType>* convex,
                           SupportSet& support_set, int& hint,
                           ShapeSupportData& support_data, size_t /*unused*/,
                           Scalar tol) {
  assert(tol > 0);
  Vec3s support;
  const Vec3s& support_dir = support_set.getNormal();
  getShapeSupportLog<SupportOptions::NoSweptSphere>(
      convex, support_dir, support, hint, support_data);
  const Scalar support_value = support.dot(support_dir);

  std::vector<int8_t>& visited = support_data.visited;
  // `visited` is guaranteed to be of right size due to previous call to convex
  // log support function.
  std::fill(support_data.visited.begin(), support_data.visited.end(), false);

  SupportSet::Polygon& polygon = support_data.polygon;
  polygon.clear();
  const Transform3s& tf = support_set.tf;

  const size_t vertex_idx = (size_t)(hint);
  convexSupportSetRecurse<_SupportOptions, IndexType>(
      convex, vertex_idx, support_dir, support_value, tf, visited, polygon,
      tol);

  computeSupportSetConvexHull(polygon, support_set.points());
}

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupportSet(const ConvexBaseTpl<IndexType>* convex,
                        SupportSet& support_set, int& hint,
                        ShapeSupportData& support_data,
                        size_t num_sampled_supports /*unused*/, Scalar tol) {
  if (convex->num_points >
          ConvexBaseTpl<IndexType>::num_vertices_large_convex_threshold &&
      convex->neighbors != nullptr) {
    getShapeSupportSetLog<_SupportOptions>(
        convex, support_set, hint, support_data, num_sampled_supports, tol);
  } else {
    getShapeSupportSetLinear<_SupportOptions>(
        convex, support_set, hint, support_data, num_sampled_supports, tol);
  }
}
getShapeSupportSetTplInstantiation(ConvexBaseTpl<Triangle16::IndexType>);
getShapeSupportSetTplInstantiation(ConvexBaseTpl<Triangle32::IndexType>);

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupportSet(const SmallConvex<IndexType>* convex,
                        SupportSet& support_set, int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t num_sampled_supports /*unused*/, Scalar tol) {
  getShapeSupportSetLinear<_SupportOptions>(
      reinterpret_cast<const ConvexBaseTpl<IndexType>*>(convex), support_set,
      hint, support_data, num_sampled_supports, tol);
}
getShapeSupportSetTplInstantiation(SmallConvex<Triangle16::IndexType>);
getShapeSupportSetTplInstantiation(SmallConvex<Triangle32::IndexType>);

// ============================================================================
template <int _SupportOptions, typename IndexType>
void getShapeSupportSet(const LargeConvex<IndexType>* convex,
                        SupportSet& support_set, int& hint,
                        ShapeSupportData& support_data,
                        size_t num_sampled_supports /*unused*/, Scalar tol) {
  getShapeSupportSetLog<_SupportOptions>(
      reinterpret_cast<const ConvexBaseTpl<IndexType>*>(convex), support_set,
      hint, support_data, num_sampled_supports, tol);
}
getShapeSupportSetTplInstantiation(LargeConvex<Triangle16::IndexType>);
getShapeSupportSetTplInstantiation(LargeConvex<Triangle32::IndexType>);

// ============================================================================
COAL_DLLAPI void computeSupportSetConvexHull(SupportSet::Polygon& cloud,
                                             SupportSet::Polygon& cvx_hull) {
  cvx_hull.clear();

  if (cloud.size() <= 2) {
    // Point or segment, nothing to do.
    for (const Vec2s& point : cloud) {
      cvx_hull.emplace_back(point);
    }
    return;
  }

  if (cloud.size() == 3) {
    // We have a triangle, we only need to arrange it in a counter clockwise
    // fashion.
    //
    // Put the vector which has the lowest y coordinate first.
    if (cloud[0](1) > cloud[1](1)) {
      std::swap(cloud[0], cloud[1]);
    }
    if (cloud[0](1) > cloud[2](1)) {
      std::swap(cloud[0], cloud[2]);
    }
    const Vec2s& a = cloud[0];
    const Vec2s& b = cloud[1];
    const Vec2s& c = cloud[2];
    const Scalar det =
        (b(0) - a(0)) * (c(1) - a(1)) - (b(1) - a(1)) * (c(0) - a(0));
    if (det < 0) {
      std::swap(cloud[1], cloud[2]);
    }

    for (const Vec2s& point : cloud) {
      cvx_hull.emplace_back(point);
    }
    return;
  }

  // The following is an implementation of the O(nlog(n)) graham scan
  // algorithm, used to compute convex-hulls in 2D.
  // See https://en.wikipedia.org/wiki/Graham_scan
  //
  // Step 1 - Compute first element of the convex-hull by computing the support
  // in the direction (0, -1) (take the element of the set which has the lowest
  // y coordinate).
  size_t support_idx = 0;
  Scalar support_val = cloud[0](1);
  for (size_t i = 1; i < cloud.size(); ++i) {
    const Scalar val = cloud[i](1);
    if (val < support_val) {
      support_val = val;
      support_idx = i;
    }
  }
  std::swap(cloud[0], cloud[support_idx]);
  cvx_hull.clear();
  cvx_hull.emplace_back(cloud[0]);
  const Vec2s& v = cvx_hull[0];

  // Step 2 - Sort the rest of the point cloud according to the angle made with
  // v. Note: we use stable_sort instead of sort because sort can fail if two
  // values are identical.
  std::stable_sort(
      cloud.begin() + 1, cloud.end(), [&v](const Vec2s& p1, const Vec2s& p2) {
        // p1 is "smaller" than p2 if det(p1 - v, p2 - v) >= 0
        const Scalar det =
            (p1(0) - v(0)) * (p2(1) - v(1)) - (p1(1) - v(1)) * (p2(0) - v(0));
        if (std::abs(det) <= Eigen::NumTraits<Scalar>::dummy_precision()) {
          // If two points are identical or (v, p1, p2) are colinear, p1 is
          // "smaller" if it is closer to v.
          return ((p1 - v).squaredNorm() <= (p2 - v).squaredNorm());
        }
        return det > 0;
      });

  // Step 3 - We iterate over the now ordered point of cloud and add the points
  // to the cvx-hull if they successively form "left turns" only. A left turn
  // is: considering the last three points of the cvx-hull, if they form a
  // right-hand basis (determinant > 0) then they make a left turn.
  auto isRightSided = [](const Vec2s& p1, const Vec2s& p2, const Vec2s& p3) {
    // Checks if (p2 - p1, p3 - p1) forms a right-sided base based on
    // det(p2 - p1, p3 - p1)
    const Scalar det =
        (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(1)) * (p3(0) - p1(0));
    // Note: we set a dummy precision threshold so that identical points or
    // colinear pionts are not added to the cvx-hull.
    return det > Eigen::NumTraits<Scalar>::dummy_precision();
  };

  // We initialize the cvx-hull algo by adding the first three
  // (distinct) points of the set.
  // These three points are guaranteed, due to the previous sorting,
  // to form a right sided basis, hence to form a left turn.
  size_t cloud_beginning_idx = 1;
  while (cvx_hull.size() < 3) {
    const Vec2s& vec = cloud[cloud_beginning_idx];
    if ((cvx_hull.back() - vec).squaredNorm() >
        Eigen::NumTraits<Scalar>::epsilon()) {
      cvx_hull.emplace_back(vec);
    }
    ++cloud_beginning_idx;
  }
  // The convex-hull should wrap counter-clockwise, i.e. three successive
  // points should always form a right-sided basis. Every time we do a turn
  // in the wrong direction, we remove the last point of the convex-hull.
  // When we do a turn in the correct direction, we add a point to the
  // convex-hull.
  for (size_t i = cloud_beginning_idx; i < cloud.size(); ++i) {
    const Vec2s& vec = cloud[i];
    while (cvx_hull.size() > 1 &&
           !isRightSided(cvx_hull[cvx_hull.size() - 2],
                         cvx_hull[cvx_hull.size() - 1], vec)) {
      cvx_hull.pop_back();
    }
    cvx_hull.emplace_back(vec);
  }
}

}  // namespace details
}  // namespace coal
