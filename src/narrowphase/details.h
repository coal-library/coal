/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2018-2019, Centre National de la Recherche Scientifique
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
/** \author Jia Pan, Florent Lamiraux */

#ifndef COAL_SRC_NARROWPHASE_DETAILS_H
#define COAL_SRC_NARROWPHASE_DETAILS_H

#include "coal/internal/traversal_node_setup.h"
#include "coal/narrowphase/narrowphase.h"

namespace coal {
namespace details {
// Compute the point on a line segment that is the closest point on the
// segment to to another point. The code is inspired by the explanation
// given by Dan Sunday's page:
//   http://geomalgorithms.com/a02-_lines.html
static inline void lineSegmentPointClosestToPoint(const Vec3s& p,
                                                  const Vec3s& s1,
                                                  const Vec3s& s2, Vec3s& sp) {
  Vec3s v = s2 - s1;
  Vec3s w = p - s1;

  Scalar c1 = w.dot(v);
  Scalar c2 = v.dot(v);

  if (c1 <= 0) {
    sp = s1;
  } else if (c2 <= c1) {
    sp = s2;
  } else {
    Scalar b = c1 / c2;
    Vec3s Pb = s1 + v * b;
    sp = Pb;
  }
}

/// @param p1 witness point on the Sphere.
/// @param p2 witness point on the Capsule.
/// @param normal pointing from shape 1 to shape 2 (sphere to capsule).
/// @return the distance between the two shapes (negative if penetration).
inline Scalar sphereCapsuleDistance(const Sphere& s1, const Transform3s& tf1,
                                    const Capsule& s2, const Transform3s& tf2,
                                    Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  Vec3s pos1(tf2.transform(Vec3s(0., 0., s2.halfLength)));
  Vec3s pos2(tf2.transform(Vec3s(0., 0., -s2.halfLength)));
  Vec3s s_c = tf1.getTranslation();

  Vec3s segment_point;

  lineSegmentPointClosestToPoint(s_c, pos1, pos2, segment_point);
  normal = segment_point - s_c;
  Scalar norm(normal.norm());
  Scalar r1 = s1.radius + s1.getSweptSphereRadius();
  Scalar r2 = s2.radius + s2.getSweptSphereRadius();
  Scalar dist = norm - r1 - r2;

  static const Scalar eps(std::numeric_limits<Scalar>::epsilon());
  if (norm > eps) {
    normal.normalize();
  } else {
    normal << 1, 0, 0;
  }
  p1 = s_c + normal * r1;
  p2 = segment_point - normal * r2;
  return dist;
}

/// @param p1 witness point on the Sphere.
/// @param p2 witness point on the Cylinder.
/// @param normal pointing from shape 1 to shape 2 (sphere to cylinder).
/// @return the distance between the two shapes (negative if penetration).
inline Scalar sphereCylinderDistance(const Sphere& s1, const Transform3s& tf1,
                                     const Cylinder& s2, const Transform3s& tf2,
                                     Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  static const Scalar eps(sqrt(std::numeric_limits<Scalar>::epsilon()));
  Scalar r1(s1.radius);
  Scalar r2(s2.radius);
  Scalar lz2(s2.halfLength);
  // boundaries of the cylinder axis
  Vec3s A(tf2.transform(Vec3s(0, 0, -lz2)));
  Vec3s B(tf2.transform(Vec3s(0, 0, lz2)));
  // Position of the center of the sphere
  Vec3s S(tf1.getTranslation());
  // axis of the cylinder
  Vec3s u(tf2.getRotation().col(2));
  /// @todo a tiny performance improvement could be achieved using the abscissa
  /// with S as the origin
  assert((B - A - (s2.halfLength * 2) * u).norm() < eps);
  Vec3s AS(S - A);
  // abscissa of S on cylinder axis with A as the origin
  Scalar s(u.dot(AS));
  Vec3s P(A + s * u);
  Vec3s PS(S - P);
  Scalar dPS = PS.norm();
  // Normal to cylinder axis such that plane (A, u, v) contains sphere
  // center
  Vec3s v(0, 0, 0);
  Scalar dist;
  if (dPS > eps) {
    // S is not on cylinder axis
    v = (1 / dPS) * PS;
  }
  if (s <= 0) {
    if (dPS <= r2) {
      // closest point on cylinder is on cylinder disc basis
      dist = -s - r1;
      p1 = S + r1 * u;
      p2 = A + dPS * v;
      normal = u;
    } else {
      // closest point on cylinder is on cylinder circle basis
      p2 = A + r2 * v;
      Vec3s Sp2(p2 - S);
      Scalar dSp2 = Sp2.norm();
      if (dSp2 > eps) {
        normal = (1 / dSp2) * Sp2;
        p1 = S + r1 * normal;
        dist = dSp2 - r1;
        assert(fabs(dist) - (p1 - p2).norm() < eps);
      } else {
        // Center of sphere is on cylinder boundary
        normal = p2 - .5 * (A + B);
        assert(u.dot(normal) >= 0);
        normal.normalize();
        dist = -r1;
        p1 = S + r1 * normal;
      }
    }
  } else if (s <= (s2.halfLength * 2)) {
    // 0 < s <= s2.lz
    normal = -v;
    dist = dPS - r1 - r2;
    p2 = P + r2 * v;
    p1 = S - r1 * v;
  } else {
    // lz < s
    if (dPS <= r2) {
      // closest point on cylinder is on cylinder disc basis
      dist = s - (s2.halfLength * 2) - r1;
      p1 = S - r1 * u;
      p2 = B + dPS * v;
      normal = -u;
    } else {
      // closest point on cylinder is on cylinder circle basis
      p2 = B + r2 * v;
      Vec3s Sp2(p2 - S);
      Scalar dSp2 = Sp2.norm();
      if (dSp2 > eps) {
        normal = (1 / dSp2) * Sp2;
        p1 = S + r1 * normal;
        dist = dSp2 - r1;
        assert(fabs(dist) - (p1 - p2).norm() < eps);
      } else {
        // Center of sphere is on cylinder boundary
        normal = p2 - .5 * (A + B);
        normal.normalize();
        p1 = S + r1 * normal;
        dist = -r1;
      }
    }
  }

  // Take swept-sphere radius into account
  const Scalar ssr1 = s1.getSweptSphereRadius();
  const Scalar ssr2 = s2.getSweptSphereRadius();
  if (ssr1 > 0 || ssr2 > 0) {
    p1 += ssr1 * normal;
    p2 -= ssr2 * normal;
    dist -= (ssr1 + ssr2);
  }

  return dist;
}

/// @param p1 witness point on the first Sphere.
/// @param p2 witness point on the second Sphere.
/// @param normal pointing from shape 1 to shape 2 (sphere1 to sphere2).
/// @return the distance between the two spheres (negative if penetration).
inline Scalar sphereSphereDistance(const Sphere& s1, const Transform3s& tf1,
                                   const Sphere& s2, const Transform3s& tf2,
                                   Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  const coal::Vec3s& center1 = tf1.getTranslation();
  const coal::Vec3s& center2 = tf2.getTranslation();
  Scalar r1 = (s1.radius + s1.getSweptSphereRadius());
  Scalar r2 = (s2.radius + s2.getSweptSphereRadius());

  Vec3s c1c2 = center2 - center1;
  Scalar cdist = c1c2.norm();
  Vec3s unit(1, 0, 0);
  if (cdist > Eigen::NumTraits<Scalar>::epsilon()) unit = c1c2 / cdist;
  Scalar dist = cdist - r1 - r2;
  normal = unit;
  p1.noalias() = center1 + r1 * unit;
  p2.noalias() = center2 - r2 * unit;
  return dist;
}

/** @brief the minimum distance from a point to a line */
inline Scalar segmentSqrDistance(const Vec3s& from, const Vec3s& to,
                                 const Vec3s& p, Vec3s& nearest) {
  Vec3s diff = p - from;
  Vec3s v = to - from;
  Scalar t = v.dot(diff);

  if (t > 0) {
    Scalar dotVV = v.squaredNorm();
    if (t < dotVV) {
      t /= dotVV;
      diff -= v * t;
    } else {
      t = 1;
      diff -= v;
    }
  } else
    t = 0;

  nearest.noalias() = from + v * t;
  return diff.squaredNorm();
}

/// @brief Whether a point's projection is in a triangle
inline bool projectInTriangle(const Vec3s& p1, const Vec3s& p2, const Vec3s& p3,
                              const Vec3s& normal, const Vec3s& p) {
  Vec3s edge1(p2 - p1);
  Vec3s edge2(p3 - p2);
  Vec3s edge3(p1 - p3);

  Vec3s p1_to_p(p - p1);
  Vec3s p2_to_p(p - p2);
  Vec3s p3_to_p(p - p3);

  Vec3s edge1_normal(edge1.cross(normal));
  Vec3s edge2_normal(edge2.cross(normal));
  Vec3s edge3_normal(edge3.cross(normal));

  Scalar r1, r2, r3;
  r1 = edge1_normal.dot(p1_to_p);
  r2 = edge2_normal.dot(p2_to_p);
  r3 = edge3_normal.dot(p3_to_p);
  if ((r1 > 0 && r2 > 0 && r3 > 0) || (r1 <= 0 && r2 <= 0 && r3 <= 0)) {
    return true;
  }
  return false;
}

/// @param p1 witness point on the first Sphere.
/// @param p2 witness point on the second Sphere.
/// @param normal pointing from shape 1 to shape 2 (sphere1 to sphere2).
/// @return the distance between the two shapes (negative if penetration).
inline Scalar sphereTriangleDistance(const Sphere& s, const Transform3s& tf1,
                                     const TriangleP& tri,
                                     const Transform3s& tf2, Vec3s& p1,
                                     Vec3s& p2, Vec3s& normal) {
  const Vec3s& P1 = tf2.transform(tri.a);
  const Vec3s& P2 = tf2.transform(tri.b);
  const Vec3s& P3 = tf2.transform(tri.c);

  Vec3s tri_normal = (P2 - P1).cross(P3 - P1);
  tri_normal.normalize();
  const Vec3s& center = tf1.getTranslation();
  // Note: comparing an object with a swept-sphere radius of r1 against another
  // object with a swept-sphere radius of r2 is equivalent to comparing the
  // first object with a swept-sphere radius of r1 + r2 against the second
  // object with a swept-sphere radius of 0.
  const Scalar& radius =
      s.radius + s.getSweptSphereRadius() + tri.getSweptSphereRadius();
  assert(radius >= 0);
  assert(s.radius >= 0);
  Vec3s p1_to_center = center - P1;
  Scalar distance_from_plane = p1_to_center.dot(tri_normal);
  Vec3s closest_point(
      Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN()));
  Scalar min_distance_sqr, distance_sqr;

  if (distance_from_plane < 0) {
    distance_from_plane *= -1;
    tri_normal *= -1;
  }

  if (projectInTriangle(P1, P2, P3, tri_normal, center)) {
    closest_point = center - tri_normal * distance_from_plane;
    min_distance_sqr = distance_from_plane * distance_from_plane;
  } else {
    // Compute distance to each edge and take minimal distance
    Vec3s nearest_on_edge;
    min_distance_sqr = segmentSqrDistance(P1, P2, center, closest_point);

    distance_sqr = segmentSqrDistance(P2, P3, center, nearest_on_edge);
    if (distance_sqr < min_distance_sqr) {
      min_distance_sqr = distance_sqr;
      closest_point = nearest_on_edge;
    }
    distance_sqr = segmentSqrDistance(P3, P1, center, nearest_on_edge);
    if (distance_sqr < min_distance_sqr) {
      min_distance_sqr = distance_sqr;
      closest_point = nearest_on_edge;
    }
  }

  normal = (closest_point - center).normalized();
  p1 = center + normal * (s.radius + s.getSweptSphereRadius());
  p2 = closest_point - normal * tri.getSweptSphereRadius();
  const Scalar distance = std::sqrt(min_distance_sqr) - radius;
  return distance;
}

/// @param p1 closest (or most penetrating) point on the Halfspace,
/// @param p2 closest (or most penetrating) point on the shape,
/// @param normal the halfspace normal.
/// @return the distance between the two shapes (negative if penetration).
inline Scalar halfspaceDistance(const Halfspace& h, const Transform3s& tf1,
                                const ShapeBase& s, const Transform3s& tf2,
                                Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  // TODO(louis): handle multiple contact points when the halfspace normal is
  // parallel to the shape's surface (every primitive except sphere and
  // ellipsoid).

  // Express halfspace in world frame
  Halfspace new_h = transform(h, tf1);

  // Express halfspace normal in shape frame
  Vec3s n_2(tf2.getRotation().transpose() * new_h.n);

  // Compute support of shape in direction of halfspace normal
  int hint = 0;
  p2.noalias() =
      getSupport<details::SupportOptions::WithSweptSphere>(&s, -n_2, hint);
  p2 = tf2.transform(p2);

  const Scalar dist = new_h.signedDistance(p2);
  p1.noalias() = p2 - dist * new_h.n;
  normal.noalias() = new_h.n;

  const Scalar dummy_precision =
      std::sqrt(Eigen::NumTraits<Scalar>::dummy_precision());
  COAL_UNUSED_VARIABLE(dummy_precision);
  assert(new_h.distance(p1) <= dummy_precision);
  return dist;
}

/// @param p1 closest (or most penetrating) point on the Plane,
/// @param p2 closest (or most penetrating) point on the shape,
/// @param normal the halfspace normal.
/// @return the distance between the two shapes (negative if penetration).
inline Scalar planeDistance(const Plane& plane, const Transform3s& tf1,
                            const ShapeBase& s, const Transform3s& tf2,
                            Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  // TODO(louis): handle multiple contact points when the plane normal is
  // parallel to the shape's surface (every primitive except sphere and
  // ellipsoid).

  // Express plane as two halfspaces in world frame
  std::array<Halfspace, 2> new_h = transformToHalfspaces(plane, tf1);

  // Express halfspace normals in shape frame
  Vec3s n_h1(tf2.getRotation().transpose() * new_h[0].n);
  Vec3s n_h2(tf2.getRotation().transpose() * new_h[1].n);

  // Compute support of shape in direction of halfspace normal and its opposite
  int hint = 0;
  Vec3s p2h1 =
      getSupport<details::SupportOptions::WithSweptSphere>(&s, -n_h1, hint);
  p2h1 = tf2.transform(p2h1);

  hint = 0;
  Vec3s p2h2 =
      getSupport<details::SupportOptions::WithSweptSphere>(&s, -n_h2, hint);
  p2h2 = tf2.transform(p2h2);

  Scalar dist1 = new_h[0].signedDistance(p2h1);
  Scalar dist2 = new_h[1].signedDistance(p2h2);

  const Scalar dummy_precision =
      std::sqrt(Eigen::NumTraits<Scalar>::dummy_precision());
  COAL_UNUSED_VARIABLE(dummy_precision);

  Scalar dist;
  if (dist1 >= dist2) {
    dist = dist1;
    p2.noalias() = p2h1;
    p1.noalias() = p2 - dist * new_h[0].n;
    normal.noalias() = new_h[0].n;
    assert(new_h[0].distance(p1) <= dummy_precision);
  } else {
    dist = dist2;
    p2.noalias() = p2h2;
    p1.noalias() = p2 - dist * new_h[1].n;
    normal.noalias() = new_h[1].n;
    assert(new_h[1].distance(p1) <= dummy_precision);
  }
  return dist;
}

/// Taken from book Real Time Collision Detection, from Christer Ericson
/// @param pb the witness point on the box surface
/// @param ps the witness point on the sphere.
/// @param normal pointing from box to sphere
/// @return the distance between the two shapes (negative if penetration).
inline Scalar boxSphereDistance(const Box& b, const Transform3s& tfb,
                                const Sphere& s, const Transform3s& tfs,
                                Vec3s& pb, Vec3s& ps, Vec3s& normal) {
  const Vec3s& os = tfs.getTranslation();
  const Vec3s& ob = tfb.getTranslation();
  const Matrix3s& Rb = tfb.getRotation();

  pb = ob;

  bool outside = false;
  const Vec3s os_in_b_frame(Rb.transpose() * (os - ob));
  int axis = -1;
  Scalar min_d = (std::numeric_limits<Scalar>::max)();
  for (int i = 0; i < 3; ++i) {
    Scalar facedist;
    if (os_in_b_frame(i) < -b.halfSide(i)) {  // outside
      pb.noalias() -= b.halfSide(i) * Rb.col(i);
      outside = true;
    } else if (os_in_b_frame(i) > b.halfSide(i)) {  // outside
      pb.noalias() += b.halfSide(i) * Rb.col(i);
      outside = true;
    } else {
      pb.noalias() += os_in_b_frame(i) * Rb.col(i);
      if (!outside &&
          (facedist = b.halfSide(i) - std::fabs(os_in_b_frame(i))) < min_d) {
        axis = i;
        min_d = facedist;
      }
    }
  }
  normal = pb - os;
  Scalar pdist = normal.norm();
  Scalar dist;    // distance between sphere and box
  if (outside) {  // pb is on the box
    dist = pdist - s.radius;
    normal /= -pdist;
  } else {  // pb is inside the box
    if (os_in_b_frame(axis) >= 0) {
      normal = Rb.col(axis);
    } else {
      normal = -Rb.col(axis);
    }
    dist = -min_d - s.radius;
  }
  ps = os - s.radius * normal;
  if (!outside || dist <= 0) {
    // project point pb onto the box's surface
    pb = ps - dist * normal;
  }

  // Take swept-sphere radius into account
  const Scalar ssrb = b.getSweptSphereRadius();
  const Scalar ssrs = s.getSweptSphereRadius();
  if (ssrb > 0 || ssrs > 0) {
    pb += ssrb * normal;
    ps -= ssrs * normal;
    dist -= (ssrb + ssrs);
  }

  return dist;
}

/// @brief return distance between two halfspaces
/// @param p1 the witness point on the first halfspace.
/// @param p2 the witness point on the second halfspace.
/// @param normal pointing from first to second halfspace.
/// @return the distance between the two shapes (negative if penetration).
///
/// @note If the two halfspaces don't have the same normal (or opposed
/// normals), they collide and their distance is set to -infinity as there is no
/// translation that can separate them; they have infinite penetration depth.
/// The points p1 and p2 are the same point and represent the origin of the
/// intersection line between the objects. The normal is the direction of this
/// line.
inline Scalar halfspaceHalfspaceDistance(const Halfspace& s1,
                                         const Transform3s& tf1,
                                         const Halfspace& s2,
                                         const Transform3s& tf2, Vec3s& p1,
                                         Vec3s& p2, Vec3s& normal) {
  Halfspace new_s1 = transform(s1, tf1);
  Halfspace new_s2 = transform(s2, tf2);

  Scalar distance;
  Vec3s dir = (new_s1.n).cross(new_s2.n);
  Scalar dir_sq_norm = dir.squaredNorm();

  if (dir_sq_norm < std::numeric_limits<Scalar>::epsilon())  // parallel
  {
    if (new_s1.n.dot(new_s2.n) > 0) {
      // If the two halfspaces have the same normal, one is inside the other
      // and they can't be separated. They have inifinte penetration depth.
      distance = -(std::numeric_limits<Scalar>::max)();
      if (new_s1.d <= new_s2.d) {
        normal = new_s1.n;
        p1 = normal * distance;
        p2 = new_s2.n * new_s2.d;
        assert(new_s2.distance(p2) <=
               Eigen::NumTraits<Scalar>::dummy_precision());
      } else {
        normal = -new_s1.n;
        p1 << new_s1.n * new_s1.d;
        p2 = -(normal * distance);
        assert(new_s1.distance(p1) <=
               Eigen::NumTraits<Scalar>::dummy_precision());
      }
    } else {
      distance = -(new_s1.d + new_s2.d);
      normal = new_s1.n;
      p1 = new_s1.n * new_s1.d;
      p2 = new_s2.n * new_s2.d;
    }
  } else {
    // If the halfspaces are not parallel, they are in collision.
    // Their distance, in the sens of the norm of separation vector, is infinite
    // (it's impossible to find a translation which separates them)
    distance = -(std::numeric_limits<Scalar>::max)();
    // p1 and p2 are the same point, corresponding to a point on the
    // intersection line between the two objects. Normal is the direction of
    // that line.
    normal = dir;
    p1 = p2 =
        ((new_s2.n * new_s1.d - new_s1.n * new_s2.d).cross(dir)) / dir_sq_norm;
    // Sources: https://en.wikipedia.org/wiki/Plane%E2%80%93plane_intersection
    // and      https://en.wikipedia.org/wiki/Cross_product
  }

  // Take swept-sphere radius into account
  const Scalar ssr1 = s1.getSweptSphereRadius();
  const Scalar ssr2 = s2.getSweptSphereRadius();
  if (ssr1 > 0 || ssr2 > 0) {
    p1 += ssr1 * normal;
    p2 -= ssr2 * normal;
    distance -= (ssr1 + ssr2);
  }

  return distance;
}

/// @brief return distance between plane and halfspace.
/// @param p1 the witness point on the halfspace.
/// @param p2 the witness point on the plane.
/// @param normal pointing from halfspace to plane.
/// @return the distance between the two shapes (negative if penetration).
///
/// @note If plane and halfspace don't have the same normal (or opposed
/// normals), they collide and their distance is set to -infinity as there is no
/// translation that can separate them; they have infinite penetration depth.
/// The points p1 and p2 are the same point and represent the origin of the
/// intersection line between the objects. The normal is the direction of this
/// line.
inline Scalar halfspacePlaneDistance(const Halfspace& s1,
                                     const Transform3s& tf1, const Plane& s2,
                                     const Transform3s& tf2, Vec3s& p1,
                                     Vec3s& p2, Vec3s& normal) {
  Halfspace new_s1 = transform(s1, tf1);
  Plane new_s2 = transform(s2, tf2);

  Scalar distance;
  Vec3s dir = (new_s1.n).cross(new_s2.n);
  Scalar dir_sq_norm = dir.squaredNorm();

  if (dir_sq_norm < std::numeric_limits<Scalar>::epsilon())  // parallel
  {
    normal = new_s1.n;
    distance = new_s1.n.dot(new_s2.n) > 0 ? (new_s2.d - new_s1.d)
                                          : -(new_s1.d + new_s2.d);
    p1 = new_s1.n * new_s1.d;
    p2 = new_s2.n * new_s2.d;
    assert(new_s1.distance(p1) <= Eigen::NumTraits<Scalar>::dummy_precision());
    assert(new_s2.distance(p2) <= Eigen::NumTraits<Scalar>::dummy_precision());
  } else {
    // If the halfspace and plane are not parallel, they are in collision.
    // Their distance, in the sens of the norm of separation vector, is infinite
    // (it's impossible to find a translation which separates them)
    distance = -(std::numeric_limits<Scalar>::max)();
    // p1 and p2 are the same point, corresponding to a point on the
    // intersection line between the two objects. Normal is the direction of
    // that line.
    normal = dir;
    p1 = p2 =
        ((new_s2.n * new_s1.d - new_s1.n * new_s2.d).cross(dir)) / dir_sq_norm;
    // Sources: https://en.wikipedia.org/wiki/Plane%E2%80%93plane_intersection
    // and      https://en.wikipedia.org/wiki/Cross_product
  }

  // Take swept-sphere radius into account
  const Scalar ssr1 = s1.getSweptSphereRadius();
  const Scalar ssr2 = s2.getSweptSphereRadius();
  if (ssr1 > 0 || ssr2 > 0) {
    p1 += ssr1 * normal;
    p2 -= ssr2 * normal;
    distance -= (ssr1 + ssr2);
  }

  return distance;
}

/// @brief return distance between two planes
/// @param p1 the witness point on the first plane.
/// @param p2 the witness point on the second plane.
/// @param normal pointing from first to second plane.
/// @return the distance between the two shapes (negative if penetration).
///
/// @note If the two planes don't have the same normal (or opposed
/// normals), they collide and their distance is set to -infinity as there is no
/// translation that can separate them; they have infinite penetration depth.
/// The points p1 and p2 are the same point and represent the origin of the
/// intersection line between the objects. The normal is the direction of this
/// line.
inline Scalar planePlaneDistance(const Plane& s1, const Transform3s& tf1,
                                 const Plane& s2, const Transform3s& tf2,
                                 Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  Plane new_s1 = transform(s1, tf1);
  Plane new_s2 = transform(s2, tf2);

  Scalar distance;
  Vec3s dir = (new_s1.n).cross(new_s2.n);
  Scalar dir_sq_norm = dir.squaredNorm();

  if (dir_sq_norm < std::numeric_limits<Scalar>::epsilon())  // parallel
  {
    p1 = new_s1.n * new_s1.d;
    p2 = new_s2.n * new_s2.d;
    assert(new_s1.distance(p1) <= Eigen::NumTraits<Scalar>::dummy_precision());
    assert(new_s2.distance(p2) <= Eigen::NumTraits<Scalar>::dummy_precision());
    distance = (p1 - p2).norm();

    if (distance > Eigen::NumTraits<Scalar>::dummy_precision()) {
      normal = (p2 - p1).normalized();
    } else {
      normal = new_s1.n;
    }
  } else {
    // If the planes are not parallel, they are in collision.
    // Their distance, in the sens of the norm of separation vector, is infinite
    // (it's impossible to find a translation which separates them)
    distance = -(std::numeric_limits<Scalar>::max)();
    // p1 and p2 are the same point, corresponding to a point on the
    // intersection line between the two objects. Normal is the direction of
    // that line.
    normal = dir;
    p1 = p2 =
        ((new_s2.n * new_s1.d - new_s1.n * new_s2.d).cross(dir)) / dir_sq_norm;
    // Sources: https://en.wikipedia.org/wiki/Plane%E2%80%93plane_intersection
    // and      https://en.wikipedia.org/wiki/Cross_product
  }

  // Take swept-sphere radius into account
  const Scalar ssr1 = s1.getSweptSphereRadius();
  const Scalar ssr2 = s2.getSweptSphereRadius();
  if (ssr1 > 0 || ssr2 > 0) {
    p1 += ssr1 * normal;
    p2 -= ssr2 * normal;
    distance -= (ssr1 + ssr2);
  }

  return distance;
}

/// See the prototype below
inline Scalar computePenetration(const Vec3s& P1, const Vec3s& P2,
                                 const Vec3s& P3, const Vec3s& Q1,
                                 const Vec3s& Q2, const Vec3s& Q3,
                                 Vec3s& normal) {
  Vec3s u((P2 - P1).cross(P3 - P1));
  normal = u.normalized();
  Scalar depth1((P1 - Q1).dot(normal));
  Scalar depth2((P1 - Q2).dot(normal));
  Scalar depth3((P1 - Q3).dot(normal));
  return std::max(depth1, std::max(depth2, depth3));
}

// Compute penetration distance and normal of two triangles in collision
// Normal is normal of triangle 1 (P1, P2, P3), penetration depth is the
// minimal distance (Q1, Q2, Q3) should be translated along the normal so
// that the triangles are collision free.
//
// Note that we compute here an upper bound of the penetration distance,
// not the exact value.
inline Scalar computePenetration(const Vec3s& P1, const Vec3s& P2,
                                 const Vec3s& P3, const Vec3s& Q1,
                                 const Vec3s& Q2, const Vec3s& Q3,
                                 const Transform3s& tf1, const Transform3s& tf2,
                                 Vec3s& normal) {
  Vec3s globalP1(tf1.transform(P1));
  Vec3s globalP2(tf1.transform(P2));
  Vec3s globalP3(tf1.transform(P3));
  Vec3s globalQ1(tf2.transform(Q1));
  Vec3s globalQ2(tf2.transform(Q2));
  Vec3s globalQ3(tf2.transform(Q3));
  return computePenetration(globalP1, globalP2, globalP3, globalQ1, globalQ2,
                            globalQ3, normal);
}

}  // namespace details
}  // namespace coal

#endif  // COAL_SRC_NARROWPHASE_DETAILS_H
