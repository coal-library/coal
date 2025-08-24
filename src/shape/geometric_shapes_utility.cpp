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

#include "coal/shape/geometric_shapes_utility.h"
#include "coal/internal/BV_fitter.h"
#include "coal/internal/tools.h"

namespace coal {

namespace details {

std::vector<Vec3s> getBoundVertices(const Box& box, const Transform3s& tf) {
  std::vector<Vec3s> result(8);
  Scalar a = box.halfSide[0];
  Scalar b = box.halfSide[1];
  Scalar c = box.halfSide[2];
  result[0] = tf.transform(Vec3s(a, b, c));
  result[1] = tf.transform(Vec3s(a, b, -c));
  result[2] = tf.transform(Vec3s(a, -b, c));
  result[3] = tf.transform(Vec3s(a, -b, -c));
  result[4] = tf.transform(Vec3s(-a, b, c));
  result[5] = tf.transform(Vec3s(-a, b, -c));
  result[6] = tf.transform(Vec3s(-a, -b, c));
  result[7] = tf.transform(Vec3s(-a, -b, -c));

  return result;
}

// we use icosahedron to bound the sphere
std::vector<Vec3s> getBoundVertices(const Sphere& sphere,
                                    const Transform3s& tf) {
  std::vector<Vec3s> result(12);
  const Scalar m = (1 + sqrt(Scalar(5))) / Scalar(2);
  Scalar edge_size = sphere.radius * 6 / (sqrt(Scalar(27)) + sqrt(Scalar(15)));

  Scalar a = edge_size;
  Scalar b = m * edge_size;
  result[0] = tf.transform(Vec3s(0, a, b));
  result[1] = tf.transform(Vec3s(0, -a, b));
  result[2] = tf.transform(Vec3s(0, a, -b));
  result[3] = tf.transform(Vec3s(0, -a, -b));
  result[4] = tf.transform(Vec3s(a, b, 0));
  result[5] = tf.transform(Vec3s(-a, b, 0));
  result[6] = tf.transform(Vec3s(a, -b, 0));
  result[7] = tf.transform(Vec3s(-a, -b, 0));
  result[8] = tf.transform(Vec3s(b, 0, a));
  result[9] = tf.transform(Vec3s(b, 0, -a));
  result[10] = tf.transform(Vec3s(-b, 0, a));
  result[11] = tf.transform(Vec3s(-b, 0, -a));

  return result;
}

// we use scaled icosahedron to bound the ellipsoid
std::vector<Vec3s> getBoundVertices(const Ellipsoid& ellipsoid,
                                    const Transform3s& tf) {
  std::vector<Vec3s> result(12);
  const Scalar phi = (1 + sqrt(Scalar(5))) / Scalar(2);

  const Scalar a = sqrt(Scalar(3)) / (phi * phi);
  const Scalar b = phi * a;

  const Scalar& A = ellipsoid.radii[0];
  const Scalar& B = ellipsoid.radii[1];
  const Scalar& C = ellipsoid.radii[2];

  Scalar Aa = A * a;
  Scalar Ab = A * b;
  Scalar Ba = B * a;
  Scalar Bb = B * b;
  Scalar Ca = C * a;
  Scalar Cb = C * b;
  result[0] = tf.transform(Vec3s(0, Ba, Cb));
  result[1] = tf.transform(Vec3s(0, -Ba, Cb));
  result[2] = tf.transform(Vec3s(0, Ba, -Cb));
  result[3] = tf.transform(Vec3s(0, -Ba, -Cb));
  result[4] = tf.transform(Vec3s(Aa, Bb, 0));
  result[5] = tf.transform(Vec3s(-Aa, Bb, 0));
  result[6] = tf.transform(Vec3s(Aa, -Bb, 0));
  result[7] = tf.transform(Vec3s(-Aa, -Bb, 0));
  result[8] = tf.transform(Vec3s(Ab, 0, Ca));
  result[9] = tf.transform(Vec3s(Ab, 0, -Ca));
  result[10] = tf.transform(Vec3s(-Ab, 0, Ca));
  result[11] = tf.transform(Vec3s(-Ab, 0, -Ca));

  return result;
}

std::vector<Vec3s> getBoundVertices(const Capsule& capsule,
                                    const Transform3s& tf) {
  std::vector<Vec3s> result(36);
  const Scalar m = (1 + sqrt(Scalar(5))) / Scalar(2);

  Scalar hl = capsule.halfLength;
  Scalar edge_size = capsule.radius * 6 / (sqrt(Scalar(27)) + sqrt(Scalar(15)));
  Scalar a = edge_size;
  Scalar b = m * edge_size;
  Scalar r2 = capsule.radius * 2 / sqrt(Scalar(3));

  result[0] = tf.transform(Vec3s(0, a, b + hl));
  result[1] = tf.transform(Vec3s(0, -a, b + hl));
  result[2] = tf.transform(Vec3s(0, a, -b + hl));
  result[3] = tf.transform(Vec3s(0, -a, -b + hl));
  result[4] = tf.transform(Vec3s(a, b, hl));
  result[5] = tf.transform(Vec3s(-a, b, hl));
  result[6] = tf.transform(Vec3s(a, -b, hl));
  result[7] = tf.transform(Vec3s(-a, -b, hl));
  result[8] = tf.transform(Vec3s(b, 0, a + hl));
  result[9] = tf.transform(Vec3s(b, 0, -a + hl));
  result[10] = tf.transform(Vec3s(-b, 0, a + hl));
  result[11] = tf.transform(Vec3s(-b, 0, -a + hl));

  result[12] = tf.transform(Vec3s(0, a, b - hl));
  result[13] = tf.transform(Vec3s(0, -a, b - hl));
  result[14] = tf.transform(Vec3s(0, a, -b - hl));
  result[15] = tf.transform(Vec3s(0, -a, -b - hl));
  result[16] = tf.transform(Vec3s(a, b, -hl));
  result[17] = tf.transform(Vec3s(-a, b, -hl));
  result[18] = tf.transform(Vec3s(a, -b, -hl));
  result[19] = tf.transform(Vec3s(-a, -b, -hl));
  result[20] = tf.transform(Vec3s(b, 0, a - hl));
  result[21] = tf.transform(Vec3s(b, 0, -a - hl));
  result[22] = tf.transform(Vec3s(-b, 0, a - hl));
  result[23] = tf.transform(Vec3s(-b, 0, -a - hl));

  Scalar c = Scalar(0.5) * r2;
  Scalar d = capsule.radius;
  result[24] = tf.transform(Vec3s(r2, 0, hl));
  result[25] = tf.transform(Vec3s(c, d, hl));
  result[26] = tf.transform(Vec3s(-c, d, hl));
  result[27] = tf.transform(Vec3s(-r2, 0, hl));
  result[28] = tf.transform(Vec3s(-c, -d, hl));
  result[29] = tf.transform(Vec3s(c, -d, hl));

  result[30] = tf.transform(Vec3s(r2, 0, -hl));
  result[31] = tf.transform(Vec3s(c, d, -hl));
  result[32] = tf.transform(Vec3s(-c, d, -hl));
  result[33] = tf.transform(Vec3s(-r2, 0, -hl));
  result[34] = tf.transform(Vec3s(-c, -d, -hl));
  result[35] = tf.transform(Vec3s(c, -d, -hl));

  return result;
}

std::vector<Vec3s> getBoundVertices(const Cone& cone, const Transform3s& tf) {
  std::vector<Vec3s> result(7);

  Scalar hl = cone.halfLength;
  Scalar r2 = cone.radius * 2 / sqrt(Scalar(3));
  Scalar a = Scalar(0.5) * r2;
  Scalar b = cone.radius;

  result[0] = tf.transform(Vec3s(r2, 0, -hl));
  result[1] = tf.transform(Vec3s(a, b, -hl));
  result[2] = tf.transform(Vec3s(-a, b, -hl));
  result[3] = tf.transform(Vec3s(-r2, 0, -hl));
  result[4] = tf.transform(Vec3s(-a, -b, -hl));
  result[5] = tf.transform(Vec3s(a, -b, -hl));

  result[6] = tf.transform(Vec3s(0, 0, hl));

  return result;
}

std::vector<Vec3s> getBoundVertices(const Cylinder& cylinder,
                                    const Transform3s& tf) {
  std::vector<Vec3s> result(12);

  Scalar hl = cylinder.halfLength;
  Scalar r2 = cylinder.radius * 2 / sqrt(Scalar(3));
  Scalar a = Scalar(0.5) * r2;
  Scalar b = cylinder.radius;

  result[0] = tf.transform(Vec3s(r2, 0, -hl));
  result[1] = tf.transform(Vec3s(a, b, -hl));
  result[2] = tf.transform(Vec3s(-a, b, -hl));
  result[3] = tf.transform(Vec3s(-r2, 0, -hl));
  result[4] = tf.transform(Vec3s(-a, -b, -hl));
  result[5] = tf.transform(Vec3s(a, -b, -hl));

  result[6] = tf.transform(Vec3s(r2, 0, hl));
  result[7] = tf.transform(Vec3s(a, b, hl));
  result[8] = tf.transform(Vec3s(-a, b, hl));
  result[9] = tf.transform(Vec3s(-r2, 0, hl));
  result[10] = tf.transform(Vec3s(-a, -b, hl));
  result[11] = tf.transform(Vec3s(a, -b, hl));

  return result;
}

std::vector<Vec3s> getBoundVertices(const TriangleP& triangle,
                                    const Transform3s& tf) {
  std::vector<Vec3s> result(3);
  result[0] = tf.transform(triangle.a);
  result[1] = tf.transform(triangle.b);
  result[2] = tf.transform(triangle.c);

  return result;
}

}  // namespace details

Halfspace transform(const Halfspace& a, const Transform3s& tf) {
  /// suppose the initial halfspace is n * x <= d
  /// after transform (R, T), x --> x' = R x + T
  /// and the new half space becomes n' * x' <= d'
  /// where n' = R * n
  ///   and d' = d + n' * T

  Vec3s n = tf.getRotation() * a.n;
  Scalar d = a.d + n.dot(tf.getTranslation());
  Halfspace result(n, d);
  result.setSweptSphereRadius(a.getSweptSphereRadius());

  return result;
}

Plane transform(const Plane& a, const Transform3s& tf) {
  /// suppose the initial halfspace is n * x <= d
  /// after transform (R, T), x --> x' = R x + T
  /// and the new half space becomes n' * x' <= d'
  /// where n' = R * n
  ///   and d' = d + n' * T

  Vec3s n = tf.getRotation() * a.n;
  Scalar d = a.d + n.dot(tf.getTranslation());
  Plane result(n, d);
  result.setSweptSphereRadius(a.getSweptSphereRadius());

  return result;
}

std::array<Halfspace, 2> transformToHalfspaces(const Plane& a,
                                               const Transform3s& tf) {
  // A plane can be represented by two halfspaces

  Vec3s n = tf.getRotation() * a.n;
  Scalar d = a.d + n.dot(tf.getTranslation());
  std::array<Halfspace, 2> result = {Halfspace(n, d), Halfspace(-n, -d)};
  result[0].setSweptSphereRadius(a.getSweptSphereRadius());
  result[1].setSweptSphereRadius(a.getSweptSphereRadius());

  return result;
}

template <>
void computeBV<AABB, Box>(const Box& s, const Transform3s& tf, AABB& bv) {
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  Vec3s v_delta(R.cwiseAbs() * s.halfSide);
  bv.max_ = T + v_delta;
  bv.min_ = T - v_delta;
}

template <>
void computeBV<AABB, Sphere>(const Sphere& s, const Transform3s& tf, AABB& bv) {
  const Vec3s& T = tf.getTranslation();

  Vec3s v_delta(Vec3s::Constant(s.radius));
  bv.max_ = T + v_delta;
  bv.min_ = T - v_delta;
}

template <>
void computeBV<AABB, Ellipsoid>(const Ellipsoid& e, const Transform3s& tf,
                                AABB& bv) {
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  Vec3s v_delta = R * e.radii;
  bv.max_ = T + v_delta;
  bv.min_ = T - v_delta;
}

template <>
void computeBV<AABB, Capsule>(const Capsule& s, const Transform3s& tf,
                              AABB& bv) {
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  Vec3s v_delta(R.col(2).cwiseAbs() * s.halfLength + Vec3s::Constant(s.radius));
  bv.max_ = T + v_delta;
  bv.min_ = T - v_delta;
}

template <>
void computeBV<AABB, Cone>(const Cone& s, const Transform3s& tf, AABB& bv) {
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  Scalar x_range = fabs(R(0, 0) * s.radius) + fabs(R(0, 1) * s.radius) +
                   fabs(R(0, 2) * s.halfLength);
  Scalar y_range = fabs(R(1, 0) * s.radius) + fabs(R(1, 1) * s.radius) +
                   fabs(R(1, 2) * s.halfLength);
  Scalar z_range = fabs(R(2, 0) * s.radius) + fabs(R(2, 1) * s.radius) +
                   fabs(R(2, 2) * s.halfLength);

  Vec3s v_delta(x_range, y_range, z_range);
  bv.max_ = T + v_delta;
  bv.min_ = T - v_delta;
}

template <>
void computeBV<AABB, Cylinder>(const Cylinder& s, const Transform3s& tf,
                               AABB& bv) {
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  Scalar x_range = fabs(R(0, 0) * s.radius) + fabs(R(0, 1) * s.radius) +
                   fabs(R(0, 2) * s.halfLength);
  Scalar y_range = fabs(R(1, 0) * s.radius) + fabs(R(1, 1) * s.radius) +
                   fabs(R(1, 2) * s.halfLength);
  Scalar z_range = fabs(R(2, 0) * s.radius) + fabs(R(2, 1) * s.radius) +
                   fabs(R(2, 2) * s.halfLength);

  Vec3s v_delta(x_range, y_range, z_range);
  bv.max_ = T + v_delta;
  bv.min_ = T - v_delta;
}

template <typename IndexType>
void computeAABBConvex(const ConvexBaseTpl<IndexType>& s, const Transform3s& tf,
                       AABB& bv) {
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  AABB bv_;
  const std::vector<Vec3s>& points_ = *(s.points);
  for (std::size_t i = 0; i < s.num_points; ++i) {
    Vec3s new_p = R * points_[i] + T;
    bv_ += new_p;
  }

  bv = bv_;
}

template <>
void computeBV<AABB, ConvexBase32>(const ConvexBase32& s, const Transform3s& tf,
                                   AABB& bv) {
  computeAABBConvex(s, tf, bv);
}

template <>
void computeBV<AABB, ConvexBase16>(const ConvexBase16& s, const Transform3s& tf,
                                   AABB& bv) {
  computeAABBConvex(s, tf, bv);
}

template <>
void computeBV<AABB, TriangleP>(const TriangleP& s, const Transform3s& tf,
                                AABB& bv) {
  bv = AABB(tf.transform(s.a), tf.transform(s.b), tf.transform(s.c));
}

template <>
void computeBV<AABB, Halfspace>(const Halfspace& s, const Transform3s& tf,
                                AABB& bv) {
  Halfspace new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  AABB bv_;
  bv_.min_ = Vec3s::Constant(-(std::numeric_limits<Scalar>::max)());
  bv_.max_ = Vec3s::Constant((std::numeric_limits<Scalar>::max)());
  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    // normal aligned with x axis
    if (n[0] < 0)
      bv_.min_[0] = -d;
    else if (n[0] > 0)
      bv_.max_[0] = d;
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    // normal aligned with y axis
    if (n[1] < 0)
      bv_.min_[1] = -d;
    else if (n[1] > 0)
      bv_.max_[1] = d;
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    // normal aligned with z axis
    if (n[2] < 0)
      bv_.min_[2] = -d;
    else if (n[2] > 0)
      bv_.max_[2] = d;
  }

  bv = bv_;
}

template <>
void computeBV<AABB, Plane>(const Plane& s, const Transform3s& tf, AABB& bv) {
  Plane new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  AABB bv_;
  bv_.min_ = Vec3s::Constant(-(std::numeric_limits<Scalar>::max)());
  bv_.max_ = Vec3s::Constant((std::numeric_limits<Scalar>::max)());
  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    // normal aligned with x axis
    if (n[0] < 0) {
      bv_.min_[0] = bv_.max_[0] = -d;
    } else if (n[0] > 0) {
      bv_.min_[0] = bv_.max_[0] = d;
    }
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    // normal aligned with y axis
    if (n[1] < 0) {
      bv_.min_[1] = bv_.max_[1] = -d;
    } else if (n[1] > 0) {
      bv_.min_[1] = bv_.max_[1] = d;
    }
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    // normal aligned with z axis
    if (n[2] < 0) {
      bv_.min_[2] = bv_.max_[2] = -d;
    } else if (n[2] > 0) {
      bv_.min_[2] = bv_.max_[2] = d;
    }
  }

  bv = bv_;
}

template <>
void computeBV<OBB, Box>(const Box& s, const Transform3s& tf, OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  bv.To = T;
  bv.axes = R;
  bv.extent = s.halfSide;
}

template <>
void computeBV<OBB, Sphere>(const Sphere& s, const Transform3s& tf, OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  const Vec3s& T = tf.getTranslation();

  bv.To.noalias() = T;
  bv.axes.setIdentity();
  bv.extent.setConstant(s.radius);
}

template <>
void computeBV<OBB, Capsule>(const Capsule& s, const Transform3s& tf, OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  bv.To.noalias() = T;
  bv.axes.noalias() = R;
  bv.extent << s.radius, s.radius, s.halfLength + s.radius;
}

template <>
void computeBV<OBB, Cone>(const Cone& s, const Transform3s& tf, OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  bv.To.noalias() = T;
  bv.axes.noalias() = R;
  bv.extent << s.radius, s.radius, s.halfLength;
}

template <>
void computeBV<OBB, Cylinder>(const Cylinder& s, const Transform3s& tf,
                              OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  bv.To.noalias() = T;
  bv.axes.noalias() = R;
  bv.extent << s.radius, s.radius, s.halfLength;
}

template <typename IndexType>
void computeOBBConvex(const ConvexBaseTpl<IndexType>& s, const Transform3s& tf,
                      OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  const Matrix3s& R = tf.getRotation();
  const Vec3s& T = tf.getTranslation();

  fit(s.points->data(), s.num_points, bv);

  bv.axes.applyOnTheLeft(R);

  bv.To = R * bv.To + T;
}

template <>
void computeBV<OBB, ConvexBase32>(const ConvexBase32& s, const Transform3s& tf,
                                  OBB& bv) {
  computeOBBConvex(s, tf, bv);
}

template <>
void computeBV<OBB, ConvexBase16>(const ConvexBase16& s, const Transform3s& tf,
                                  OBB& bv) {
  computeOBBConvex(s, tf, bv);
}

template <>
void computeBV<OBB, Halfspace>(const Halfspace& s, const Transform3s&,
                               OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  /// Half space can only have very rough OBB
  bv.axes.setIdentity();
  bv.To.setZero();
  bv.extent.setConstant(((std::numeric_limits<Scalar>::max)()));
}

template <>
void computeBV<RSS, Halfspace>(const Halfspace& s, const Transform3s&,
                               RSS& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  /// Half space can only have very rough RSS
  bv.axes.setIdentity();
  bv.Tr.setZero();
  bv.length[0] = bv.length[1] = bv.radius =
      (std::numeric_limits<Scalar>::max)();
}

template <>
void computeBV<OBBRSS, Halfspace>(const Halfspace& s, const Transform3s& tf,
                                  OBBRSS& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  computeBV<OBB, Halfspace>(s, tf, bv.obb);
  computeBV<RSS, Halfspace>(s, tf, bv.rss);
}

template <>
void computeBV<kIOS, Halfspace>(const Halfspace& s, const Transform3s& tf,
                                kIOS& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  bv.num_spheres = 1;
  computeBV<OBB, Halfspace>(s, tf, bv.obb);
  bv.spheres[0].o = Vec3s();
  bv.spheres[0].r = (std::numeric_limits<Scalar>::max)();
}

template <>
void computeBV<KDOP<16>, Halfspace>(const Halfspace& s, const Transform3s& tf,
                                    KDOP<16>& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Halfspace new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  const short D = 8;
  for (short i = 0; i < D; ++i)
    bv.dist(i) = -(std::numeric_limits<Scalar>::max)();
  for (short i = D; i < 2 * D; ++i)
    bv.dist(i) = (std::numeric_limits<Scalar>::max)();

  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D) = d;
    else
      bv.dist(0) = -d;
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(D + 1) = d;
    else
      bv.dist(1) = -d;
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    if (n[2] > 0)
      bv.dist(D + 2) = d;
    else
      bv.dist(2) = -d;
  } else if (n[2] == (Scalar)0.0 && n[0] == n[1]) {
    if (n[0] > 0)
      bv.dist(D + 3) = n[0] * d * 2;
    else
      bv.dist(3) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] == n[2]) {
    if (n[1] > 0)
      bv.dist(D + 4) = n[0] * d * 2;
    else
      bv.dist(4) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] == n[2]) {
    if (n[1] > 0)
      bv.dist(D + 5) = n[1] * d * 2;
    else
      bv.dist(5) = n[1] * d * 2;
  } else if (n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 6) = n[0] * d * 2;
    else
      bv.dist(6) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 7) = n[0] * d * 2;
    else
      bv.dist(7) = n[0] * d * 2;
  }
}

template <>
void computeBV<KDOP<18>, Halfspace>(const Halfspace& s, const Transform3s& tf,
                                    KDOP<18>& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Halfspace new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  const short D = 9;

  for (short i = 0; i < D; ++i)
    bv.dist(i) = -(std::numeric_limits<Scalar>::max)();
  for (short i = D; i < 2 * D; ++i)
    bv.dist(i) = (std::numeric_limits<Scalar>::max)();

  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D) = d;
    else
      bv.dist(0) = -d;
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(D + 1) = d;
    else
      bv.dist(1) = -d;
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    if (n[2] > 0)
      bv.dist(D + 2) = d;
    else
      bv.dist(2) = -d;
  } else if (n[2] == (Scalar)0.0 && n[0] == n[1]) {
    if (n[0] > 0)
      bv.dist(D + 3) = n[0] * d * 2;
    else
      bv.dist(3) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] == n[2]) {
    if (n[1] > 0)
      bv.dist(D + 4) = n[0] * d * 2;
    else
      bv.dist(4) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] == n[2]) {
    if (n[1] > 0)
      bv.dist(D + 5) = n[1] * d * 2;
    else
      bv.dist(5) = n[1] * d * 2;
  } else if (n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 6) = n[0] * d * 2;
    else
      bv.dist(6) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 7) = n[0] * d * 2;
    else
      bv.dist(7) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] + n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(D + 8) = n[1] * d * 2;
    else
      bv.dist(8) = n[1] * d * 2;
  }
}

template <>
void computeBV<KDOP<24>, Halfspace>(const Halfspace& s, const Transform3s& tf,
                                    KDOP<24>& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Halfspace new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  const short D = 12;

  for (short i = 0; i < D; ++i)
    bv.dist(i) = -(std::numeric_limits<Scalar>::max)();
  for (short i = D; i < 2 * D; ++i)
    bv.dist(i) = (std::numeric_limits<Scalar>::max)();

  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D) = d;
    else
      bv.dist(0) = -d;
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(D + 1) = d;
    else
      bv.dist(1) = -d;
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    if (n[2] > 0)
      bv.dist(D + 2) = d;
    else
      bv.dist(2) = -d;
  } else if (n[2] == (Scalar)0.0 && n[0] == n[1]) {
    if (n[0] > 0)
      bv.dist(D + 3) = n[0] * d * 2;
    else
      bv.dist(3) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] == n[2]) {
    if (n[1] > 0)
      bv.dist(D + 4) = n[0] * d * 2;
    else
      bv.dist(4) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] == n[2]) {
    if (n[1] > 0)
      bv.dist(D + 5) = n[1] * d * 2;
    else
      bv.dist(5) = n[1] * d * 2;
  } else if (n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 6) = n[0] * d * 2;
    else
      bv.dist(6) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 7) = n[0] * d * 2;
    else
      bv.dist(7) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] + n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(D + 8) = n[1] * d * 2;
    else
      bv.dist(8) = n[1] * d * 2;
  } else if (n[0] + n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 9) = n[0] * d * 3;
    else
      bv.dist(9) = n[0] * d * 3;
  } else if (n[0] + n[1] == (Scalar)0.0 && n[1] + n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(D + 10) = n[0] * d * 3;
    else
      bv.dist(10) = n[0] * d * 3;
  } else if (n[0] + n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(D + 11) = n[1] * d * 3;
    else
      bv.dist(11) = n[1] * d * 3;
  }
}

template <>
void computeBV<OBB, Plane>(const Plane& s, const Transform3s& tf, OBB& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Vec3s n = tf.getRotation() * s.n;
  generateCoordinateSystem(n, bv.axes.col(1), bv.axes.col(2));
  bv.axes.col(0).noalias() = n;

  bv.extent << 0, (std::numeric_limits<Scalar>::max)(),
      (std::numeric_limits<Scalar>::max)();

  Vec3s p = s.n * s.d;
  bv.To =
      tf.transform(p);  /// n'd' = R * n * (d + (R * n) * T) = R * (n * d) + T
}

template <>
void computeBV<RSS, Plane>(const Plane& s, const Transform3s& tf, RSS& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Vec3s n = tf.getRotation() * s.n;

  generateCoordinateSystem(n, bv.axes.col(1), bv.axes.col(2));
  bv.axes.col(0).noalias() = n;

  bv.length[0] = (std::numeric_limits<Scalar>::max)();
  bv.length[1] = (std::numeric_limits<Scalar>::max)();

  bv.radius = 0;

  Vec3s p = s.n * s.d;
  bv.Tr = tf.transform(p);
}

template <>
void computeBV<OBBRSS, Plane>(const Plane& s, const Transform3s& tf,
                              OBBRSS& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  computeBV<OBB, Plane>(s, tf, bv.obb);
  computeBV<RSS, Plane>(s, tf, bv.rss);
}

template <>
void computeBV<kIOS, Plane>(const Plane& s, const Transform3s& tf, kIOS& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  bv.num_spheres = 1;
  computeBV<OBB, Plane>(s, tf, bv.obb);
  bv.spheres[0].o = Vec3s();
  bv.spheres[0].r = (std::numeric_limits<Scalar>::max)();
}

template <>
void computeBV<KDOP<16>, Plane>(const Plane& s, const Transform3s& tf,
                                KDOP<16>& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Plane new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  const short D = 8;

  for (short i = 0; i < D; ++i)
    bv.dist(i) = -(std::numeric_limits<Scalar>::max)();
  for (short i = D; i < 2 * D; ++i)
    bv.dist(i) = (std::numeric_limits<Scalar>::max)();

  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(0) = bv.dist(D) = d;
    else
      bv.dist(0) = bv.dist(D) = -d;
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(1) = bv.dist(D + 1) = d;
    else
      bv.dist(1) = bv.dist(D + 1) = -d;
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    if (n[2] > 0)
      bv.dist(2) = bv.dist(D + 2) = d;
    else
      bv.dist(2) = bv.dist(D + 2) = -d;
  } else if (n[2] == (Scalar)0.0 && n[0] == n[1]) {
    bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] == n[2]) {
    bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] == n[2]) {
    bv.dist(6) = bv.dist(D + 5) = n[1] * d * 2;
  } else if (n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
  }
}

template <>
void computeBV<KDOP<18>, Plane>(const Plane& s, const Transform3s& tf,
                                KDOP<18>& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Plane new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  const short D = 9;

  for (short i = 0; i < D; ++i)
    bv.dist(i) = -(std::numeric_limits<Scalar>::max)();
  for (short i = D; i < 2 * D; ++i)
    bv.dist(i) = (std::numeric_limits<Scalar>::max)();

  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(0) = bv.dist(D) = d;
    else
      bv.dist(0) = bv.dist(D) = -d;
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(1) = bv.dist(D + 1) = d;
    else
      bv.dist(1) = bv.dist(D + 1) = -d;
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    if (n[2] > 0)
      bv.dist(2) = bv.dist(D + 2) = d;
    else
      bv.dist(2) = bv.dist(D + 2) = -d;
  } else if (n[2] == (Scalar)0.0 && n[0] == n[1]) {
    bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] == n[2]) {
    bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] == n[2]) {
    bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
  } else if (n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] + n[2] == (Scalar)0.0) {
    bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
  }
}

template <>
void computeBV<KDOP<24>, Plane>(const Plane& s, const Transform3s& tf,
                                KDOP<24>& bv) {
  if (s.getSweptSphereRadius() > 0) {
    COAL_THROW_PRETTY("Swept-sphere radius not yet supported.",
                      std::runtime_error);
  }
  Plane new_s = transform(s, tf);
  const Vec3s& n = new_s.n;
  const Scalar& d = new_s.d;

  const short D = 12;

  for (short i = 0; i < D; ++i)
    bv.dist(i) = -(std::numeric_limits<Scalar>::max)();
  for (short i = D; i < 2 * D; ++i)
    bv.dist(i) = (std::numeric_limits<Scalar>::max)();

  if (n[1] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[0] > 0)
      bv.dist(0) = bv.dist(D) = d;
    else
      bv.dist(0) = bv.dist(D) = -d;
  } else if (n[0] == (Scalar)0.0 && n[2] == (Scalar)0.0) {
    if (n[1] > 0)
      bv.dist(1) = bv.dist(D + 1) = d;
    else
      bv.dist(1) = bv.dist(D + 1) = -d;
  } else if (n[0] == (Scalar)0.0 && n[1] == (Scalar)0.0) {
    if (n[2] > 0)
      bv.dist(2) = bv.dist(D + 2) = d;
    else
      bv.dist(2) = bv.dist(D + 2) = -d;
  } else if (n[2] == (Scalar)0.0 && n[0] == n[1]) {
    bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] == n[2]) {
    bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] == n[2]) {
    bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
  } else if (n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
  } else if (n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
  } else if (n[0] == (Scalar)0.0 && n[1] + n[2] == (Scalar)0.0) {
    bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
  } else if (n[0] + n[2] == (Scalar)0.0 && n[0] + n[1] == (Scalar)0.0) {
    bv.dist(9) = bv.dist(D + 9) = n[0] * d * 3;
  } else if (n[0] + n[1] == (Scalar)0.0 && n[1] + n[2] == (Scalar)0.0) {
    bv.dist(10) = bv.dist(D + 10) = n[0] * d * 3;
  } else if (n[0] + n[1] == (Scalar)0.0 && n[0] + n[2] == (Scalar)0.0) {
    bv.dist(11) = bv.dist(D + 11) = n[1] * d * 3;
  }
}

void constructBox(const AABB& bv, Box& box, Transform3s& tf) {
  box = Box(bv.max_ - bv.min_);
  tf = Transform3s(bv.center());
}

void constructBox(const OBB& bv, Box& box, Transform3s& tf) {
  box = Box(bv.extent * 2);
  tf = Transform3s(bv.axes, bv.To);
}

void constructBox(const OBBRSS& bv, Box& box, Transform3s& tf) {
  box = Box(bv.obb.extent * 2);
  tf = Transform3s(bv.obb.axes, bv.obb.To);
}

void constructBox(const kIOS& bv, Box& box, Transform3s& tf) {
  box = Box(bv.obb.extent * 2);
  tf = Transform3s(bv.obb.axes, bv.obb.To);
}

void constructBox(const RSS& bv, Box& box, Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = Transform3s(bv.axes, bv.Tr);
}

void constructBox(const KDOP<16>& bv, Box& box, Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = Transform3s(bv.center());
}

void constructBox(const KDOP<18>& bv, Box& box, Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = Transform3s(bv.center());
}

void constructBox(const KDOP<24>& bv, Box& box, Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = Transform3s(bv.center());
}

void constructBox(const AABB& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.max_ - bv.min_);
  tf = tf_bv * Transform3s(bv.center());
}

void constructBox(const OBB& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.extent * 2);
  tf = tf_bv * Transform3s(bv.axes, bv.To);
}

void constructBox(const OBBRSS& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.obb.extent * 2);
  tf = tf_bv * Transform3s(bv.obb.axes, bv.obb.To);
}

void constructBox(const kIOS& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.obb.extent * 2);
  tf = tf_bv * Transform3s(bv.obb.axes, bv.obb.To);
}

void constructBox(const RSS& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Transform3s(bv.axes, bv.Tr);
}

void constructBox(const KDOP<16>& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Transform3s(bv.center());
}

void constructBox(const KDOP<18>& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Transform3s(bv.center());
}

void constructBox(const KDOP<24>& bv, const Transform3s& tf_bv, Box& box,
                  Transform3s& tf) {
  box = Box(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Transform3s(bv.center());
}

}  // namespace coal
