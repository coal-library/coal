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

#include "coal/internal/BV_fitter.h"
#include "coal/BVH/BVH_utility.h"
#include "coal/internal/tools.h"

#include <limits>

namespace coal {

static const Scalar kIOS_RATIO = 1.5;
static const Scalar invSinA = 2;
static const Scalar cosA = sqrt(Scalar(3)) / Scalar(2);

static inline void axisFromEigen(Vec3s eigenV[3], Scalar eigenS[3],
                                 Matrix3s& axes) {
  int min, mid, max;
  if (eigenS[0] > eigenS[1]) {
    max = 0;
    min = 1;
  } else {
    min = 0;
    max = 1;
  }
  if (eigenS[2] < eigenS[min]) {
    mid = min;
    min = 2;
  } else if (eigenS[2] > eigenS[max]) {
    mid = max;
    max = 2;
  } else {
    mid = 2;
  }

  axes.col(0) << eigenV[0][max], eigenV[1][max], eigenV[2][max];
  axes.col(1) << eigenV[0][mid], eigenV[1][mid], eigenV[2][mid];
  axes.col(2) << eigenV[1][max] * eigenV[2][mid] -
                     eigenV[1][mid] * eigenV[2][max],
      eigenV[0][mid] * eigenV[2][max] - eigenV[0][max] * eigenV[2][mid],
      eigenV[0][max] * eigenV[1][mid] - eigenV[0][mid] * eigenV[1][max];
}

namespace OBB_fit_functions {

void fit1(Vec3s* ps, OBB& bv) {
  bv.To.noalias() = ps[0];
  bv.axes.setIdentity();
  bv.extent.setZero();
}

void fit2(Vec3s* ps, OBB& bv) {
  const Vec3s& p1 = ps[0];
  const Vec3s& p2 = ps[1];
  Vec3s p1p2 = p1 - p2;
  Scalar len_p1p2 = p1p2.norm();
  p1p2.normalize();

  bv.axes.col(0).noalias() = p1p2;
  generateCoordinateSystem(bv.axes.col(0), bv.axes.col(1), bv.axes.col(2));

  bv.extent << len_p1p2 * Scalar(0.5), 0, 0;
  bv.To.noalias() = (p1 + p2) / 2;
}

void fit3(Vec3s* ps, OBB& bv) {
  const Vec3s& p1 = ps[0];
  const Vec3s& p2 = ps[1];
  const Vec3s& p3 = ps[2];
  Vec3s e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  Scalar len[3];
  len[0] = e[0].squaredNorm();
  len[1] = e[1].squaredNorm();
  len[2] = e[2].squaredNorm();

  int imax = 0;
  if (len[1] > len[0]) imax = 1;
  if (len[2] > len[imax]) imax = 2;

  bv.axes.col(2).noalias() = e[0].cross(e[1]).normalized();
  bv.axes.col(0).noalias() = e[imax].normalized();
  bv.axes.col(1).noalias() = bv.axes.col(2).cross(bv.axes.col(0));

  getExtentAndCenter(ps, NULL, NULL, NULL, 3, bv.axes, bv.To, bv.extent);
}

void fit6(Vec3s* ps, OBB& bv) {
  OBB bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

void fitn(Vec3s* ps, unsigned int n, OBB& bv) {
  Matrix3s M;
  Vec3s E[3];
  Scalar s[3] = {0, 0, 0};  // three eigen values

  getCovariance(ps, NULL, NULL, NULL, n, M);
  eigen(M, s, E);
  axisFromEigen(E, s, bv.axes);

  // set obb centers and extensions
  getExtentAndCenter(ps, NULL, NULL, NULL, n, bv.axes, bv.To, bv.extent);
}

}  // namespace OBB_fit_functions

namespace RSS_fit_functions {
void fit1(Vec3s* ps, RSS& bv) {
  bv.Tr.noalias() = ps[0];
  bv.axes.setIdentity();
  bv.length[0] = 0;
  bv.length[1] = 0;
  bv.radius = 0;
}

void fit2(Vec3s* ps, RSS& bv) {
  const Vec3s& p1 = ps[0];
  const Vec3s& p2 = ps[1];
  bv.axes.col(0).noalias() = p1 - p2;
  Scalar len_p1p2 = bv.axes.col(0).norm();
  bv.axes.col(0) /= len_p1p2;

  generateCoordinateSystem(bv.axes.col(0), bv.axes.col(1), bv.axes.col(2));
  bv.length[0] = len_p1p2;
  bv.length[1] = 0;

  bv.Tr = p2;
  bv.radius = 0;
}

void fit3(Vec3s* ps, RSS& bv) {
  const Vec3s& p1 = ps[0];
  const Vec3s& p2 = ps[1];
  const Vec3s& p3 = ps[2];
  Vec3s e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  Scalar len[3];
  len[0] = e[0].squaredNorm();
  len[1] = e[1].squaredNorm();
  len[2] = e[2].squaredNorm();

  int imax = 0;
  if (len[1] > len[0]) imax = 1;
  if (len[2] > len[imax]) imax = 2;

  bv.axes.col(2).noalias() = e[0].cross(e[1]).normalized();
  bv.axes.col(0).noalias() = e[imax].normalized();
  bv.axes.col(1).noalias() = bv.axes.col(2).cross(bv.axes.col(0));

  getRadiusAndOriginAndRectangleSize(ps, NULL, NULL, NULL, 3, bv.axes, bv.Tr,
                                     bv.length, bv.radius);
}

void fit6(Vec3s* ps, RSS& bv) {
  RSS bv1, bv2;
  fit3(ps, bv1);
  fit3(ps + 3, bv2);
  bv = bv1 + bv2;
}

void fitn(Vec3s* ps, unsigned int n, RSS& bv) {
  Matrix3s M;  // row first matrix
  Vec3s E[3];  // row first eigen-vectors
  Scalar s[3] = {0, 0, 0};

  getCovariance(ps, NULL, NULL, NULL, n, M);
  eigen(M, s, E);
  axisFromEigen(E, s, bv.axes);

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize(ps, NULL, NULL, NULL, n, bv.axes, bv.Tr,
                                     bv.length, bv.radius);
}

}  // namespace RSS_fit_functions

namespace kIOS_fit_functions {

void fit1(Vec3s* ps, kIOS& bv) {
  bv.num_spheres = 1;
  bv.spheres[0].o.noalias() = ps[0];
  bv.spheres[0].r = 0;

  bv.obb.axes.setIdentity();
  bv.obb.extent.setZero();
  bv.obb.To.noalias() = ps[0];
}

void fit2(Vec3s* ps, kIOS& bv) {
  bv.num_spheres = 5;

  const Vec3s& p1 = ps[0];
  const Vec3s& p2 = ps[1];
  Vec3s p1p2 = p1 - p2;
  Scalar len_p1p2 = p1p2.norm();
  p1p2.normalize();

  Matrix3s& axes = bv.obb.axes;
  axes.col(0).noalias() = p1p2;
  generateCoordinateSystem(axes.col(0), axes.col(1), axes.col(2));

  Scalar r0 = len_p1p2 * Scalar(0.5);
  bv.obb.extent << r0, 0, 0;
  bv.obb.To = (p1 + p2) * 0.5;

  bv.spheres[0].o = bv.obb.To;
  bv.spheres[0].r = r0;

  Scalar r1 = r0 * invSinA;
  Scalar r1cosA = r1 * cosA;
  bv.spheres[1].r = r1;
  bv.spheres[2].r = r1;
  Vec3s delta = axes.col(1) * r1cosA;
  bv.spheres[1].o = bv.spheres[0].o - delta;
  bv.spheres[2].o = bv.spheres[0].o + delta;

  bv.spheres[3].r = r1;
  bv.spheres[4].r = r1;
  delta = axes.col(2) * r1cosA;
  bv.spheres[3].o = bv.spheres[0].o - delta;
  bv.spheres[4].o = bv.spheres[0].o + delta;
}

void fit3(Vec3s* ps, kIOS& bv) {
  bv.num_spheres = 3;

  const Vec3s& p1 = ps[0];
  const Vec3s& p2 = ps[1];
  const Vec3s& p3 = ps[2];
  Vec3s e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  Scalar len[3];
  len[0] = e[0].squaredNorm();
  len[1] = e[1].squaredNorm();
  len[2] = e[2].squaredNorm();

  int imax = 0;
  if (len[1] > len[0]) imax = 1;
  if (len[2] > len[imax]) imax = 2;

  bv.obb.axes.col(2).noalias() = e[0].cross(e[1]).normalized();
  bv.obb.axes.col(0).noalias() = e[imax].normalized();
  bv.obb.axes.col(1).noalias() = bv.obb.axes.col(2).cross(bv.obb.axes.col(0));

  getExtentAndCenter(ps, NULL, NULL, NULL, 3, bv.obb.axes, bv.obb.To,
                     bv.obb.extent);

  // compute radius and center
  Scalar r0;
  Vec3s center;
  circumCircleComputation(p1, p2, p3, center, r0);

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  Scalar r1 = r0 * invSinA;
  Vec3s delta = bv.obb.axes.col(2) * (r1 * cosA);

  bv.spheres[1].r = r1;
  bv.spheres[1].o = center - delta;
  bv.spheres[2].r = r1;
  bv.spheres[2].o = center + delta;
}

void fitn(Vec3s* ps, unsigned int n, kIOS& bv) {
  Matrix3s M;
  Vec3s E[3];
  Scalar s[3] = {0, 0, 0};  // three eigen values;

  getCovariance(ps, NULL, NULL, NULL, n, M);
  eigen(M, s, E);

  Matrix3s& axes = bv.obb.axes;
  axisFromEigen(E, s, axes);

  getExtentAndCenter(ps, NULL, NULL, NULL, n, axes, bv.obb.To, bv.obb.extent);

  // get center and extension
  const Vec3s& center = bv.obb.To;
  const Vec3s& extent = bv.obb.extent;
  Scalar r0 = maximumDistance(ps, NULL, NULL, NULL, n, center);

  // decide the k in kIOS
  if (extent[0] > kIOS_RATIO * extent[2]) {
    if (extent[0] > kIOS_RATIO * extent[1])
      bv.num_spheres = 5;
    else
      bv.num_spheres = 3;
  } else
    bv.num_spheres = 1;

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  if (bv.num_spheres >= 3) {
    Scalar r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * invSinA;
    Vec3s delta = axes.col(2) * (r10 * cosA - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;

    Scalar r11 = 0, r12 = 0;
    r11 = maximumDistance(ps, NULL, NULL, NULL, n, bv.spheres[1].o);
    r12 = maximumDistance(ps, NULL, NULL, NULL, n, bv.spheres[2].o);
    bv.spheres[1].o += axes.col(2) * (-r10 + r11);
    bv.spheres[2].o += axes.col(2) * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if (bv.num_spheres >= 5) {
    Scalar r10 = bv.spheres[1].r;
    Vec3s delta =
        axes.col(1) *
        (sqrt(r10 * r10 - extent[0] * extent[0] - extent[2] * extent[2]) -
         extent[1]);
    bv.spheres[3].o = bv.spheres[0].o - delta;
    bv.spheres[4].o = bv.spheres[0].o + delta;

    Scalar r21 = 0, r22 = 0;
    r21 = maximumDistance(ps, NULL, NULL, NULL, n, bv.spheres[3].o);
    r22 = maximumDistance(ps, NULL, NULL, NULL, n, bv.spheres[4].o);

    bv.spheres[3].o += axes.col(1) * (-r10 + r21);
    bv.spheres[4].o += axes.col(1) * (r10 - r22);

    bv.spheres[3].r = r10;
    bv.spheres[4].r = r10;
  }
}

}  // namespace kIOS_fit_functions

namespace OBBRSS_fit_functions {
void fit1(Vec3s* ps, OBBRSS& bv) {
  OBB_fit_functions::fit1(ps, bv.obb);
  RSS_fit_functions::fit1(ps, bv.rss);
}

void fit2(Vec3s* ps, OBBRSS& bv) {
  OBB_fit_functions::fit2(ps, bv.obb);
  RSS_fit_functions::fit2(ps, bv.rss);
}

void fit3(Vec3s* ps, OBBRSS& bv) {
  OBB_fit_functions::fit3(ps, bv.obb);
  RSS_fit_functions::fit3(ps, bv.rss);
}

void fitn(Vec3s* ps, unsigned int n, OBBRSS& bv) {
  OBB_fit_functions::fitn(ps, n, bv.obb);
  RSS_fit_functions::fitn(ps, n, bv.rss);
}

}  // namespace OBBRSS_fit_functions

template <>
void fit(Vec3s* ps, unsigned int n, OBB& bv) {
  switch (n) {
    case 1:
      OBB_fit_functions::fit1(ps, bv);
      break;
    case 2:
      OBB_fit_functions::fit2(ps, bv);
      break;
    case 3:
      OBB_fit_functions::fit3(ps, bv);
      break;
    case 6:
      OBB_fit_functions::fit6(ps, bv);
      break;
    default:
      OBB_fit_functions::fitn(ps, n, bv);
  }
}

template <>
void fit(Vec3s* ps, unsigned int n, RSS& bv) {
  switch (n) {
    case 1:
      RSS_fit_functions::fit1(ps, bv);
      break;
    case 2:
      RSS_fit_functions::fit2(ps, bv);
      break;
    case 3:
      RSS_fit_functions::fit3(ps, bv);
      break;
    default:
      RSS_fit_functions::fitn(ps, n, bv);
  }
}

template <>
void fit(Vec3s* ps, unsigned int n, kIOS& bv) {
  switch (n) {
    case 1:
      kIOS_fit_functions::fit1(ps, bv);
      break;
    case 2:
      kIOS_fit_functions::fit2(ps, bv);
      break;
    case 3:
      kIOS_fit_functions::fit3(ps, bv);
      break;
    default:
      kIOS_fit_functions::fitn(ps, n, bv);
  }
}

template <>
void fit(Vec3s* ps, unsigned int n, OBBRSS& bv) {
  switch (n) {
    case 1:
      OBBRSS_fit_functions::fit1(ps, bv);
      break;
    case 2:
      OBBRSS_fit_functions::fit2(ps, bv);
      break;
    case 3:
      OBBRSS_fit_functions::fit3(ps, bv);
      break;
    default:
      OBBRSS_fit_functions::fitn(ps, n, bv);
  }
}

template <>
void fit(Vec3s* ps, unsigned int n, AABB& bv) {
  if (n <= 0) return;
  bv = AABB(ps[0]);
  for (unsigned int i = 1; i < n; ++i) {
    bv += ps[i];
  }
}

OBB BVFitter<OBB>::fit(unsigned int* primitive_indices,
                       unsigned int num_primitives) {
  OBB bv;

  Matrix3s M;   // row first matrix
  Vec3s E[3];   // row first eigen-vectors
  Scalar s[3];  // three eigen values

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices,
                num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.axes);

  // set obb centers and extensions
  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices,
                     num_primitives, bv.axes, bv.To, bv.extent);

  return bv;
}

OBBRSS BVFitter<OBBRSS>::fit(unsigned int* primitive_indices,
                             unsigned int num_primitives) {
  OBBRSS bv;
  Matrix3s M;
  Vec3s E[3];
  Scalar s[3];

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices,
                num_primitives, M);
  eigen(M, s, E);

  axisFromEigen(E, s, bv.obb.axes);
  bv.rss.axes.noalias() = bv.obb.axes;

  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices,
                     num_primitives, bv.obb.axes, bv.obb.To, bv.obb.extent);

  Vec3s origin;
  Scalar l[2];
  Scalar r;
  getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices,
                                     primitive_indices, num_primitives,
                                     bv.rss.axes, origin, l, r);

  bv.rss.Tr = origin;
  bv.rss.length[0] = l[0];
  bv.rss.length[1] = l[1];
  bv.rss.radius = r;

  return bv;
}

RSS BVFitter<RSS>::fit(unsigned int* primitive_indices,
                       unsigned int num_primitives) {
  RSS bv;

  Matrix3s M;   // row first matrix
  Vec3s E[3];   // row first eigen-vectors
  Scalar s[3];  // three eigen values
  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices,
                num_primitives, M);
  eigen(M, s, E);
  axisFromEigen(E, s, bv.axes);

  // set rss origin, rectangle size and radius

  Vec3s origin;
  Scalar l[2];
  Scalar r;
  getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices,
                                     primitive_indices, num_primitives, bv.axes,
                                     origin, l, r);

  bv.Tr = origin;
  bv.length[0] = l[0];
  bv.length[1] = l[1];
  bv.radius = r;

  return bv;
}

kIOS BVFitter<kIOS>::fit(unsigned int* primitive_indices,
                         unsigned int num_primitives) {
  kIOS bv;

  Matrix3s M;  // row first matrix
  Vec3s E[3];  // row first eigen-vectors
  Scalar s[3];

  getCovariance(vertices, prev_vertices, tri_indices, primitive_indices,
                num_primitives, M);
  eigen(M, s, E);

  Matrix3s& axes = bv.obb.axes;
  axisFromEigen(E, s, axes);

  // get centers and extensions
  getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices,
                     num_primitives, axes, bv.obb.To, bv.obb.extent);

  const Vec3s& center = bv.obb.To;
  const Vec3s& extent = bv.obb.extent;
  Scalar r0 = maximumDistance(vertices, prev_vertices, tri_indices,
                              primitive_indices, num_primitives, center);

  // decide k in kIOS
  if (extent[0] > kIOS_RATIO * extent[2]) {
    if (extent[0] > kIOS_RATIO * extent[1])
      bv.num_spheres = 5;
    else
      bv.num_spheres = 3;
  } else
    bv.num_spheres = 1;

  bv.spheres[0].o = center;
  bv.spheres[0].r = r0;

  if (bv.num_spheres >= 3) {
    Scalar r10 = sqrt(r0 * r0 - extent[2] * extent[2]) * invSinA;
    Vec3s delta = axes.col(2) * (r10 * cosA - extent[2]);
    bv.spheres[1].o = center - delta;
    bv.spheres[2].o = center + delta;

    Scalar r11 =
        maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices,
                        num_primitives, bv.spheres[1].o);
    Scalar r12 =
        maximumDistance(vertices, prev_vertices, tri_indices, primitive_indices,
                        num_primitives, bv.spheres[2].o);

    bv.spheres[1].o += axes.col(2) * (-r10 + r11);
    bv.spheres[2].o += axes.col(2) * (r10 - r12);

    bv.spheres[1].r = r10;
    bv.spheres[2].r = r10;
  }

  if (bv.num_spheres >= 5) {
    Scalar r10 = bv.spheres[1].r;
    Vec3s delta =
        axes.col(1) *
        (sqrt(r10 * r10 - extent[0] * extent[0] - extent[2] * extent[2]) -
         extent[1]);
    bv.spheres[3].o = bv.spheres[0].o - delta;
    bv.spheres[4].o = bv.spheres[0].o + delta;

    Scalar r21 = 0, r22 = 0;
    r21 = maximumDistance(vertices, prev_vertices, tri_indices,
                          primitive_indices, num_primitives, bv.spheres[3].o);
    r22 = maximumDistance(vertices, prev_vertices, tri_indices,
                          primitive_indices, num_primitives, bv.spheres[4].o);

    bv.spheres[3].o += axes.col(1) * (-r10 + r21);
    bv.spheres[4].o += axes.col(1) * (r10 - r22);

    bv.spheres[3].r = r10;
    bv.spheres[4].r = r10;
  }

  return bv;
}

AABB BVFitter<AABB>::fit(unsigned int* primitive_indices,
                         unsigned int num_primitives) {
  AABB bv;
  if (num_primitives == 0) return bv;

  if (type == BVH_MODEL_TRIANGLES)  /// The primitive is triangle
  {
    Triangle32 t0 = tri_indices[primitive_indices[0]];
    bv = AABB(vertices[t0[0]]);

    for (unsigned int i = 0; i < num_primitives; ++i) {
      Triangle32 t = tri_indices[primitive_indices[i]];
      bv += vertices[t[0]];
      bv += vertices[t[1]];
      bv += vertices[t[2]];

      if (prev_vertices)  /// can fitting both current and previous frame
      {
        bv += prev_vertices[t[0]];
        bv += prev_vertices[t[1]];
        bv += prev_vertices[t[2]];
      }
    }
    return bv;
  } else if (type == BVH_MODEL_POINTCLOUD)  /// The primitive is point
  {
    bv = AABB(vertices[primitive_indices[0]]);
    for (unsigned int i = 0; i < num_primitives; ++i) {
      bv += vertices[primitive_indices[i]];

      if (prev_vertices)  /// can fitting both current and previous frame
      {
        bv += prev_vertices[primitive_indices[i]];
      }
    }
  }
  return bv;
}

}  // namespace coal
