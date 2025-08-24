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

#ifndef COAL_INTERSECT_HXX
#define COAL_INTERSECT_HXX

#include "coal/internal/intersect.h"
#include "coal/internal/tools.h"

#include <iostream>
#include <limits>
#include <vector>
#include <cmath>

namespace coal {

template <typename Scalar>
inline typename Project<Scalar>::ProjectResult Project<Scalar>::projectLine(
    const Vec3& a, const Vec3& b, const Vec3& p) {
  ProjectResult res;

  const Vec3 d = b - a;
  const Scalar l = d.squaredNorm();

  if (l > 0) {
    const Scalar t = (p - a).dot(d);
    res.parameterization[1] = (t >= l) ? 1 : ((t <= 0) ? 0 : (t / l));
    res.parameterization[0] = 1 - res.parameterization[1];
    if (t >= l) {
      res.sqr_distance = (p - b).squaredNorm();
      res.encode = 2; /* 0x10 */
    } else if (t <= 0) {
      res.sqr_distance = (p - a).squaredNorm();
      res.encode = 1; /* 0x01 */
    } else {
      res.sqr_distance = (a + d * res.parameterization[1] - p).squaredNorm();
      res.encode = 3; /* 0x00 */
    }
  }

  return res;
}

template <typename Scalar>
inline typename Project<Scalar>::ProjectResult Project<Scalar>::projectTriangle(
    const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p) {
  ProjectResult res;

  static const size_t nexti[3] = {1, 2, 0};
  const Vec3* vt[] = {&a, &b, &c};
  const Vec3 dl[] = {a - b, b - c, c - a};
  const Vec3& n = dl[0].cross(dl[1]);
  const Scalar l = n.squaredNorm();

  if (l > 0) {
    Scalar mindist = -1;
    for (size_t i = 0; i < 3; ++i) {
      if ((*vt[i] - p).dot(dl[i].cross(n)) >
          0)  // origin is to the outside part of the triangle edge, then the
              // optimal can only be on the edge
      {
        size_t j = nexti[i];
        ProjectResult res_line = projectLine(*vt[i], *vt[j], p);

        if (mindist < 0 || res_line.sqr_distance < mindist) {
          mindist = res_line.sqr_distance;
          res.encode =
              static_cast<unsigned int>(((res_line.encode & 1) ? 1 << i : 0) +
                                        ((res_line.encode & 2) ? 1 << j : 0));
          res.parameterization[i] = res_line.parameterization[0];
          res.parameterization[j] = res_line.parameterization[1];
          res.parameterization[nexti[j]] = 0;
        }
      }
    }

    if (mindist < 0)  // the origin project is within the triangle
    {
      Scalar d = (a - p).dot(n);
      Scalar s = std::sqrt(l);
      Vec3 p_to_project = n * (d / l);
      mindist = p_to_project.squaredNorm();
      res.encode = 7;  // m = 0x111
      res.parameterization[0] = dl[1].cross(b - p - p_to_project).norm() / s;
      res.parameterization[1] = dl[2].cross(c - p - p_to_project).norm() / s;
      res.parameterization[2] =
          1 - res.parameterization[0] - res.parameterization[1];
    }

    res.sqr_distance = mindist;
  }

  return res;
}

template <typename Scalar>
inline typename Project<Scalar>::ProjectResult
Project<Scalar>::projectTetrahedra(const Vec3& a, const Vec3& b, const Vec3& c,
                                   const Vec3& d, const Vec3& p) {
  ProjectResult res;

  static const size_t nexti[] = {1, 2, 0};
  const Vec3* vt[] = {&a, &b, &c, &d};
  const Vec3 dl[3] = {a - d, b - d, c - d};
  Scalar vl = triple(dl[0], dl[1], dl[2]);
  bool ng = (vl * (a - p).dot((b - c).cross(a - b))) <= 0;
  if (ng &&
      std::abs(vl) > 0)  // abs(vl) == 0, the tetrahedron is degenerated; if ng
                         // is false, then the last vertex in the tetrahedron
                         // does not grow toward the origin (in fact origin is
                         // on the other side of the abc face)
  {
    Scalar mindist = -1;

    for (size_t i = 0; i < 3; ++i) {
      size_t j = nexti[i];
      Scalar s = vl * (d - p).dot(dl[i].cross(dl[j]));
      if (s > 0)  // the origin is to the outside part of a triangle face, then
                  // the optimal can only be on the triangle face
      {
        ProjectResult res_triangle = projectTriangle(*vt[i], *vt[j], d, p);
        if (mindist < 0 || res_triangle.sqr_distance < mindist) {
          mindist = res_triangle.sqr_distance;
          res.encode =
              static_cast<unsigned int>((res_triangle.encode & 1 ? 1 << i : 0) +
                                        (res_triangle.encode & 2 ? 1 << j : 0) +
                                        (res_triangle.encode & 4 ? 8 : 0));
          res.parameterization[i] = res_triangle.parameterization[0];
          res.parameterization[j] = res_triangle.parameterization[1];
          res.parameterization[nexti[j]] = 0;
          res.parameterization[3] = res_triangle.parameterization[2];
        }
      }
    }

    if (mindist < 0) {
      mindist = 0;
      res.encode = 15;
      res.parameterization[0] = triple(c - p, b - p, d - p) / vl;
      res.parameterization[1] = triple(a - p, c - p, d - p) / vl;
      res.parameterization[2] = triple(b - p, a - p, d - p) / vl;
      res.parameterization[3] =
          1 - (res.parameterization[0] + res.parameterization[1] +
               res.parameterization[2]);
    }

    res.sqr_distance = mindist;
  } else if (!ng) {
    res = projectTriangle(a, b, c, p);
    res.parameterization[3] = 0;
  }
  return res;
}

template <typename Scalar>
inline typename Project<Scalar>::ProjectResult
Project<Scalar>::projectLineOrigin(const Vec3& a, const Vec3& b) {
  ProjectResult res;

  const Vec3 d = b - a;
  const Scalar l = d.squaredNorm();

  if (l > 0) {
    const Scalar t = -a.dot(d);
    res.parameterization[1] = (t >= l) ? 1 : ((t <= 0) ? 0 : (t / l));
    res.parameterization[0] = 1 - res.parameterization[1];
    if (t >= l) {
      res.sqr_distance = b.squaredNorm();
      res.encode = 2; /* 0x10 */
    } else if (t <= 0) {
      res.sqr_distance = a.squaredNorm();
      res.encode = 1; /* 0x01 */
    } else {
      res.sqr_distance = (a + d * res.parameterization[1]).squaredNorm();
      res.encode = 3; /* 0x00 */
    }
  }

  return res;
}

template <typename Scalar>
inline typename Project<Scalar>::ProjectResult
Project<Scalar>::projectTriangleOrigin(const Vec3& a, const Vec3& b,
                                       const Vec3& c) {
  ProjectResult res;

  static const size_t nexti[3] = {1, 2, 0};
  const Vec3* vt[] = {&a, &b, &c};
  const Vec3 dl[] = {a - b, b - c, c - a};
  const Vec3& n = dl[0].cross(dl[1]);
  const Scalar l = n.squaredNorm();

  if (l > 0) {
    Scalar mindist = -1;
    for (size_t i = 0; i < 3; ++i) {
      if (vt[i]->dot(dl[i].cross(n)) >
          0)  // origin is to the outside part of the triangle edge, then the
              // optimal can only be on the edge
      {
        size_t j = nexti[i];
        ProjectResult res_line = projectLineOrigin(*vt[i], *vt[j]);

        if (mindist < 0 || res_line.sqr_distance < mindist) {
          mindist = res_line.sqr_distance;
          res.encode =
              static_cast<unsigned int>(((res_line.encode & 1) ? 1 << i : 0) +
                                        ((res_line.encode & 2) ? 1 << j : 0));
          res.parameterization[i] = res_line.parameterization[0];
          res.parameterization[j] = res_line.parameterization[1];
          res.parameterization[nexti[j]] = 0;
        }
      }
    }

    if (mindist < 0)  // the origin project is within the triangle
    {
      Scalar d = a.dot(n);
      Scalar s = std::sqrt(l);
      Vec3 o_to_project = n * (d / l);
      mindist = o_to_project.squaredNorm();
      res.encode = 7;  // m = 0x111
      res.parameterization[0] = dl[1].cross(b - o_to_project).norm() / s;
      res.parameterization[1] = dl[2].cross(c - o_to_project).norm() / s;
      res.parameterization[2] =
          1 - res.parameterization[0] - res.parameterization[1];
    }

    res.sqr_distance = mindist;
  }

  return res;
}

template <typename Scalar>
inline typename Project<Scalar>::ProjectResult
Project<Scalar>::projectTetrahedraOrigin(const Vec3& a, const Vec3& b,
                                         const Vec3& c, const Vec3& d) {
  ProjectResult res;

  static const size_t nexti[] = {1, 2, 0};
  const Vec3* vt[] = {&a, &b, &c, &d};
  const Vec3 dl[3] = {a - d, b - d, c - d};
  Scalar vl = triple(dl[0], dl[1], dl[2]);
  bool ng = (vl * a.dot((b - c).cross(a - b))) <= 0;
  if (ng &&
      std::abs(vl) > 0)  // abs(vl) == 0, the tetrahedron is degenerated; if ng
                         // is false, then the last vertex in the tetrahedron
                         // does not grow toward the origin (in fact origin is
                         // on the other side of the abc face)
  {
    Scalar mindist = -1;

    for (size_t i = 0; i < 3; ++i) {
      size_t j = nexti[i];
      Scalar s = vl * d.dot(dl[i].cross(dl[j]));
      if (s > 0)  // the origin is to the outside part of a triangle face, then
                  // the optimal can only be on the triangle face
      {
        ProjectResult res_triangle = projectTriangleOrigin(*vt[i], *vt[j], d);
        if (mindist < 0 || res_triangle.sqr_distance < mindist) {
          mindist = res_triangle.sqr_distance;
          res.encode =
              static_cast<unsigned int>((res_triangle.encode & 1 ? 1 << i : 0) +
                                        (res_triangle.encode & 2 ? 1 << j : 0) +
                                        (res_triangle.encode & 4 ? 8 : 0));
          res.parameterization[i] = res_triangle.parameterization[0];
          res.parameterization[j] = res_triangle.parameterization[1];
          res.parameterization[nexti[j]] = 0;
          res.parameterization[3] = res_triangle.parameterization[2];
        }
      }
    }

    if (mindist < 0) {
      mindist = 0;
      res.encode = 15;
      res.parameterization[0] = triple(c, b, d) / vl;
      res.parameterization[1] = triple(a, c, d) / vl;
      res.parameterization[2] = triple(b, a, d) / vl;
      res.parameterization[3] =
          1 - (res.parameterization[0] + res.parameterization[1] +
               res.parameterization[2]);
    }

    res.sqr_distance = mindist;
  } else if (!ng) {
    res = projectTriangleOrigin(a, b, c);
    res.parameterization[3] = 0;
  }
  return res;
}

}  // namespace coal

#endif  // COAL_INTERSECT_HXX
