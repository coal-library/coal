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

namespace internal {

/// @brief SAT separating-axis helper.
/// Returns true iff projections of {p1,p2,p3} and {q1,q2,q3} onto @p ax
/// overlap (i.e., no separating gap exists on this axis).
template <typename Vec3>
inline bool project6(const Vec3& ax, const Vec3& p1, const Vec3& p2,
                     const Vec3& p3, const Vec3& q1, const Vec3& q2,
                     const Vec3& q3) {
  using Scalar = typename Vec3::Scalar;
  const Scalar P1 = ax.dot(p1), P2 = ax.dot(p2), P3 = ax.dot(p3);
  const Scalar Q1 = ax.dot(q1), Q2 = ax.dot(q2), Q3 = ax.dot(q3);
  const Scalar mn1 = std::min(P1, std::min(P2, P3));
  const Scalar mx2 = std::max(Q1, std::max(Q2, Q3));
  if (mn1 > mx2) return false;
  const Scalar mx1 = std::max(P1, std::max(P2, P3));
  const Scalar mn2 = std::min(Q1, std::min(Q2, Q3));
  return mn2 <= mx1;
}

inline bool triangleTriangleOverlap(const Vec3s& P1, const Vec3s& P2,
                                    const Vec3s& P3, const Vec3s& Q1,
                                    const Vec3s& Q2, const Vec3s& Q3,
                                    Scalar& sqrDistLowerBound) {
  /// SAT-based triangle-triangle overlap test (boolean only).
  ///
  /// Adapted from FCL's Intersect<S>::intersect_Triangle
  /// (Copyright Willow Garage / Open Source Robotics Foundation, BSD-2).
  ///
  /// Uses 17 separating axes: 2 face normals, 9 edge×edge cross-products, and
  /// 3+3 edge×face-normal axes. The SAT correctly handles all configurations
  /// including coplanar triangles — no special-case fallback is needed.
  ///
  /// Both triangles must be expressed in the same coordinate frame.
  /// Returns true iff P1P2P3 and Q1Q2Q3 share at least one point.
  /// On separation, sqrDistLowerBound is set to gap²/‖ax‖² for the first
  /// separating axis found (a valid but potentially loose lower bound).

  sqrDistLowerBound = 0;

  // Translate so that P1 is at the origin; p1 becomes the zero vector.
  const Vec3s p1 = Vec3s::Zero();
  const Vec3s p2 = P2 - P1;
  const Vec3s p3 = P3 - P1;
  const Vec3s q1 = Q1 - P1;
  const Vec3s q2 = Q2 - P1;
  const Vec3s q3 = Q3 - P1;

  const Vec3s f1 = q2 - q1;
  const Vec3s f2 = q3 - q2;
  const Vec3s f3 = q1 - q3;

  // On the first separating axis found, capture gap²/‖ax‖² as a lower bound
  // on the squared distance. For a unit axis u = ax/‖ax‖, the 1D separation
  // gap_u = gap/‖ax‖ satisfies dist_3D ≥ gap_u, so gap_u² is valid.
  auto checkAxis = [&](const Vec3s& ax) -> bool {
    const Scalar P1p = ax.dot(p1), P2p = ax.dot(p2), P3p = ax.dot(p3);
    const Scalar Q1p = ax.dot(q1), Q2p = ax.dot(q2), Q3p = ax.dot(q3);
    const Scalar mn1 = std::min(P1p, std::min(P2p, P3p));
    const Scalar mx2 = std::max(Q1p, std::max(Q2p, Q3p));
    if (mn1 > mx2) {
      const Scalar gap = mn1 - mx2;
      sqrDistLowerBound = gap * gap / ax.squaredNorm();
      return false;
    }
    const Scalar mx1 = std::max(P1p, std::max(P2p, P3p));
    const Scalar mn2 = std::min(Q1p, std::min(Q2p, Q3p));
    if (mn2 > mx1) {
      const Scalar gap = mn2 - mx1;
      sqrDistLowerBound = gap * gap / ax.squaredNorm();
      return false;
    }
    return true;
  };

  const Vec3s e1 = p2;  // = p2 - p1  (p1 = 0)
  if (!checkAxis(e1.cross(f1))) return false;
  if (!checkAxis(e1.cross(f2))) return false;
  if (!checkAxis(e1.cross(f3))) return false;

  const Vec3s e2 = p3 - p2;
  if (!checkAxis(e2.cross(f1))) return false;
  if (!checkAxis(e2.cross(f2))) return false;
  if (!checkAxis(e2.cross(f3))) return false;

  const Vec3s e3 = -p3;  // = p1 - p3  (p1 = 0)
  if (!checkAxis(e3.cross(f1))) return false;
  if (!checkAxis(e3.cross(f2))) return false;
  if (!checkAxis(e3.cross(f3))) return false;

  const Vec3s n1 = e1.cross(e2);
  if (!checkAxis(n1)) return false;

  if (!checkAxis(e1.cross(n1))) return false;
  if (!checkAxis(e2.cross(n1))) return false;
  if (!checkAxis(e3.cross(n1))) return false;

  const Vec3s m1 = f1.cross(f2);
  if (!checkAxis(m1)) return false;

  if (!checkAxis(f1.cross(m1))) return false;
  if (!checkAxis(f2.cross(m1))) return false;
  if (!checkAxis(f3.cross(m1))) return false;

  return true;
}

inline bool segmentTriangleIntersection(const Vec3s& A, const Vec3s& B,
                                        const Vec3s& P, const Vec3s& Q,
                                        const Vec3s& R, Vec3s& X) {
  using Scalar = typename Vec3s::Scalar;

  const Vec3s AB = B - A;
  const Vec3s N = (Q - P).cross(R - P);  // unnormalized triangle normal

  const Scalar denom = N.dot(AB);
  // Segment parallel (or nearly so) to the triangle's plane.
  if (std::abs(denom) <= std::numeric_limits<Scalar>::epsilon() *
                             (std::max)(N.norm() * AB.norm(), Scalar(1)))
    return false;

  const Scalar t = N.dot(P - A) / denom;
  if (t < 0 || t > 1) return false;

  X = A + t * AB;

  // Inside-triangle test: X must be on the interior side of every directed
  // edge (cross product with N must be non-negative).
  if (N.dot((Q - P).cross(X - P)) < 0) return false;
  if (N.dot((R - Q).cross(X - Q)) < 0) return false;
  if (N.dot((P - R).cross(X - R)) < 0) return false;

  return true;
}

inline void computeTriangleTriangleContact(const Vec3s& P1, const Vec3s& P2,
                                           const Vec3s& P3, const Vec3s& Q1,
                                           const Vec3s& Q2, const Vec3s& Q3,
                                           Scalar& signed_distance,
                                           Vec3s& normal, Vec3s& p1,
                                           Vec3s& p2) {
  // Normal from triangle 1's plane; penetration depth is an upper bound.
  // Note: we make the assumption that the triangles are properly
  // oriented, so the normal points outside of triangle 1 (this is
  // coherent with the normal definiction in `Contact`, see
  // `collision-data.h`).
  normal = (P2 - P1).cross(P3 - P1).normalized();
  Scalar depth1((P1 - Q1).dot(normal));
  Scalar depth2((P1 - Q2).dot(normal));
  Scalar depth3((P1 - Q3).dot(normal));
  signed_distance = -std::max(depth1, std::max(depth2, depth3));

  // Find a point in the intersection via edge-triangle sweep.
  // For non-coplanar intersecting triangles the intersection is a
  // segment; we collect all edge-piercing points and average them.
  // For coplanar triangles (fast-path fallthrough) no edge pierces the
  // other triangle's plane, so we fall back to the vertex average.
  Vec3s P_sum = Vec3s::Zero();
  int num_hits = 0;
  auto testEdge = [&](const Vec3s& E0, const Vec3s& E1, const Vec3s& T0,
                      const Vec3s& T1, const Vec3s& T2) {
    Vec3s X;
    if (segmentTriangleIntersection(E0, E1, T0, T1, T2, X)) {
      P_sum += X;
      ++num_hits;
    }
  };
  testEdge(P1, P2, Q1, Q2, Q3);
  testEdge(P2, P3, Q1, Q2, Q3);
  testEdge(P3, P1, Q1, Q2, Q3);
  testEdge(Q1, Q2, P1, P2, P3);
  testEdge(Q2, Q3, P1, P2, P3);
  testEdge(Q3, Q1, P1, P2, P3);
  if (num_hits > 0)
    p1 = p2 = P_sum / num_hits;
  else
    p1 = p2 = (P1 + P2 + P3 + Q1 + Q2 + Q3) / 6;
}
}  // namespace internal

}  // namespace coal

#endif  // COAL_INTERSECT_HXX
