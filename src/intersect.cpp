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

#include "coal/internal/intersect.h"
#include "coal/internal/tools.h"

#include <iostream>
#include <limits>
#include <vector>
#include <cmath>

namespace coal {

bool Intersect::buildTrianglePlane(const Vec3s& v1, const Vec3s& v2,
                                   const Vec3s& v3, Vec3s* n, Scalar* t) {
  Vec3s n_ = (v2 - v1).cross(v3 - v1);
  Scalar norm2 = n_.squaredNorm();
  if (norm2 > 0) {
    *n = n_ / sqrt(norm2);
    *t = n->dot(v1);
    return true;
  }
  return false;
}

void TriangleDistance::segPoints(const Vec3s& P, const Vec3s& A, const Vec3s& Q,
                                 const Vec3s& B, Vec3s& VEC, Vec3s& X,
                                 Vec3s& Y) {
  Vec3s T;
  Scalar A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
  Vec3s TMP;

  T = Q - P;
  A_dot_A = A.dot(A);
  B_dot_B = B.dot(B);
  A_dot_B = A.dot(B);
  A_dot_T = A.dot(T);
  B_dot_T = B.dot(T);

  // t parameterizes ray P,A
  // u parameterizes ray Q,B

  Scalar t, u;

  // compute t for the closest point on ray P,A to
  // ray Q,B

  Scalar denom = A_dot_A * B_dot_B - A_dot_B * A_dot_B;

  t = (A_dot_T * B_dot_B - B_dot_T * A_dot_B) / denom;

  // clamp result so t is on the segment P,A

  if ((t < 0) || std::isnan(t))
    t = 0;
  else if (t > 1)
    t = 1;

  // find u for point on ray Q,B closest to point at t

  u = (t * A_dot_B - B_dot_T) / B_dot_B;

  // if u is on segment Q,B, t and u correspond to
  // closest points, otherwise, clamp u, recompute and
  // clamp t

  if ((u <= 0) || std::isnan(u)) {
    Y = Q;

    t = A_dot_T / A_dot_A;

    if ((t <= 0) || std::isnan(t)) {
      X = P;
      VEC = Q - P;
    } else if (t >= 1) {
      X = P + A;
      VEC = Q - X;
    } else {
      X = P + A * t;
      TMP = T.cross(A);
      VEC = A.cross(TMP);
    }
  } else if (u >= 1) {
    Y = Q + B;

    t = (A_dot_B + A_dot_T) / A_dot_A;

    if ((t <= 0) || std::isnan(t)) {
      X = P;
      VEC = Y - P;
    } else if (t >= 1) {
      X = P + A;
      VEC = Y - X;
    } else {
      X = P + A * t;
      T = Y - P;
      TMP = T.cross(A);
      VEC = A.cross(TMP);
    }
  } else {
    Y = Q + B * u;

    if ((t <= 0) || std::isnan(t)) {
      X = P;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    } else if (t >= 1) {
      X = P + A;
      T = Q - X;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    } else {
      X = P + A * t;
      VEC = A.cross(B);
      if (VEC.dot(T) < 0) {
        VEC = VEC * (-1);
      }
    }
  }
}

Scalar TriangleDistance::sqrTriDistance(const Vec3s S[3], const Vec3s T[3],
                                        Vec3s& P, Vec3s& Q) {
  // Compute vectors along the 6 sides

  Vec3s Sv[3];
  Vec3s Tv[3];
  Vec3s VEC;

  Sv[0] = S[1] - S[0];
  Sv[1] = S[2] - S[1];
  Sv[2] = S[0] - S[2];

  Tv[0] = T[1] - T[0];
  Tv[1] = T[2] - T[1];
  Tv[2] = T[0] - T[2];

  // For each edge pair, the vector connecting the closest points
  // of the edges defines a slab (parallel planes at head and tail
  // enclose the slab). If we can show that the off-edge vertex of
  // each triangle is outside of the slab, then the closest points
  // of the edges are the closest points for the triangles.
  // Even if these tests fail, it may be helpful to know the closest
  // points found, and whether the triangles were shown disjoint

  Vec3s V, Z, minP, minQ;
  Scalar mindd;
  int shown_disjoint = 0;

  mindd = (S[0] - T[0]).squaredNorm() + 1;  // Set first minimum safely high

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      // Find closest points on edges i & j, plus the
      // vector (and distance squared) between these points
      segPoints(S[i], Sv[i], T[j], Tv[j], VEC, P, Q);

      V = Q - P;
      Scalar dd = V.dot(V);

      // Verify this closest point pair only if the distance
      // squared is less than the minimum found thus far.

      if (dd <= mindd) {
        minP = P;
        minQ = Q;
        mindd = dd;

        Z = S[(i + 2) % 3] - P;
        Scalar a = Z.dot(VEC);
        Z = T[(j + 2) % 3] - Q;
        Scalar b = Z.dot(VEC);

        if ((a <= 0) && (b >= 0)) return dd;

        Scalar p = V.dot(VEC);

        if (a < 0) a = 0;
        if (b > 0) b = 0;
        if ((p - a + b) > 0) shown_disjoint = 1;
      }
    }
  }

  // No edge pairs contained the closest points.
  // either:
  // 1. one of the closest points is a vertex, and the
  //    other point is interior to a face.
  // 2. the triangles are overlapping.
  // 3. an edge of one triangle is parallel to the other's face. If
  //    cases 1 and 2 are not true, then the closest points from the 9
  //    edge pairs checks above can be taken as closest points for the
  //    triangles.
  // 4. possibly, the triangles were degenerate.  When the
  //    triangle points are nearly colinear or coincident, one
  //    of above tests might fail even though the edges tested
  //    contain the closest points.

  // First check for case 1

  Vec3s Sn;
  Scalar Snl;

  Sn = Sv[0].cross(Sv[1]);  // Compute normal to S triangle
  Snl = Sn.dot(Sn);         // Compute square of length of normal

  // If cross product is long enough,

  if (Snl > 1e-15) {
    // Get projection lengths of T points

    Vec3s Tp;

    V = S[0] - T[0];
    Tp[0] = V.dot(Sn);

    V = S[0] - T[1];
    Tp[1] = V.dot(Sn);

    V = S[0] - T[2];
    Tp[2] = V.dot(Sn);

    // If Sn is a separating direction,
    // find point with smallest projection

    int point = -1;
    if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0)) {
      if (Tp[0] < Tp[1])
        point = 0;
      else
        point = 1;
      if (Tp[2] < Tp[point]) point = 2;
    } else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0)) {
      if (Tp[0] > Tp[1])
        point = 0;
      else
        point = 1;
      if (Tp[2] > Tp[point]) point = 2;
    }

    // If Sn is a separating direction,

    if (point >= 0) {
      shown_disjoint = 1;

      // Test whether the point found, when projected onto the
      // other triangle, lies within the face.

      V = T[point] - S[0];
      Z = Sn.cross(Sv[0]);
      if (V.dot(Z) > 0) {
        V = T[point] - S[1];
        Z = Sn.cross(Sv[1]);
        if (V.dot(Z) > 0) {
          V = T[point] - S[2];
          Z = Sn.cross(Sv[2]);
          if (V.dot(Z) > 0) {
            // T[point] passed the test - it's a closest point for
            // the T triangle; the other point is on the face of S
            P = T[point] + Sn * (Tp[point] / Snl);
            Q = T[point];
            return (P - Q).squaredNorm();
          }
        }
      }
    }
  }

  Vec3s Tn;
  Scalar Tnl;

  Tn = Tv[0].cross(Tv[1]);
  Tnl = Tn.dot(Tn);

  if (Tnl > 1e-15) {
    Vec3s Sp;

    V = T[0] - S[0];
    Sp[0] = V.dot(Tn);

    V = T[0] - S[1];
    Sp[1] = V.dot(Tn);

    V = T[0] - S[2];
    Sp[2] = V.dot(Tn);

    int point = -1;
    if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0)) {
      if (Sp[0] < Sp[1])
        point = 0;
      else
        point = 1;
      if (Sp[2] < Sp[point]) point = 2;
    } else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0)) {
      if (Sp[0] > Sp[1])
        point = 0;
      else
        point = 1;
      if (Sp[2] > Sp[point]) point = 2;
    }

    if (point >= 0) {
      shown_disjoint = 1;

      V = S[point] - T[0];
      Z = Tn.cross(Tv[0]);
      if (V.dot(Z) > 0) {
        V = S[point] - T[1];
        Z = Tn.cross(Tv[1]);
        if (V.dot(Z) > 0) {
          V = S[point] - T[2];
          Z = Tn.cross(Tv[2]);
          if (V.dot(Z) > 0) {
            P = S[point];
            Q = S[point] + Tn * (Sp[point] / Tnl);
            return (P - Q).squaredNorm();
          }
        }
      }
    }
  }

  // Case 1 can't be shown.
  // If one of these tests showed the triangles disjoint,
  // we assume case 3 or 4, otherwise we conclude case 2,
  // that the triangles overlap.

  if (shown_disjoint) {
    P = minP;
    Q = minQ;
    return mindd;
  } else
    return 0;
}

Scalar TriangleDistance::sqrTriDistance(const Vec3s& S1, const Vec3s& S2,
                                        const Vec3s& S3, const Vec3s& T1,
                                        const Vec3s& T2, const Vec3s& T3,
                                        Vec3s& P, Vec3s& Q) {
  Vec3s S[3];
  Vec3s T[3];
  S[0] = S1;
  S[1] = S2;
  S[2] = S3;
  T[0] = T1;
  T[1] = T2;
  T[2] = T3;

  return sqrTriDistance(S, T, P, Q);
}

Scalar TriangleDistance::sqrTriDistance(const Vec3s S[3], const Vec3s T[3],
                                        const Matrix3s& R, const Vec3s& Tl,
                                        Vec3s& P, Vec3s& Q) {
  Vec3s T_transformed[3];
  T_transformed[0] = R * T[0] + Tl;
  T_transformed[1] = R * T[1] + Tl;
  T_transformed[2] = R * T[2] + Tl;

  return sqrTriDistance(S, T_transformed, P, Q);
}

Scalar TriangleDistance::sqrTriDistance(const Vec3s S[3], const Vec3s T[3],
                                        const Transform3s& tf, Vec3s& P,
                                        Vec3s& Q) {
  Vec3s T_transformed[3];
  T_transformed[0] = tf.transform(T[0]);
  T_transformed[1] = tf.transform(T[1]);
  T_transformed[2] = tf.transform(T[2]);

  return sqrTriDistance(S, T_transformed, P, Q);
}

Scalar TriangleDistance::sqrTriDistance(const Vec3s& S1, const Vec3s& S2,
                                        const Vec3s& S3, const Vec3s& T1,
                                        const Vec3s& T2, const Vec3s& T3,
                                        const Matrix3s& R, const Vec3s& Tl,
                                        Vec3s& P, Vec3s& Q) {
  Vec3s T1_transformed = R * T1 + Tl;
  Vec3s T2_transformed = R * T2 + Tl;
  Vec3s T3_transformed = R * T3 + Tl;
  return sqrTriDistance(S1, S2, S3, T1_transformed, T2_transformed,
                        T3_transformed, P, Q);
}

Scalar TriangleDistance::sqrTriDistance(const Vec3s& S1, const Vec3s& S2,
                                        const Vec3s& S3, const Vec3s& T1,
                                        const Vec3s& T2, const Vec3s& T3,
                                        const Transform3s& tf, Vec3s& P,
                                        Vec3s& Q) {
  Vec3s T1_transformed = tf.transform(T1);
  Vec3s T2_transformed = tf.transform(T2);
  Vec3s T3_transformed = tf.transform(T3);
  return sqrTriDistance(S1, S2, S3, T1_transformed, T2_transformed,
                        T3_transformed, P, Q);
}

}  // namespace coal
