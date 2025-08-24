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

#include "coal/BV/RSS.h"
#include "coal/BVH/BVH_utility.h"
#include "coal/internal/tools.h"
#include "coal/collision_data.h"

#include <iostream>

namespace coal {

/// @brief Clip value between a and b
void clipToRange(Scalar& val, Scalar a, Scalar b) {
  if (val < a)
    val = a;
  else if (val > b)
    val = b;
}

/// @brief Finds the parameters t & u corresponding to the two closest points on
/// a pair of line segments. The first segment is defined as Pa + A*t, 0 <= t <=
/// a,  where "Pa" is one endpoint of the segment, "A" is a unit vector pointing
/// to the other endpoint, and t is a scalar that produces all the points
/// between the two endpoints. Since "A" is a unit vector, "a" is the segment's
/// length. The second segment is defined as Pb + B*u, 0 <= u <= b. Many of the
/// terms needed by the algorithm are already computed for other purposes,so we
/// just pass these terms into the function instead of complete specifications
/// of each segment. "T" in the dot products is the vector betweeen Pa and Pb.
/// Reference: "On fast computation of distance between line segments." Vladimir
/// J. Lumelsky, in Information Processing Letters, no. 21, pages 55-61, 1985.
void segCoords(Scalar& t, Scalar& u, Scalar a, Scalar b, Scalar A_dot_B,
               Scalar A_dot_T, Scalar B_dot_T) {
  Scalar denom = 1 - A_dot_B * A_dot_B;

  if (denom == 0)
    t = 0;
  else {
    t = (A_dot_T - B_dot_T * A_dot_B) / denom;
    clipToRange(t, 0, a);
  }

  u = t * A_dot_B - B_dot_T;
  if (u < 0) {
    u = 0;
    t = A_dot_T;
    clipToRange(t, 0, a);
  } else if (u > b) {
    u = b;
    t = u * A_dot_B + A_dot_T;
    clipToRange(t, 0, a);
  }
}

/// @brief Returns whether the nearest point on rectangle edge
/// Pb + B*u, 0 <= u <= b, to the rectangle edge,
/// Pa + A*t, 0 <= t <= a, is within the half space
/// determined by the point Pa and the direction Anorm.
/// A,B, and Anorm are unit vectors. T is the vector between Pa and Pb.
bool inVoronoi(Scalar a, Scalar b, Scalar Anorm_dot_B, Scalar Anorm_dot_T,
               Scalar A_dot_B, Scalar A_dot_T, Scalar B_dot_T) {
  if (fabs(Anorm_dot_B) < 1e-7) return false;

  Scalar t, u, v;

  u = -Anorm_dot_T / Anorm_dot_B;
  clipToRange(u, 0, b);

  t = u * A_dot_B + A_dot_T;
  clipToRange(t, 0, a);

  v = t * A_dot_B - B_dot_T;

  if (Anorm_dot_B > 0) {
    if (v > (u + 1e-7)) return true;
  } else {
    if (v < (u - 1e-7)) return true;
  }
  return false;
}

/// @brief Distance between two oriented rectangles; P and Q (optional return
/// values) are the closest points in the rectangles, both are in the local
/// frame of the first rectangle.
Scalar rectDistance(const Matrix3s& Rab, Vec3s const& Tab, const Scalar a[2],
                    const Scalar b[2], Vec3s* P = NULL, Vec3s* Q = NULL) {
  Scalar A0_dot_B0, A0_dot_B1, A1_dot_B0, A1_dot_B1;

  A0_dot_B0 = Rab(0, 0);
  A0_dot_B1 = Rab(0, 1);
  A1_dot_B0 = Rab(1, 0);
  A1_dot_B1 = Rab(1, 1);

  Scalar aA0_dot_B0, aA0_dot_B1, aA1_dot_B0, aA1_dot_B1;
  Scalar bA0_dot_B0, bA0_dot_B1, bA1_dot_B0, bA1_dot_B1;

  aA0_dot_B0 = a[0] * A0_dot_B0;
  aA0_dot_B1 = a[0] * A0_dot_B1;
  aA1_dot_B0 = a[1] * A1_dot_B0;
  aA1_dot_B1 = a[1] * A1_dot_B1;
  bA0_dot_B0 = b[0] * A0_dot_B0;
  bA1_dot_B0 = b[0] * A1_dot_B0;
  bA0_dot_B1 = b[1] * A0_dot_B1;
  bA1_dot_B1 = b[1] * A1_dot_B1;

  Vec3s Tba(Rab.transpose() * Tab);

  Vec3s S;
  Scalar t, u;

  // determine if any edge pair contains the closest points

  Scalar ALL_x, ALU_x, AUL_x, AUU_x;
  Scalar BLL_x, BLU_x, BUL_x, BUU_x;
  Scalar LA1_lx, LA1_ux, UA1_lx, UA1_ux, LB1_lx, LB1_ux, UB1_lx, UB1_ux;

  ALL_x = -Tba[0];
  ALU_x = ALL_x + aA1_dot_B0;
  AUL_x = ALL_x + aA0_dot_B0;
  AUU_x = ALU_x + aA0_dot_B0;

  if (ALL_x < ALU_x) {
    LA1_lx = ALL_x;
    LA1_ux = ALU_x;
    UA1_lx = AUL_x;
    UA1_ux = AUU_x;
  } else {
    LA1_lx = ALU_x;
    LA1_ux = ALL_x;
    UA1_lx = AUU_x;
    UA1_ux = AUL_x;
  }

  BLL_x = Tab[0];
  BLU_x = BLL_x + bA0_dot_B1;
  BUL_x = BLL_x + bA0_dot_B0;
  BUU_x = BLU_x + bA0_dot_B0;

  if (BLL_x < BLU_x) {
    LB1_lx = BLL_x;
    LB1_ux = BLU_x;
    UB1_lx = BUL_x;
    UB1_ux = BUU_x;
  } else {
    LB1_lx = BLU_x;
    LB1_ux = BLL_x;
    UB1_lx = BUU_x;
    UB1_ux = BUL_x;
  }

  // UA1, UB1

  if ((UA1_ux > b[0]) && (UB1_ux > a[0])) {
    if (((UA1_lx > b[0]) ||
         inVoronoi(b[1], a[1], A1_dot_B0, aA0_dot_B0 - b[0] - Tba[0], A1_dot_B1,
                   aA0_dot_B1 - Tba[1], -Tab[1] - bA1_dot_B0)) &&
        ((UB1_lx > a[0]) ||
         inVoronoi(a[1], b[1], A0_dot_B1, Tab[0] + bA0_dot_B0 - a[0], A1_dot_B1,
                   Tab[1] + bA1_dot_B0, Tba[1] - aA0_dot_B1))) {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1] + bA1_dot_B0,
                Tba[1] - aA0_dot_B1);

      S[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u - a[0];
      S[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u - t;
      S[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if (P && Q) {
        *P << a[0], t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // UA1, LB1

  if ((UA1_lx < 0) && (LB1_ux > a[0])) {
    if (((UA1_ux < 0) || inVoronoi(b[1], a[1], -A1_dot_B0, Tba[0] - aA0_dot_B0,
                                   A1_dot_B1, aA0_dot_B1 - Tba[1], -Tab[1])) &&
        ((LB1_lx > a[0]) ||
         inVoronoi(a[1], b[1], A0_dot_B1, Tab[0] - a[0], A1_dot_B1, Tab[1],
                   Tba[1] - aA0_dot_B1))) {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1], Tba[1] - aA0_dot_B1);

      S[0] = Tab[0] + Rab(0, 1) * u - a[0];
      S[1] = Tab[1] + Rab(1, 1) * u - t;
      S[2] = Tab[2] + Rab(2, 1) * u;

      if (P && Q) {
        *P << a[0], t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA1, UB1

  if ((LA1_ux > b[0]) && (UB1_lx < 0)) {
    if (((LA1_lx > b[0]) ||
         inVoronoi(b[1], a[1], A1_dot_B0, -Tba[0] - b[0], A1_dot_B1, -Tba[1],
                   -Tab[1] - bA1_dot_B0)) &&
        ((UB1_ux < 0) || inVoronoi(a[1], b[1], -A0_dot_B1, -Tab[0] - bA0_dot_B0,
                                   A1_dot_B1, Tab[1] + bA1_dot_B0, Tba[1]))) {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1] + bA1_dot_B0, Tba[1]);

      S[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u;
      S[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u - t;
      S[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if (P && Q) {
        *P << 0, t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA1, LB1

  if ((LA1_lx < 0) && (LB1_lx < 0)) {
    if (((LA1_ux < 0) || inVoronoi(b[1], a[1], -A1_dot_B0, Tba[0], A1_dot_B1,
                                   -Tba[1], -Tab[1])) &&
        ((LB1_ux < 0) || inVoronoi(a[1], b[1], -A0_dot_B1, -Tab[0], A1_dot_B1,
                                   Tab[1], Tba[1]))) {
      segCoords(t, u, a[1], b[1], A1_dot_B1, Tab[1], Tba[1]);

      S[0] = Tab[0] + Rab(0, 1) * u;
      S[1] = Tab[1] + Rab(1, 1) * u - t;
      S[2] = Tab[2] + Rab(2, 1) * u;

      if (P && Q) {
        *P << 0, t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  Scalar ALL_y, ALU_y, AUL_y, AUU_y;

  ALL_y = -Tba[1];
  ALU_y = ALL_y + aA1_dot_B1;
  AUL_y = ALL_y + aA0_dot_B1;
  AUU_y = ALU_y + aA0_dot_B1;

  Scalar LA1_ly, LA1_uy, UA1_ly, UA1_uy, LB0_lx, LB0_ux, UB0_lx, UB0_ux;

  if (ALL_y < ALU_y) {
    LA1_ly = ALL_y;
    LA1_uy = ALU_y;
    UA1_ly = AUL_y;
    UA1_uy = AUU_y;
  } else {
    LA1_ly = ALU_y;
    LA1_uy = ALL_y;
    UA1_ly = AUU_y;
    UA1_uy = AUL_y;
  }

  if (BLL_x < BUL_x) {
    LB0_lx = BLL_x;
    LB0_ux = BUL_x;
    UB0_lx = BLU_x;
    UB0_ux = BUU_x;
  } else {
    LB0_lx = BUL_x;
    LB0_ux = BLL_x;
    UB0_lx = BUU_x;
    UB0_ux = BLU_x;
  }

  // UA1, UB0

  if ((UA1_uy > b[1]) && (UB0_ux > a[0])) {
    if (((UA1_ly > b[1]) ||
         inVoronoi(b[0], a[1], A1_dot_B1, aA0_dot_B1 - Tba[1] - b[1], A1_dot_B0,
                   aA0_dot_B0 - Tba[0], -Tab[1] - bA1_dot_B1)) &&
        ((UB0_lx > a[0]) ||
         inVoronoi(a[1], b[0], A0_dot_B0, Tab[0] - a[0] + bA0_dot_B1, A1_dot_B0,
                   Tab[1] + bA1_dot_B1, Tba[0] - aA0_dot_B0))) {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1] + bA1_dot_B1,
                Tba[0] - aA0_dot_B0);

      S[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u - a[0];
      S[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u - t;
      S[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if (P && Q) {
        *P << a[0], t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // UA1, LB0

  if ((UA1_ly < 0) && (LB0_ux > a[0])) {
    if (((UA1_uy < 0) || inVoronoi(b[0], a[1], -A1_dot_B1, Tba[1] - aA0_dot_B1,
                                   A1_dot_B0, aA0_dot_B0 - Tba[0], -Tab[1])) &&
        ((LB0_lx > a[0]) ||
         inVoronoi(a[1], b[0], A0_dot_B0, Tab[0] - a[0], A1_dot_B0, Tab[1],
                   Tba[0] - aA0_dot_B0))) {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1], Tba[0] - aA0_dot_B0);

      S[0] = Tab[0] + Rab(0, 0) * u - a[0];
      S[1] = Tab[1] + Rab(1, 0) * u - t;
      S[2] = Tab[2] + Rab(2, 0) * u;

      if (P && Q) {
        *P << a[0], t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA1, UB0

  if ((LA1_uy > b[1]) && (UB0_lx < 0)) {
    if (((LA1_ly > b[1]) ||
         inVoronoi(b[0], a[1], A1_dot_B1, -Tba[1] - b[1], A1_dot_B0, -Tba[0],
                   -Tab[1] - bA1_dot_B1)) &&

        ((UB0_ux < 0) || inVoronoi(a[1], b[0], -A0_dot_B0, -Tab[0] - bA0_dot_B1,
                                   A1_dot_B0, Tab[1] + bA1_dot_B1, Tba[0]))) {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1] + bA1_dot_B1, Tba[0]);

      S[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u;
      S[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u - t;
      S[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if (P && Q) {
        *P << 0, t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA1, LB0

  if ((LA1_ly < 0) && (LB0_lx < 0)) {
    if (((LA1_uy < 0) || inVoronoi(b[0], a[1], -A1_dot_B1, Tba[1], A1_dot_B0,
                                   -Tba[0], -Tab[1])) &&
        ((LB0_ux < 0) || inVoronoi(a[1], b[0], -A0_dot_B0, -Tab[0], A1_dot_B0,
                                   Tab[1], Tba[0]))) {
      segCoords(t, u, a[1], b[0], A1_dot_B0, Tab[1], Tba[0]);

      S[0] = Tab[0] + Rab(0, 0) * u;
      S[1] = Tab[1] + Rab(1, 0) * u - t;
      S[2] = Tab[2] + Rab(2, 0) * u;

      if (P && Q) {
        *P << 0, t, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  Scalar BLL_y, BLU_y, BUL_y, BUU_y;

  BLL_y = Tab[1];
  BLU_y = BLL_y + bA1_dot_B1;
  BUL_y = BLL_y + bA1_dot_B0;
  BUU_y = BLU_y + bA1_dot_B0;

  Scalar LA0_lx, LA0_ux, UA0_lx, UA0_ux, LB1_ly, LB1_uy, UB1_ly, UB1_uy;

  if (ALL_x < AUL_x) {
    LA0_lx = ALL_x;
    LA0_ux = AUL_x;
    UA0_lx = ALU_x;
    UA0_ux = AUU_x;
  } else {
    LA0_lx = AUL_x;
    LA0_ux = ALL_x;
    UA0_lx = AUU_x;
    UA0_ux = ALU_x;
  }

  if (BLL_y < BLU_y) {
    LB1_ly = BLL_y;
    LB1_uy = BLU_y;
    UB1_ly = BUL_y;
    UB1_uy = BUU_y;
  } else {
    LB1_ly = BLU_y;
    LB1_uy = BLL_y;
    UB1_ly = BUU_y;
    UB1_uy = BUL_y;
  }

  // UA0, UB1

  if ((UA0_ux > b[0]) && (UB1_uy > a[1])) {
    if (((UA0_lx > b[0]) ||
         inVoronoi(b[1], a[0], A0_dot_B0, aA1_dot_B0 - Tba[0] - b[0], A0_dot_B1,
                   aA1_dot_B1 - Tba[1], -Tab[0] - bA0_dot_B0)) &&
        ((UB1_ly > a[1]) ||
         inVoronoi(a[0], b[1], A1_dot_B1, Tab[1] - a[1] + bA1_dot_B0, A0_dot_B1,
                   Tab[0] + bA0_dot_B0, Tba[1] - aA1_dot_B1))) {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0] + bA0_dot_B0,
                Tba[1] - aA1_dot_B1);

      S[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u - t;
      S[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u - a[1];
      S[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if (P && Q) {
        *P << t, a[1], 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // UA0, LB1

  if ((UA0_lx < 0) && (LB1_uy > a[1])) {
    if (((UA0_ux < 0) || inVoronoi(b[1], a[0], -A0_dot_B0, Tba[0] - aA1_dot_B0,
                                   A0_dot_B1, aA1_dot_B1 - Tba[1], -Tab[0])) &&
        ((LB1_ly > a[1]) ||
         inVoronoi(a[0], b[1], A1_dot_B1, Tab[1] - a[1], A0_dot_B1, Tab[0],
                   Tba[1] - aA1_dot_B1))) {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0], Tba[1] - aA1_dot_B1);

      S[0] = Tab[0] + Rab(0, 1) * u - t;
      S[1] = Tab[1] + Rab(1, 1) * u - a[1];
      S[2] = Tab[2] + Rab(2, 1) * u;

      if (P && Q) {
        *P << t, a[1], 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA0, UB1

  if ((LA0_ux > b[0]) && (UB1_ly < 0)) {
    if (((LA0_lx > b[0]) ||
         inVoronoi(b[1], a[0], A0_dot_B0, -b[0] - Tba[0], A0_dot_B1, -Tba[1],
                   -bA0_dot_B0 - Tab[0])) &&
        ((UB1_uy < 0) || inVoronoi(a[0], b[1], -A1_dot_B1, -Tab[1] - bA1_dot_B0,
                                   A0_dot_B1, Tab[0] + bA0_dot_B0, Tba[1]))) {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0] + bA0_dot_B0, Tba[1]);

      S[0] = Tab[0] + Rab(0, 0) * b[0] + Rab(0, 1) * u - t;
      S[1] = Tab[1] + Rab(1, 0) * b[0] + Rab(1, 1) * u;
      S[2] = Tab[2] + Rab(2, 0) * b[0] + Rab(2, 1) * u;

      if (P && Q) {
        *P << t, 0, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA0, LB1

  if ((LA0_lx < 0) && (LB1_ly < 0)) {
    if (((LA0_ux < 0) || inVoronoi(b[1], a[0], -A0_dot_B0, Tba[0], A0_dot_B1,
                                   -Tba[1], -Tab[0])) &&
        ((LB1_uy < 0) || inVoronoi(a[0], b[1], -A1_dot_B1, -Tab[1], A0_dot_B1,
                                   Tab[0], Tba[1]))) {
      segCoords(t, u, a[0], b[1], A0_dot_B1, Tab[0], Tba[1]);

      S[0] = Tab[0] + Rab(0, 1) * u - t;
      S[1] = Tab[1] + Rab(1, 1) * u;
      S[2] = Tab[2] + Rab(2, 1) * u;

      if (P && Q) {
        *P << t, 0, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  Scalar LA0_ly, LA0_uy, UA0_ly, UA0_uy, LB0_ly, LB0_uy, UB0_ly, UB0_uy;

  if (ALL_y < AUL_y) {
    LA0_ly = ALL_y;
    LA0_uy = AUL_y;
    UA0_ly = ALU_y;
    UA0_uy = AUU_y;
  } else {
    LA0_ly = AUL_y;
    LA0_uy = ALL_y;
    UA0_ly = AUU_y;
    UA0_uy = ALU_y;
  }

  if (BLL_y < BUL_y) {
    LB0_ly = BLL_y;
    LB0_uy = BUL_y;
    UB0_ly = BLU_y;
    UB0_uy = BUU_y;
  } else {
    LB0_ly = BUL_y;
    LB0_uy = BLL_y;
    UB0_ly = BUU_y;
    UB0_uy = BLU_y;
  }

  // UA0, UB0

  if ((UA0_uy > b[1]) && (UB0_uy > a[1])) {
    if (((UA0_ly > b[1]) ||
         inVoronoi(b[0], a[0], A0_dot_B1, aA1_dot_B1 - Tba[1] - b[1], A0_dot_B0,
                   aA1_dot_B0 - Tba[0], -Tab[0] - bA0_dot_B1)) &&
        ((UB0_ly > a[1]) ||
         inVoronoi(a[0], b[0], A1_dot_B0, Tab[1] - a[1] + bA1_dot_B1, A0_dot_B0,
                   Tab[0] + bA0_dot_B1, Tba[0] - aA1_dot_B0))) {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0] + bA0_dot_B1,
                Tba[0] - aA1_dot_B0);

      S[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u - t;
      S[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u - a[1];
      S[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if (P && Q) {
        *P << t, a[1], 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // UA0, LB0

  if ((UA0_ly < 0) && (LB0_uy > a[1])) {
    if (((UA0_uy < 0) || inVoronoi(b[0], a[0], -A0_dot_B1, Tba[1] - aA1_dot_B1,
                                   A0_dot_B0, aA1_dot_B0 - Tba[0], -Tab[0])) &&
        ((LB0_ly > a[1]) ||
         inVoronoi(a[0], b[0], A1_dot_B0, Tab[1] - a[1], A0_dot_B0, Tab[0],
                   Tba[0] - aA1_dot_B0))) {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0], Tba[0] - aA1_dot_B0);

      S[0] = Tab[0] + Rab(0, 0) * u - t;
      S[1] = Tab[1] + Rab(1, 0) * u - a[1];
      S[2] = Tab[2] + Rab(2, 0) * u;

      if (P && Q) {
        *P << t, a[1], 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA0, UB0

  if ((LA0_uy > b[1]) && (UB0_ly < 0)) {
    if (((LA0_ly > b[1]) ||
         inVoronoi(b[0], a[0], A0_dot_B1, -Tba[1] - b[1], A0_dot_B0, -Tba[0],
                   -Tab[0] - bA0_dot_B1)) &&

        ((UB0_uy < 0) || inVoronoi(a[0], b[0], -A1_dot_B0, -Tab[1] - bA1_dot_B1,
                                   A0_dot_B0, Tab[0] + bA0_dot_B1, Tba[0]))) {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0] + bA0_dot_B1, Tba[0]);

      S[0] = Tab[0] + Rab(0, 1) * b[1] + Rab(0, 0) * u - t;
      S[1] = Tab[1] + Rab(1, 1) * b[1] + Rab(1, 0) * u;
      S[2] = Tab[2] + Rab(2, 1) * b[1] + Rab(2, 0) * u;

      if (P && Q) {
        *P << t, 0, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // LA0, LB0

  if ((LA0_ly < 0) && (LB0_ly < 0)) {
    if (((LA0_uy < 0) || inVoronoi(b[0], a[0], -A0_dot_B1, Tba[1], A0_dot_B0,
                                   -Tba[0], -Tab[0])) &&
        ((LB0_uy < 0) || inVoronoi(a[0], b[0], -A1_dot_B0, -Tab[1], A0_dot_B0,
                                   Tab[0], Tba[0]))) {
      segCoords(t, u, a[0], b[0], A0_dot_B0, Tab[0], Tba[0]);

      S[0] = Tab[0] + Rab(0, 0) * u - t;
      S[1] = Tab[1] + Rab(1, 0) * u;
      S[2] = Tab[2] + Rab(2, 0) * u;

      if (P && Q) {
        *P << t, 0, 0;
        *Q = S + (*P);
      }

      return S.norm();
    }
  }

  // no edges passed, take max separation along face normals

  Scalar sep1, sep2;

  if (Tab[2] > 0.0) {
    sep1 = Tab[2];
    if (Rab(2, 0) < 0.0) sep1 += b[0] * Rab(2, 0);
    if (Rab(2, 1) < 0.0) sep1 += b[1] * Rab(2, 1);
  } else {
    sep1 = -Tab[2];
    if (Rab(2, 0) > 0.0) sep1 -= b[0] * Rab(2, 0);
    if (Rab(2, 1) > 0.0) sep1 -= b[1] * Rab(2, 1);
  }

  if (Tba[2] < 0) {
    sep2 = -Tba[2];
    if (Rab(0, 2) < 0.0) sep2 += a[0] * Rab(0, 2);
    if (Rab(1, 2) < 0.0) sep2 += a[1] * Rab(1, 2);
  } else {
    sep2 = Tba[2];
    if (Rab(0, 2) > 0.0) sep2 -= a[0] * Rab(0, 2);
    if (Rab(1, 2) > 0.0) sep2 -= a[1] * Rab(1, 2);
  }

  if (sep1 >= sep2 && sep1 >= 0) {
    if (Tab[2] > 0)
      S << 0, 0, sep1;
    else
      S << 0, 0, -sep1;

    if (P && Q) {
      *Q = S;
      P->setZero();
    }
  }

  if (sep2 >= sep1 && sep2 >= 0) {
    Vec3s Q_(Tab[0], Tab[1], Tab[2]);
    Vec3s P_;
    if (Tba[2] < 0) {
      P_[0] = Rab(0, 2) * sep2 + Tab[0];
      P_[1] = Rab(1, 2) * sep2 + Tab[1];
      P_[2] = Rab(2, 2) * sep2 + Tab[2];
    } else {
      P_[0] = -Rab(0, 2) * sep2 + Tab[0];
      P_[1] = -Rab(1, 2) * sep2 + Tab[1];
      P_[2] = -Rab(2, 2) * sep2 + Tab[2];
    }

    S = Q_ - P_;

    if (P && Q) {
      *P = P_;
      *Q = Q_;
    }
  }

  Scalar sep = (sep1 > sep2 ? sep1 : sep2);
  return (sep > 0 ? sep : 0);
}

bool RSS::overlap(const RSS& other) const {
  /// compute what transform [R,T] that takes us from cs1 to cs2.
  /// [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  /// First compute the rotation part, then translation part

  /// Then compute R1'(T2 - T1)
  Vec3s T(axes.transpose() * (other.Tr - Tr));

  /// Now compute R1'R2
  Matrix3s R(axes.transpose() * other.axes);

  Scalar dist = rectDistance(R, T, length, other.length);
  return (dist <= (radius + other.radius));
}

bool overlap(const Matrix3s& R0, const Vec3s& T0, const RSS& b1,
             const RSS& b2) {
  // ROb2 = R0 . b2
  // where b2 = [ b2.axis [0] | b2.axis [1] | b2.axis [2] ]

  // (1 0 0)^T R0b2^T axis [0] = (1 0 0)^T b2^T R0^T axis [0]
  // R = b2^T RO^T b1
  Vec3s Ttemp(R0.transpose() * (b2.Tr - T0) - b1.Tr);
  Vec3s T(b1.axes.transpose() * Ttemp);
  Matrix3s R(b1.axes.transpose() * R0.transpose() * b2.axes);

  Scalar dist = rectDistance(R, T, b1.length, b2.length);
  return (dist <= (b1.radius + b2.radius));
}

bool overlap(const Matrix3s& R0, const Vec3s& T0, const RSS& b1, const RSS& b2,
             const CollisionRequest& request, Scalar& sqrDistLowerBound) {
  // ROb2 = R0 . b2
  // where b2 = [ b2.axis [0] | b2.axis [1] | b2.axis [2] ]

  // (1 0 0)^T R0b2^T axis [0] = (1 0 0)^T b2^T R0^T axis [0]
  // R = b2^T RO^T b1
  Vec3s Ttemp(R0.transpose() * (b2.Tr - T0) - b1.Tr);
  Vec3s T(b1.axes.transpose() * Ttemp);
  Matrix3s R(b1.axes.transpose() * R0.transpose() * b2.axes);

  Scalar dist = rectDistance(R, T, b1.length, b2.length) - b1.radius -
                b2.radius - request.security_margin;
  if (dist <= 0) return true;
  sqrDistLowerBound = dist * dist;
  return false;
}

bool RSS::contain(const Vec3s& p) const {
  Vec3s local_p = p - Tr;
  // FIXME: Vec3s proj (axes.transpose() * local_p);
  Scalar proj0 = local_p.dot(axes.col(0));
  Scalar proj1 = local_p.dot(axes.col(1));
  Scalar proj2 = local_p.dot(axes.col(2));
  Scalar abs_proj2 = fabs(proj2);
  Vec3s proj(proj0, proj1, proj2);

  /// projection is within the rectangle
  if ((proj0 < length[0]) && (proj0 > 0) && (proj1 < length[1]) &&
      (proj1 > 0)) {
    return (abs_proj2 < radius);
  } else if ((proj0 < length[0]) && (proj0 > 0) &&
             ((proj1 < 0) || (proj1 > length[1]))) {
    Scalar y = (proj1 > 0) ? length[1] : 0;
    Vec3s v(proj0, y, 0);
    return ((proj - v).squaredNorm() < radius * radius);
  } else if ((proj1 < length[1]) && (proj1 > 0) &&
             ((proj0 < 0) || (proj0 > length[0]))) {
    Scalar x = (proj0 > 0) ? length[0] : 0;
    Vec3s v(x, proj1, 0);
    return ((proj - v).squaredNorm() < radius * radius);
  } else {
    Scalar x = (proj0 > 0) ? length[0] : 0;
    Scalar y = (proj1 > 0) ? length[1] : 0;
    Vec3s v(x, y, 0);
    return ((proj - v).squaredNorm() < radius * radius);
  }
}

RSS& RSS::operator+=(const Vec3s& p) {
  Vec3s local_p = p - Tr;
  Scalar proj0 = local_p.dot(axes.col(0));
  Scalar proj1 = local_p.dot(axes.col(1));
  Scalar proj2 = local_p.dot(axes.col(2));
  Scalar abs_proj2 = fabs(proj2);
  Vec3s proj(proj0, proj1, proj2);

  // projection is within the rectangle
  if ((proj0 < length[0]) && (proj0 > 0) && (proj1 < length[1]) &&
      (proj1 > 0)) {
    if (abs_proj2 < radius)
      ;  // do nothing
    else {
      const Scalar half = Scalar(0.5);
      radius = half * (radius + abs_proj2);  // enlarge the r
      // change RSS origin position
      if (proj2 > 0)
        Tr[2] += half * (abs_proj2 - radius);
      else
        Tr[2] -= half * (abs_proj2 - radius);
    }
  } else if ((proj0 < length[0]) && (proj0 > 0) &&
             ((proj1 < 0) || (proj1 > length[1]))) {
    Scalar y = (proj1 > 0) ? length[1] : 0;
    Vec3s v(proj0, y, 0);
    Scalar new_r_sqr = (proj - v).squaredNorm();
    if (new_r_sqr < radius * radius)
      ;  // do nothing
    else {
      if (abs_proj2 < radius) {
        Scalar delta_y =
            -std::sqrt(radius * radius - proj2 * proj2) + fabs(proj1 - y);
        length[1] += delta_y;
        if (proj1 < 0) Tr[1] -= delta_y;
      } else {
        Scalar delta_y = fabs(proj1 - y);
        length[1] += delta_y;
        if (proj1 < 0) Tr[1] -= delta_y;

        const Scalar half = Scalar(0.5);
        if (proj2 > 0)
          Tr[2] += half * (abs_proj2 - radius);
        else
          Tr[2] -= half * (abs_proj2 - radius);
      }
    }
  } else if ((proj1 < length[1]) && (proj1 > 0) &&
             ((proj0 < 0) || (proj0 > length[0]))) {
    Scalar x = (proj0 > 0) ? length[0] : 0;
    Vec3s v(x, proj1, 0);
    Scalar new_r_sqr = (proj - v).squaredNorm();
    if (new_r_sqr < radius * radius)
      ;  // do nothing
    else {
      if (abs_proj2 < radius) {
        Scalar delta_x =
            -std::sqrt(radius * radius - proj2 * proj2) + fabs(proj0 - x);
        length[0] += delta_x;
        if (proj0 < 0) Tr[0] -= delta_x;
      } else {
        Scalar delta_x = fabs(proj0 - x);
        length[0] += delta_x;
        if (proj0 < 0) Tr[0] -= delta_x;

        const Scalar half = Scalar(0.5);
        if (proj2 > 0)
          Tr[2] += half * (abs_proj2 - radius);
        else
          Tr[2] -= half * (abs_proj2 - radius);
      }
    }
  } else {
    Scalar x = (proj0 > 0) ? length[0] : 0;
    Scalar y = (proj1 > 0) ? length[1] : 0;
    Vec3s v(x, y, 0);
    Scalar new_r_sqr = (proj - v).squaredNorm();
    if (new_r_sqr < radius * radius)
      ;  // do nothing
    else {
      if (abs_proj2 < radius) {
        Scalar diag = std::sqrt(new_r_sqr - proj2 * proj2);
        Scalar delta_diag = -std::sqrt(radius * radius - proj2 * proj2) + diag;

        Scalar delta_x = delta_diag / diag * fabs(proj0 - x);
        Scalar delta_y = delta_diag / diag * fabs(proj1 - y);
        length[0] += delta_x;
        length[1] += delta_y;

        if (proj0 < 0 && proj1 < 0) {
          Tr[0] -= delta_x;
          Tr[1] -= delta_y;
        }
      } else {
        Scalar delta_x = fabs(proj0 - x);
        Scalar delta_y = fabs(proj1 - y);

        length[0] += delta_x;
        length[1] += delta_y;

        if (proj0 < 0 && proj1 < 0) {
          Tr[0] -= delta_x;
          Tr[1] -= delta_y;
        }

        const Scalar half = Scalar(0.5);
        if (proj2 > 0)
          Tr[2] += half * (abs_proj2 - radius);
        else
          Tr[2] -= half * (abs_proj2 - radius);
      }
    }
  }

  return *this;
}

RSS RSS::operator+(const RSS& other) const {
  RSS bv;

  Vec3s v[16];
  Vec3s d0_pos(other.axes.col(0) * (other.length[0] + other.radius));
  Vec3s d1_pos(other.axes.col(1) * (other.length[1] + other.radius));
  Vec3s d0_neg(other.axes.col(0) * (-other.radius));
  Vec3s d1_neg(other.axes.col(1) * (-other.radius));
  Vec3s d2_pos(other.axes.col(2) * other.radius);
  Vec3s d2_neg(other.axes.col(2) * (-other.radius));

  v[0].noalias() = other.Tr + d0_pos + d1_pos + d2_pos;
  v[1].noalias() = other.Tr + d0_pos + d1_pos + d2_neg;
  v[2].noalias() = other.Tr + d0_pos + d1_neg + d2_pos;
  v[3].noalias() = other.Tr + d0_pos + d1_neg + d2_neg;
  v[4].noalias() = other.Tr + d0_neg + d1_pos + d2_pos;
  v[5].noalias() = other.Tr + d0_neg + d1_pos + d2_neg;
  v[6].noalias() = other.Tr + d0_neg + d1_neg + d2_pos;
  v[7].noalias() = other.Tr + d0_neg + d1_neg + d2_neg;

  d0_pos.noalias() = axes.col(0) * (length[0] + radius);
  d1_pos.noalias() = axes.col(1) * (length[1] + radius);
  d0_neg.noalias() = axes.col(0) * (-radius);
  d1_neg.noalias() = axes.col(1) * (-radius);
  d2_pos.noalias() = axes.col(2) * radius;
  d2_neg.noalias() = axes.col(2) * (-radius);

  v[8].noalias() = Tr + d0_pos + d1_pos + d2_pos;
  v[9].noalias() = Tr + d0_pos + d1_pos + d2_neg;
  v[10].noalias() = Tr + d0_pos + d1_neg + d2_pos;
  v[11].noalias() = Tr + d0_pos + d1_neg + d2_neg;
  v[12].noalias() = Tr + d0_neg + d1_pos + d2_pos;
  v[13].noalias() = Tr + d0_neg + d1_pos + d2_neg;
  v[14].noalias() = Tr + d0_neg + d1_neg + d2_pos;
  v[15].noalias() = Tr + d0_neg + d1_neg + d2_neg;

  Matrix3s M;  // row first matrix
  Vec3s E[3];  // row first eigen-vectors
  Scalar s[3] = {0, 0, 0};

  getCovariance(v, NULL, NULL, NULL, 16, M);
  eigen(M, s, E);

  int min, mid, max;
  if (s[0] > s[1]) {
    max = 0;
    min = 1;
  } else {
    min = 0;
    max = 1;
  }
  if (s[2] < s[min]) {
    mid = min;
    min = 2;
  } else if (s[2] > s[max]) {
    mid = max;
    max = 2;
  } else {
    mid = 2;
  }

  // column first matrix, as the axis in RSS
  bv.axes.col(0) << E[0][max], E[1][max], E[2][max];
  bv.axes.col(1) << E[0][mid], E[1][mid], E[2][mid];
  bv.axes.col(2) << E[1][max] * E[2][mid] - E[1][mid] * E[2][max],
      E[0][mid] * E[2][max] - E[0][max] * E[2][mid],
      E[0][max] * E[1][mid] - E[0][mid] * E[1][max];

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize(v, NULL, NULL, NULL, 16, bv.axes, bv.Tr,
                                     bv.length, bv.radius);

  return bv;
}

Scalar RSS::distance(const RSS& other, Vec3s* P, Vec3s* Q) const {
  // compute what transform [R,T] that takes us from cs1 to cs2.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part
  Matrix3s R(axes.transpose() * other.axes);
  Vec3s T(axes.transpose() * (other.Tr - Tr));

  Scalar dist = rectDistance(R, T, length, other.length, P, Q);
  dist -= (radius + other.radius);
  return (dist < (Scalar)0.0) ? (Scalar)0.0 : dist;
}

Scalar distance(const Matrix3s& R0, const Vec3s& T0, const RSS& b1,
                const RSS& b2, Vec3s* P, Vec3s* Q) {
  Matrix3s R(b1.axes.transpose() * R0 * b2.axes);
  Vec3s Ttemp(R0 * b2.Tr + T0 - b1.Tr);

  Vec3s T(b1.axes.transpose() * Ttemp);

  Scalar dist = rectDistance(R, T, b1.length, b2.length, P, Q);
  dist -= (b1.radius + b2.radius);
  return (dist < (Scalar)0.0) ? (Scalar)0.0 : dist;
}

RSS translate(const RSS& bv, const Vec3s& t) {
  RSS res(bv);
  res.Tr += t;
  return res;
}

}  // namespace coal
