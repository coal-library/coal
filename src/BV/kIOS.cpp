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

#include "coal/BV/kIOS.h"
#include "coal/BVH/BVH_utility.h"
#include "coal/math/transform.h"

#include <iostream>
#include <limits>

namespace coal {

bool kIOS::overlap(const kIOS& other) const {
  for (unsigned int i = 0; i < num_spheres; ++i) {
    for (unsigned int j = 0; j < other.num_spheres; ++j) {
      Scalar o_dist = (spheres[i].o - other.spheres[j].o).squaredNorm();
      Scalar sum_r = spheres[i].r + other.spheres[j].r;
      if (o_dist > sum_r * sum_r) return false;
    }
  }

  return obb.overlap(other.obb);
}

bool kIOS::overlap(const kIOS& other, const CollisionRequest& request,
                   Scalar& sqrDistLowerBound) const {
  for (unsigned int i = 0; i < num_spheres; ++i) {
    for (unsigned int j = 0; j < other.num_spheres; ++j) {
      Scalar o_dist = (spheres[i].o - other.spheres[j].o).squaredNorm();
      Scalar sum_r = spheres[i].r + other.spheres[j].r;
      if (o_dist > sum_r * sum_r) {
        o_dist = sqrt(o_dist) - sum_r;
        sqrDistLowerBound = o_dist * o_dist;
        return false;
      }
    }
  }

  return obb.overlap(other.obb, request, sqrDistLowerBound);
}

bool kIOS::contain(const Vec3s& p) const {
  for (unsigned int i = 0; i < num_spheres; ++i) {
    Scalar r = spheres[i].r;
    if ((spheres[i].o - p).squaredNorm() > r * r) return false;
  }

  return true;
}

kIOS& kIOS::operator+=(const Vec3s& p) {
  for (unsigned int i = 0; i < num_spheres; ++i) {
    Scalar r = spheres[i].r;
    Scalar new_r_sqr = (p - spheres[i].o).squaredNorm();
    if (new_r_sqr > r * r) {
      spheres[i].r = sqrt(new_r_sqr);
    }
  }

  obb += p;
  return *this;
}

kIOS kIOS::operator+(const kIOS& other) const {
  kIOS result;
  unsigned int new_num_spheres = std::min(num_spheres, other.num_spheres);
  for (unsigned int i = 0; i < new_num_spheres; ++i) {
    result.spheres[i] = encloseSphere(spheres[i], other.spheres[i]);
  }

  result.num_spheres = new_num_spheres;

  result.obb = obb + other.obb;

  return result;
}

Scalar kIOS::width() const { return obb.width(); }

Scalar kIOS::height() const { return obb.height(); }

Scalar kIOS::depth() const { return obb.depth(); }

Scalar kIOS::volume() const { return obb.volume(); }

Scalar kIOS::size() const { return volume(); }

Scalar kIOS::distance(const kIOS& other, Vec3s* P, Vec3s* Q) const {
  Scalar d_max = 0;
  long id_a = -1, id_b = -1;
  for (unsigned int i = 0; i < num_spheres; ++i) {
    for (unsigned int j = 0; j < other.num_spheres; ++j) {
      Scalar d = (spheres[i].o - other.spheres[j].o).norm() -
                 (spheres[i].r + other.spheres[j].r);
      if (d_max < d) {
        d_max = d;
        if (P && Q) {
          id_a = (long)i;
          id_b = (long)j;
        }
      }
    }
  }

  if (P && Q) {
    if (id_a != -1 && id_b != -1) {
      const Vec3s v = spheres[id_a].o - spheres[id_b].o;
      Scalar len_v = v.norm();
      *P = spheres[id_a].o - v * (spheres[id_a].r / len_v);
      *Q = spheres[id_b].o + v * (spheres[id_b].r / len_v);
    }
  }

  return d_max;
}

bool overlap(const Matrix3s& R0, const Vec3s& T0, const kIOS& b1,
             const kIOS& b2) {
  kIOS b2_temp = b2;
  for (unsigned int i = 0; i < b2_temp.num_spheres; ++i) {
    b2_temp.spheres[i].o.noalias() =
        R0.transpose() * (b2_temp.spheres[i].o - T0);
  }

  b2_temp.obb.To.noalias() = R0.transpose() * (b2_temp.obb.To - T0);
  b2_temp.obb.axes.applyOnTheLeft(R0.transpose());

  return b1.overlap(b2_temp);
}

bool overlap(const Matrix3s& R0, const Vec3s& T0, const kIOS& b1,
             const kIOS& b2, const CollisionRequest& request,
             Scalar& sqrDistLowerBound) {
  kIOS b2_temp = b2;
  for (unsigned int i = 0; i < b2_temp.num_spheres; ++i) {
    b2_temp.spheres[i].o.noalias() =
        R0.transpose() * (b2_temp.spheres[i].o - T0);
  }

  b2_temp.obb.To.noalias() = R0.transpose() * (b2_temp.obb.To - T0);
  b2_temp.obb.axes.applyOnTheLeft(R0.transpose());

  return b1.overlap(b2_temp, request, sqrDistLowerBound);
}

Scalar distance(const Matrix3s& R0, const Vec3s& T0, const kIOS& b1,
                const kIOS& b2, Vec3s* P, Vec3s* Q) {
  kIOS b2_temp = b2;
  for (unsigned int i = 0; i < b2_temp.num_spheres; ++i) {
    b2_temp.spheres[i].o = R0 * b2_temp.spheres[i].o + T0;
  }

  return b1.distance(b2_temp, P, Q);
}

kIOS translate(const kIOS& bv, const Vec3s& t) {
  kIOS res(bv);
  for (size_t i = 0; i < res.num_spheres; ++i) {
    res.spheres[i].o += t;
  }

  translate(res.obb, t);
  return res;
}

}  // namespace coal
