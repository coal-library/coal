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

#ifndef COAL_GEOMETRIC_SHAPE_TO_BVH_MODEL_H
#define COAL_GEOMETRIC_SHAPE_TO_BVH_MODEL_H

#include "coal/shape/geometric_shapes.h"
#include "coal/BVH/BVH_model.h"
#include <boost/math/constants/constants.hpp>

namespace coal {

/// @brief Generate BVH model from box
template <typename BV>
void generateBVHModel(BVHModel<BV>& model, const Box& shape,
                      const Transform3s& pose) {
  Scalar a = shape.halfSide[0];
  Scalar b = shape.halfSide[1];
  Scalar c = shape.halfSide[2];
  std::vector<Vec3s> points(8);
  std::vector<Triangle32> tri_indices(12);
  points[0] = Vec3s(a, -b, c);
  points[1] = Vec3s(a, b, c);
  points[2] = Vec3s(-a, b, c);
  points[3] = Vec3s(-a, -b, c);
  points[4] = Vec3s(a, -b, -c);
  points[5] = Vec3s(a, b, -c);
  points[6] = Vec3s(-a, b, -c);
  points[7] = Vec3s(-a, -b, -c);

  tri_indices[0].set(0, 4, 1);
  tri_indices[1].set(1, 4, 5);
  tri_indices[2].set(2, 6, 3);
  tri_indices[3].set(3, 6, 7);
  tri_indices[4].set(3, 0, 2);
  tri_indices[5].set(2, 0, 1);
  tri_indices[6].set(6, 5, 7);
  tri_indices[7].set(7, 5, 4);
  tri_indices[8].set(1, 5, 2);
  tri_indices[9].set(2, 5, 6);
  tri_indices[10].set(3, 7, 0);
  tri_indices[11].set(0, 7, 4);

  for (unsigned int i = 0; i < points.size(); ++i) {
    points[i] = pose.transform(points[i]).eval();
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from sphere, given the number of segments along
/// longitude and number of rings along latitude.
template <typename BV>
void generateBVHModel(BVHModel<BV>& model, const Sphere& shape,
                      const Transform3s& pose, unsigned int seg,
                      unsigned int ring) {
  std::vector<Vec3s> points;
  std::vector<Triangle32> tri_indices;

  Scalar r = shape.radius;
  Scalar phi, phid;
  const Scalar pi = boost::math::constants::pi<Scalar>();
  phid = pi * 2 / Scalar(seg);
  phi = 0;

  Scalar theta, thetad;
  thetad = pi / Scalar(ring + 1);
  theta = 0;

  for (unsigned int i = 0; i < ring; ++i) {
    Scalar theta_ = theta + thetad * Scalar(i + 1);
    for (unsigned int j = 0; j < seg; ++j) {
      points.push_back(Vec3s(r * sin(theta_) * cos(phi + Scalar(j) * phid),
                             r * sin(theta_) * sin(phi + Scalar(j) * phid),
                             r * cos(theta_)));
    }
  }
  points.push_back(Vec3s(0, 0, r));
  points.push_back(Vec3s(0, 0, -r));

  for (unsigned int i = 0; i < ring - 1; ++i) {
    for (unsigned int j = 0; j < seg; ++j) {
      unsigned int a, b, c, d;
      a = i * seg + j;
      b = (j == seg - 1) ? (i * seg) : (i * seg + j + 1);
      c = (i + 1) * seg + j;
      d = (j == seg - 1) ? ((i + 1) * seg) : ((i + 1) * seg + j + 1);
      tri_indices.push_back(Triangle32(a, c, b));
      tri_indices.push_back(Triangle32(b, c, d));
    }
  }

  for (unsigned int j = 0; j < seg; ++j) {
    unsigned int a, b;
    a = j;
    b = (j == seg - 1) ? 0 : (j + 1);
    tri_indices.push_back(Triangle32(ring * seg, a, b));

    a = (ring - 1) * seg + j;
    b = (j == seg - 1) ? (ring - 1) * seg : ((ring - 1) * seg + j + 1);
    tri_indices.push_back(Triangle32(a, ring * seg + 1, b));
  }

  for (unsigned int i = 0; i < points.size(); ++i) {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from sphere
/// The difference between generateBVHModel is that it gives the number of
/// triangles faces N for a sphere with unit radius. For sphere of radius r,
/// then the number of triangles is r * r * N so that the area represented by a
/// single triangle is approximately the same.s
template <typename BV>
void generateBVHModel(BVHModel<BV>& model, const Sphere& shape,
                      const Transform3s& pose,
                      unsigned int n_faces_for_unit_sphere) {
  Scalar r = shape.radius;
  Scalar n_low_bound =
      std::sqrt((Scalar)n_faces_for_unit_sphere / Scalar(2.)) * r * r;
  unsigned int ring = (unsigned int)ceil(n_low_bound);
  unsigned int seg = (unsigned int)ceil(n_low_bound);

  generateBVHModel(model, shape, pose, seg, ring);
}

/// @brief Generate BVH model from cylinder, given the number of segments along
/// circle and the number of segments along axis.
template <typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cylinder& shape,
                      const Transform3s& pose, unsigned int tot,
                      unsigned int h_num) {
  std::vector<Vec3s> points;
  std::vector<Triangle32> tri_indices;

  Scalar r = shape.radius;
  Scalar h = shape.halfLength;
  Scalar phi, phid;
  const Scalar pi = boost::math::constants::pi<Scalar>();
  phid = pi * 2 / Scalar(tot);
  phi = 0;

  Scalar hd = 2 * h / Scalar(h_num);

  for (unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3s(r * cos(phi + phid * Scalar(i)),
                           r * sin(phi + phid * Scalar(i)), h));

  for (unsigned int i = 0; i < h_num - 1; ++i) {
    for (unsigned int j = 0; j < tot; ++j) {
      points.push_back(Vec3s(r * cos(phi + phid * Scalar(j)),
                             r * sin(phi + phid * Scalar(j)),
                             h - Scalar(i + 1) * hd));
    }
  }

  for (unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3s(r * cos(phi + phid * Scalar(i)),
                           r * sin(phi + phid * Scalar(i)), -h));

  points.push_back(Vec3s(0, 0, h));
  points.push_back(Vec3s(0, 0, -h));

  for (unsigned int i = 0; i < tot; ++i) {
    Triangle32 tmp((h_num + 1) * tot, i, ((i == tot - 1) ? 0 : (i + 1)));
    tri_indices.push_back(tmp);
  }

  for (unsigned int i = 0; i < tot; ++i) {
    Triangle32 tmp((h_num + 1) * tot + 1,
                   h_num * tot + ((i == tot - 1) ? 0 : (i + 1)),
                   h_num * tot + i);
    tri_indices.push_back(tmp);
  }

  for (unsigned int i = 0; i < h_num; ++i) {
    for (unsigned int j = 0; j < tot; ++j) {
      unsigned int a, b, c, d;
      a = j;
      b = (j == tot - 1) ? 0 : (j + 1);
      c = j + tot;
      d = (j == tot - 1) ? tot : (j + 1 + tot);

      unsigned int start = i * tot;
      tri_indices.push_back(Triangle32(start + b, start + a, start + c));
      tri_indices.push_back(Triangle32(start + b, start + c, start + d));
    }
  }

  for (unsigned int i = 0; i < points.size(); ++i) {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from cylinder
/// Difference from generateBVHModel: is that it gives the circle split number
/// tot for a cylinder with unit radius. For cylinder with larger radius, the
/// number of circle split number is r * tot.
template <typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cylinder& shape,
                      const Transform3s& pose,
                      unsigned int tot_for_unit_cylinder) {
  Scalar r = shape.radius;
  Scalar h = 2 * shape.halfLength;

  const Scalar pi = boost::math::constants::pi<Scalar>();
  unsigned int tot = (unsigned int)(Scalar(tot_for_unit_cylinder) * r);
  Scalar phid = pi * 2 / Scalar(tot);

  Scalar circle_edge = phid * r;
  unsigned int h_num = (unsigned int)ceil(h / circle_edge);

  generateBVHModel(model, shape, pose, tot, h_num);
}

/// @brief Generate BVH model from cone, given the number of segments along
/// circle and the number of segments along axis.
template <typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cone& shape,
                      const Transform3s& pose, unsigned int tot,
                      unsigned int h_num) {
  std::vector<Vec3s> points;
  std::vector<Triangle32> tri_indices;

  Scalar r = shape.radius;
  Scalar h = shape.halfLength;

  Scalar phi, phid;
  const Scalar pi = boost::math::constants::pi<Scalar>();
  phid = pi * 2 / Scalar(tot);
  phi = 0;

  Scalar hd = 2 * h / Scalar(h_num);

  for (unsigned int i = 0; i < h_num - 1; ++i) {
    Scalar h_i = h - Scalar(i + 1) * hd;
    Scalar rh = r * (Scalar(0.5) - h_i / h / 2);
    for (unsigned int j = 0; j < tot; ++j) {
      points.push_back(Vec3s(rh * cos(phi + phid * Scalar(j)),
                             rh * sin(phi + phid * Scalar(j)), h_i));
    }
  }

  for (unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3s(r * cos(phi + phid * Scalar(i)),
                           r * sin(phi + phid * Scalar(i)), -h));

  points.push_back(Vec3s(0, 0, h));
  points.push_back(Vec3s(0, 0, -h));

  for (unsigned int i = 0; i < tot; ++i) {
    Triangle32 tmp(h_num * tot, i, (i == tot - 1) ? 0 : (i + 1));
    tri_indices.push_back(tmp);
  }

  for (unsigned int i = 0; i < tot; ++i) {
    Triangle32 tmp(h_num * tot + 1,
                   (h_num - 1) * tot + ((i == tot - 1) ? 0 : (i + 1)),
                   (h_num - 1) * tot + i);
    tri_indices.push_back(tmp);
  }

  for (unsigned int i = 0; i < h_num - 1; ++i) {
    for (unsigned int j = 0; j < tot; ++j) {
      unsigned int a, b, c, d;
      a = j;
      b = (j == tot - 1) ? 0 : (j + 1);
      c = j + tot;
      d = (j == tot - 1) ? tot : (j + 1 + tot);

      unsigned int start = i * tot;
      tri_indices.push_back(Triangle32(start + b, start + a, start + c));
      tri_indices.push_back(Triangle32(start + b, start + c, start + d));
    }
  }

  for (unsigned int i = 0; i < points.size(); ++i) {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from cone
/// Difference from generateBVHModel: is that it gives the circle split number
/// tot for a cylinder with unit radius. For cone with larger radius, the number
/// of circle split number is r * tot.
template <typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cone& shape,
                      const Transform3s& pose, unsigned int tot_for_unit_cone) {
  Scalar r = shape.radius;
  Scalar h = 2 * shape.halfLength;

  const Scalar pi = boost::math::constants::pi<Scalar>();
  unsigned int tot = (unsigned int)(Scalar(tot_for_unit_cone) * r);
  Scalar phid = pi * 2 / Scalar(tot);

  Scalar circle_edge = phid * r;
  unsigned int h_num = (unsigned int)ceil(h / circle_edge);

  generateBVHModel(model, shape, pose, tot, h_num);
}

}  // namespace coal

#endif
