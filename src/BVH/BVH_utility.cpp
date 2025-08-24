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

#include "coal/BVH/BVH_utility.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes_utility.h"
#include "coal/internal/shape_shape_func.h"

namespace coal {

namespace details {
template <typename BV>
BVHModel<BV>* BVHExtract(const BVHModel<BV>& model, const Transform3s& pose,
                         const AABB& _aabb) {
  assert(model.getModelType() == BVH_MODEL_TRIANGLES);
  const Matrix3s& q = pose.getRotation();
  AABB aabb = translate(_aabb, -pose.getTranslation());

  Transform3s box_pose;
  Box box;
  constructBox(_aabb, box, box_pose);
  box_pose = pose.inverseTimes(box_pose);

  GJKSolver gjk;
  // Early-stop GJK as soon as a separating plane is found
  gjk.gjk.setDistanceEarlyBreak(Scalar(1e-3));

  // Check what triangles should be kept.
  // TODO use the BV hierarchy
  std::vector<bool> keep_vertex(model.num_vertices, false);
  std::vector<bool> keep_tri(model.num_tris, false);
  unsigned int ntri = 0;
  const std::vector<Vec3s>& model_vertices_ = *(model.vertices);
  const std::vector<Triangle32>& model_tri_indices_ = *(model.tri_indices);
  for (unsigned int i = 0; i < model.num_tris; ++i) {
    const Triangle32& t = model_tri_indices_[i];

    bool keep_this_tri =
        keep_vertex[t[0]] || keep_vertex[t[1]] || keep_vertex[t[2]];

    if (!keep_this_tri) {
      for (unsigned int j = 0; j < 3; ++j) {
        if (aabb.contain(q * model_vertices_[t[j]])) {
          keep_this_tri = true;
          break;
        }
      }

      const TriangleP tri(model_vertices_[t[0]], model_vertices_[t[1]],
                          model_vertices_[t[2]]);
      const bool enable_nearest_points = false;
      const bool compute_penetration = false;  // we only need to know if there
                                               // is a collision or not
      const DistanceRequest distanceRequest(enable_nearest_points,
                                            compute_penetration);
      DistanceResult distanceResult;
      const Scalar distance = ShapeShapeDistance<Box, TriangleP>(
          &box, box_pose, &tri, Transform3s(), &gjk, distanceRequest,
          distanceResult);
      bool is_collision =
          (distance <= gjk.getDistancePrecision(compute_penetration));

      if (!keep_this_tri && is_collision) {
        keep_this_tri = true;
      }
    }
    if (keep_this_tri) {
      keep_vertex[t[0]] = keep_vertex[t[1]] = keep_vertex[t[2]] = true;
      keep_tri[i] = true;
      ntri++;
    }
  }

  if (ntri == 0) return NULL;

  BVHModel<BV>* new_model(new BVHModel<BV>());
  new_model->beginModel(ntri, std::min(ntri * 3, model.num_vertices));
  std::vector<unsigned int> idxConversion(model.num_vertices);
  assert(new_model->num_vertices == 0);
  std::vector<Vec3s>& new_model_vertices_ = *(new_model->vertices);
  for (unsigned int i = 0; i < keep_vertex.size(); ++i) {
    if (keep_vertex[i]) {
      idxConversion[i] = new_model->num_vertices;
      new_model_vertices_[new_model->num_vertices] = model_vertices_[i];
      new_model->num_vertices++;
    }
  }
  assert(new_model->num_tris == 0);
  std::vector<Triangle32>& new_model_tri_indices_ = *(new_model->tri_indices);
  for (unsigned int i = 0; i < keep_tri.size(); ++i) {
    if (keep_tri[i]) {
      new_model_tri_indices_[new_model->num_tris].set(
          idxConversion[model_tri_indices_[i][0]],
          idxConversion[model_tri_indices_[i][1]],
          idxConversion[model_tri_indices_[i][2]]);
      new_model->num_tris++;
    }
  }
  if (new_model->endModel() != BVH_OK) {
    delete new_model;
    return NULL;
  }
  return new_model;
}
}  // namespace details

template <>
BVHModel<OBB>* BVHExtract(const BVHModel<OBB>& model, const Transform3s& pose,
                          const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}
template <>
BVHModel<AABB>* BVHExtract(const BVHModel<AABB>& model, const Transform3s& pose,
                           const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}
template <>
BVHModel<RSS>* BVHExtract(const BVHModel<RSS>& model, const Transform3s& pose,
                          const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}
template <>
BVHModel<kIOS>* BVHExtract(const BVHModel<kIOS>& model, const Transform3s& pose,
                           const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}
template <>
BVHModel<OBBRSS>* BVHExtract(const BVHModel<OBBRSS>& model,
                             const Transform3s& pose, const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}
template <>
BVHModel<KDOP<16> >* BVHExtract(const BVHModel<KDOP<16> >& model,
                                const Transform3s& pose, const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}
template <>
BVHModel<KDOP<18> >* BVHExtract(const BVHModel<KDOP<18> >& model,
                                const Transform3s& pose, const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}
template <>
BVHModel<KDOP<24> >* BVHExtract(const BVHModel<KDOP<24> >& model,
                                const Transform3s& pose, const AABB& aabb) {
  return details::BVHExtract(model, pose, aabb);
}

void getCovariance(Vec3s* ps, Vec3s* ps2, Triangle32* ts, unsigned int* indices,
                   unsigned int n, Matrix3s& M) {
  Vec3s S1(Vec3s::Zero());
  Vec3s S2[3] = {Vec3s::Zero(), Vec3s::Zero(), Vec3s::Zero()};

  if (ts) {
    for (unsigned int i = 0; i < n; ++i) {
      const Triangle32& t = (indices) ? ts[indices[i]] : ts[i];

      const Vec3s& p1 = ps[t[0]];
      const Vec3s& p2 = ps[t[1]];
      const Vec3s& p3 = ps[t[2]];

      S1[0] += (p1[0] + p2[0] + p3[0]);
      S1[1] += (p1[1] + p2[1] + p3[1]);
      S1[2] += (p1[2] + p2[2] + p3[2]);
      S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
      S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
      S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
      S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
      S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
      S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);

      if (ps2) {
        const Vec3s& p1 = ps2[t[0]];
        const Vec3s& p2 = ps2[t[1]];
        const Vec3s& p3 = ps2[t[2]];

        S1[0] += (p1[0] + p2[0] + p3[0]);
        S1[1] += (p1[1] + p2[1] + p3[1]);
        S1[2] += (p1[2] + p2[2] + p3[2]);

        S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
        S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
        S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
        S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
        S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
        S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
      }
    }
  } else {
    for (unsigned int i = 0; i < n; ++i) {
      const Vec3s& p = (indices) ? ps[indices[i]] : ps[i];
      S1 += p;
      S2[0][0] += (p[0] * p[0]);
      S2[1][1] += (p[1] * p[1]);
      S2[2][2] += (p[2] * p[2]);
      S2[0][1] += (p[0] * p[1]);
      S2[0][2] += (p[0] * p[2]);
      S2[1][2] += (p[1] * p[2]);

      if (ps2)  // another frame
      {
        const Vec3s& p = (indices) ? ps2[indices[i]] : ps2[i];
        S1 += p;
        S2[0][0] += (p[0] * p[0]);
        S2[1][1] += (p[1] * p[1]);
        S2[2][2] += (p[2] * p[2]);
        S2[0][1] += (p[0] * p[1]);
        S2[0][2] += (p[0] * p[2]);
        S2[1][2] += (p[1] * p[2]);
      }
    }
  }

  unsigned int n_points = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  M(0, 0) = S2[0][0] - S1[0] * S1[0] / Scalar(n_points);
  M(1, 1) = S2[1][1] - S1[1] * S1[1] / Scalar(n_points);
  M(2, 2) = S2[2][2] - S1[2] * S1[2] / Scalar(n_points);
  M(0, 1) = S2[0][1] - S1[0] * S1[1] / Scalar(n_points);
  M(1, 2) = S2[1][2] - S1[1] * S1[2] / Scalar(n_points);
  M(0, 2) = S2[0][2] - S1[0] * S1[2] / Scalar(n_points);
  M(1, 0) = M(0, 1);
  M(2, 0) = M(0, 2);
  M(2, 1) = M(1, 2);
}

/** @brief Compute the RSS bounding volume parameters: radius, rectangle size
 * and the origin. The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3s* ps, Vec3s* ps2, Triangle32* ts,
                                        unsigned int* indices, unsigned int n,
                                        const Matrix3s& axes, Vec3s& origin,
                                        Scalar l[2], Scalar& r) {
  bool indirect_index = true;
  if (!indices) indirect_index = false;

  unsigned int size_P = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  Scalar(*P)[3] = new Scalar[size_P][3];

  int P_id = 0;

  if (ts) {
    for (unsigned int i = 0; i < n; ++i) {
      unsigned int index = indirect_index ? indices[i] : i;
      const Triangle32& t = ts[index];

      for (Triangle32::IndexType j = 0; j < 3; ++j) {
        Triangle32::IndexType point_id = t[j];
        const Vec3s& p = ps[point_id];
        Vec3s v(p[0], p[1], p[2]);
        P[P_id][0] = axes.col(0).dot(v);
        P[P_id][1] = axes.col(1).dot(v);
        P[P_id][2] = axes.col(2).dot(v);
        P_id++;
      }

      if (ps2) {
        for (Triangle32::IndexType j = 0; j < 3; ++j) {
          Triangle32::IndexType point_id = t[j];
          const Vec3s& p = ps2[point_id];
          // FIXME Is this right ?????
          Vec3s v(p[0], p[1], p[2]);
          P[P_id][0] = axes.col(0).dot(v);
          P[P_id][1] = axes.col(0).dot(v);
          P[P_id][2] = axes.col(1).dot(v);
          P_id++;
        }
      }
    }
  } else {
    for (unsigned int i = 0; i < n; ++i) {
      unsigned int index = indirect_index ? indices[i] : i;

      const Vec3s& p = ps[index];
      Vec3s v(p[0], p[1], p[2]);
      P[P_id][0] = axes.col(0).dot(v);
      P[P_id][1] = axes.col(1).dot(v);
      P[P_id][2] = axes.col(2).dot(v);
      P_id++;

      if (ps2) {
        const Vec3s& v = ps2[index];
        P[P_id][0] = axes.col(0).dot(v);
        P[P_id][1] = axes.col(1).dot(v);
        P[P_id][2] = axes.col(2).dot(v);
        P_id++;
      }
    }
  }

  Scalar minx, maxx, miny, maxy, minz, maxz;

  Scalar cz, radsqr;

  minz = maxz = P[0][2];

  for (unsigned int i = 1; i < size_P; ++i) {
    Scalar z_value = P[i][2];
    if (z_value < minz)
      minz = z_value;
    else if (z_value > maxz)
      maxz = z_value;
  }

  r = (Scalar)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (Scalar)0.5 * (maxz + minz);

  // compute an initial norm of rectangle along x direction

  // find minx and maxx as starting points

  unsigned int minindex = 0, maxindex = 0;
  Scalar mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for (unsigned int i = 1; i < size_P; ++i) {
    Scalar x_value = P[i][0];
    if (x_value < mintmp) {
      minindex = i;
      mintmp = x_value;
    } else if (x_value > maxtmp) {
      maxindex = i;
      maxtmp = x_value;
    }
  }

  Scalar x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));

  // grow minx/maxx

  for (unsigned int i = 0; i < size_P; ++i) {
    if (P[i][0] < minx) {
      dz = P[i][2] - cz;
      x = P[i][0] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if (x < minx) minx = x;
    } else if (P[i][0] > maxx) {
      dz = P[i][2] - cz;
      x = P[i][0] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if (x > maxx) maxx = x;
    }
  }

  // compute an initial norm of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for (unsigned int i = 1; i < size_P; ++i) {
    Scalar y_value = P[i][1];
    if (y_value < mintmp) {
      minindex = i;
      mintmp = y_value;
    } else if (y_value > maxtmp) {
      maxindex = i;
      maxtmp = y_value;
    }
  }

  Scalar y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));

  // grow miny/maxy

  for (unsigned int i = 0; i < size_P; ++i) {
    if (P[i][1] < miny) {
      dz = P[i][2] - cz;
      y = P[i][1] + std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if (y < miny) miny = y;
    } else if (P[i][1] > maxy) {
      dz = P[i][2] - cz;
      y = P[i][1] - std::sqrt(std::max<Scalar>(radsqr - dz * dz, 0));
      if (y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if
  // necessary quite conservative (can be improved)
  Scalar dx, dy, u, t;
  Scalar a = std::sqrt((Scalar)0.5);
  for (unsigned int i = 0; i < size_P; ++i) {
    if (P[i][0] > maxx) {
      if (P[i][1] > maxy) {
        dx = P[i][0] - maxx;
        dy = P[i][1] - maxy;
        u = dx * a + dy * a;
        t = (a * u - dx) * (a * u - dx) + (a * u - dy) * (a * u - dy) +
            (cz - P[i][2]) * (cz - P[i][2]);
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
        if (u > 0) {
          maxx += u * a;
          maxy += u * a;
        }
      } else if (P[i][1] < miny) {
        dx = P[i][0] - maxx;
        dy = P[i][1] - miny;
        u = dx * a - dy * a;
        t = (a * u - dx) * (a * u - dx) + (-a * u - dy) * (-a * u - dy) +
            (cz - P[i][2]) * (cz - P[i][2]);
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
        if (u > 0) {
          maxx += u * a;
          miny -= u * a;
        }
      }
    } else if (P[i][0] < minx) {
      if (P[i][1] > maxy) {
        dx = P[i][0] - minx;
        dy = P[i][1] - maxy;
        u = dy * a - dx * a;
        t = (-a * u - dx) * (-a * u - dx) + (a * u - dy) * (a * u - dy) +
            (cz - P[i][2]) * (cz - P[i][2]);
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
        if (u > 0) {
          minx -= u * a;
          maxy += u * a;
        }
      } else if (P[i][1] < miny) {
        dx = P[i][0] - minx;
        dy = P[i][1] - miny;
        u = -dx * a - dy * a;
        t = (-a * u - dx) * (-a * u - dx) + (-a * u - dy) * (-a * u - dy) +
            (cz - P[i][2]) * (cz - P[i][2]);
        u = u - std::sqrt(std::max<Scalar>(radsqr - t, 0));
        if (u > 0) {
          minx -= u * a;
          miny -= u * a;
        }
      }
    }
  }

  origin.noalias() = axes * Vec3s(minx, miny, cz);

  l[0] = std::max<Scalar>(maxx - minx, 0);
  l[1] = std::max<Scalar>(maxy - miny, 0);

  delete[] P;
}

/** @brief Compute the bounding volume extent and center for a set or subset of
 * points. The bounding volume axes are known.
 */
static inline void getExtentAndCenter_pointcloud(Vec3s* ps, Vec3s* ps2,
                                                 unsigned int* indices,
                                                 unsigned int n, Matrix3s& axes,
                                                 Vec3s& center, Vec3s& extent) {
  bool indirect_index = true;
  if (!indices) indirect_index = false;

  Scalar real_max = (std::numeric_limits<Scalar>::max)();

  Vec3s min_coord(real_max, real_max, real_max);
  Vec3s max_coord(-real_max, -real_max, -real_max);

  for (unsigned int i = 0; i < n; ++i) {
    unsigned int index = indirect_index ? indices[i] : i;

    const Vec3s& p = ps[index];
    Vec3s proj(axes.transpose() * p);

    for (int j = 0; j < 3; ++j) {
      if (proj[j] > max_coord[j]) max_coord[j] = proj[j];
      if (proj[j] < min_coord[j]) min_coord[j] = proj[j];
    }

    if (ps2) {
      const Vec3s& v = ps2[index];
      proj.noalias() = axes.transpose() * v;

      for (int j = 0; j < 3; ++j) {
        if (proj[j] > max_coord[j]) max_coord[j] = proj[j];
        if (proj[j] < min_coord[j]) min_coord[j] = proj[j];
      }
    }
  }

  center.noalias() = axes * (max_coord + min_coord) / 2;

  extent.noalias() = (max_coord - min_coord) / 2;
}

/** @brief Compute the bounding volume extent and center for a set or subset of
 * points. The bounding volume axes are known.
 */
static inline void getExtentAndCenter_mesh(Vec3s* ps, Vec3s* ps2,
                                           Triangle32* ts,
                                           unsigned int* indices,
                                           unsigned int n, Matrix3s& axes,
                                           Vec3s& center, Vec3s& extent) {
  bool indirect_index = true;
  if (!indices) indirect_index = false;

  Scalar real_max = (std::numeric_limits<Scalar>::max)();

  Vec3s min_coord(real_max, real_max, real_max);
  Vec3s max_coord(-real_max, -real_max, -real_max);

  for (unsigned int i = 0; i < n; ++i) {
    unsigned int index = indirect_index ? indices[i] : i;
    const Triangle32& t = ts[index];

    for (Triangle32::IndexType j = 0; j < 3; ++j) {
      Triangle32::IndexType point_id = t[j];
      const Vec3s& p = ps[point_id];
      Vec3s proj(axes.transpose() * p);

      for (int k = 0; k < 3; ++k) {
        if (proj[k] > max_coord[k]) max_coord[k] = proj[k];
        if (proj[k] < min_coord[k]) min_coord[k] = proj[k];
      }
    }

    if (ps2) {
      for (Triangle32::IndexType j = 0; j < 3; ++j) {
        Triangle32::IndexType point_id = t[j];
        const Vec3s& p = ps2[point_id];
        Vec3s proj(axes.transpose() * p);

        for (int k = 0; k < 3; ++k) {
          if (proj[k] > max_coord[k]) max_coord[k] = proj[k];
          if (proj[k] < min_coord[k]) min_coord[k] = proj[k];
        }
      }
    }
  }

  Vec3s o((max_coord + min_coord) / 2);

  center.noalias() = axes * o;

  extent.noalias() = (max_coord - min_coord) / 2;
}

void getExtentAndCenter(Vec3s* ps, Vec3s* ps2, Triangle32* ts,
                        unsigned int* indices, unsigned int n, Matrix3s& axes,
                        Vec3s& center, Vec3s& extent) {
  if (ts)
    getExtentAndCenter_mesh(ps, ps2, ts, indices, n, axes, center, extent);
  else
    getExtentAndCenter_pointcloud(ps, ps2, indices, n, axes, center, extent);
}

void circumCircleComputation(const Vec3s& a, const Vec3s& b, const Vec3s& c,
                             Vec3s& center, Scalar& radius) {
  Vec3s e1 = a - c;
  Vec3s e2 = b - c;
  Scalar e1_len2 = e1.squaredNorm();
  Scalar e2_len2 = e2.squaredNorm();
  Vec3s e3 = e1.cross(e2);
  Scalar e3_len2 = e3.squaredNorm();
  radius = e1_len2 * e2_len2 * (e1 - e2).squaredNorm() / e3_len2;
  radius = std::sqrt(radius) * Scalar(0.5);

  center = (e2 * e1_len2 - e1 * e2_len2).cross(e3) * (0.5 * 1 / e3_len2) + c;
}

static inline Scalar maximumDistance_mesh(Vec3s* ps, Vec3s* ps2, Triangle32* ts,
                                          unsigned int* indices, unsigned int n,
                                          const Vec3s& query) {
  bool indirect_index = true;
  if (!indices) indirect_index = false;

  Scalar maxD = 0;
  for (unsigned int i = 0; i < n; ++i) {
    unsigned int index = indirect_index ? indices[i] : i;
    const Triangle32& t = ts[index];

    for (Triangle32::IndexType j = 0; j < 3; ++j) {
      Triangle32::IndexType point_id = t[j];
      const Vec3s& p = ps[point_id];

      Scalar d = (p - query).squaredNorm();
      if (d > maxD) maxD = d;
    }

    if (ps2) {
      for (Triangle32::IndexType j = 0; j < 3; ++j) {
        Triangle32::IndexType point_id = t[j];
        const Vec3s& p = ps2[point_id];

        Scalar d = (p - query).squaredNorm();
        if (d > maxD) maxD = d;
      }
    }
  }

  return std::sqrt(maxD);
}

static inline Scalar maximumDistance_pointcloud(Vec3s* ps, Vec3s* ps2,
                                                unsigned int* indices,
                                                unsigned int n,
                                                const Vec3s& query) {
  bool indirect_index = true;
  if (!indices) indirect_index = false;

  Scalar maxD = 0;
  for (unsigned int i = 0; i < n; ++i) {
    unsigned int index = indirect_index ? indices[i] : i;

    const Vec3s& p = ps[index];
    Scalar d = (p - query).squaredNorm();
    if (d > maxD) maxD = d;

    if (ps2) {
      const Vec3s& v = ps2[index];
      Scalar d = (v - query).squaredNorm();
      if (d > maxD) maxD = d;
    }
  }

  return std::sqrt(maxD);
}

Scalar maximumDistance(Vec3s* ps, Vec3s* ps2, Triangle32* ts,
                       unsigned int* indices, unsigned int n,
                       const Vec3s& query) {
  if (ts)
    return maximumDistance_mesh(ps, ps2, ts, indices, n, query);
  else
    return maximumDistance_pointcloud(ps, ps2, indices, n, query);
}

}  // namespace coal
