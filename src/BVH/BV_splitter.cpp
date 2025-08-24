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

#include "coal/internal/BV_splitter.h"

namespace coal {

template <typename BV>
void computeSplitVector(const BV& bv, Vec3s& split_vector) {
  split_vector = bv.axes.col(0);
}

template <>
void computeSplitVector<kIOS>(const kIOS& bv, Vec3s& split_vector) {
  /*
    switch(bv.num_spheres)
    {
    case 1:
    split_vector = Vec3s(1, 0, 0);
    break;
    case 3:
    {
    Vec3s v[3];
    v[0] = bv.spheres[1].o - bv.spheres[0].o;
    v[0].normalize();
    generateCoordinateSystem(v[0], v[1], v[2]);
    split_vector = v[1];
    }
    break;
    case 5:
    {
    Vec3s v[2];
    v[0] = bv.spheres[1].o - bv.spheres[0].o;
    v[1] = bv.spheres[3].o - bv.spheres[0].o;
    split_vector = v[0].cross(v[1]);
    split_vector.normalize();
    }
    break;
    default:
    ;
    }
  */
  split_vector = bv.obb.axes.col(0);
}

template <>
void computeSplitVector<OBBRSS>(const OBBRSS& bv, Vec3s& split_vector) {
  split_vector = bv.obb.axes.col(0);
}

template <typename BV>
void computeSplitValue_bvcenter(const BV& bv, Scalar& split_value) {
  Vec3s center = bv.center();
  split_value = center[0];
}

template <typename BV>
void computeSplitValue_mean(const BV&, Vec3s* vertices, Triangle32* triangles,
                            unsigned int* primitive_indices,
                            unsigned int num_primitives, BVHModelType type,
                            const Vec3s& split_vector, Scalar& split_value) {
  if (type == BVH_MODEL_TRIANGLES) {
    Vec3s c(Vec3s::Zero());

    for (unsigned int i = 0; i < num_primitives; ++i) {
      const Triangle32& t = triangles[primitive_indices[i]];
      const Vec3s& p1 = vertices[t[0]];
      const Vec3s& p2 = vertices[t[1]];
      const Vec3s& p3 = vertices[t[2]];

      c += p1 + p2 + p3;
    }
    split_value = c.dot(split_vector) / Scalar(3 * num_primitives);
  } else if (type == BVH_MODEL_POINTCLOUD) {
    Scalar sum = 0;
    for (unsigned int i = 0; i < num_primitives; ++i) {
      const Vec3s& p = vertices[primitive_indices[i]];
      sum += p.dot(split_vector);
    }

    split_value = sum / Scalar(num_primitives);
  }
}

template <typename BV>
void computeSplitValue_median(const BV&, Vec3s* vertices, Triangle32* triangles,
                              unsigned int* primitive_indices,
                              unsigned int num_primitives, BVHModelType type,
                              const Vec3s& split_vector, Scalar& split_value) {
  std::vector<Scalar> proj(num_primitives);

  if (type == BVH_MODEL_TRIANGLES) {
    for (unsigned int i = 0; i < num_primitives; ++i) {
      const Triangle32& t = triangles[primitive_indices[i]];
      const Vec3s& p1 = vertices[t[0]];
      const Vec3s& p2 = vertices[t[1]];
      const Vec3s& p3 = vertices[t[2]];
      Vec3s centroid3(p1[0] + p2[0] + p3[0], p1[1] + p2[1] + p3[1],
                      p1[2] + p2[2] + p3[2]);

      proj[i] = centroid3.dot(split_vector) / 3;
    }
  } else if (type == BVH_MODEL_POINTCLOUD) {
    for (unsigned int i = 0; i < num_primitives; ++i) {
      const Vec3s& p = vertices[primitive_indices[i]];
      Vec3s v(p[0], p[1], p[2]);
      proj[i] = v.dot(split_vector);
    }
  }

  std::sort(proj.begin(), proj.end());

  if (num_primitives % 2 == 1) {
    split_value = proj[(num_primitives - 1) / 2];
  } else {
    split_value = (proj[num_primitives / 2] + proj[num_primitives / 2 - 1]) / 2;
  }
}

template <>
void BVSplitter<OBB>::computeRule_bvcenter(const OBB& bv, unsigned int*,
                                           unsigned int) {
  computeSplitVector<OBB>(bv, split_vector);
  computeSplitValue_bvcenter<OBB>(bv, split_value);
}

template <>
void BVSplitter<OBB>::computeRule_mean(const OBB& bv,
                                       unsigned int* primitive_indices,
                                       unsigned int num_primitives) {
  computeSplitVector<OBB>(bv, split_vector);
  computeSplitValue_mean<OBB>(bv, vertices, tri_indices, primitive_indices,
                              num_primitives, type, split_vector, split_value);
}

template <>
void BVSplitter<OBB>::computeRule_median(const OBB& bv,
                                         unsigned int* primitive_indices,
                                         unsigned int num_primitives) {
  computeSplitVector<OBB>(bv, split_vector);
  computeSplitValue_median<OBB>(bv, vertices, tri_indices, primitive_indices,
                                num_primitives, type, split_vector,
                                split_value);
}

template <>
void BVSplitter<RSS>::computeRule_bvcenter(const RSS& bv, unsigned int*,
                                           unsigned int) {
  computeSplitVector<RSS>(bv, split_vector);
  computeSplitValue_bvcenter<RSS>(bv, split_value);
}

template <>
void BVSplitter<RSS>::computeRule_mean(const RSS& bv,
                                       unsigned int* primitive_indices,
                                       unsigned int num_primitives) {
  computeSplitVector<RSS>(bv, split_vector);
  computeSplitValue_mean<RSS>(bv, vertices, tri_indices, primitive_indices,
                              num_primitives, type, split_vector, split_value);
}

template <>
void BVSplitter<RSS>::computeRule_median(const RSS& bv,
                                         unsigned int* primitive_indices,
                                         unsigned int num_primitives) {
  computeSplitVector<RSS>(bv, split_vector);
  computeSplitValue_median<RSS>(bv, vertices, tri_indices, primitive_indices,
                                num_primitives, type, split_vector,
                                split_value);
}

template <>
void BVSplitter<kIOS>::computeRule_bvcenter(const kIOS& bv, unsigned int*,
                                            unsigned int) {
  computeSplitVector<kIOS>(bv, split_vector);
  computeSplitValue_bvcenter<kIOS>(bv, split_value);
}

template <>
void BVSplitter<kIOS>::computeRule_mean(const kIOS& bv,
                                        unsigned int* primitive_indices,
                                        unsigned int num_primitives) {
  computeSplitVector<kIOS>(bv, split_vector);
  computeSplitValue_mean<kIOS>(bv, vertices, tri_indices, primitive_indices,
                               num_primitives, type, split_vector, split_value);
}

template <>
void BVSplitter<kIOS>::computeRule_median(const kIOS& bv,
                                          unsigned int* primitive_indices,
                                          unsigned int num_primitives) {
  computeSplitVector<kIOS>(bv, split_vector);
  computeSplitValue_median<kIOS>(bv, vertices, tri_indices, primitive_indices,
                                 num_primitives, type, split_vector,
                                 split_value);
}

template <>
void BVSplitter<OBBRSS>::computeRule_bvcenter(const OBBRSS& bv, unsigned int*,
                                              unsigned int) {
  computeSplitVector<OBBRSS>(bv, split_vector);
  computeSplitValue_bvcenter<OBBRSS>(bv, split_value);
}

template <>
void BVSplitter<OBBRSS>::computeRule_mean(const OBBRSS& bv,
                                          unsigned int* primitive_indices,
                                          unsigned int num_primitives) {
  computeSplitVector<OBBRSS>(bv, split_vector);
  computeSplitValue_mean<OBBRSS>(bv, vertices, tri_indices, primitive_indices,
                                 num_primitives, type, split_vector,
                                 split_value);
}

template <>
void BVSplitter<OBBRSS>::computeRule_median(const OBBRSS& bv,
                                            unsigned int* primitive_indices,
                                            unsigned int num_primitives) {
  computeSplitVector<OBBRSS>(bv, split_vector);
  computeSplitValue_median<OBBRSS>(bv, vertices, tri_indices, primitive_indices,
                                   num_primitives, type, split_vector,
                                   split_value);
}

template <>
bool BVSplitter<OBB>::apply(const Vec3s& q) const {
  return split_vector.dot(Vec3s(q[0], q[1], q[2])) > split_value;
}

template <>
bool BVSplitter<RSS>::apply(const Vec3s& q) const {
  return split_vector.dot(Vec3s(q[0], q[1], q[2])) > split_value;
}

template <>
bool BVSplitter<kIOS>::apply(const Vec3s& q) const {
  return split_vector.dot(Vec3s(q[0], q[1], q[2])) > split_value;
}

template <>
bool BVSplitter<OBBRSS>::apply(const Vec3s& q) const {
  return split_vector.dot(Vec3s(q[0], q[1], q[2])) > split_value;
}

template class BVSplitter<RSS>;
template class BVSplitter<OBBRSS>;
template class BVSplitter<OBB>;
template class BVSplitter<kIOS>;

}  // namespace coal
