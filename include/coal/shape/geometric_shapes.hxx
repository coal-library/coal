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

#ifndef COAL_GEOMETRIC_SHAPES_HXX
#define COAL_GEOMETRIC_SHAPES_HXX

#include "coal/shape/convex.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/narrowphase/support_data.h"

#include <iostream>

namespace coal {

template <typename IndexType>
NODE_TYPE ConvexBaseTpl<IndexType>::getNodeType() const {
  COAL_THROW_PRETTY("Uknown vertex index type for ConvexBase.",
                    std::runtime_error);
  return NODE_COUNT;
}

template <>
inline NODE_TYPE ConvexBaseTpl<std::uint16_t>::getNodeType() const {
  return GEOM_CONVEX16;
}

template <>
inline NODE_TYPE ConvexBaseTpl<std::uint32_t>::getNodeType() const {
  return GEOM_CONVEX32;
}

template <typename IndexType>
void ConvexBaseTpl<IndexType>::initialize(
    std::shared_ptr<std::vector<Vec3s>> points_, unsigned int num_points_) {
  this->points = points_;
  this->num_points = num_points_;
  COAL_ASSERT(this->points->size() == this->num_points,
              "The number of points is not consistent with the size of the "
              "points vector",
              std::logic_error);
  this->num_normals_and_offsets = 0;
  this->normals.reset();
  this->offsets.reset();
  this->computeCenter();
}

template <typename IndexType>
void ConvexBaseTpl<IndexType>::set(std::shared_ptr<std::vector<Vec3s>> points_,
                                   unsigned int num_points_) {
  initialize(points_, num_points_);
}

template <typename IndexType>
ConvexBaseTpl<IndexType>& ConvexBaseTpl<IndexType>::operator=(
    const ConvexBaseTpl& other) {
  if (this != &other) {
    // Copy the base
    this->base() = other.base();

    // Shallow copy the rest of the data
    this->points = other.points;
    this->num_points = other.num_points;
    this->normals = other.normals;
    this->offsets = other.offsets;
    this->num_normals_and_offsets = other.num_normals_and_offsets;
    this->neighbors = other.neighbors;
    this->nneighbors_ = other.nneighbors_;
    this->center = other.center;
    this->support_warm_starts = other.support_warm_starts;
  }

  return *this;
}

template <typename IndexType>
template <typename OtherIndexType>
void ConvexBaseTpl<IndexType>::deepcopy(const ConvexBaseTpl<IndexType>* source,
                                        ConvexBaseTpl<OtherIndexType>* copy) {
  if (source == nullptr || copy == nullptr) {
    return;
  }

  // Copy the base
  copy->base() = source->base();

  // Copy the non-templated data
  if (source->points != nullptr) {
    copy->points.reset(new std::vector<Vec3s>(*source->points));
  } else {
    copy->points.reset();
    assert(source->num_points == 0);
  }
  copy->num_points = source->num_points;

  if (source->normals != nullptr) {
    copy->normals.reset(new std::vector<Vec3s>(*source->normals));
  } else {
    copy->normals.reset();
    assert(source->num_normals_and_offsets == 0);
  }
  if (source->offsets != nullptr) {
    copy->offsets.reset(new std::vector<Scalar>(*source->offsets));
  } else {
    copy->offsets.reset();
    assert(source->num_normals_and_offsets == 0);
  }
  copy->num_normals_and_offsets = source->num_normals_and_offsets;

  copy->center = source->center;
  copy->support_warm_starts =
      source->support_warm_starts.template cast<OtherIndexType>();

  // Convert neighbors to new type
  if (source->points->size() >=
      (std::size_t)(std::numeric_limits<OtherIndexType>::max())) {
    COAL_THROW_PRETTY(
        "The source has more points than the max of OtherIndexType.",
        std::runtime_error);
  }

  if (source->nneighbors_ != nullptr) {
    const std::vector<IndexType>& source_nneighbors = *(source->nneighbors_);
    copy->nneighbors_.reset(
        new std::vector<OtherIndexType>(source_nneighbors.size()));
    std::vector<OtherIndexType>& copy_nneighbors = *(copy->nneighbors_);
    for (std::size_t i = 0; i < source_nneighbors.size(); ++i) {
      copy_nneighbors[i] = OtherIndexType(source_nneighbors[i]);
    }
  } else {
    copy->nneighbors_.reset();
  }

  if (source->neighbors != nullptr) {
    typedef typename ConvexBaseTpl<OtherIndexType>::Neighbors OtherNeighbors;
    assert(source->neighbors->size() == source->points->size());
    const std::vector<Neighbors>& source_neighbors = *(source->neighbors);
    copy->neighbors.reset(
        new std::vector<OtherNeighbors>(source_neighbors.size()));
    std::vector<OtherNeighbors>& copy_neighbors = *(copy->neighbors);
    for (std::size_t i = 0; i < source_neighbors.size(); ++i) {
      copy_neighbors[i].count = source_neighbors[i].count;
      copy_neighbors[i].begin_id = OtherIndexType(source_neighbors[i].begin_id);
    }
  } else {
    copy->neighbors.reset();
  }
}

template <typename IndexType>
void ConvexBaseTpl<IndexType>::computeCenter() {
  center.setZero();
  const std::vector<Vec3s>& points_ = *points;
  for (std::size_t i = 0; i < num_points; ++i)
    center += points_[i];  // TODO(jcarpent): vectorization
  center /= Scalar(num_points);
}

// forward declaration for ConvexBase
template <typename BV, typename S>
void computeBV(const S& s, const Transform3s& tf, BV& bv);

template <typename IndexType>
void ConvexBaseTpl<IndexType>::computeLocalAABB() {
  computeBV<AABB, ConvexBaseTpl<IndexType>>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

// Reorders `tri` such that the dot product between the normal of triangle and
// the vector `triangle barycentre - convex_tri.center` is positive.
template <typename IndexType>
void reorderTriangle(const ConvexTpl<TriangleTpl<IndexType>>* convex_tri,
                     TriangleTpl<IndexType>& tri) {
  Vec3s p0, p1, p2;
  p0 = (*(convex_tri->points))[tri[0]];
  p1 = (*(convex_tri->points))[tri[1]];
  p2 = (*(convex_tri->points))[tri[2]];

  Vec3s barycentre_tri, center_barycenter;
  barycentre_tri = (p0 + p1 + p2) / 3;
  center_barycenter = barycentre_tri - convex_tri->center;

  Vec3s edge_tri1, edge_tri2, n_tri;
  edge_tri1 = p1 - p0;
  edge_tri2 = p2 - p1;
  n_tri = edge_tri1.cross(edge_tri2);

  if (center_barycenter.dot(n_tri) < 0) {
    tri.set(tri[1], tri[0], tri[2]);
  }
}

}  // namespace coal

#endif  // COAL_GEOMETRIC_SHAPES_HXX
