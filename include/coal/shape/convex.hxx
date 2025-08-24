/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CNRS - LAAS
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

/** \author Joseph Mirabel */

#ifndef COAL_SHAPE_CONVEX_HXX
#define COAL_SHAPE_CONVEX_HXX

#include <set>
#include <vector>
#include <iostream>

#include "coal/shape/convex.h"

namespace coal {

template <typename PolygonT>
ConvexTpl<PolygonT>::ConvexTpl(std::shared_ptr<std::vector<Vec3s>> points_,
                               unsigned int num_points_,
                               std::shared_ptr<std::vector<PolygonT>> polygons_,
                               unsigned int num_polygons_)
    : Base(), polygons(polygons_), num_polygons(num_polygons_) {
  this->initialize(points_, num_points_);
  this->fillNeighbors();
  this->buildSupportWarmStart();
}

template <typename PolygonT>
ConvexTpl<PolygonT>& ConvexTpl<PolygonT>::operator=(const ConvexTpl& other) {
  if (this != &other) {
    // Copy the base
    this->base() = other.base();

    // Shallow copy the polygons
    this->num_polygons = other.num_polygons;
    this->polygons = other.polygons;
  }

  return *this;
}

template <typename PolygonT>
template <typename OtherPolygonT>
void ConvexTpl<PolygonT>::deepcopy(const ConvexTpl<PolygonT>* source,
                                   ConvexTpl<OtherPolygonT>* copy) {
  if (source == nullptr || copy == nullptr) {
    return;
  }

  // Deep copy the base
  Base::deepcopy(source, copy);

  // Deep copy the polygons
  typedef typename OtherPolygonT::IndexType OtherIndexType;
  copy->num_polygons = source->num_polygons;
  if (source->polygons != nullptr) {
    const std::vector<PolygonT>& source_polygons = *(source->polygons);
    copy->polygons.reset(
        new std::vector<OtherPolygonT>(source_polygons.size()));
    std::vector<OtherPolygonT>& copy_polygons = *(copy->polygons);
    for (std::size_t i = 0; i < source_polygons.size(); ++i) {
      copy_polygons[i] = source_polygons[i].template cast<OtherIndexType>();
    }
  } else {
    copy->polygons.reset();
  }
}

template <typename PolygonT>
void ConvexTpl<PolygonT>::set(std::shared_ptr<std::vector<Vec3s>> points_,
                              unsigned int num_points_,
                              std::shared_ptr<std::vector<PolygonT>> polygons_,
                              unsigned int num_polygons_) {
  Base::set(points_, num_points_);

  this->num_polygons = num_polygons_;
  this->polygons = polygons_;

  this->fillNeighbors();
  this->buildSupportWarmStart();
}

template <typename PolygonT>
Matrix3s ConvexTpl<PolygonT>::computeMomentofInertia() const {
  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::IndexType IndexType;

  Matrix3s C = Matrix3s::Zero();

  Matrix3s C_canonical;
  C_canonical << Scalar(1 / 60.0),  //
      Scalar(1 / 120.0),            //
      Scalar(1 / 120.0),            //
      Scalar(1 / 120.0),            //
      Scalar(1 / 60.0),             //
      Scalar(1 / 120.0),            //
      Scalar(1 / 120.0),            //
      Scalar(1 / 120.0),            //
      Scalar(1 / 60.0);

  if (!(points.get())) {
    std::cerr << "Error in `ConvexTpl::computeMomentofInertia`! ConvexTpl has "
                 "no vertices."
              << std::endl;
    return C;
  }
  const std::vector<Vec3s>& points_ = *points;
  if (!(polygons.get())) {
    std::cerr << "Error in `ConvexTpl::computeMomentofInertia`! ConvexTpl has "
                 "no polygons."
              << std::endl;
    return C;
  }
  const std::vector<PolygonT>& polygons_ = *polygons;
  for (unsigned int i = 0; i < num_polygons; ++i) {
    const PolygonT& polygon = polygons_[i];

    // compute the center of the polygon
    Vec3s plane_center(0, 0, 0);
    for (size_type j = 0; j < polygon.size(); ++j)
      plane_center += points_[polygon[(IndexType)j]];
    plane_center /= Scalar(polygon.size());

    // compute the volume of tetrahedron making by neighboring two points, the
    // plane center and the reference point (zero) of the convex shape
    const Vec3s& v3 = plane_center;
    for (size_type j = 0; j < polygon.size(); ++j) {
      IndexType e_first = polygon[static_cast<IndexType>(j)];
      IndexType e_second =
          polygon[static_cast<IndexType>((j + 1) % polygon.size())];
      const Vec3s& v1 = points_[e_first];
      const Vec3s& v2 = points_[e_second];
      Matrix3s A;
      A << v1.transpose(), v2.transpose(),
          v3.transpose();  // this is A' in the original document
      C += A.transpose() * C_canonical * A * (v1.cross(v2)).dot(v3);
    }
  }

  return C.trace() * Matrix3s::Identity() - C;
}

template <typename PolygonT>
Vec3s ConvexTpl<PolygonT>::computeCOM() const {
  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::IndexType IndexType;

  Vec3s com(0, 0, 0);
  Scalar vol = 0;
  if (!(points.get())) {
    std::cerr << "Error in `ConvexTpl::computeCOM`! ConvexTpl has no vertices."
              << std::endl;
    return com;
  }
  const std::vector<Vec3s>& points_ = *points;
  if (!(polygons.get())) {
    std::cerr << "Error in `ConvexTpl::computeCOM`! ConvexTpl has no polygons."
              << std::endl;
    return com;
  }
  const std::vector<PolygonT>& polygons_ = *polygons;
  for (unsigned int i = 0; i < num_polygons; ++i) {
    const PolygonT& polygon = polygons_[i];
    // compute the center of the polygon
    Vec3s plane_center(0, 0, 0);
    for (size_type j = 0; j < polygon.size(); ++j)
      plane_center += points_[polygon[(IndexType)j]];
    plane_center /= Scalar(polygon.size());

    // compute the volume of tetrahedron making by neighboring two points, the
    // plane center and the reference point (zero) of the convex shape
    const Vec3s& v3 = plane_center;
    for (size_type j = 0; j < polygon.size(); ++j) {
      IndexType e_first = polygon[static_cast<IndexType>(j)];
      IndexType e_second =
          polygon[static_cast<IndexType>((j + 1) % polygon.size())];
      const Vec3s& v1 = points_[e_first];
      const Vec3s& v2 = points_[e_second];
      Scalar d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
      com += (points_[e_first] + points_[e_second] + plane_center) * d_six_vol;
    }
  }

  return com / (vol * 4);  // here we choose zero as the reference
}

template <typename PolygonT>
Scalar ConvexTpl<PolygonT>::computeVolume() const {
  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::IndexType IndexType;

  Scalar vol = 0;
  if (!(points.get())) {
    std::cerr
        << "Error in `ConvexTpl::computeVolume`! ConvexTpl has no vertices."
        << std::endl;
    return vol;
  }
  const std::vector<Vec3s>& points_ = *points;
  if (!(polygons.get())) {
    std::cerr
        << "Error in `ConvexTpl::computeVolume`! ConvexTpl has no polygons."
        << std::endl;
    return vol;
  }
  const std::vector<PolygonT>& polygons_ = *polygons;
  for (unsigned int i = 0; i < num_polygons; ++i) {
    const PolygonT& polygon = polygons_[i];

    // compute the center of the polygon
    Vec3s plane_center(0, 0, 0);
    for (size_type j = 0; j < polygon.size(); ++j)
      plane_center += points_[polygon[(IndexType)j]];
    plane_center /= Scalar(polygon.size());

    // compute the volume of tetrahedron making by neighboring two points, the
    // plane center and the reference point (zero point) of the convex shape
    const Vec3s& v3 = plane_center;
    for (size_type j = 0; j < polygon.size(); ++j) {
      IndexType e_first = polygon[static_cast<IndexType>(j)];
      IndexType e_second =
          polygon[static_cast<IndexType>((j + 1) % polygon.size())];
      const Vec3s& v1 = points_[e_first];
      const Vec3s& v2 = points_[e_second];
      Scalar d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
    }
  }

  return vol / 6;
}

template <typename PolygonT>
void ConvexTpl<PolygonT>::fillNeighbors() {
  neighbors.reset(new std::vector<Neighbors>(num_points));

  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::IndexType IndexType;

  std::vector<std::set<IndexType>> nneighbors(num_points);
  unsigned int c_nneighbors = 0;

  if (!(polygons.get())) {
    std::cerr
        << "Error in `ConvexTpl::fillNeighbors`! ConvexTpl has no polygons."
        << std::endl;
  }
  const std::vector<PolygonT>& polygons_ = *polygons;
  for (unsigned int l = 0; l < num_polygons; ++l) {
    const PolygonT& polygon = polygons_[l];
    const size_type n = polygon.size();

    for (size_type j = 0; j < polygon.size(); ++j) {
      size_type i = (j == 0) ? n - 1 : j - 1;
      size_type k = (j == n - 1) ? 0 : j + 1;
      IndexType pi = polygon[(IndexType)i], pj = polygon[(IndexType)j],
                pk = polygon[(IndexType)k];
      // Update neighbors of pj;
      if (nneighbors[pj].count(pi) == 0) {
        c_nneighbors++;
        nneighbors[pj].insert(pi);
      }
      if (nneighbors[pj].count(pk) == 0) {
        c_nneighbors++;
        nneighbors[pj].insert(pk);
      }
    }
  }

  nneighbors_.reset(new std::vector<IndexType>(c_nneighbors));

  std::vector<Neighbors>& neighbors_ = *neighbors;
  std::vector<IndexType>& nneighbors__ = *(nneighbors_);
  IndexType begin_id = 0;
  for (unsigned int i = 0; i < num_points; ++i) {
    Neighbors& n = neighbors_[i];
    if (nneighbors[i].size() >= (std::numeric_limits<unsigned char>::max)())
      COAL_THROW_PRETTY("Too many neighbors.", std::logic_error);
    n.count = (unsigned char)nneighbors[i].size();
    n.begin_id = begin_id;
    IndexType j = 0;
    for (IndexType idx : nneighbors[i]) {
      nneighbors__[n.begin_id + j] = idx;
      j++;
    }
    begin_id += n.count;
  }
}

}  // namespace coal

#endif
