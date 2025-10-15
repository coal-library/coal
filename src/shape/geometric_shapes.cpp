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

#include "coal/shape/geometric_shapes.h"
#include "coal/shape/geometric_shapes_utility.h"

namespace coal {

void ConvexBase::initialize(std::shared_ptr<std::vector<Vec3s>> points_,
                            unsigned int num_points_) {
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

void ConvexBase::set(std::shared_ptr<std::vector<Vec3s>> points_,
                     unsigned int num_points_) {
  initialize(points_, num_points_);
}

ConvexBase::ConvexBase(const ConvexBase& other)
    : ShapeBase(other),
      num_points(other.num_points),
      num_normals_and_offsets(other.num_normals_and_offsets),
      center(other.center) {
  if (other.points.get() && other.points->size() > 0) {
    // Deep copy of other points
    points.reset(new std::vector<Vec3s>(*other.points));
  } else
    points.reset();

  if (other.nneighbors_.get() && other.nneighbors_->size() > 0) {
    // Deep copy the list of all the neighbors of all the points
    nneighbors_.reset(new std::vector<unsigned int>(*(other.nneighbors_)));
    if (other.neighbors.get() && other.neighbors->size() > 0) {
      // Fill each neighbors for each point in the Convex object.
      neighbors.reset(new std::vector<Neighbors>(other.neighbors->size()));
      assert(neighbors->size() == points->size());
      unsigned int* p_nneighbors = nneighbors_->data();

      std::vector<Neighbors>& neighbors_ = *neighbors;
      const std::vector<Neighbors>& other_neighbors_ = *(other.neighbors);
      for (size_t i = 0; i < neighbors->size(); ++i) {
        Neighbors& n = neighbors_[i];
        n.count_ = other_neighbors_[i].count_;
        n.n_ = p_nneighbors;
        p_nneighbors += n.count_;
      }
    } else
      neighbors.reset();
  } else
    nneighbors_.reset();

  if (other.normals.get() && other.normals->size() > 0) {
    normals.reset(new std::vector<Vec3s>(*(other.normals)));
  } else
    normals.reset();

  if (other.offsets.get() && other.offsets->size() > 0) {
    offsets.reset(new std::vector<double>(*(other.offsets)));
  } else
    offsets.reset();

  support_warm_starts = other.support_warm_starts;

  ShapeBase::operator=(*this);
}

ConvexBase::~ConvexBase() {}

void ConvexBase::computeCenter() {
  center.setZero();
  const std::vector<Vec3s>& points_ = *points;
  for (std::size_t i = 0; i < num_points; ++i)
    center += points_[i];  // TODO(jcarpent): vectorization
  center /= num_points;
}

void Halfspace::unitNormalTest() {
  CoalScalar l = n.norm();
  if (l > 0) {
    CoalScalar inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  } else {
    n << 1, 0, 0;
    d = 0;
  }
}

void Plane::unitNormalTest() {
  CoalScalar l = n.norm();
  if (l > 0) {
    CoalScalar inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  } else {
    n << 1, 0, 0;
    d = 0;
  }
}

void Box::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Sphere::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = radius;
}

void Ellipsoid::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Capsule::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cone::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cylinder::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void ConvexBase::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Halfspace::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Plane::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void TriangleP::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const CoalScalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

}  // namespace coal
