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

#ifndef COAL_SHAPE_CONVEX_H
#define COAL_SHAPE_CONVEX_H

#include "coal/shape/geometric_shapes.h"

namespace coal {

/// @brief Convex polytope
/// @tparam PolygonT the polygon class. It must have method \c size() and
///         \c operator[](int i)
template <typename PolygonT>
class Convex : public ConvexBase {
 public:
  /// @brief Construct an uninitialized convex object
  Convex() : ConvexBase(), num_polygons(0) {}

  /// @brief Constructing a convex, providing normal and offset of each polytype
  /// surface, and the points and shape topology information \param ownStorage
  /// whether this class owns the pointers of points and
  ///                    polygons. If owned, they are deleted upon destruction.
  /// \param points_ list of 3D points
  /// \param num_points_ number of 3D points
  /// \param polygons_ \copydoc Convex::polygons
  /// \param num_polygons_ the number of polygons.
  /// \note num_polygons_ is not the allocated size of polygons_.
  Convex(std::shared_ptr<std::vector<Vec3s>> points_, unsigned int num_points_,
         std::shared_ptr<std::vector<PolygonT>> polygons_,
         unsigned int num_polygons_);

  /// @brief Copy constructor
  /// Only the list of neighbors is copied.
  Convex(const Convex& other);

  ~Convex();

  /// based on http://number-none.com/blow/inertia/bb_inertia.doc
  Matrix3s computeMomentofInertia() const;

  Vec3s computeCOM() const;

  CoalScalar computeVolume() const;

  ///
  /// @brief Set the current Convex from a list of points and polygons.
  ///
  /// \param ownStorage whether this class owns the pointers of points and
  ///                    polygons. If owned, they are deleted upon destruction.
  /// \param points list of 3D points
  /// \param num_points number of 3D points
  /// \param polygons \copydoc Convex::polygons
  /// \param num_polygons the number of polygons.
  /// \note num_polygons is not the allocated size of polygons.
  ///
  void set(std::shared_ptr<std::vector<Vec3s>> points, unsigned int num_points,
           std::shared_ptr<std::vector<PolygonT>> polygons,
           unsigned int num_polygons);

  /// @brief Clone (deep copy)
  virtual Convex<PolygonT>* clone() const;

  /// @brief An array of PolygonT object.
  /// PolygonT should contains a list of vertices for each polygon,
  /// in counter clockwise order.
  std::shared_ptr<std::vector<PolygonT>> polygons;
  unsigned int num_polygons;

 protected:
  void fillNeighbors();
};

}  // namespace coal

#include "coal/shape/details/convex.hxx"

#endif
