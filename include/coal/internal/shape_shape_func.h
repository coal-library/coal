/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CNRS-LAAS
 *  Copyright (c) 2024, INRIA
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Florent Lamiraux */

#ifndef COAL_INTERNAL_SHAPE_SHAPE_FUNC_H
#define COAL_INTERNAL_SHAPE_SHAPE_FUNC_H

/// @cond INTERNAL

#include "coal/collision_data.h"
#include "coal/collision_utility.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes_traits.h"

#include "coal/tracy.hh"

namespace coal {

template <typename ShapeType1, typename ShapeType2>
struct ShapeShapeDistancer {
  static Scalar run(const CollisionGeometry* o1, const Transform3s& tf1,
                    const CollisionGeometry* o2, const Transform3s& tf2,
                    const GJKSolver* nsolver, const DistanceRequest& request,
                    DistanceResult& result) {
    if (request.isSatisfied(result)) return result.min_distance;

    // Witness points on shape1 and shape2, normal pointing from shape1 to
    // shape2.
    Vec3s p1, p2, normal;
    const Scalar distance = ShapeShapeDistancer<ShapeType1, ShapeType2>::run(
        o1, tf1, o2, tf2, nsolver, request.enable_signed_distance, p1, p2,
        normal);

    result.update(distance, o1, o2, DistanceResult::NONE, DistanceResult::NONE,
                  p1, p2, normal);

    return distance;
  }

  static Scalar run(const CollisionGeometry* o1, const Transform3s& tf1,
                    const CollisionGeometry* o2, const Transform3s& tf2,
                    const GJKSolver* nsolver,
                    const bool compute_signed_distance, Vec3s& p1, Vec3s& p2,
                    Vec3s& normal) {
    const ShapeType1* obj1 = static_cast<const ShapeType1*>(o1);
    const ShapeType2* obj2 = static_cast<const ShapeType2*>(o2);
    return nsolver->shapeDistance(*obj1, tf1, *obj2, tf2,
                                  compute_signed_distance, p1, p2, normal);
  }
};

/// @brief Shape-shape distance computation.
/// Assumes that `nsolver` has already been set up by a `DistanceRequest` or
/// a `CollisionRequest`.
///
/// @note This function is typically used for collision pairs containing two
/// primitive shapes.
/// @note This function might be specialized for some pairs of shapes.
template <typename ShapeType1, typename ShapeType2>
Scalar ShapeShapeDistance(const CollisionGeometry* o1, const Transform3s& tf1,
                          const CollisionGeometry* o2, const Transform3s& tf2,
                          const GJKSolver* nsolver,
                          const DistanceRequest& request,
                          DistanceResult& result) {
  COAL_TRACY_ZONE_SCOPED_N("coal::ShapeShapeDistance");
  return ShapeShapeDistancer<ShapeType1, ShapeType2>::run(
      o1, tf1, o2, tf2, nsolver, request, result);
}

namespace internal {
/// @brief Shape-shape distance computation.
/// Assumes that `nsolver` has already been set up by a `DistanceRequest` or
/// a `CollisionRequest`.
///
/// @note This function is typically used for collision pairs complex structures
/// like BVHs, HeightFields or Octrees. These structures contain sets of
/// primitive shapes.
/// This function is meant to be called on the pairs of primitive shapes of
/// these structures.
/// @note This function might be specialized for some pairs of shapes.
template <typename ShapeType1, typename ShapeType2>
Scalar ShapeShapeDistance(const CollisionGeometry* o1, const Transform3s& tf1,
                          const CollisionGeometry* o2, const Transform3s& tf2,
                          const GJKSolver* nsolver,
                          const bool compute_signed_distance, Vec3s& p1,
                          Vec3s& p2, Vec3s& normal) {
  return ::coal::ShapeShapeDistancer<ShapeType1, ShapeType2>::run(
      o1, tf1, o2, tf2, nsolver, compute_signed_distance, p1, p2, normal);
}
}  // namespace internal

/// @brief Shape-shape collision detection.
/// Assumes that `nsolver` has already been set up by a `DistanceRequest` or
/// a `CollisionRequest`.
///
/// @note This function is typically used for collision pairs containing two
/// primitive shapes.
/// Complex structures like BVHs, HeightFields or Octrees contain sets of
/// primitive shapes should use the `ShapeShapeDistance` function to do their
/// internal collision detection checks.
template <typename ShapeType1, typename ShapeType2>
struct ShapeShapeCollider {
  static std::size_t run(const CollisionGeometry* o1, const Transform3s& tf1,
                         const CollisionGeometry* o2, const Transform3s& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result) {
    if (request.isSatisfied(result)) return result.numContacts();

    const bool compute_penetration =
        request.enable_contact || (request.security_margin < 0);
    Vec3s p1, p2, normal;
    Scalar distance = internal::ShapeShapeDistance<ShapeType1, ShapeType2>(
        o1, tf1, o2, tf2, nsolver, compute_penetration, p1, p2, normal);

    size_t num_contacts = 0;
    const Scalar distToCollision = distance - request.security_margin;

    internal::updateDistanceLowerBoundFromLeaf(request, result, distToCollision,
                                               p1, p2, normal);
    if (distToCollision <= request.collision_distance_threshold &&
        result.numContacts() < request.num_max_contacts) {
      if (result.numContacts() < request.num_max_contacts) {
        Contact contact(o1, o2, Contact::NONE, Contact::NONE, p1, p2, normal,
                        distance);
        result.addContact(contact);
      }
      num_contacts = result.numContacts();
    }

    return num_contacts;
  }
};

template <typename ShapeType1, typename ShapeType2>
std::size_t ShapeShapeCollide(const CollisionGeometry* o1,
                              const Transform3s& tf1,
                              const CollisionGeometry* o2,
                              const Transform3s& tf2, const GJKSolver* nsolver,
                              const CollisionRequest& request,
                              CollisionResult& result) {
  COAL_TRACY_ZONE_SCOPED_N("coal::ShapeShapeCollide");
  return ShapeShapeCollider<ShapeType1, ShapeType2>::run(
      o1, tf1, o2, tf2, nsolver, request, result);
}

// clang-format off
// ==============================================================================================================
// ==============================================================================================================
// ==============================================================================================================
// Shape distance algorithms based on:
// - built-in function: 0
// - GJK:               1
//
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// |            | box | sphere | capsule | cone | cylinder | plane | half-space | triangle | ellipsoid | convex |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | box        |  1  |   0    |    1    |   1  |    1     |   0   |      0     |    1     |    1      |    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | sphere     |/////|   0    |    0    |   1  |    0     |   0   |      0     |    0     |    1      |    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | capsule    |/////|////////|    0    |   1  |    1     |   0   |      0     |    1     |    1      |    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | cone       |/////|////////|/////////|   1  |    1     |   0   |      0     |    1     |    1      |    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | cylinder   |/////|////////|/////////|//////|    1     |   0   |      0     |    1     |    1      |    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | plane      |/////|////////|/////////|//////|//////////|   ?   |      ?     |    0     |    0      |    0   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | half-space |/////|////////|/////////|//////|//////////|///////|      ?     |    0     |    0      |    0   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | triangle   |/////|////////|/////////|//////|//////////|///////|////////////|    0     |    1      |    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | ellipsoid  |/////|////////|/////////|//////|//////////|///////|////////////|//////////|    1      |    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
// | convex     |/////|////////|/////////|//////|//////////|///////|////////////|//////////|///////////|    1   |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+-----------+--------+
//
// Number of pairs: 55
//   - Specialized: 26
//   - GJK:         29
// clang-format on

#define SHAPE_SHAPE_DISTANCE_SPECIALIZATION(T1, T2)                            \
  template <>                                                                  \
  COAL_DLLAPI Scalar internal::ShapeShapeDistance<T1, T2>(                     \
      const CollisionGeometry* o1, const Transform3s& tf1,                     \
      const CollisionGeometry* o2, const Transform3s& tf2,                     \
      const GJKSolver* nsolver, const bool compute_signed_distance, Vec3s& p1, \
      Vec3s& p2, Vec3s& normal);                                               \
  template <>                                                                  \
  COAL_DLLAPI Scalar internal::ShapeShapeDistance<T2, T1>(                     \
      const CollisionGeometry* o1, const Transform3s& tf1,                     \
      const CollisionGeometry* o2, const Transform3s& tf2,                     \
      const GJKSolver* nsolver, const bool compute_signed_distance, Vec3s& p1, \
      Vec3s& p2, Vec3s& normal);                                               \
  template <>                                                                  \
  inline COAL_DLLAPI Scalar ShapeShapeDistance<T1, T2>(                        \
      const CollisionGeometry* o1, const Transform3s& tf1,                     \
      const CollisionGeometry* o2, const Transform3s& tf2,                     \
      const GJKSolver* nsolver, const DistanceRequest& request,                \
      DistanceResult& result) {                                                \
    result.o1 = o1;                                                            \
    result.o2 = o2;                                                            \
    result.b1 = DistanceResult::NONE;                                          \
    result.b2 = DistanceResult::NONE;                                          \
    result.min_distance = internal::ShapeShapeDistance<T1, T2>(                \
        o1, tf1, o2, tf2, nsolver, request.enable_signed_distance,             \
        result.nearest_points[0], result.nearest_points[1], result.normal);    \
    return result.min_distance;                                                \
  }                                                                            \
  template <>                                                                  \
  inline COAL_DLLAPI Scalar ShapeShapeDistance<T2, T1>(                        \
      const CollisionGeometry* o1, const Transform3s& tf1,                     \
      const CollisionGeometry* o2, const Transform3s& tf2,                     \
      const GJKSolver* nsolver, const DistanceRequest& request,                \
      DistanceResult& result) {                                                \
    result.o1 = o1;                                                            \
    result.o2 = o2;                                                            \
    result.b1 = DistanceResult::NONE;                                          \
    result.b2 = DistanceResult::NONE;                                          \
    result.min_distance = internal::ShapeShapeDistance<T2, T1>(                \
        o1, tf1, o2, tf2, nsolver, request.enable_signed_distance,             \
        result.nearest_points[0], result.nearest_points[1], result.normal);    \
    return result.min_distance;                                                \
  }

#define SHAPE_SELF_DISTANCE_SPECIALIZATION(T)                                  \
  template <>                                                                  \
  COAL_DLLAPI Scalar internal::ShapeShapeDistance<T, T>(                       \
      const CollisionGeometry* o1, const Transform3s& tf1,                     \
      const CollisionGeometry* o2, const Transform3s& tf2,                     \
      const GJKSolver* nsolver, const bool compute_signed_distance, Vec3s& p1, \
      Vec3s& p2, Vec3s& normal);                                               \
  template <>                                                                  \
  inline COAL_DLLAPI Scalar ShapeShapeDistance<T, T>(                          \
      const CollisionGeometry* o1, const Transform3s& tf1,                     \
      const CollisionGeometry* o2, const Transform3s& tf2,                     \
      const GJKSolver* nsolver, const DistanceRequest& request,                \
      DistanceResult& result) {                                                \
    result.o1 = o1;                                                            \
    result.o2 = o2;                                                            \
    result.b1 = DistanceResult::NONE;                                          \
    result.b2 = DistanceResult::NONE;                                          \
    result.min_distance = internal::ShapeShapeDistance<T, T>(                  \
        o1, tf1, o2, tf2, nsolver, request.enable_signed_distance,             \
        result.nearest_points[0], result.nearest_points[1], result.normal);    \
    return result.min_distance;                                                \
  }

SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Plane)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Sphere)
SHAPE_SELF_DISTANCE_SPECIALIZATION(Capsule)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Plane)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cone, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cone, Plane)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cylinder, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cylinder, Plane)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Plane)
SHAPE_SELF_DISTANCE_SPECIALIZATION(Sphere)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Cylinder)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Capsule)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Ellipsoid, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Ellipsoid, Plane)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(ConvexBase16, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(ConvexBase32, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(ConvexBase16, Plane)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(ConvexBase32, Plane)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(TriangleP, Halfspace)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(TriangleP, Plane)
SHAPE_SELF_DISTANCE_SPECIALIZATION(TriangleP)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(TriangleP, Sphere)
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Plane, Halfspace)
SHAPE_SELF_DISTANCE_SPECIALIZATION(Plane)
SHAPE_SELF_DISTANCE_SPECIALIZATION(Halfspace)

#undef SHAPE_SHAPE_DISTANCE_SPECIALIZATION
#undef SHAPE_SELF_DISTANCE_SPECIALIZATION

}  // namespace coal

/// @endcond

#endif
