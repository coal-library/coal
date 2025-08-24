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

#ifndef COAL_COLLISION_OBJECT_BASE_H
#define COAL_COLLISION_OBJECT_BASE_H

#include <limits>
#include <typeinfo>

#include "coal/deprecated.hh"
#include "coal/fwd.hh"
#include "coal/BV/AABB.h"
#include "coal/math/transform.h"

namespace coal {

/// @brief object type: BVH (mesh, points), basic geometry, octree
enum OBJECT_TYPE {
  OT_UNKNOWN,
  OT_BVH,
  OT_GEOM,
  OT_OCTREE,
  OT_HFIELD,
  OT_COUNT
};

/// @brief traversal node type: bounding volume (AABB, OBB, RSS, kIOS, OBBRSS,
/// KDOP16, KDOP18, kDOP24), basic shape (box, sphere, ellipsoid, capsule, cone,
/// cylinder, convex, plane, triangle), and octree
enum NODE_TYPE {
  BV_UNKNOWN,
  BV_AABB,
  BV_OBB,
  BV_RSS,
  BV_kIOS,
  BV_OBBRSS,
  BV_KDOP16,
  BV_KDOP18,
  BV_KDOP24,
  GEOM_BOX,
  GEOM_SPHERE,
  GEOM_CAPSULE,
  GEOM_CONE,
  GEOM_CYLINDER,
  GEOM_CONVEX16,
  GEOM_CONVEX32,
  GEOM_CONVEX = GEOM_CONVEX32,
  GEOM_PLANE,
  GEOM_HALFSPACE,
  GEOM_TRIANGLE,
  GEOM_OCTREE,
  GEOM_ELLIPSOID,
  HF_AABB,
  HF_OBBRSS,
  NODE_COUNT
};

/// @addtogroup Construction_Of_BVH
/// @{

/// @brief The geometry for the object for collision or distance computation
class COAL_DLLAPI CollisionGeometry {
 public:
  CollisionGeometry()
      : aabb_center(Vec3s::Constant((std::numeric_limits<Scalar>::max)())),
        aabb_radius(-1),
        cost_density(1),
        threshold_occupied(1),
        threshold_free(0) {}

  /// \brief Copy constructor
  CollisionGeometry(const CollisionGeometry& other) = default;

  virtual ~CollisionGeometry() {}

  /// @brief Clone *this into a new CollisionGeometry
  virtual CollisionGeometry* clone() const = 0;

  /// \brief Equality operator
  bool operator==(const CollisionGeometry& other) const {
    return cost_density == other.cost_density &&
           threshold_occupied == other.threshold_occupied &&
           threshold_free == other.threshold_free &&
           aabb_center == other.aabb_center &&
           aabb_radius == other.aabb_radius && aabb_local == other.aabb_local &&
           isEqual(other);
  }

  /// \brief Difference operator
  bool operator!=(const CollisionGeometry& other) const {
    return isNotEqual(other);
  }

  /// @brief get the type of the object
  virtual OBJECT_TYPE getObjectType() const { return OT_UNKNOWN; }

  /// @brief get the node type
  virtual NODE_TYPE getNodeType() const { return BV_UNKNOWN; }

  /// @brief compute the AABB for object in local coordinate
  virtual void computeLocalAABB() = 0;

  /// @brief get user data in geometry
  void* getUserData() const { return user_data; }

  /// @brief set user data in geometry
  void setUserData(void* data) { user_data = data; }

  /// @brief whether the object is completely occupied
  inline bool isOccupied() const { return cost_density >= threshold_occupied; }

  /// @brief whether the object is completely free
  inline bool isFree() const { return cost_density <= threshold_free; }

  /// @brief whether the object has some uncertainty
  bool isUncertain() const;

  /// @brief AABB center in local coordinate
  Vec3s aabb_center;

  /// @brief AABB radius
  Scalar aabb_radius;

  /// @brief AABB in local coordinate, used for tight AABB when only translation
  /// transform
  AABB aabb_local;

  /// @brief pointer to user defined data specific to this object
  void* user_data;

  /// @brief collision cost for unit volume
  Scalar cost_density;

  /// @brief threshold for occupied ( >= is occupied)
  Scalar threshold_occupied;

  /// @brief threshold for free (<= is free)
  Scalar threshold_free;

  /// @brief compute center of mass
  virtual Vec3s computeCOM() const { return Vec3s::Zero(); }

  /// @brief compute the inertia matrix, related to the origin
  virtual Matrix3s computeMomentofInertia() const {
    return Matrix3s::Constant(NAN);
  }

  /// @brief compute the volume
  virtual Scalar computeVolume() const { return 0; }

  /// @brief compute the inertia matrix, related to the com
  virtual Matrix3s computeMomentofInertiaRelatedToCOM() const {
    Matrix3s C = computeMomentofInertia();
    Vec3s com = computeCOM();
    Scalar V = computeVolume();

    return (Matrix3s() << C(0, 0) - V * (com[1] * com[1] + com[2] * com[2]),
            C(0, 1) + V * com[0] * com[1], C(0, 2) + V * com[0] * com[2],
            C(1, 0) + V * com[1] * com[0],
            C(1, 1) - V * (com[0] * com[0] + com[2] * com[2]),
            C(1, 2) + V * com[1] * com[2], C(2, 0) + V * com[2] * com[0],
            C(2, 1) + V * com[2] * com[1],
            C(2, 2) - V * (com[0] * com[0] + com[1] * com[1]))
        .finished();
  }

 private:
  /// @brief equal operator with another object of derived type.
  virtual bool isEqual(const CollisionGeometry& other) const = 0;

  /// @brief not equal operator with another object of derived type.
  virtual bool isNotEqual(const CollisionGeometry& other) const {
    return !(*this == other);
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief the object for collision or distance computation, contains the
/// geometry and the transform information
class COAL_DLLAPI CollisionObject {
 public:
  CollisionObject(const shared_ptr<CollisionGeometry>& cgeom_,
                  bool compute_local_aabb = true)
      : cgeom(cgeom_), user_data(nullptr) {
    init(compute_local_aabb);
  }

  CollisionObject(const shared_ptr<CollisionGeometry>& cgeom_,
                  const Transform3s& tf, bool compute_local_aabb = true)
      : cgeom(cgeom_), t(tf), user_data(nullptr) {
    init(compute_local_aabb);
  }

  CollisionObject(const shared_ptr<CollisionGeometry>& cgeom_,
                  const Matrix3s& R, const Vec3s& T,
                  bool compute_local_aabb = true)
      : cgeom(cgeom_), t(R, T), user_data(nullptr) {
    init(compute_local_aabb);
  }

  bool operator==(const CollisionObject& other) const {
    return cgeom == other.cgeom && t == other.t && user_data == other.user_data;
  }

  bool operator!=(const CollisionObject& other) const {
    return !(*this == other);
  }

  ~CollisionObject() {}

  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const { return cgeom->getObjectType(); }

  /// @brief get the node type
  NODE_TYPE getNodeType() const { return cgeom->getNodeType(); }

  /// @brief get the AABB in world space
  const AABB& getAABB() const { return aabb; }

  /// @brief get the AABB in world space
  AABB& getAABB() { return aabb; }

  /// @brief compute the AABB in world space
  void computeAABB() {
    if (t.getRotation().isIdentity()) {
      aabb = translate(cgeom->aabb_local, t.getTranslation());
    } else {
      aabb.min_ = aabb.max_ = t.getTranslation();

      Vec3s min_world, max_world;
      for (int k = 0; k < 3; ++k) {
        min_world.array() = t.getRotation().row(k).array() *
                            cgeom->aabb_local.min_.transpose().array();
        max_world.array() = t.getRotation().row(k).array() *
                            cgeom->aabb_local.max_.transpose().array();

        aabb.min_[k] += (min_world.array().min)(max_world.array()).sum();
        aabb.max_[k] += (min_world.array().max)(max_world.array()).sum();
      }
    }
  }

  /// @brief get user data in object
  void* getUserData() const { return user_data; }

  /// @brief set user data in object
  void setUserData(void* data) { user_data = data; }

  /// @brief get translation of the object
  inline const Vec3s& getTranslation() const { return t.getTranslation(); }

  /// @brief get matrix rotation of the object
  inline const Matrix3s& getRotation() const { return t.getRotation(); }

  /// @brief get object's transform
  inline const Transform3s& getTransform() const { return t; }

  /// @brief set object's rotation matrix
  void setRotation(const Matrix3s& R) { t.setRotation(R); }

  /// @brief set object's translation
  void setTranslation(const Vec3s& T) { t.setTranslation(T); }

  /// @brief set object's transform
  void setTransform(const Matrix3s& R, const Vec3s& T) { t.setTransform(R, T); }

  /// @brief set object's transform
  void setTransform(const Transform3s& tf) { t = tf; }

  /// @brief whether the object is in local coordinate
  bool isIdentityTransform() const { return t.isIdentity(); }

  /// @brief set the object in local coordinate
  void setIdentityTransform() { t.setIdentity(); }

  /// @brief get shared pointer to collision geometry of the object instance
  const shared_ptr<const CollisionGeometry> collisionGeometry() const {
    return cgeom;
  }

  /// @brief get shared pointer to collision geometry of the object instance
  const shared_ptr<CollisionGeometry>& collisionGeometry() { return cgeom; }

  /// @brief get raw pointer to collision geometry of the object instance
  const CollisionGeometry* collisionGeometryPtr() const { return cgeom.get(); }

  /// @brief get raw pointer to collision geometry of the object instance
  CollisionGeometry* collisionGeometryPtr() { return cgeom.get(); }

  /// @brief Associate a new CollisionGeometry
  ///
  /// @param[in] collision_geometry The new CollisionGeometry
  /// @param[in] compute_local_aabb Whether the local aabb of the input new has
  /// to be computed.
  ///
  void setCollisionGeometry(
      const shared_ptr<CollisionGeometry>& collision_geometry,
      bool compute_local_aabb = true) {
    if (collision_geometry.get() != cgeom.get()) {
      cgeom = collision_geometry;
      init(compute_local_aabb);
    }
  }

 protected:
  void init(bool compute_local_aabb = true) {
    if (cgeom) {
      if (compute_local_aabb) cgeom->computeLocalAABB();
      computeAABB();
    }
  }

  shared_ptr<CollisionGeometry> cgeom;

  Transform3s t;

  /// @brief AABB in global coordinate
  mutable AABB aabb;

  /// @brief pointer to user defined data specific to this object
  void* user_data;
};

}  // namespace coal

#endif
