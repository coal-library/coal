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

#ifndef COAL_GEOMETRIC_SHAPES_H
#define COAL_GEOMETRIC_SHAPES_H

#include <vector>
#include <memory>

#include <boost/math/constants/constants.hpp>

#include "coal/collision_object.h"
#include "coal/data_types.h"

#ifdef COAL_HAS_QHULL
namespace orgQhull {
class Qhull;
}
#endif

namespace coal {

/// @brief Base class for all basic geometric shapes
class COAL_DLLAPI ShapeBase : public CollisionGeometry {
 public:
  ShapeBase() {}

  ///  \brief Copy constructor
  ShapeBase(const ShapeBase& other)
      : CollisionGeometry(other),
        m_swept_sphere_radius(other.m_swept_sphere_radius) {}

  ShapeBase& operator=(const ShapeBase& other) = default;

  virtual ~ShapeBase() {};

  /// @brief Get object type: a geometric shape
  OBJECT_TYPE getObjectType() const { return OT_GEOM; }

  /// @brief Set radius of sphere swept around the shape.
  /// Must be >= 0.
  void setSweptSphereRadius(Scalar radius) {
    if (radius < 0) {
      COAL_THROW_PRETTY("Swept-sphere radius must be positive.",
                        std::invalid_argument);
    }
    this->m_swept_sphere_radius = radius;
  }

  /// @brief Get radius of sphere swept around the shape.
  /// This radius is always >= 0.
  Scalar getSweptSphereRadius() const { return this->m_swept_sphere_radius; }

 protected:
  /// \brief Radius of the sphere swept around the shape.
  /// Default value is 0.
  /// Note: this property differs from `inflated` method of certain
  /// derived classes (e.g. Box, Sphere, Ellipsoid, Capsule, Cone, Cylinder)
  /// in the sense that inflated returns a new shape which can be inflated but
  /// also deflated.
  /// Also, an inflated shape is not rounded. It simply has a different size.
  /// Sweeping a shape with a sphere is a different operation (a Minkowski sum),
  /// which rounds the sharp corners of a shape.
  /// The swept sphere radius is a property of the shape itself and can be
  /// manually updated between collision checks.
  Scalar m_swept_sphere_radius{0};
};

/// @defgroup Geometric_Shapes Geometric shapes
/// Classes of different types of geometric shapes.
/// @{

/// @brief Triangle stores the points instead of only indices of points
class COAL_DLLAPI TriangleP : public ShapeBase {
 public:
  TriangleP() {};

  TriangleP(const Vec3s& a_, const Vec3s& b_, const Vec3s& c_)
      : ShapeBase(), a(a_), b(b_), c(c_) {}

  TriangleP(const TriangleP& other)
      : ShapeBase(other), a(other.a), b(other.b), c(other.c) {}

  /// @brief Clone *this into a new TriangleP
  virtual TriangleP* clone() const { return new TriangleP(*this); };

  /// @brief virtual function of compute AABB in local coordinate
  void computeLocalAABB();

  NODE_TYPE getNodeType() const { return GEOM_TRIANGLE; }

  //  std::pair<ShapeBase*, Transform3s> inflated(const Scalar value) const
  //  {
  //    if (value == 0) return std::make_pair(new TriangleP(*this),
  //    Transform3s()); Vec3s AB(b - a), BC(c - b), CA(a - c); AB.normalize();
  //    BC.normalize();
  //    CA.normalize();
  //
  //    Vec3s new_a(a + value * Vec3s(-AB + CA).normalized());
  //    Vec3s new_b(b + value * Vec3s(-BC + AB).normalized());
  //    Vec3s new_c(c + value * Vec3s(-CA + BC).normalized());
  //
  //    return std::make_pair(new TriangleP(new_a, new_b, new_c),
  //    Transform3s());
  //  }
  //
  //  Scalar minInflationValue() const
  //  {
  //    return (std::numeric_limits<Scalar>::max)(); // TODO(jcarpent):
  //    implement
  //  }

  Vec3s a, b, c;

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const TriangleP* other_ptr = dynamic_cast<const TriangleP*>(&_other);
    if (other_ptr == nullptr) return false;
    const TriangleP& other = *other_ptr;

    return a == other.a && b == other.b && c == other.c &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Center at zero point, axis aligned box
class COAL_DLLAPI Box : public ShapeBase {
 public:
  Box(Scalar x, Scalar y, Scalar z)
      : ShapeBase(), halfSide(x / 2, y / 2, z / 2) {}

  Box(const Vec3s& side_) : ShapeBase(), halfSide(side_ / 2) {}

  Box(const Box& other) : ShapeBase(other), halfSide(other.halfSide) {}

  Box& operator=(const Box& other) {
    if (this == &other) return *this;

    this->halfSide = other.halfSide;
    return *this;
  }

  /// @brief Clone *this into a new Box
  virtual Box* clone() const { return new Box(*this); };

  /// @brief Default constructor
  Box() {}

  /// @brief box side half-length
  Vec3s halfSide;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a box
  NODE_TYPE getNodeType() const { return GEOM_BOX; }

  Scalar computeVolume() const { return 8 * halfSide.prod(); }

  Matrix3s computeMomentofInertia() const {
    Scalar V = computeVolume();
    Vec3s s(halfSide.cwiseAbs2() * V);
    return (Vec3s(s[1] + s[2], s[0] + s[2], s[0] + s[1]) / 3).asDiagonal();
  }

  Scalar minInflationValue() const { return -halfSide.minCoeff(); }

  /// \brief Inflate the box by an amount given by `value`.
  /// This value can be positive or negative but must always >=
  /// `minInflationValue()`.
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated box and the related transform to account for the
  /// change of shape frame
  std::pair<Box, Transform3s> inflated(const Scalar value) const {
    if (value <= minInflationValue())
      COAL_THROW_PRETTY("value (" << value << ") "
                                  << "is two small. It should be at least: "
                                  << minInflationValue(),
                        std::invalid_argument);
    return std::make_pair(Box(2 * (halfSide + Vec3s::Constant(value))),
                          Transform3s());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Box* other_ptr = dynamic_cast<const Box*>(&_other);
    if (other_ptr == nullptr) return false;
    const Box& other = *other_ptr;

    return halfSide == other.halfSide &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Center at zero point sphere
class COAL_DLLAPI Sphere : public ShapeBase {
 public:
  /// @brief Default constructor
  Sphere() {}

  explicit Sphere(Scalar radius_) : ShapeBase(), radius(radius_) {}

  Sphere(const Sphere& other) : ShapeBase(other), radius(other.radius) {}

  /// @brief Clone *this into a new Sphere
  virtual Sphere* clone() const { return new Sphere(*this); };

  /// @brief Radius of the sphere
  Scalar radius;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const { return GEOM_SPHERE; }

  Matrix3s computeMomentofInertia() const {
    Scalar I = Scalar(0.4) * radius * radius * computeVolume();
    return I * Matrix3s::Identity();
  }

  Scalar computeVolume() const {
    return 4 * boost::math::constants::pi<Scalar>() * radius * radius * radius /
           3;
  }

  Scalar minInflationValue() const { return -radius; }

  /// \brief Inflate the sphere by an amount given by `value`.
  /// This value can be positive or negative but must always >=
  /// `minInflationValue()`.
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated sphere and the related transform to account for
  /// the change of shape frame
  std::pair<Sphere, Transform3s> inflated(const Scalar value) const {
    if (value <= minInflationValue())
      COAL_THROW_PRETTY("value (" << value
                                  << ") is two small. It should be at least: "
                                  << minInflationValue(),
                        std::invalid_argument);
    return std::make_pair(Sphere(radius + value), Transform3s());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Sphere* other_ptr = dynamic_cast<const Sphere*>(&_other);
    if (other_ptr == nullptr) return false;
    const Sphere& other = *other_ptr;

    return radius == other.radius &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Ellipsoid centered at point zero
class COAL_DLLAPI Ellipsoid : public ShapeBase {
 public:
  /// @brief Default constructor
  Ellipsoid() {}

  Ellipsoid(Scalar rx, Scalar ry, Scalar rz) : ShapeBase(), radii(rx, ry, rz) {}

  explicit Ellipsoid(const Vec3s& radii) : radii(radii) {}

  Ellipsoid(const Ellipsoid& other) : ShapeBase(other), radii(other.radii) {}

  /// @brief Clone *this into a new Ellipsoid
  virtual Ellipsoid* clone() const { return new Ellipsoid(*this); };

  /// @brief Radii of the Ellipsoid (such that on boundary: x^2/rx^2 + y^2/ry^2
  /// + z^2/rz^2 = 1)
  Vec3s radii;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: an ellipsoid
  NODE_TYPE getNodeType() const { return GEOM_ELLIPSOID; }

  Matrix3s computeMomentofInertia() const {
    Scalar V = computeVolume();
    Scalar a2 = V * radii[0] * radii[0];
    Scalar b2 = V * radii[1] * radii[1];
    Scalar c2 = V * radii[2] * radii[2];
    Scalar alpha = Scalar(0.2);
    return (Matrix3s() << alpha * (b2 + c2), 0, 0, 0, alpha * (a2 + c2), 0, 0,
            0, alpha * (a2 + b2))
        .finished();
  }

  Scalar computeVolume() const {
    return 4 * boost::math::constants::pi<Scalar>() * radii[0] * radii[1] *
           radii[2] / 3;
  }

  Scalar minInflationValue() const { return -radii.minCoeff(); }

  /// \brief Inflate the ellipsoid by an amount given by `value`.
  /// This value can be positive or negative but must always >=
  /// `minInflationValue()`.
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated ellipsoid and the related transform to account for
  /// the change of shape frame
  std::pair<Ellipsoid, Transform3s> inflated(const Scalar value) const {
    if (value <= minInflationValue())
      COAL_THROW_PRETTY("value (" << value
                                  << ") is two small. It should be at least: "
                                  << minInflationValue(),
                        std::invalid_argument);
    return std::make_pair(Ellipsoid(radii + Vec3s::Constant(value)),
                          Transform3s());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Ellipsoid* other_ptr = dynamic_cast<const Ellipsoid*>(&_other);
    if (other_ptr == nullptr) return false;
    const Ellipsoid& other = *other_ptr;

    return radii == other.radii &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Capsule
/// It is \f$ { x~\in~\mathbb{R}^3, d(x, AB) \leq radius } \f$
/// where \f$ d(x, AB) \f$ is the distance between the point x and the capsule
/// segment AB, with \f$ A = (0,0,-halfLength), B = (0,0,halfLength) \f$.
class COAL_DLLAPI Capsule : public ShapeBase {
 public:
  /// @brief Default constructor
  Capsule() {}

  Capsule(Scalar radius_, Scalar lz_) : ShapeBase(), radius(radius_) {
    halfLength = lz_ / 2;
  }

  Capsule(const Capsule& other)
      : ShapeBase(other), radius(other.radius), halfLength(other.halfLength) {}

  /// @brief Clone *this into a new Capsule
  virtual Capsule* clone() const { return new Capsule(*this); };

  /// @brief Radius of capsule
  Scalar radius;

  /// @brief Half Length along z axis
  Scalar halfLength;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a capsule
  NODE_TYPE getNodeType() const { return GEOM_CAPSULE; }

  Scalar computeVolume() const {
    return boost::math::constants::pi<Scalar>() * radius * radius *
           ((halfLength * 2) + radius * 4 / Scalar(3));
  }

  Matrix3s computeMomentofInertia() const {
    Scalar v_cyl = radius * radius * (halfLength * 2) *
                   boost::math::constants::pi<Scalar>();
    Scalar v_sph = radius * radius * radius *
                   boost::math::constants::pi<Scalar>() * 4 / Scalar(3);

    Scalar h2 = halfLength * halfLength;
    Scalar r2 = radius * radius;
    Scalar ix =
        v_cyl * (h2 / Scalar(3) + r2 / Scalar(4)) +
        v_sph * (Scalar(0.4) * r2 + h2 + Scalar(0.75) * radius * halfLength);
    Scalar iz = (Scalar(0.5) * v_cyl + Scalar(0.4) * v_sph) * radius * radius;

    return (Matrix3s() << ix, 0, 0, 0, ix, 0, 0, 0, iz).finished();
  }

  Scalar minInflationValue() const { return -radius; }

  /// \brief Inflate the capsule by an amount given by `value`.
  /// This value can be positive or negative but must always >=
  /// `minInflationValue()`.
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated capsule and the related transform to account for
  /// the change of shape frame
  std::pair<Capsule, Transform3s> inflated(const Scalar value) const {
    if (value <= minInflationValue())
      COAL_THROW_PRETTY("value (" << value
                                  << ") is two small. It should be at least: "
                                  << minInflationValue(),
                        std::invalid_argument);
    return std::make_pair(Capsule(radius + value, 2 * halfLength),
                          Transform3s());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Capsule* other_ptr = dynamic_cast<const Capsule*>(&_other);
    if (other_ptr == nullptr) return false;
    const Capsule& other = *other_ptr;

    return radius == other.radius && halfLength == other.halfLength &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Cone
/// The base of the cone is at \f$ z = - halfLength \f$ and the top is at
/// \f$ z = halfLength \f$.
class COAL_DLLAPI Cone : public ShapeBase {
 public:
  /// @brief Default constructor
  Cone() {}

  Cone(Scalar radius_, Scalar lz_) : ShapeBase(), radius(radius_) {
    halfLength = lz_ / 2;
  }

  Cone(const Cone& other)
      : ShapeBase(other), radius(other.radius), halfLength(other.halfLength) {}

  /// @brief Clone *this into a new Cone
  virtual Cone* clone() const { return new Cone(*this); };

  /// @brief Radius of the cone
  Scalar radius;

  /// @brief Half Length along z axis
  Scalar halfLength;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a cone
  NODE_TYPE getNodeType() const { return GEOM_CONE; }

  Scalar computeVolume() const {
    return boost::math::constants::pi<Scalar>() * radius * radius *
           (halfLength * 2) / 3;
  }

  Matrix3s computeMomentofInertia() const {
    Scalar V = computeVolume();
    Scalar ix =
        V * (Scalar(0.4) * halfLength * halfLength + 3 * radius * radius / 20);
    Scalar iz = Scalar(0.3) * V * radius * radius;

    return (Matrix3s() << ix, 0, 0, 0, ix, 0, 0, 0, iz).finished();
  }

  Vec3s computeCOM() const { return Vec3s(0, 0, -Scalar(0.5) * halfLength); }

  Scalar minInflationValue() const { return -(std::min)(radius, halfLength); }

  /// \brief Inflate the cone by an amount given by `value`.
  /// This value can be positive or negative but must always >=
  /// `minInflationValue()`.
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated cone and the related transform to account for the
  /// change of shape frame
  std::pair<Cone, Transform3s> inflated(const Scalar value) const {
    if (value <= minInflationValue())
      COAL_THROW_PRETTY("value (" << value
                                  << ") is two small. It should be at least: "
                                  << minInflationValue(),
                        std::invalid_argument);

    // tan(alpha) = 2*halfLength/radius;
    const Scalar tan_alpha = 2 * halfLength / radius;
    const Scalar sin_alpha = tan_alpha / std::sqrt(1 + tan_alpha * tan_alpha);
    const Scalar top_inflation = value / sin_alpha;
    const Scalar bottom_inflation = value;

    const Scalar new_lz = 2 * halfLength + top_inflation + bottom_inflation;
    const Scalar new_cz = (top_inflation + bottom_inflation) / Scalar(2);
    const Scalar new_radius = new_lz / tan_alpha;

    return std::make_pair(Cone(new_radius, new_lz),
                          Transform3s(Vec3s(0., 0., new_cz)));
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Cone* other_ptr = dynamic_cast<const Cone*>(&_other);
    if (other_ptr == nullptr) return false;
    const Cone& other = *other_ptr;

    return radius == other.radius && halfLength == other.halfLength &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Cylinder along Z axis.
/// The cylinder is defined at its centroid.
class COAL_DLLAPI Cylinder : public ShapeBase {
 public:
  /// @brief Default constructor
  Cylinder() {}

  Cylinder(Scalar radius_, Scalar lz_) : ShapeBase(), radius(radius_) {
    halfLength = lz_ / 2;
  }

  Cylinder(const Cylinder& other)
      : ShapeBase(other), radius(other.radius), halfLength(other.halfLength) {}

  Cylinder& operator=(const Cylinder& other) {
    if (this == &other) return *this;

    this->radius = other.radius;
    this->halfLength = other.halfLength;
    return *this;
  }

  /// @brief Clone *this into a new Cylinder
  virtual Cylinder* clone() const { return new Cylinder(*this); };

  /// @brief Radius of the cylinder
  Scalar radius;

  /// @brief Half Length along z axis
  Scalar halfLength;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a cylinder
  NODE_TYPE getNodeType() const { return GEOM_CYLINDER; }

  Scalar computeVolume() const {
    return boost::math::constants::pi<Scalar>() * radius * radius *
           (halfLength * 2);
  }

  Matrix3s computeMomentofInertia() const {
    Scalar V = computeVolume();
    Scalar ix = V * (radius * radius / 4 + halfLength * halfLength / 3);
    Scalar iz = V * radius * radius / 2;
    return (Matrix3s() << ix, 0, 0, 0, ix, 0, 0, 0, iz).finished();
  }

  Scalar minInflationValue() const { return -(std::min)(radius, halfLength); }

  /// \brief Inflate the cylinder by an amount given by `value`.
  /// This value can be positive or negative but must always >=
  /// `minInflationValue()`.
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated cylinder and the related transform to account for
  /// the change of shape frame
  std::pair<Cylinder, Transform3s> inflated(const Scalar value) const {
    if (value <= minInflationValue())
      COAL_THROW_PRETTY("value (" << value
                                  << ") is two small. It should be at least: "
                                  << minInflationValue(),
                        std::invalid_argument);
    return std::make_pair(Cylinder(radius + value, 2 * (halfLength + value)),
                          Transform3s());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Cylinder* other_ptr = dynamic_cast<const Cylinder*>(&_other);
    if (other_ptr == nullptr) return false;
    const Cylinder& other = *other_ptr;

    return radius == other.radius && halfLength == other.halfLength &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename _IndexType>
struct ConvexBaseTplNeighbors {
  typedef _IndexType IndexType;

  unsigned char count;
  IndexType begin_id;

  bool operator==(const ConvexBaseTplNeighbors& other) const {
    if (count != other.count) return false;
    if (begin_id != other.begin_id) return false;

    return true;
  }

  bool operator!=(const ConvexBaseTplNeighbors& other) const {
    return !(*this == other);
  }
};

// The support warm start polytope contains certain points of `this`
// which are support points in specific directions of space.
// This struct is used to warm start the support function computation for
// large meshes (`num_points` > 32).
template <typename _IndexType>
struct ConvexBaseTplSupportWarmStartPolytope {
  typedef _IndexType IndexType;

  // Array of support points to warm start the support function
  // computation.
  std::vector<Vec3s> points;

  // Indices of the support points warm starts.
  // These are the indices of the real convex, not the indices of points in
  // the warm start polytope.
  std::vector<IndexType> indices;

  // Cast to a different index type.
  template <typename OtherIndexType>
  ConvexBaseTplSupportWarmStartPolytope<OtherIndexType> cast() const {
    typedef ConvexBaseTplSupportWarmStartPolytope<OtherIndexType> ResType;
    ResType res;
    res.points = this->points;
    res.indices.clear();
    for (size_t i = 0; i < this->indices.size(); ++i) {
      res.indices.push_back(OtherIndexType(this->indices[i]));
    }
    return res;
  }
};

/// @brief Base for convex polytope.
/// @tparam _IndexType type of vertices indexes.
/// @note Inherited classes are responsible for filling ConvexBase::neighbors;
template <typename _IndexType>
class ConvexBaseTpl : public ShapeBase {
 public:
  // clang-format off
  COAL_DEPRECATED_MESSAGE(Use IndexType) typedef _IndexType index_type;
  // clang-format on
  typedef _IndexType IndexType;
  typedef ShapeBase Base;

  template <typename OtherIndexType>
  friend class ConvexBaseTpl;

  /// @brief Build a convex hull based on Qhull library
  /// and store the vertices and optionally the triangles
  /// \param points, num_points the points whose convex hull should be computed.
  /// \param keepTriangles if \c true, returns a Convex<Triangle> object which
  ///        contains the triangle of the shape.
  /// \param qhullCommand the command sent to qhull.
  ///        - if \c keepTriangles is \c true, this parameter should include
  ///          "Qt". If \c NULL, "Qt" is passed to Qhull.
  ///        - if \c keepTriangles is \c false, an empty string is passed to
  ///          Qhull.
  /// \note Coal must have been compiled with option \c COAL_HAS_QHULL set
  ///       to \c ON.
  static COAL_DLLAPI ConvexBaseTpl* convexHull(
      std::shared_ptr<std::vector<Vec3s>>& points, unsigned int num_points,
      bool keepTriangles, const char* qhullCommand = NULL);

  // TODO(louis): put this method in private sometime in the future.
  COAL_DEPRECATED static COAL_DLLAPI ConvexBaseTpl* convexHull(
      const Vec3s* points, unsigned int num_points, bool keepTriangles,
      const char* qhullCommand = NULL);

  virtual ~ConvexBaseTpl() {}

  /// @brief Cast ConvexBaseTpl to ShapeBase.
  /// This method should never be marked as virtual
  Base& base() { return static_cast<Base&>(*this); }

  /// @brief Const cast ConvexBaseTpl to ShapeBase.
  /// This method should never be marked as virtual
  const Base& base() const { return static_cast<const Base&>(*this); }

  /// @brief Copy constructor.
  /// The copy constructor only shallow copies the data (it copies the shared
  /// pointers but does not deep clones the data).
  ConvexBaseTpl(const ConvexBaseTpl& other) { *this = other; }

  /// @brief Copy assignment operator.
  /// The copy assignment operator shallow copies the data, just as the copy
  /// constructor.
  ConvexBaseTpl& operator=(const ConvexBaseTpl& other);

  /// @brief Clone (deep copy).
  COAL_DEPRECATED_MESSAGE(Use deepcopy instead.)
  virtual ConvexBaseTpl* clone() const { return this->deepcopy(); }

  /// @brief Deep copy of the ConvexBaseTpl.
  /// This method deep copies every field of the class.
  virtual ConvexBaseTpl* deepcopy() const {
    ConvexBaseTpl* copy = new ConvexBaseTpl();
    deepcopy(this, copy);
    return copy;
  }

  /// @brief Cast this ConvexBase vertex indices to OtherIndexType.
  /// This effectively deep copies this ConvexBaseTpl into a new one.
  template <typename OtherIndexType>
  ConvexBaseTpl<OtherIndexType> cast() const {
    ConvexBaseTpl<OtherIndexType> res;
    deepcopy(this, &res);
    return res;
  }

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a convex polytope
  NODE_TYPE getNodeType() const;

#ifdef COAL_HAS_QHULL
  /// @brief Builds the double description of the convex polytope, i.e. the set
  /// of hyperplanes which intersection form the polytope.
  void COAL_DLLAPI buildDoubleDescription();
#endif

  using Neighbors = coal::ConvexBaseTplNeighbors<IndexType>;

  /// @brief Get the index of the j-th neighbor of the i-th vertex.
  IndexType neighbor(IndexType i, IndexType j) const {
    assert(i < IndexType(num_points));
    const std::vector<Neighbors>& nns = *neighbors;
    IndexType begin_id = nns[i].begin_id;
#ifndef NDEBUG
    unsigned char count = nns[i].count;
    assert(j < count);
#endif
    const std::vector<IndexType>& nns_vec = *nneighbors_;
    return nns_vec[begin_id + j];
  }

  /// @brief Above this threshold, the convex polytope is considered large.
  /// This influcences the way the support function is computed.
  static constexpr size_t num_vertices_large_convex_threshold = 32;

  /// @brief An array of the points of the polygon.
  std::shared_ptr<std::vector<Vec3s>> points;
  unsigned int num_points;

  /// @brief An array of the normals of the polygon.
  std::shared_ptr<std::vector<Vec3s>> normals;
  /// @brief An array of the offsets to the normals of the polygon.
  /// Note: there are as many offsets as normals.
  std::shared_ptr<std::vector<Scalar>> offsets;
  unsigned int num_normals_and_offsets;

  /// @brief Neighbors of each vertex.
  /// It is an array of size num_points. For each vertex, it contains the number
  /// of neighbors and a list of indices pointing to them.
  std::shared_ptr<std::vector<Neighbors>> neighbors;

  /// @brief center of the convex polytope, this is used for collision: center
  /// is guaranteed in the internal of the polytope (as it is convex)
  Vec3s center;

  using SupportWarmStartPolytope =
      ConvexBaseTplSupportWarmStartPolytope<IndexType>;

  /// @brief Number of support warm starts.
  static constexpr size_t num_support_warm_starts = 14;

  /// @brief Support warm start polytopes.
  SupportWarmStartPolytope support_warm_starts;

 protected:
  /// @brief Construct an uninitialized convex object
  /// Initialization is done with ConvexBase::initialize.
  ConvexBaseTpl()
      : ShapeBase(),
        num_points(0),
        num_normals_and_offsets(0),
        center(Vec3s::Zero()) {}

  /// @brief Initialize the points of the convex shape
  /// This also initializes the ConvexBase::center.
  ///
  /// \param points_ list of 3D points  ///
  /// \param num_points_ number of 3D points
  void initialize(std::shared_ptr<std::vector<Vec3s>> points_,
                  unsigned int num_points_);

  /// @brief Set the points of the convex shape.
  ///
  /// \param points_ list of 3D points  ///
  /// \param num_points_ number of 3D points
  void set(std::shared_ptr<std::vector<Vec3s>> points_,
           unsigned int num_points_);

#ifdef COAL_HAS_QHULL
  void COAL_DLLAPI
  buildDoubleDescriptionFromQHullResult(const orgQhull::Qhull& qh);
#endif

  /// @brief Build the support points warm starts.
  void COAL_DLLAPI buildSupportWarmStart();

  /// @brief Array of indices of the neighbors of each vertex.
  /// Since we don't know a priori the number of neighbors of each vertex, we
  /// store the indices of the neighbors in a single array.
  /// The `neighbors` attribute, an array of `Neighbors`, is used to point each
  /// vertex to the right indices in the `nneighbors_` array.
  std::shared_ptr<std::vector<IndexType>> nneighbors_;

 protected:
  /// @brief Deep copy of a ConvexBaseTpl.
  /// This method deep copies every field of the class.
  template <typename OtherIndexType>
  static void deepcopy(const ConvexBaseTpl<IndexType>* source,
                       ConvexBaseTpl<OtherIndexType>* copy);

  void computeCenter();

  virtual bool isEqual(const CollisionGeometry& _other) const {
    const ConvexBaseTpl* other_ptr =
        dynamic_cast<const ConvexBaseTpl*>(&_other);
    if (other_ptr == nullptr) return false;
    const ConvexBaseTpl& other = *other_ptr;

    if (num_points != other.num_points) return false;

    if ((!(points.get()) && other.points.get()) ||
        (points.get() && !(other.points.get())))
      return false;
    if (points.get() && other.points.get()) {
      const std::vector<Vec3s>& points_ = *points;
      const std::vector<Vec3s>& other_points_ = *(other.points);
      for (unsigned int i = 0; i < num_points; ++i) {
        if (points_[i] != (other_points_)[i]) return false;
      }
    }

    if ((!(neighbors.get()) && other.neighbors.get()) ||
        (neighbors.get() && !(other.neighbors.get())))
      return false;
    if (neighbors.get() && other.neighbors.get()) {
      const std::vector<Neighbors>& neighbors_ = *neighbors;
      const std::vector<Neighbors>& other_neighbors_ = *(other.neighbors);
      for (unsigned int i = 0; i < num_points; ++i) {
        if (neighbors_[i] != other_neighbors_[i]) return false;
      }
    }

    if ((!(normals.get()) && other.normals.get()) ||
        (normals.get() && !(other.normals.get())))
      return false;
    if (normals.get() && other.normals.get()) {
      const std::vector<Vec3s>& normals_ = *normals;
      const std::vector<Vec3s>& other_normals_ = *(other.normals);
      for (unsigned int i = 0; i < num_normals_and_offsets; ++i) {
        if (normals_[i] != other_normals_[i]) return false;
      }
    }

    if ((!(offsets.get()) && other.offsets.get()) ||
        (offsets.get() && !(other.offsets.get())))
      return false;
    if (offsets.get() && other.offsets.get()) {
      const std::vector<Scalar>& offsets_ = *offsets;
      const std::vector<Scalar>& other_offsets_ = *(other.offsets);
      for (unsigned int i = 0; i < num_normals_and_offsets; ++i) {
        if (offsets_[i] != other_offsets_[i]) return false;
      }
    }

    if (this->support_warm_starts.points.size() !=
            other.support_warm_starts.points.size() ||
        this->support_warm_starts.indices.size() !=
            other.support_warm_starts.indices.size()) {
      return false;
    }

    for (size_t i = 0; i < this->support_warm_starts.points.size(); ++i) {
      if (this->support_warm_starts.points[i] !=
              other.support_warm_starts.points[i] ||
          this->support_warm_starts.indices[i] !=
              other.support_warm_starts.indices[i]) {
        return false;
      }
    }

    return center == other.center &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef ConvexBaseTpl<Triangle16::IndexType> ConvexBase16;
typedef ConvexBaseTpl<Triangle32::IndexType> ConvexBase32;
COAL_DEPRECATED_MESSAGE(Use ConvexBase32 instead.)
typedef ConvexBase32 ConvexBase;

template <typename PolygonT>
class ConvexTpl;

/// @brief Half Space: this is equivalent to the Plane in ODE.
/// A Half space has a priviledged direction: the direction of the normal.
/// The separation plane is defined as n * x = d; Points in the negative side of
/// the separation plane (i.e. {x | n * x < d}) are inside the half space and
/// points in the positive side of the separation plane (i.e. {x | n * x > d})
/// are outside the half space.
/// Note: prefer using a Halfspace instead of a Plane if possible, it has better
/// behavior w.r.t. collision detection algorithms.
class COAL_DLLAPI Halfspace : public ShapeBase {
 public:
  /// @brief Construct a half space with normal direction and offset
  Halfspace(const Vec3s& n_, Scalar d_) : ShapeBase(), n(n_), d(d_) {
    unitNormalTest();
  }

  /// @brief Construct a plane with normal direction and offset
  Halfspace(Scalar a, Scalar b, Scalar c, Scalar d_)
      : ShapeBase(), n(a, b, c), d(d_) {
    unitNormalTest();
  }

  Halfspace() : ShapeBase(), n(1, 0, 0), d(0) {}

  Halfspace(const Halfspace& other)
      : ShapeBase(other), n(other.n), d(other.d) {}

  /// @brief operator =
  Halfspace& operator=(const Halfspace& other) {
    n = other.n;
    d = other.d;
    return *this;
  }

  /// @brief Clone *this into a new Halfspace
  virtual Halfspace* clone() const { return new Halfspace(*this); };

  Scalar signedDistance(const Vec3s& p) const {
    return n.dot(p) - (d + this->getSweptSphereRadius());
  }

  Scalar distance(const Vec3s& p) const {
    return std::abs(this->signedDistance(p));
  }

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a half space
  NODE_TYPE getNodeType() const { return GEOM_HALFSPACE; }

  Scalar minInflationValue() const {
    return std::numeric_limits<Scalar>::lowest();
  }

  /// \brief Inflate the halfspace by an amount given by `value`.
  /// This value can be positive or negative but must always >=
  /// `minInflationValue()`.
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated halfspace and the related transform to account for
  /// the change of shape frame
  std::pair<Halfspace, Transform3s> inflated(const Scalar value) const {
    if (value <= minInflationValue())
      COAL_THROW_PRETTY("value (" << value
                                  << ") is two small. It should be at least: "
                                  << minInflationValue(),
                        std::invalid_argument);
    return std::make_pair(Halfspace(n, d + value), Transform3s());
  }

  /// @brief Plane normal
  Vec3s n;

  /// @brief Plane offset
  Scalar d;

 protected:
  /// @brief Turn non-unit normal into unit
  void unitNormalTest();

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Halfspace* other_ptr = dynamic_cast<const Halfspace*>(&_other);
    if (other_ptr == nullptr) return false;
    const Halfspace& other = *other_ptr;

    return n == other.n && d == other.d &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Infinite plane.
/// A plane can be viewed as two half spaces; it has no priviledged direction.
/// Note: prefer using a Halfspace instead of a Plane if possible, it has better
/// behavior w.r.t. collision detection algorithms.
class COAL_DLLAPI Plane : public ShapeBase {
 public:
  /// @brief Construct a plane with normal direction and offset
  Plane(const Vec3s& n_, Scalar d_) : ShapeBase(), n(n_), d(d_) {
    unitNormalTest();
  }

  /// @brief Construct a plane with normal direction and offset
  Plane(Scalar a, Scalar b, Scalar c, Scalar d_)
      : ShapeBase(), n(a, b, c), d(d_) {
    unitNormalTest();
  }

  Plane() : ShapeBase(), n(1, 0, 0), d(0) {}

  Plane(const Plane& other) : ShapeBase(other), n(other.n), d(other.d) {}

  /// @brief operator =
  Plane& operator=(const Plane& other) {
    n = other.n;
    d = other.d;
    return *this;
  }

  /// @brief Clone *this into a new Plane
  virtual Plane* clone() const { return new Plane(*this); };

  Scalar signedDistance(const Vec3s& p) const {
    const Scalar dist = n.dot(p) - d;
    Scalar signed_dist = std::abs(n.dot(p) - d) - this->getSweptSphereRadius();
    if (dist >= 0) {
      return signed_dist;
    }
    if (signed_dist >= 0) {
      return -signed_dist;
    }
    return signed_dist;
  }

  Scalar distance(const Vec3s& p) const {
    return std::abs(std::abs(n.dot(p) - d) - this->getSweptSphereRadius());
  }

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a plane
  NODE_TYPE getNodeType() const { return GEOM_PLANE; }

  /// @brief Plane normal
  Vec3s n;

  /// @brief Plane offset
  Scalar d;

 protected:
  /// @brief Turn non-unit normal into unit
  void unitNormalTest();

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Plane* other_ptr = dynamic_cast<const Plane*>(&_other);
    if (other_ptr == nullptr) return false;
    const Plane& other = *other_ptr;

    return n == other.n && d == other.d &&
           getSweptSphereRadius() == other.getSweptSphereRadius();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @} */  // end of Geometric_Shapes

}  // namespace coal

#include "coal/shape/geometric_shapes.hxx"

#endif
