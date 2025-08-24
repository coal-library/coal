//
// Copyright (c) 2021-2024 INRIA
//

#ifndef COAL_SERIALIZATION_GEOMETRIC_SHAPES_H
#define COAL_SERIALIZATION_GEOMETRIC_SHAPES_H

#include "coal/shape/geometric_shapes.h"
#include "coal/serialization/fwd.h"
#include "coal/serialization/collision_object.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::ShapeBase& shape_base,
               const unsigned int /*version*/) {
  ar& make_nvp(
      "base",
      boost::serialization::base_object<coal::CollisionGeometry>(shape_base));
  ::coal::Scalar radius = shape_base.getSweptSphereRadius();
  ar& make_nvp("swept_sphere_radius", radius);

  if (Archive::is_loading::value) {
    shape_base.setSweptSphereRadius(radius);
  }
}

template <class Archive>
void serialize(Archive& ar, coal::TriangleP& triangle,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(triangle));
  ar& make_nvp("a", triangle.a);
  ar& make_nvp("b", triangle.b);
  ar& make_nvp("c", triangle.c);
}

template <class Archive>
void serialize(Archive& ar, coal::Box& box, const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<coal::ShapeBase>(box));
  ar& make_nvp("halfSide", box.halfSide);
}

template <class Archive>
void serialize(Archive& ar, coal::Sphere& sphere,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(sphere));
  ar& make_nvp("radius", sphere.radius);
}

template <class Archive>
void serialize(Archive& ar, coal::Ellipsoid& ellipsoid,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(ellipsoid));
  ar& make_nvp("radii", ellipsoid.radii);
}

template <class Archive>
void serialize(Archive& ar, coal::Capsule& capsule,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(capsule));
  ar& make_nvp("radius", capsule.radius);
  ar& make_nvp("halfLength", capsule.halfLength);
}

template <class Archive>
void serialize(Archive& ar, coal::Cone& cone, const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(cone));
  ar& make_nvp("radius", cone.radius);
  ar& make_nvp("halfLength", cone.halfLength);
}

template <class Archive>
void serialize(Archive& ar, coal::Cylinder& cylinder,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(cylinder));
  ar& make_nvp("radius", cylinder.radius);
  ar& make_nvp("halfLength", cylinder.halfLength);
}

template <class Archive>
void serialize(Archive& ar, coal::Halfspace& half_space,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(half_space));
  ar& make_nvp("n", half_space.n);
  ar& make_nvp("d", half_space.d);
}

template <class Archive>
void serialize(Archive& ar, coal::Plane& plane,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::ShapeBase>(plane));
  ar& make_nvp("n", plane.n);
  ar& make_nvp("d", plane.d);
}

}  // namespace serialization
}  // namespace boost

COAL_SERIALIZATION_DECLARE_EXPORT(::coal::ShapeBase)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::CollisionGeometry)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::TriangleP)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Box)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Sphere)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Ellipsoid)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Capsule)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Cone)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Cylinder)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Halfspace)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::Plane)

#endif  // ifndef COAL_SERIALIZATION_GEOMETRIC_SHAPES_H
