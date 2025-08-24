//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_COLLISION_OBJECT_H
#define COAL_SERIALIZATION_COLLISION_OBJECT_H

#include "coal/collision_object.h"

#include "coal/serialization/fwd.h"
#include "coal/serialization/AABB.h"

namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar, const coal::CollisionGeometry& collision_geometry,
          const unsigned int /*version*/) {
  ar& make_nvp("aabb_center", collision_geometry.aabb_center);
  ar& make_nvp("aabb_radius", collision_geometry.aabb_radius);
  ar& make_nvp("aabb_local", collision_geometry.aabb_local);
  ar& make_nvp("cost_density", collision_geometry.cost_density);
  ar& make_nvp("threshold_occupied", collision_geometry.threshold_occupied);
  ar& make_nvp("threshold_free", collision_geometry.threshold_free);
}

template <class Archive>
void load(Archive& ar, coal::CollisionGeometry& collision_geometry,
          const unsigned int /*version*/) {
  ar >> make_nvp("aabb_center", collision_geometry.aabb_center);
  ar >> make_nvp("aabb_radius", collision_geometry.aabb_radius);
  ar >> make_nvp("aabb_local", collision_geometry.aabb_local);
  ar >> make_nvp("cost_density", collision_geometry.cost_density);
  ar >> make_nvp("threshold_occupied", collision_geometry.threshold_occupied);
  ar >> make_nvp("threshold_free", collision_geometry.threshold_free);
  collision_geometry.user_data = NULL;  // no way to recover this
}

COAL_SERIALIZATION_SPLIT(coal::CollisionGeometry)

}  // namespace serialization
}  // namespace boost

namespace coal {

// fwd declaration
template <typename BV>
class HeightField;

template <typename PolygonT>
class ConvexTpl;

struct OBB;
struct OBBRSS;
class AABB;

class OcTree;
class Box;
class Sphere;
class Ellipsoid;
class Capsule;
class Cone;
class TriangleP;
class Cylinder;
class Halfspace;
class Plane;

namespace serialization {
template <>
struct register_type<CollisionGeometry> {
  template <class Archive>
  static void on(Archive& ar) {
    ar.template register_type<Box>();
    ar.template register_type<Sphere>();
    ar.template register_type<Ellipsoid>();
    ar.template register_type<TriangleP>();
    ar.template register_type<Capsule>();
    ar.template register_type<Cone>();
    ar.template register_type<Cylinder>();
    ar.template register_type<Halfspace>();
    ar.template register_type<Plane>();
    ar.template register_type<OcTree>();
    ar.template register_type<HeightField<OBB>>();
    ar.template register_type<HeightField<OBBRSS>>();
    ar.template register_type<HeightField<AABB>>();
    ar.template register_type<ConvexTpl<Triangle32>>();
    ;
  }
};
}  // namespace serialization
}  // namespace coal

#endif  // ifndef COAL_SERIALIZATION_COLLISION_OBJECT_H
