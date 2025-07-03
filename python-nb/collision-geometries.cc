/// Copyright 2025 INRIA

#include "coal/fwd.hh"

#include "coal/hfield.h"
#include "coal/serialization/AABB.h"

#include "pickle.hh"
#include "serializable.hh"

#include "fwd.h"
#include <nanobind/operators.h>
#include <nanobind/stl/shared_ptr.h>

using namespace coal;
using namespace nb::literals;

void exposeAABB(nb::module_& m);
void exposeShapes(nb::module_& m);
void exposeBVHModels(nb::module_& m);
void exposeHeightFields(nb::module_& m);
void exposeComputeMemoryFootprint(nb::module_& m);

void exposeCollisionGeometries(nb::module_& m) {
  nb::enum_<BVHModelType>(m, "BVHModelType")
      .value("BVH_MODEL_UNKNOWN", BVH_MODEL_UNKNOWN)
      .value("BVH_MODEL_TRIANGLES", BVH_MODEL_TRIANGLES)
      .value("BVH_MODEL_POINTCLOUD", BVH_MODEL_POINTCLOUD)
      .export_values();

  nb::enum_<BVHBuildState>(m, "BVHBuildState")
      .value("BVH_BUILD_STATE_EMPTY", BVH_BUILD_STATE_EMPTY)
      .value("BVH_BUILD_STATE_BEGUN", BVH_BUILD_STATE_BEGUN)
      .value("BVH_BUILD_STATE_PROCESSED", BVH_BUILD_STATE_PROCESSED)
      .value("BVH_BUILD_STATE_UPDATE_BEGUN", BVH_BUILD_STATE_UPDATE_BEGUN)
      .value("BVH_BUILD_STATE_UPDATED", BVH_BUILD_STATE_UPDATED)
      .value("BVH_BUILD_STATE_REPLACE_BEGUN", BVH_BUILD_STATE_REPLACE_BEGUN)
      .export_values();

  nb::enum_<OBJECT_TYPE>(m, "OBJECT_TYPE")
      .value("OT_UNKNOWN", OT_UNKNOWN)
      .value("OT_BVH", OT_BVH)
      .value("OT_GEOM", OT_GEOM)
      .value("OT_OCTREE", OT_OCTREE)
      .value("OT_HFIELD", OT_HFIELD)
      .export_values();

  nb::enum_<NODE_TYPE>(m, "NODE_TYPE")
      .value("BV_UNKNOWN", BV_UNKNOWN)
      .value("BV_AABB", BV_AABB)
      .value("BV_OBB", BV_OBB)
      .value("BV_RSS", BV_RSS)
      .value("BV_kIOS", BV_kIOS)
      .value("BV_OBBRSS", BV_OBBRSS)
      .value("BV_KDOP16", BV_KDOP16)
      .value("BV_KDOP18", BV_KDOP18)
      .value("BV_KDOP24", BV_KDOP24)
      .value("GEOM_BOX", GEOM_BOX)
      .value("GEOM_SPHERE", GEOM_SPHERE)
      .value("GEOM_ELLIPSOID", GEOM_ELLIPSOID)
      .value("GEOM_CAPSULE", GEOM_CAPSULE)
      .value("GEOM_CONE", GEOM_CONE)
      .value("GEOM_CYLINDER", GEOM_CYLINDER)
      .value("GEOM_CONVEX", GEOM_CONVEX)
      .value("GEOM_PLANE", GEOM_PLANE)
      .value("GEOM_HALFSPACE", GEOM_HALFSPACE)
      .value("GEOM_TRIANGLE", GEOM_TRIANGLE)
      .value("GEOM_OCTREE", GEOM_OCTREE)
      .value("HF_AABB", HF_AABB)
      .value("HF_OBBRSS", HF_OBBRSS)
      .export_values();

  m.def(
      "translate",
      [](const AABB& aabb, const Vec3s& t) { return translate(aabb, t); },
      "aabb"_a, "t"_a, "Translate the center of AABB by t");

  m.def(
      "rotate",
      [](const AABB& aabb, const Matrix3s& R) { return rotate(aabb, R); },
      "aabb"_a, "R"_a, "Rotate the AABB object by R");

  nb::class_<CollisionGeometry>(m, "CollisionGeometry")
      .DEF_CLASS_FUNC(CollisionGeometry, getObjectType)
      .DEF_CLASS_FUNC(CollisionGeometry, getNodeType)

      .DEF_CLASS_FUNC(CollisionGeometry, computeLocalAABB)
      .DEF_CLASS_FUNC(CollisionGeometry, computeCOM)
      .DEF_CLASS_FUNC(CollisionGeometry, computeMomentofInertia)
      .DEF_CLASS_FUNC(CollisionGeometry, computeVolume)
      .DEF_CLASS_FUNC(CollisionGeometry, computeMomentofInertiaRelatedToCOM)

      .def_rw("aabb_radius", &CollisionGeometry::aabb_radius)
      .def_rw("aabb_center", &CollisionGeometry::aabb_center)
      .def_rw("aabb_local", &CollisionGeometry::aabb_local)

      .def("isOccupied", &CollisionGeometry::isOccupied)
      .def("isFree", &CollisionGeometry::isFree)
      .def("isUncertain", &CollisionGeometry::isUncertain)

      .def_rw("cost_density", &CollisionGeometry::cost_density)
      .def_rw("threshold_occupied", &CollisionGeometry::threshold_occupied)
      .def_rw("threshold_free", &CollisionGeometry::threshold_free)

      .def(nb::self == nb::self)
      .def(nb::self != nb::self);

  exposeAABB(m);
  exposeShapes(m);
  exposeBVHModels(m);
  exposeHeightFields(m);
  exposeComputeMemoryFootprint(m);
}

void exposeCollisionObject(nb::module_& m) {
  nb::class_<CollisionObject>(m, "CollisionObject")
      .def(nb::init<const CollisionGeometryPtr_t&, bool>(), "cgeom"_a,
           "compute_local_aabb"_a = true)
      .def(nb::init<const CollisionGeometryPtr_t&, const Transform3s&, bool>(),
           "cgeom"_a, "tf"_a, "compute_local_aabb"_a = true)
      .def(nb::init<const CollisionGeometryPtr_t&, const Matrix3s&,
                    const Vec3s&, bool>(),
           "cgeom"_a, "R"_a, "t"_a, "compute_local_aabb"_a = true)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)

      .DEF_CLASS_FUNC(CollisionObject, getObjectType)
      .DEF_CLASS_FUNC(CollisionObject, getNodeType)

      // properties
      .def("getTranslation", &CollisionObject::getTranslation,
           nb::rv_policy::copy)
      .def("setTranslation", &CollisionObject::setTranslation)
      .def("getRotation", &CollisionObject::getRotation, nb::rv_policy::copy)
      .def("setRotation", &CollisionObject::setRotation)
      .def("getTransform", &CollisionObject::getTransform, nb::rv_policy::copy)
      .def("setTransform", [](CollisionObject& o,
                              const Transform3s& tf) { o.setTransform(tf); })
      .def("setTransform", [](CollisionObject& o, const Matrix3s& R,
                              const Vec3s& t) { o.setTransform(R, t); })

      .def("isIdentityTransform", &CollisionObject::isIdentityTransform)
      .def("setIdentityTransform", &CollisionObject::setIdentityTransform)

      .def(
          "getAABB", [](CollisionObject& o) -> AABB& { return o.getAABB(); },
          nb::rv_policy::automatic_reference)
      .DEF_CLASS_FUNC(CollisionObject, computeAABB)

      .def("setCollisionGeometry", &CollisionObject::setCollisionGeometry,
           "cgeom"_a, "compute_local_aabb"_a = true,
           nb::rv_policy::reference_internal)
      .def(
          "collisionGeometry",
          [](CollisionObject& o) { return o.collisionGeometry(); },
          nb::rv_policy::copy);
}
