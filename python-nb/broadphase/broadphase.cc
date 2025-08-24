/// Copyrigh 2025 INRIA

#include "coal/fwd.hh"

#include "coal/broadphase/broadphase_dynamic_AABB_tree.h"
#include "coal/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "coal/broadphase/broadphase_bruteforce.h"
#include "coal/broadphase/broadphase_SaP.h"
#include "coal/broadphase/broadphase_SSaP.h"
#include "coal/broadphase/broadphase_interval_tree.h"
#include "coal/broadphase/broadphase_spatialhash.h"
#include "coal/broadphase/default_broadphase_callbacks.h"

#include "broadphase_callbacks_collision_manager.hh"

#include "../fwd.h"
#include <nanobind/stl/pair.h>

using namespace coal;
using namespace nb::literals;

COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
void exposeBroadPhase(nb::module_ &m) {
  CollisionCallBackBaseWrapper::expose(m);
  DistanceCallBackBaseWrapper::expose(m);

  // CollisionCallBackDefault
  nb::class_<CollisionCallBackDefault, CollisionCallBackBase>(
      m, "CollisionCallBackDefault")
      .def(nb::init<>())
      .DEF_RW_CLASS_ATTRIB(CollisionCallBackDefault, data);

  // DistanceCallBackDefault
  nb::class_<DistanceCallBackDefault, DistanceCallBackBase>(
      m, "DistanceCallBackDefault")
      .def(nb::init<>())
      .DEF_RW_CLASS_ATTRIB(DistanceCallBackDefault, data);

  // CollisionCallBackCollect
  nb::class_<CollisionCallBackCollect, CollisionCallBackBase>(
      m, "CollisionCallBackCollect")
      .def(nb::init<const size_t>(), "max_size"_a)
      .DEF_CLASS_FUNC(CollisionCallBackCollect, numCollisionPairs)
      .def("getCollisionPairs", &CollisionCallBackCollect::getCollisionPairs,
           nb::rv_policy::copy)
      .def("exist",
           [](const CollisionCallBackCollect &self,
              const CollisionCallBackCollect::CollisionPair &pair) {
             return self.exist(pair);
           })
      .def("exist",
           [](const CollisionCallBackCollect &self, CollisionObject *obj1,
              CollisionObject *obj2) { return self.exist(obj1, obj2); });

  nb::class_<CollisionData>(m, "CollisionData")
      .def(nb::init<>())
      .DEF_RW_CLASS_ATTRIB(CollisionData, request)
      .DEF_RW_CLASS_ATTRIB(CollisionData, result)
      .DEF_RW_CLASS_ATTRIB(CollisionData, done)
      .DEF_CLASS_FUNC(CollisionData, clear);

  nb::class_<DistanceData>(m, "DistanceData")
      .def(nb::init<>())
      .DEF_RW_CLASS_ATTRIB(DistanceData, request)
      .DEF_RW_CLASS_ATTRIB(DistanceData, result)
      .DEF_RW_CLASS_ATTRIB(DistanceData, done)
      .DEF_CLASS_FUNC(DistanceData, clear);

  BroadPhaseCollisionManagerWrapper::expose(m);

  BroadPhaseCollisionManagerWrapper::exposeDerived<
      DynamicAABBTreeCollisionManager>(m, "DynamicAABBTreeCollisionManager");
  BroadPhaseCollisionManagerWrapper::exposeDerived<
      DynamicAABBTreeArrayCollisionManager>(
      m, "DynamicAABBTreeArrayCollisionManager");
  BroadPhaseCollisionManagerWrapper::exposeDerived<
      IntervalTreeCollisionManager>(m, "IntervalTreeCollisionManager");
  BroadPhaseCollisionManagerWrapper::exposeDerived<SSaPCollisionManager>(
      m, "SSaPCollisionManager");
  BroadPhaseCollisionManagerWrapper::exposeDerived<SaPCollisionManager>(
      m, "SaPCollisionManager");
  BroadPhaseCollisionManagerWrapper::exposeDerived<NaiveCollisionManager>(
      m, "NaiveCollisionManager");

  // Specific case of SpatialHashingCollisionManager
  using HashTable =
      detail::SimpleHashTable<AABB, CollisionObject *, detail::SpatialHash>;
  using Derived = SpatialHashingCollisionManager<HashTable>;
  nb::class_<Derived, BroadPhaseCollisionManager>(
      m, "SpatialHashingCollisionManager")
      .def(nb::init<Scalar, const Vec3s &, const Vec3s &, unsigned int>(),
           "cell_size"_a, "scene_min"_a, "scene_max"_a,
           "default_table_size"_a = 1000);
}
COAL_COMPILER_DIAGNOSTIC_POP