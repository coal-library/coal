/// Copyright 2025 INRIA

#ifndef COAL_PYTHON_NB_BROADPHASE_BROADPHASE_CALLBACKS_HH
#define COAL_PYTHON_NB_BROADPHASE_BROADPHASE_CALLBACKS_HH

#include "coal/fwd.hh"
#include "coal/broadphase/broadphase_callbacks.h"
#include "coal/broadphase/broadphase_collision_manager.h"

#include "../fwd.h"
#include <nanobind/trampoline.h>
#include <nanobind/stl/function.h>

namespace coal {

struct CollisionCallBackBaseWrapper : CollisionCallBackBase {
  NB_TRAMPOLINE(CollisionCallBackBase, 2);
  using Base = CollisionCallBackBase;

  void init() override { NB_OVERRIDE_PURE(init); }

  bool collide(CollisionObject *o1, CollisionObject *o2) override {
    NB_OVERRIDE_PURE(collide, o1, o2);
  }

  static void expose(nb::module_ &m) {
    nb::class_<CollisionCallBackBase, CollisionCallBackBaseWrapper>(
        m, "CollisionCallBackBase")
        .def("init", &Base::init)
        .def("collide", &Base::collide)
        .def("__call__", &Base::operator());
  }
};  // CollisionCallBackBaseWrapper

struct DistanceCallBackBaseWrapper : DistanceCallBackBase {
  NB_TRAMPOLINE(DistanceCallBackBase, 2);
  using Base = DistanceCallBackBase;
  using Self = DistanceCallBackBaseWrapper;

  void init() override { NB_OVERRIDE_PURE(init); }

  bool distance(CollisionObject *o1, CollisionObject *o2,
                Eigen::Matrix<Scalar, 1, 1> &dist) {
    return distance(o1, o2, dist.coeffRef(0, 0));
  }

  bool distance(CollisionObject *o1, CollisionObject *o2,
                Scalar &dist) override {
    NB_OVERRIDE_PURE(distance, o1, o2, dist);
  }

  static void expose(nb::module_ &m) {
    nb::class_<DistanceCallBackBase, DistanceCallBackBaseWrapper>(
        m, "DistanceCallBackBase")
        .def("init", &Base::init)
        .def("distance",
             [](Self &self, CollisionObject *o1, CollisionObject *o2,
                Eigen::Matrix<Scalar, 1, 1> &dist) {
               return self.distance(o1, o2, dist);
             })
        .def("__call__", &Base::operator());
  }
};  // CollisionCallBackBaseWrapper

struct BroadPhaseCollisionManagerWrapper : BroadPhaseCollisionManager {
  NB_TRAMPOLINE(BroadPhaseCollisionManager, 17);
  using Base = BroadPhaseCollisionManager;

  void registerObjects(
      const std::vector<CollisionObject *> &other_objs) override {
    NB_OVERRIDE_PURE(registerObjects, other_objs);
  }
  void registerObject(CollisionObject *obj) override {
    NB_OVERRIDE_PURE(registerObject, obj);
  }
  void unregisterObject(CollisionObject *obj) override {
    NB_OVERRIDE_PURE(unregisterObject, obj);
  }

  void update(const std::vector<CollisionObject *> &other_objs) override {
    NB_OVERRIDE_PURE(update, other_objs);
  }
  void update(CollisionObject *obj) override { NB_OVERRIDE_PURE(update, obj); }
  void update() override { NB_OVERRIDE_PURE(update); }

  void setup() override { NB_OVERRIDE_PURE(setup); }
  void clear() override { NB_OVERRIDE_PURE(clear); }

  std::vector<CollisionObject *> getObjects() const override {
    NB_OVERRIDE_PURE(getObjects);
  }

  void collide(CollisionCallBackBase *callback) const override {
    NB_OVERRIDE_PURE(collide, callback);
  }
  void collide(CollisionObject *obj,
               CollisionCallBackBase *callback) const override {
    NB_OVERRIDE_PURE(collide, obj, callback);
  }
  void collide(BroadPhaseCollisionManager *other_manager,
               CollisionCallBackBase *callback) const override {
    NB_OVERRIDE_PURE(collide, other_manager, callback);
  }

  void distance(DistanceCallBackBase *callback) const override {
    NB_OVERRIDE_PURE(distance, callback);
  }
  void distance(CollisionObject *obj,
                DistanceCallBackBase *callback) const override {
    NB_OVERRIDE_PURE(distance, obj, callback);
  }
  void distance(BroadPhaseCollisionManager *other_manager,
                DistanceCallBackBase *callback) const override {
    NB_OVERRIDE_PURE(distance, other_manager, callback);
  }

  bool empty() const override { NB_OVERRIDE_PURE(empty); }
  size_t size() const override { NB_OVERRIDE_PURE(size); }

  static void expose(nb::module_ &m) {
    nb::class_<BroadPhaseCollisionManager, BroadPhaseCollisionManagerWrapper>(
        m, "BroadPhaseCollisionManager")
        .def("registerObjects", &Base::registerObjects)
        .def("registerObject", &Base::registerObject)
        .def("unregisterObject", &Base::unregisterObject)

        .def("update", [](Base &self) { self.update(); })
        .def("update",
             [](Base &self, const std::vector<CollisionObject *> &objects) {
               self.update(objects);
             })
        .def("update",
             [](Base &self, CollisionObject *obj) { self.update(obj); })

        .def("setup", &Base::setup)
        .def("clear", &Base::clear)
        .def("empty", &Base::empty)
        .def("size", &Base::size)

        .def("getObjects", [](const Base &self) { return self.getObjects(); })

        .def("collide",
             [](const Base &self, CollisionCallBackBase *callback) {
               self.collide(callback);
             })
        .def("collide",
             [](const Base &self, CollisionObject *obj,
                CollisionCallBackBase *callback) {
               self.collide(obj, callback);
             })
        .def("collide",
             [](const Base &self, BroadPhaseCollisionManager *manager,
                CollisionCallBackBase *callback) {
               self.collide(manager, callback);
             })

        .def("collide",
             [](const Base &self, const CollisionCallBackFunctor &callback) {
               self.collide(callback);
             })
        .def("collide",
             [](const Base &self, CollisionObject *obj,
                const CollisionCallBackFunctor &callback) {
               self.collide(obj, callback);
             })
        .def("collide",
             [](const Base &self, BroadPhaseCollisionManager *manager,
                const CollisionCallBackFunctor &callback) {
               self.collide(manager, callback);
             })

        .def("distance",
             [](const Base &self, DistanceCallBackBase *callback) {
               self.distance(callback);
             })
        .def("distance",
             [](const Base &self, CollisionObject *obj,
                DistanceCallBackBase *callback) {
               self.distance(obj, callback);
             })
        .def("distance",
             [](const Base &self, BroadPhaseCollisionManager *manager,
                DistanceCallBackBase *callback) {
               self.distance(manager, callback);
             })

        .def("distance",
             [](const Base &self, const DistanceCallBackFunctor &callback) {
               self.distance(callback);
             })
        .def("distance",
             [](const Base &self, CollisionObject *obj,
                const DistanceCallBackFunctor &callback) {
               self.distance(obj, callback);
             })
        .def("distance",
             [](const Base &self, BroadPhaseCollisionManager *manager,
                const DistanceCallBackFunctor &callback) {
               self.distance(manager, callback);
             });
  }

  template <typename Derived>
  static void exposeDerived(nb::module_ &m, const char *name) {
    nb::class_<Derived, BroadPhaseCollisionManager>(m, name).def(nb::init<>());
  }

};  // BroadPhaseCollisionManagerWrapper

}  // namespace coal

#endif  // ifndef COAL_PYTHON_NB_BROADPHASE_BROADPHASE_CALLBACKS_HH