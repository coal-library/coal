//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2022 INRIA
//  Author: Justin Carpentier
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of INRIA nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef COAL_PYTHON_BROADPHASE_BROADPHASE_COLLISION_MANAGER_HH
#define COAL_PYTHON_BROADPHASE_BROADPHASE_COLLISION_MANAGER_HH

#include <eigenpy/eigenpy.hpp>

#include "coal/fwd.hh"
#include "coal/broadphase/broadphase_collision_manager.h"
#include "coal/broadphase/default_broadphase_callbacks.h"

#include "../coal.hh"

#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/coal/broadphase/broadphase_collision_manager.h"
#endif

#include <boost/algorithm/string/replace.hpp>
#include <boost/type_index.hpp>

namespace coal {

struct BroadPhaseCollisionManagerWrapper
    : BroadPhaseCollisionManager,
      bp::wrapper<BroadPhaseCollisionManager> {
  typedef BroadPhaseCollisionManager Base;

  void registerObjects(const std::vector<CollisionObject *> &other_objs) {
    this->get_override("registerObjects")(other_objs);
  }
  void registerObject(CollisionObject *obj) {
    this->get_override("registerObjects")(obj);
  }
  void unregisterObject(CollisionObject *obj) {
    this->get_override("unregisterObject")(obj);
  }

  void update(const std::vector<CollisionObject *> &other_objs) {
    this->get_override("update")(other_objs);
  }
  void update(CollisionObject *obj) { this->get_override("update")(obj); }
  void update() { this->get_override("update")(); }

  void setup() { this->get_override("setup")(); }
  void clear() { this->get_override("clear")(); }

  std::vector<CollisionObject *> getObjects() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
    return this->get_override("getObjects")();
#pragma GCC diagnostic pop
  }

  void collide(CollisionCallBackBase *callback) const {
    this->get_override("collide")(callback);
  }
  void collide(CollisionObject *obj, CollisionCallBackBase *callback) const {
    this->get_override("collide")(obj, callback);
  }
  void collide(BroadPhaseCollisionManager *other_manager,
               CollisionCallBackBase *callback) const {
    this->get_override("collide")(other_manager, callback);
  }

  void distance(DistanceCallBackBase *callback) const {
    this->get_override("distance")(callback);
  }
  void distance(CollisionObject *obj, DistanceCallBackBase *callback) const {
    this->get_override("collide")(obj, callback);
  }
  void distance(BroadPhaseCollisionManager *other_manager,
                DistanceCallBackBase *callback) const {
    this->get_override("collide")(other_manager, callback);
  }

  bool empty() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
    return this->get_override("empty")();
#pragma GCC diagnostic pop
  }
  size_t size() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
    return this->get_override("size")();
#pragma GCC diagnostic pop
  }

  static void expose() {
    bp::class_<BroadPhaseCollisionManagerWrapper, boost::noncopyable>(
        "BroadPhaseCollisionManager", bp::no_init)
        .def("registerObjects", bp::pure_virtual(&Base::registerObjects),
             doxygen::member_func_doc(&Base::registerObjects),
             bp::with_custodian_and_ward_postcall<1, 2>())
        .def("registerObject", bp::pure_virtual(&Base::registerObject),
             doxygen::member_func_doc(&Base::registerObject),
             bp::with_custodian_and_ward_postcall<1, 2>())
        .def("unregisterObject", bp::pure_virtual(&Base::unregisterObject),
             doxygen::member_func_doc(&Base::unregisterObject))

        .def("update", bp::pure_virtual((void (Base::*)())&Base::update),
             doxygen::member_func_doc((void (Base::*)())(&Base::update)))
        .def("update",
             bp::pure_virtual((void (Base::*)(
                 const std::vector<CollisionObject *> &))&Base::update),
             doxygen::member_func_doc((void (Base::*)(
                 const std::vector<CollisionObject *> &))(&Base::update)),
             bp::with_custodian_and_ward_postcall<1, 2>())
        .def("update",
             bp::pure_virtual(
                 (void (Base::*)(CollisionObject *obj))&Base::update),
             doxygen::member_func_doc(
                 (void (Base::*)(CollisionObject *obj))(&Base::update)),
             bp::with_custodian_and_ward_postcall<1, 2>())

        .def("setup", bp::pure_virtual(&Base::setup),
             doxygen::member_func_doc(&Base::setup))
        .def("clear", bp::pure_virtual(&Base::clear),
             doxygen::member_func_doc(&Base::clear))
        .def("empty", bp::pure_virtual(&Base::empty),
             doxygen::member_func_doc(&Base::empty))
        .def("size", bp::pure_virtual(&Base::size),
             doxygen::member_func_doc(&Base::size))

        .def("getObjects",
             bp::pure_virtual(
                 (std::vector<CollisionObject *> (Base::*)() const) &
                 Base::getObjects),
             doxygen::member_func_doc(
                 (std::vector<CollisionObject *> (Base::*)() const) &
                 Base::getObjects),
             bp::with_custodian_and_ward_postcall<0, 1>())

        .def("collide",
             bp::pure_virtual((void (Base::*)(CollisionCallBackBase *) const) &
                              Base::collide),
             doxygen::member_func_doc(
                 (void (Base::*)(CollisionCallBackBase *) const) &
                 Base::collide))
        .def("collide",
             bp::pure_virtual((void (Base::*)(CollisionObject *,
                                              CollisionCallBackBase *) const) &
                              Base::collide),
             doxygen::member_func_doc(
                 (void (Base::*)(CollisionObject *, CollisionCallBackBase *)
                      const) &
                 Base::collide))
        .def("collide",
             bp::pure_virtual((void (Base::*)(BroadPhaseCollisionManager *,
                                              CollisionCallBackBase *) const) &
                              Base::collide),
             doxygen::member_func_doc(
                 (void (Base::*)(BroadPhaseCollisionManager *,
                                 CollisionCallBackBase *) const) &
                 Base::collide))

        .def("distance",
             bp::pure_virtual((void (Base::*)(DistanceCallBackBase *) const) &
                              Base::distance),
             doxygen::member_func_doc(
                 (void (Base::*)(DistanceCallBackBase *) const) &
                 Base::distance))
        .def("distance",
             bp::pure_virtual((void (Base::*)(CollisionObject *,
                                              DistanceCallBackBase *) const) &
                              Base::distance),
             doxygen::member_func_doc(
                 (void (Base::*)(CollisionObject *, DistanceCallBackBase *)
                      const) &
                 Base::distance))
        .def("distance",
             bp::pure_virtual((void (Base::*)(BroadPhaseCollisionManager *,
                                              DistanceCallBackBase *) const) &
                              Base::distance),
             doxygen::member_func_doc(
                 (void (Base::*)(BroadPhaseCollisionManager *,
                                 DistanceCallBackBase *) const) &
                 Base::distance));
  }

  template <typename Derived>
  static void exposeDerived() {
    std::string class_name = boost::typeindex::type_id<Derived>().pretty_name();
    boost::algorithm::replace_all(class_name, "coal::", "");
#if defined(WIN32)
    boost::algorithm::replace_all(class_name, "class ", "");
#endif

    bp::class_<Derived, bp::bases<BroadPhaseCollisionManager> >(
        class_name.c_str(), bp::no_init)
        .def(dv::init<Derived>());
  }

};  // BroadPhaseCollisionManagerWrapper

}  // namespace coal

#endif  // ifndef COAL_PYTHON_BROADPHASE_BROADPHASE_COLLISION_MANAGER_HH
