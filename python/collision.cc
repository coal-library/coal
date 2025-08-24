//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2019-2021 CNRS-LAAS, INRIA
//  Author: Joseph Mirabel
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
//   * Neither the name of CNRS-LAAS. nor the names of its
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

#include <eigenpy/eigenpy.hpp>

#include "coal/fwd.hh"
COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
#include "coal/collision.h"
#include "coal/serialization/collision_data.h"
COAL_COMPILER_DIAGNOSTIC_POP

#include "coal.hh"
#include "deprecation.hh"
#include "serializable.hh"

#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/coal/collision_data.h"
#endif

#include "../doc/python/doxygen.hh"
#include "../doc/python/doxygen-boost.hh"

using namespace boost::python;
using namespace coal;
using namespace coal::python;

namespace dv = doxygen::visitor;

template <int index>
const CollisionGeometry* geto(const Contact& c) {
  return index == 1 ? c.o1 : c.o2;
}

struct ContactWrapper {
  static Vec3s getNearestPoint1(const Contact& contact) {
    return contact.nearest_points[0];
  }
  static Vec3s getNearestPoint2(const Contact& contact) {
    return contact.nearest_points[1];
  }
};

void exposeCollisionAPI() {
  if (!eigenpy::register_symbolic_link_to_registered_type<
          CollisionRequestFlag>()) {
    enum_<CollisionRequestFlag>("CollisionRequestFlag")
        .value("CONTACT", CONTACT)
        .value("DISTANCE_LOWER_BOUND", DISTANCE_LOWER_BOUND)
        .value("NO_REQUEST", NO_REQUEST)
        .export_values();
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<CPUTimes>()) {
    class_<CPUTimes>("CPUTimes", no_init)
        .def_readonly("wall", &CPUTimes::wall,
                      "wall time in micro seconds (us)")
        .def_readonly("user", &CPUTimes::user,
                      "user time in micro seconds (us)")
        .def_readonly("system", &CPUTimes::system,
                      "system time in micro seconds (us)")
        .def("clear", &CPUTimes::clear, arg("self"), "Reset the time values.");
  }

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  if (!eigenpy::register_symbolic_link_to_registered_type<QueryRequest>()) {
    class_<QueryRequest>("QueryRequest", doxygen::class_doc<QueryRequest>(),
                         no_init)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_tolerance)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_max_iterations)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_variant)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_convergence_criterion)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_convergence_criterion_type)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_initial_guess)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, enable_cached_gjk_guess)
        .add_property(
            "enable_cached_gjk_guess",
            bp::make_function(
                +[](QueryRequest& self) -> bool {
                  return self.enable_cached_gjk_guess;
                },
                deprecated_warning_policy<>(
                    "enable_cached_gjk_guess has been marked as deprecated and "
                    "will be removed in a future release.\n"
                    "Please use gjk_initial_guess instead.")),
            bp::make_function(
                +[](QueryRequest& self, const bool value) -> void {
                  self.enable_cached_gjk_guess = value;
                },
                deprecated_warning_policy<>(
                    "enable_cached_gjk_guess has been marked as deprecated and "
                    "will be removed in a future release.\n"
                    "Please use gjk_initial_guess instead.")),
            doxygen::class_attrib_doc<QueryRequest>("enable_cached_gjk_guess"))
        .DEF_RW_CLASS_ATTRIB(QueryRequest, cached_gjk_guess)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, cached_support_func_guess)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, epa_max_iterations)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, epa_tolerance)
        .DEF_RW_CLASS_ATTRIB(QueryRequest, enable_timings)
        .DEF_CLASS_FUNC(QueryRequest, updateGuess);
  }
  COAL_COMPILER_DIAGNOSTIC_POP

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  if (!eigenpy::register_symbolic_link_to_registered_type<CollisionRequest>()) {
    class_<CollisionRequest, bases<QueryRequest> >(
        "CollisionRequest", doxygen::class_doc<CollisionRequest>(), no_init)
        .def(dv::init<CollisionRequest>())
        .def(dv::init<CollisionRequest, const CollisionRequestFlag, size_t>())
        .DEF_RW_CLASS_ATTRIB(CollisionRequest, num_max_contacts)
        .DEF_RW_CLASS_ATTRIB(CollisionRequest, enable_contact)
        .add_property(
            "enable_distance_lower_bound",
            bp::make_function(
                +[](CollisionRequest& self) -> bool {
                  return self.enable_distance_lower_bound;
                },
                deprecated_warning_policy<>(
                    "enable_distance_lower_bound has been marked as "
                    "deprecated. "
                    "A lower bound on distance is always computed.\n")),
            bp::make_function(
                +[](CollisionRequest& self, const bool value) -> void {
                  self.enable_distance_lower_bound = value;
                },
                deprecated_warning_policy<>(
                    "enable_distance_lower_bound has been marked as "
                    "deprecated. "
                    "A lower bound on distance is always computed.\n")),
            doxygen::class_attrib_doc<CollisionRequest>(
                "enable_distance_lower_bound"))
        .DEF_RW_CLASS_ATTRIB(CollisionRequest, security_margin)
        .DEF_RW_CLASS_ATTRIB(CollisionRequest, break_distance)
        .DEF_RW_CLASS_ATTRIB(CollisionRequest, distance_upper_bound)
        .def(SerializableVisitor<CollisionRequest>());
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<CollisionRequest> >()) {
    class_<std::vector<CollisionRequest> >("StdVec_CollisionRequest")
        .def(vector_indexing_suite<std::vector<CollisionRequest> >());
  }
  COAL_COMPILER_DIAGNOSTIC_POP

  if (!eigenpy::register_symbolic_link_to_registered_type<Contact>()) {
    class_<Contact>("Contact", doxygen::class_doc<Contact>(),
                    init<>(arg("self"), "Default constructor"))
        .def(dv::init<Contact, const CollisionGeometry*,
                      const CollisionGeometry*, int, int>())
        .def(dv::init<Contact, const CollisionGeometry*,
                      const CollisionGeometry*, int, int, const Vec3s&,
                      const Vec3s&, Scalar>())
        .add_property(
            "o1",
            make_function(&geto<1>,
                          return_value_policy<reference_existing_object>()),
            doxygen::class_attrib_doc<Contact>("o1"))
        .add_property(
            "o2",
            make_function(&geto<2>,
                          return_value_policy<reference_existing_object>()),
            doxygen::class_attrib_doc<Contact>("o2"))
        .def("getNearestPoint1", &ContactWrapper::getNearestPoint1,
             doxygen::class_attrib_doc<Contact>("nearest_points"))
        .def("getNearestPoint2", &ContactWrapper::getNearestPoint2,
             doxygen::class_attrib_doc<Contact>("nearest_points"))
        .DEF_RW_CLASS_ATTRIB(Contact, b1)
        .DEF_RW_CLASS_ATTRIB(Contact, b2)
        .DEF_RW_CLASS_ATTRIB(Contact, normal)
        .DEF_RW_CLASS_ATTRIB(Contact, nearest_points)
        .DEF_RW_CLASS_ATTRIB(Contact, pos)
        .DEF_RW_CLASS_ATTRIB(Contact, penetration_depth)
        .def(self == self)
        .def(self != self);
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<Contact> >()) {
    class_<std::vector<Contact> >("StdVec_Contact")
        .def(vector_indexing_suite<std::vector<Contact> >());
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<QueryResult>()) {
    class_<QueryResult>("QueryResult", doxygen::class_doc<QueryResult>(),
                        no_init)
        .DEF_RW_CLASS_ATTRIB(QueryResult, cached_gjk_guess)
        .DEF_RW_CLASS_ATTRIB(QueryResult, cached_support_func_guess)
        .DEF_RW_CLASS_ATTRIB(QueryResult, timings);
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<CollisionResult>()) {
    class_<CollisionResult, bases<QueryResult> >(
        "CollisionResult", doxygen::class_doc<CollisionResult>(), no_init)
        .def(dv::init<CollisionResult>())
        .DEF_CLASS_FUNC(CollisionResult, isCollision)
        .DEF_CLASS_FUNC(CollisionResult, numContacts)
        .DEF_CLASS_FUNC(CollisionResult, addContact)
        .DEF_CLASS_FUNC(CollisionResult, clear)
        .DEF_CLASS_FUNC2(CollisionResult, getContact,
                         return_value_policy<copy_const_reference>())
        .def(dv::member_func(
            "getContacts",
            static_cast<void (CollisionResult::*)(std::vector<Contact>&) const>(
                &CollisionResult::getContacts)))
        .def("getContacts",
             static_cast<const std::vector<Contact>& (CollisionResult::*)()
                             const>(&CollisionResult::getContacts),
             doxygen::member_func_doc(
                 static_cast<const std::vector<Contact>& (CollisionResult::*)()
                                 const>(&CollisionResult::getContacts)),
             return_internal_reference<>())

        .DEF_RW_CLASS_ATTRIB(CollisionResult, distance_lower_bound)
        .def(SerializableVisitor<CollisionResult>());
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<CollisionResult> >()) {
    class_<std::vector<CollisionResult> >("StdVec_CollisionResult")
        .def(vector_indexing_suite<std::vector<CollisionResult> >());
  }

  doxygen::def("collide",
               static_cast<std::size_t (*)(
                   const CollisionObject*, const CollisionObject*,
                   const CollisionRequest&, CollisionResult&)>(&collide));
  doxygen::def(
      "collide",
      static_cast<std::size_t (*)(const CollisionGeometry*, const Transform3s&,
                                  const CollisionGeometry*, const Transform3s&,
                                  const CollisionRequest&, CollisionResult&)>(
          &collide));

  class_<ComputeCollision>("ComputeCollision",
                           doxygen::class_doc<ComputeCollision>(), no_init)
      .def(dv::init<ComputeCollision, const CollisionGeometry*,
                    const CollisionGeometry*>())
      .def("__call__",
           static_cast<std::size_t (ComputeCollision::*)(
               const Transform3s&, const Transform3s&, const CollisionRequest&,
               CollisionResult&) const>(&ComputeCollision::operator()));
}
