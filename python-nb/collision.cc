/// Copyright 2025 INRIA

#include "coal/fwd.hh"
COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
#include "coal/collision.h"
#include "coal/serialization/collision_data.h"
COAL_COMPILER_DIAGNOSTIC_POP

#include "serializable.hh"

#include "fwd.h"
#include <nanobind/operators.h>

using namespace coal;
using namespace nb::literals;

void exposeCollisionAPI(nb::module_& m) {
  nb::enum_<CollisionRequestFlag>(m, "CollisionRequestFlag")
      .value("CONTACT", CONTACT)
      .value("DISTANCE_LOWER_BOUND", DISTANCE_LOWER_BOUND)
      .value("NO_REQUEST", NO_REQUEST)
      .export_values();

  nb::class_<CPUTimes>(m, "CPUTimes")
      .def_ro("wall", &CPUTimes::wall, "wall time in micro seconds (us)")
      .def_ro("user", &CPUTimes::user, "user time in micro seconds (us)")
      .def_ro("system", &CPUTimes::system, "system time in micro seconds (us)")
      .def("clear", &CPUTimes::clear, "Reset the time values.");

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  nb::class_<QueryRequest>(m, "QueryRequest")
      .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_tolerance)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_max_iterations)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_variant)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_convergence_criterion)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_convergence_criterion_type)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, gjk_initial_guess)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, enable_cached_gjk_guess)
      .def_prop_rw(
          "enable_cached_gjk_guess",
          [](QueryRequest& self) -> bool {
            return self.enable_cached_gjk_guess;
          },
          [](QueryRequest& self, bool value) {
            self.enable_cached_gjk_guess = value;
          })
      .DEF_RW_CLASS_ATTRIB(QueryRequest, cached_gjk_guess)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, cached_support_func_guess)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, epa_max_iterations)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, epa_tolerance)
      .DEF_RW_CLASS_ATTRIB(QueryRequest, enable_timings)
      .DEF_CLASS_FUNC(QueryRequest, updateGuess);
  COAL_COMPILER_DIAGNOSTIC_POP

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  nb::class_<CollisionRequest, QueryRequest>(m, "CollisionRequest")
      .def(nb::init<>())
      .def(nb::init<const CollisionRequestFlag, size_t>(), "flag"_a,
           "num_max_contacts_"_a)
      .DEF_RW_CLASS_ATTRIB(CollisionRequest, num_max_contacts)
      .DEF_RW_CLASS_ATTRIB(CollisionRequest, enable_contact)
      .def_prop_rw(
          "enable_distance_lower_bound",
          [](CollisionRequest& self) -> bool {
            return self.enable_distance_lower_bound;
          },
          [](CollisionRequest& self, bool value) {
            self.enable_distance_lower_bound = value;
          })
      .DEF_RW_CLASS_ATTRIB(CollisionRequest, security_margin)
      .DEF_RW_CLASS_ATTRIB(CollisionRequest, break_distance)
      .DEF_RW_CLASS_ATTRIB(CollisionRequest, distance_upper_bound)
      .def(python::v2::SerializableVisitor<CollisionRequest>());

  nb::bind_vector<std::vector<CollisionRequest>>(m, "StdVec_CollisionRequest");
  COAL_COMPILER_DIAGNOSTIC_POP

  nb::class_<Contact>(m, "Contact")
      .def(nb::init<>())
      .def(nb::init<const CollisionGeometry*, const CollisionGeometry*, int,
                    int>(),
           "o1_"_a, "o2_"_a, "b1_"_a, "b2_"_a)
      .def(nb::init<const CollisionGeometry*, const CollisionGeometry*, int,
                    int, const Vec3s&, const Vec3s&, Scalar>(),
           "o1_"_a, "o2_"_a, "b1_"_a, "b2_"_a, "pos_"_a, "normal_"_a,
           "depth_"_a)
      .def_prop_ro("o1",
                   [](Contact& self) -> CollisionGeometry* {
                     return const_cast<CollisionGeometry*>(self.o1);
                   })
      .def_prop_ro("o2",
                   [](Contact& self) -> CollisionGeometry* {
                     return const_cast<CollisionGeometry*>(self.o2);
                   })
      .def("getNearestPoint1",
           [](const Contact& self) -> Vec3s { return self.nearest_points[0]; })
      .def("getNearestPoint2",
           [](const Contact& self) -> Vec3s { return self.nearest_points[1]; })
      .DEF_RW_CLASS_ATTRIB(Contact, b1)
      .DEF_RW_CLASS_ATTRIB(Contact, b2)
      .DEF_RW_CLASS_ATTRIB(Contact, normal)
      .DEF_RW_CLASS_ATTRIB(Contact, nearest_points)
      .DEF_RW_CLASS_ATTRIB(Contact, pos)
      .DEF_RW_CLASS_ATTRIB(Contact, penetration_depth)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self);

  nb::bind_vector<std::vector<Contact>>(m, "StdVec_Contact");

  nb::class_<QueryResult>(m, "QueryResult")
      .DEF_RW_CLASS_ATTRIB(QueryResult, cached_gjk_guess)
      .DEF_RW_CLASS_ATTRIB(QueryResult, cached_support_func_guess)
      .DEF_RW_CLASS_ATTRIB(QueryResult, timings);

  nb::class_<CollisionResult, QueryResult>(m, "CollisionResult")
      .def(nb::init<>())
      .DEF_CLASS_FUNC(CollisionResult, isCollision)
      .DEF_CLASS_FUNC(CollisionResult, numContacts)
      .DEF_CLASS_FUNC(CollisionResult, addContact)
      .DEF_CLASS_FUNC(CollisionResult, clear)
      .def("getContact", &CollisionResult::getContact, nb::rv_policy::copy)
      .def("getContacts",
           [](const CollisionResult& self, std::vector<Contact>& contacts) {
             self.getContacts(contacts);
           })
      .def(
          "getContacts",
          [](const CollisionResult& self) -> const std::vector<Contact>& {
            return self.getContacts();
          },
          nb::rv_policy::reference_internal)
      .DEF_RW_CLASS_ATTRIB(CollisionResult, distance_lower_bound)
      .def(python::v2::SerializableVisitor<CollisionResult>());

  nb::bind_vector<std::vector<CollisionResult>>(m, "StdVec_CollisionResult");

  m.def(
      "collide",
      [](const CollisionObject* o1, const CollisionObject* o2,
         const CollisionRequest& request,
         CollisionResult& result) -> std::size_t {
        return collide(o1, o2, request, result);
      },
      "o1"_a, "o2"_a, "request"_a, "result"_a);

  m.def(
      "collide",
      [](const CollisionGeometry* o1, const Transform3s& tf1,
         const CollisionGeometry* o2, const Transform3s& tf2,
         const CollisionRequest& request,
         CollisionResult& result) -> std::size_t {
        return collide(o1, tf1, o2, tf2, request, result);
      },
      "geom1"_a, "transform1"_a, "geom2"_a, "transform2"_a, "request"_a,
      "result"_a);

  nb::class_<ComputeCollision>(m, "ComputeCollision")
      .def(nb::init<const CollisionGeometry*, const CollisionGeometry*>(),
           "o1"_a, "o2"_a)
      .def("call",
           [](const ComputeCollision& self, const Transform3s& tf1,
              const Transform3s& tf2, const CollisionRequest& request,
              CollisionResult& result) -> std::size_t {
             return self(tf1, tf2, request, result);
           });
}