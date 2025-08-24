/// Copyright 2025 INRIA

#include "coal/fwd.hh"
#include "coal/hfield.h"

#include "coal/serialization/AABB.h"

#include "pickle.hh"
#include "serializable.hh"

#include "fwd.h"
#include <nanobind/operators.h>

using namespace coal;
using namespace nb::literals;

void exposeAABB(nb::module_& m) {
  nb::class_<AABB>(
      m, "AABB",
      "A class describing the AABB collision structure, which is a "
      "box in 3D space determined by two diagonal points")
      .def(nb::init<>())
      .def(nb::init<AABB>(), "other"_a)
      .def(nb::init<Vec3s>(), "v"_a)
      .def(nb::init<Vec3s, Vec3s>(), "a"_a, "b"_a)
      .def(nb::init<AABB, Vec3s>(), "core"_a, "delta"_a)
      .def(nb::init<Vec3s, Vec3s, Vec3s>(), "a"_a, "b"_a, "c"_a)

      .def(
          "contain",
          [](const AABB& self, const Vec3s& p) { return self.contain(p); },
          "p"_a, "Check whether the AABB contains a point p.")
      .def(
          "contain",
          [](const AABB& self, const AABB& other) {
            return self.contain(other);
          },
          "other"_a, "Check whether the AABB contains another AABB.")
      .def(
          "overlap",
          [](const AABB& self, const AABB& other) {
            return self.overlap(other);
          },
          "other"_a, "Check whether two AABB overlap.")
      .def(
          "overlap",
          [](const AABB& self, const AABB& other, AABB& overlapping_part) {
            return self.overlap(other, overlapping_part);
          },
          "other"_a, "overlapping_part"_a,
          "Check whether two AABB overlap and return the overlapping part if "
          "true.")
      .def(
          "distance",
          [](const AABB& self, const AABB& other) {
            return self.distance(other);
          },
          "other"_a, "Distance between two AABBs.")

      .def_prop_rw(
          "min_", [](AABB& self) -> Vec3s& { return self.min_; },
          [](AABB& self, const Vec3s& min_) { self.min_ = min_; },
          "The min point in the AABB.")
      .def_prop_rw(
          "max_", [](AABB& self) -> Vec3s& { return self.max_; },
          [](AABB& self, const Vec3s& max_) { self.max_ = max_; },
          "The max point in the AABB.")

      .def(nb::self == nb::self)
      .def(nb::self != nb::self)
      .def(nb::self + nb::self)
      .def(nb::self += nb::self)
      .def(nb::self += Vec3s())

      .def("size", &AABB::volume)
      .def("center", &AABB::center)
      .def("width", &AABB::width)
      .def("height", &AABB::height)
      .def("depth", &AABB::depth)
      .def("volume", &AABB::volume)

      .def(
          "expand",
          [](AABB& self, const AABB& other, Scalar scalar) -> AABB& {
            return self.expand(other, scalar);
          },
          nb::rv_policy::reference_internal)
      .def(
          "expand",
          [](AABB& self, Scalar scalar) -> AABB& {
            return self.expand(scalar);
          },
          nb::rv_policy::reference_internal)
      .def(
          "expand",
          [](AABB& self, const Vec3s& vec) -> AABB& {
            return self.expand(vec);
          },
          nb::rv_policy::reference_internal)

      .def(python::v2::PickleVisitor<AABB>())
      .def(python::v2::SerializableVisitor<AABB>())
      .def(nanoeigenpy::IdVisitor());
}
