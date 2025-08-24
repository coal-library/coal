/// Copyright 2025 INRIA

#include "coal/data_types.h"
#include "coal/math/transform.h"
#include "coal/serialization/transform.h"

#include "serializable.hh"
#include "pickle.hh"

#include "fwd.h"
#include <nanobind/operators.h>
#include <nanoeigenpy/geometry.hpp>

using namespace coal;
using namespace nb::literals;

template <typename IndexType>
void exposeTriangle(nb::module_ &m, const std::string &classname) {
  using TriangleType = TriangleTpl<IndexType>;

  nb::class_<TriangleType>(m, classname.c_str())
      .def(nb::init<>())
      .def(nb::init<IndexType, IndexType, IndexType>(), "p1"_a, "p2"_a, "p3"_a)
      .def("__getitem__",
           [](TriangleType &m, Py_ssize_t i) {
             if (i >= 3 || i <= -3) {
               throw nb::index_error("Index out of range");
             }
             return m[IndexType(i % 3)];
           })
      .def("__setitem__",
           [](TriangleType &m, Py_ssize_t i, IndexType v) {
             if (i >= 3 || i <= -3) {
               throw nb::index_error("Index out of range");
             }
             m[IndexType(i % 3)] = v;
           })
      .def("set", &TriangleType::set)
      .def_static("size", &TriangleType::size)
      .def(nb::self == nb::self);
}

void exposeMaths(nb::module_ &m) {
  nanoeigenpy::exposeQuaternion<Scalar>(m, "Quaternion");
  nanoeigenpy::exposeAngleAxis<Scalar>(m, "AngleAxis");

  nb::class_<Transform3s>(m, "Transform3s")
      .def(nb::init<>())
      .def(nb::init<const Matrix3s &, Vec3s>(), "R"_a, "t"_a)
      .def(nb::init<const Quats &, Vec3s>(), "q"_a, "t"_a)
      .def(nb::init<const Matrix3s &>(), "R"_a)
      .def(nb::init<const Quats &>(), "q"_a)
      .def(nb::init<const Vec3s &>(), "t"_a)
      .def(nb::init<const Transform3s &>())

      .def("getQuatRotation", &Transform3s::getQuatRotation,
           nb::rv_policy::automatic_reference)
      .def("getTranslation", &Transform3s::getTranslation, nb::rv_policy::copy)
      .def("setTranslation",
           [](Transform3s &t, Vec3s v) { t.setTranslation(v); })
      .def("getRotation", &Transform3s::getRotation, nb::rv_policy::copy)
      .def("setRotation", [](Transform3s &t, Matrix3s R) { t.setRotation(R); })
      .def("isIdentity", &Transform3s::isIdentity)

      .def("setQuatRotation", &Transform3s::setQuatRotation)
      .def("setTransform",
           [](Transform3s &self, const Matrix3s &R, const Vec3s &t) {
             self.setTransform<Matrix3s, Vec3s>(R, t);
           })
      .def("setTransform",
           [](Transform3s &self, const Quats &quat, const Vec3s &vec) {
             self.setTransform(quat, vec);
           })
      .def("setIdentity", &Transform3s::setIdentity)
      .def_static("Identity", &Transform3s::Identity)

      .def("setRandom", &Transform3s::setRandom)
      .def_static("Random", &Transform3s::Random)

      .def("transform", [](const Transform3s &self,
                           const Vec3s &v) { return self.transform<Vec3s>(v); })
      .def("inverseInPlace", &Transform3s::inverseInPlace,
           nb::rv_policy::automatic_reference)
      .def("inverse", &Transform3s::inverse)
      .def("inverseTimes", &Transform3s::inverseTimes)

      .def(nb::self * nb::self)
      .def(nb::self *= nb::self)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)

      .def(python::v2::PickleVisitor<Transform3s>())
      .def(python::v2::SerializableVisitor<Transform3s>());

  exposeTriangle<Triangle32::IndexType>(m, "Triangle32");
  m.attr("Triangle") = m.attr("Triangle32");
  exposeTriangle<Triangle16::IndexType>(m, "Triangle16");

  nb::bind_vector<std::vector<Triangle32>>(m, "StdVec_Triangle32");
  m.attr("StdVec_Triangle") = m.attr("StdVec_Triangle32");
  nb::bind_vector<std::vector<Triangle16>>(m, "StdVec_Triangle16");

  nb::bind_vector<std::vector<Vec3s>>(m, "StdVec_Vec3s");
}
