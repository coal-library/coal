/// Copyright 2025 INRIA

#include "coal/fwd.hh"
#include "coal/hfield.h"

#include "coal/serialization/hfield.h"
#include "coal/serialization/geometric_shapes.h"

#include "pickle.hh"
#include "serializable.hh"

#include "fwd.h"

using namespace coal;
using namespace nb::literals;

template <typename BV>
void exposeHeightField(nb::module_& m, const char* name) {
  using Geometry = HeightField<BV>;
  using Base = typename Geometry::Base;
  using Node = typename Geometry::Node;
  nb::class_<Geometry, Base>(m, name)
      .def(nb::init<>())
      .def(nb::init<const HeightField<BV>&>(), "other"_a)
      .def(nb::init<Scalar, Scalar, const MatrixXs&, Scalar>(), "x_dim"_a,
           "y_dim"_a, "heights"_a, "min_height"_a = Scalar(0))

      .DEF_CLASS_FUNC(Geometry, getXDim)
      .DEF_CLASS_FUNC(Geometry, getYDim)
      .DEF_CLASS_FUNC(Geometry, getMinHeight)
      .DEF_CLASS_FUNC(Geometry, getMaxHeight)
      .DEF_CLASS_FUNC(Geometry, getNodeType)
      .DEF_CLASS_FUNC(Geometry, updateHeights)

      .def("clone", &Geometry::clone, nb::rv_policy::take_ownership)
      .def("getXGrid", &Geometry::getXGrid, nb::rv_policy::copy)
      .def("getYGrid", &Geometry::getYGrid, nb::rv_policy::copy)
      .def("getHeights", &Geometry::getHeights, nb::rv_policy::copy)
      .def(
          "getBV",
          [](Geometry& self, unsigned int index) -> Node& {
            return self.getBV(index);
          },
          nanobind::rv_policy::reference_internal)

      .def(python::v2::PickleVisitor<Geometry>())
      .def(python::v2::SerializableVisitor<Geometry>())

      .def(nanoeigenpy::IdVisitor());
}

void exposeHeightFields(nb::module_& m) {
  exposeHeightField<OBBRSS>(m, "HeightFieldOBBRSS");
  exposeHeightField<AABB>(m, "HeightFieldAABB");
}
