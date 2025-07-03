/// Copyright 2025 INRIA

#include "coal/fwd.hh"
#include "coal/octree.h"

#include "fwd.h"

using namespace coal;
using namespace nb::literals;

nb::bytes tobytes(const OcTree &self) {
  std::vector<uint8_t> bytes = self.tobytes();
  return nb::bytes(reinterpret_cast<const char *>(bytes.data()), bytes.size());
}

void exposeOctree(nb::module_ &m) {
  nb::class_<OcTree, CollisionGeometry>(m, "OcTree")
      .def(nb::init<Scalar>(), "resolution"_a)
      .def("clone", &OcTree::clone, nb::rv_policy::take_ownership)
      .def("getTreeDepth", &OcTree::getTreeDepth)
      .def("size", &OcTree::size)
      .def("getResolution", &OcTree::getResolution)
      .def("getOccupancyThres", &OcTree::getOccupancyThres)
      .def("getFreeThres", &OcTree::getFreeThres)
      .def("getDefaultOccupancy", &OcTree::getDefaultOccupancy)
      .def("setCellDefaultOccupancy", &OcTree::setCellDefaultOccupancy)
      .def("setOccupancyThres", &OcTree::setOccupancyThres)
      .def("setFreeThres", &OcTree::setFreeThres)
      .def("getRootBV", &OcTree::getRootBV)
      .def("toBoxes", &OcTree::toBoxes)
      .def("tobytes", &tobytes);
}
