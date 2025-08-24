
#include "coal.hh"

#include <eigenpy/std-vector.hpp>

#include "coal/fwd.hh"
#include "coal/octree.h"

#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/coal/octree.h"
#endif

bp::object toPyBytes(std::vector<uint8_t>& bytes) {
#if PY_MAJOR_VERSION == 2
  PyObject* py_buf =
      PyBuffer_FromMemory((char*)bytes.data(), Py_ssize_t(bytes.size()));
  return bp::object(bp::handle<>(py_buf));
#else
  PyObject* py_buf = PyMemoryView_FromMemory(
      (char*)bytes.data(), Py_ssize_t(bytes.size()), PyBUF_READ);
  return bp::object(bp::handle<>(PyBytes_FromObject(py_buf)));
#endif
}

bp::object tobytes(const coal::OcTree& self) {
  std::vector<uint8_t> bytes = self.tobytes();
  return toPyBytes(bytes);
}

void exposeOctree() {
  using namespace coal;
  namespace bp = boost::python;
  namespace dv = doxygen::visitor;

  bp::class_<OcTree, bp::bases<CollisionGeometry>, shared_ptr<OcTree> >(
      "OcTree", doxygen::class_doc<OcTree>(), bp::no_init)
      .def(dv::init<OcTree, Scalar>())
      .def("clone", &OcTree::clone, doxygen::member_func_doc(&OcTree::clone),
           bp::return_value_policy<bp::manage_new_object>())
      .def(dv::member_func("getTreeDepth", &OcTree::getTreeDepth))
      .def(dv::member_func("size", &OcTree::size))
      .def(dv::member_func("getResolution", &OcTree::getResolution))
      .def(dv::member_func("getOccupancyThres", &OcTree::getOccupancyThres))
      .def(dv::member_func("getFreeThres", &OcTree::getFreeThres))
      .def(dv::member_func("getDefaultOccupancy", &OcTree::getDefaultOccupancy))
      .def(dv::member_func("setCellDefaultOccupancy",
                           &OcTree::setCellDefaultOccupancy))
      .def(dv::member_func("setOccupancyThres", &OcTree::setOccupancyThres))
      .def(dv::member_func("setFreeThres", &OcTree::setFreeThres))
      .def(dv::member_func("getRootBV", &OcTree::getRootBV))
      .def(dv::member_func("toBoxes", &OcTree::toBoxes))
      .def("tobytes", tobytes, doxygen::member_func_doc(&OcTree::tobytes));

  doxygen::def("makeOctree", &makeOctree);
  eigenpy::enableEigenPySpecific<Vec6s>();
  eigenpy::StdVectorPythonVisitor<std::vector<Vec6s>, true>::expose(
      "StdVec_Vec6");
}
