/// Copyright 2025 INRIA

#ifndef COAL_PYTHON_NB_COPYABLE_HH
#define COAL_PYTHON_NB_COPYABLE_HH

#include "fwd.h"

namespace coal::python {
namespace v2 {

/// Binds `copy()`, `__copy__` and `__deepcopy__` to the copy constructor of
/// `T`. Geometries whose copy constructor shares memory (e.g. Convex) expose
/// `clone()` to get a deep copy.
template <typename T>
struct CopyableVisitor : nb::def_visitor<CopyableVisitor<T>> {
  template <class PyClass, class... Extra>
  void execute(PyClass& cl, const Extra&...) const {
    const auto copy = [](const T& self) -> T { return T(self); };
    cl.def("copy", copy, "Returns a copy of this object.")
        .def("__copy__", copy, "Returns a copy of this object.")
        .def("__deepcopy__",
             [](const T& self, nb::dict) -> T { return T(self); });
  }
};

}  // namespace v2
}  // namespace coal::python

#endif  // ifndef COAL_PYTHON_NB_COPYABLE_HH
