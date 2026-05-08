//
// Copyright (c) 2026 INRIA
// This file was borrowed from the Pinocchio library:
// https://github.com/stack-of-tasks/pinocchio
//

#ifndef COAL_PYTHON_PRINTABLE_H
#define COAL_PYTHON_PRINTABLE_H

#include <boost/python.hpp>

namespace coal {
namespace python {

namespace bp = boost::python;

///
/// \brief Set the Python method __str__ and __repr__ to use the overloading
/// operator<<.
///
template <class C>
struct PrintableVisitor : public bp::def_visitor<PrintableVisitor<C>> {
  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::self_ns::str(bp::self_ns::self))
        .def(bp::self_ns::repr(bp::self_ns::self));
  }
};

}  // namespace python
}  // namespace coal

#endif  // ifndef COAL_PYTHON_PRINTABLE_H
