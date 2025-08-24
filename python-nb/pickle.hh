/// Copyright 2025 INRIA

#ifndef COAL_PYTHON_NB_PICKLE_HH
#define COAL_PYTHON_NB_PICKLE_HH

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <sstream>

#include "fwd.h"
#include <nanobind/stl/string.h>

namespace coal::python {
namespace v2 {

/// See: https://nanobind.readthedocs.io/en/latest/classes.html#pickling
template <typename T>
struct PickleVisitor : nb::def_visitor<PickleVisitor<T>> {
  template <class... Ts>
  void execute(nb::class_<T, Ts...> &cl) {
    using namespace nb::literals;
    cl.def("__getstate__", [](const T &obj) -> std::string {
        std::stringstream ss;
        boost::archive::text_oarchive oa(ss);
        oa & obj;
        return ss.str();
      }).def("__setstate__", [](T &obj, const std::string &state) {
      std::istringstream is(state);
      boost::archive::text_iarchive ia(is, boost::archive::no_codecvt);
      new (&obj) T();
      ia >> obj;
    });
  }
};

}  // namespace v2
}  // namespace coal::python

#endif  // ifndef COAL_PYTHON_NB_PICKLE_HH
