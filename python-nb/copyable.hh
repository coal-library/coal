/// Copyright 2025 INRIA

#ifndef COAL_PYTHON_NB_COPYABLE_HH
#define COAL_PYTHON_NB_COPYABLE_HH

#include <type_traits>

#include "fwd.h"

namespace coal::python {
namespace v2 {

namespace internal {
template <typename T, typename = void>
struct has_clone : std::false_type {};

template <typename T>
struct has_clone<T, std::void_t<decltype(std::declval<const T&>().clone())>>
    : std::true_type {};
}  // namespace internal

/// Binds `copy()` and `__copy__` to the copy constructor of `T`, and
/// `__deepcopy__` to `T::clone()` when `T` has one (e.g. geometries, whose
/// copy constructor may share memory), to the copy constructor otherwise.
template <typename T>
struct CopyableVisitor : nb::def_visitor<CopyableVisitor<T>> {
  template <class PyClass, class... Extra>
  void execute(PyClass& cl, const Extra&...) const {
    using namespace nb::literals;
    cl.def("copy", &copy, "Returns a copy of this object.",
           nb::rv_policy::take_ownership)
        .def("__copy__", &copy, "Returns a copy of this object.",
             nb::rv_policy::take_ownership)
        .def(
            "__deepcopy__",
            [](const T& self, nb::dict) -> T* { return deepcopy(self); },
            "memo"_a, "Returns a deep copy of this object.",
            nb::rv_policy::take_ownership);
  }

 private:
  static T* copy(const T& self) { return new T(self); }

  static T* deepcopy(const T& self) {
    if constexpr (internal::has_clone<T>::value) {
      using Clone = decltype(self.clone());
      if constexpr (std::is_pointer_v<Clone>) {
        // Catches a T that inherits clone() without overriding it: that
        // clone() copies a base class object, not a T.
        static_assert(std::is_convertible_v<Clone, T*>,
                      "T::clone() must return a T*.");
        return self.clone();
      } else {
        return new T(self.clone());
      }
    } else {
      return new T(self);
    }
  }
};

}  // namespace v2
}  // namespace coal::python

#endif  // ifndef COAL_PYTHON_NB_COPYABLE_HH
