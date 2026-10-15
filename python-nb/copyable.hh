/// Copyright 2025 INRIA

#ifndef COAL_PYTHON_NB_COPYABLE_HH
#define COAL_PYTHON_NB_COPYABLE_HH

#include <type_traits>

#include "fwd.h"

namespace coal::python {
namespace v2 {

namespace internal {
template <typename T, typename = void>
struct has_deepcopy : std::false_type {};

template <typename T>
struct has_deepcopy<T,
                    std::void_t<decltype(std::declval<const T&>().deepcopy())>>
    : std::true_type {};
}  // namespace internal

/// Binds `copy()`, `__copy__` and `__deepcopy__`, so that `copy()`,
/// `copy.copy` and `copy.deepcopy` return an independent copy of the object.
/// The copy is made with `T::deepcopy()` when `T` has one (e.g. the copy
/// constructor of a Convex shares its points), with the copy constructor
/// otherwise.
template <typename T>
struct CopyableVisitor : nb::def_visitor<CopyableVisitor<T>> {
  template <class... Ts>
  void execute(nb::class_<T, Ts...>& cl) {
    using namespace nb::literals;
    cl.def("copy", &copy, "Returns a copy of this object.",
           nb::rv_policy::take_ownership)
        .def("__copy__", &copy, "Returns a copy of this object.",
             nb::rv_policy::take_ownership)
        .def(
            "__deepcopy__",
            [](const T& self, nb::dict) -> T* { return copy(self); }, "memo"_a,
            "Returns a copy of this object.", nb::rv_policy::take_ownership);
  }

 private:
  static T* copy(const T& self) {
    if constexpr (internal::has_deepcopy<T>::value) {
      return self.deepcopy();
    } else {
      return new T(self);
    }
  }
};

}  // namespace v2
}  // namespace coal::python

#endif  // ifndef COAL_PYTHON_NB_COPYABLE_HH
