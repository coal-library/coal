/// Copyright 2025 INRIA

#ifndef COAL_PYTHON_NB_SERIALIZABLE_HH
#define COAL_PYTHON_NB_SERIALIZABLE_HH

#include "fwd.h"

#include "coal/serialization/archive.h"
#include "coal/serialization/serializer.h"

namespace coal::python {
namespace v2 {
using Serializer = ::coal::serialization::Serializer;

template <typename Derived>
struct SerializableVisitor : nb::def_visitor<SerializableVisitor<Derived>> {
  template <class... Ts>
  void execute(nb::class_<Derived, Ts...>& cl) {
    using namespace nb::literals;
    cl.def("saveToText", &Serializer::saveToText<Derived>, nb::arg("filename"),
           "Saves *this inside a text file.")
        .def("loadFromText", &Serializer::loadFromText<Derived>,
             nb::arg("filename"), "Loads *this from a text file.")

        .def("saveToString", &Serializer::saveToString<Derived>,
             "Parses the current object to a string.")
        .def("loadFromString", &Serializer::loadFromString<Derived>, "string"_a,
             "Parses from the input string the content of the current object.")

        .def("saveToXML", &Serializer::saveToXML<Derived>, "filename"_a,
             "tag_name"_a, "Saves *this inside a XML file.")
        .def("loadFromXML", &Serializer::loadFromXML<Derived>, "filename"_a,
             "tag_name"_a, "Loads *this from a XML file.")

        .def("saveToBinary", &Serializer::saveToBinary<Derived>, "filename"_a,
             "Saves *this inside a binary file.")
        .def("loadFromBinary", &Serializer::loadFromBinary<Derived>,
             "filename"_a, "Loads *this from a binary file.")

        .def("saveToBuffer", &Serializer::saveToBuffer<Derived>, "buffer"_a,
             "Saves *this inside a binary buffer.")
        .def("loadFromBuffer", &Serializer::loadFromBuffer<Derived>, "buffer"_a,
             "Loads *this from a binary buffer.");
  }
};

}  // namespace v2
}  // namespace coal::python

#endif  // ifndef COAL_PYTHON_NB_SERIALIZABLE_HH
