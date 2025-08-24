//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2019 CNRS-LAAS INRIA
//  Author: Joseph Mirabel
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of CNRS-LAAS. nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#include <eigenpy/eigenpy.hpp>

#include "coal.hh"

COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
#include "coal/fwd.hh"
#include "coal/distance.h"
#include "coal/serialization/collision_data.h"

#include "deprecation.hh"
COAL_COMPILER_DIAGNOSTIC_POP

#include "serializable.hh"

#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/coal/collision_data.h"
#endif

using namespace boost::python;
using namespace coal;
using namespace coal::python;

namespace dv = doxygen::visitor;

struct DistanceResultWrapper {
  static Vec3s getNearestPoint1(const DistanceResult& res) {
    return res.nearest_points[0];
  }
  static Vec3s getNearestPoint2(const DistanceResult& res) {
    return res.nearest_points[1];
  }
};

void exposeDistanceAPI() {
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  if (!eigenpy::register_symbolic_link_to_registered_type<DistanceRequest>()) {
    class_<DistanceRequest, bases<QueryRequest> >(
        "DistanceRequest", doxygen::class_doc<DistanceRequest>(),
        init<optional<bool, Scalar, Scalar> >(
            (arg("self"), arg("enable_nearest_points"), arg("rel_err"),
             arg("abs_err")),
            "Constructor"))
        .add_property(
            "enable_nearest_points",
            bp::make_function(
                +[](DistanceRequest& self) -> bool {
                  return self.enable_nearest_points;
                },
                deprecated_warning_policy<>(
                    "enable_nearest_points has been marked as deprecated. "
                    "Nearest points are always computed when computing "
                    "distance. They are the points of the shapes that achieve "
                    "a distance of "
                    "DistanceResult::min_distance.\n"
                    "Use `enable_signed_distance` if you want to compute a "
                    "signed minimum "
                    "distance (and thus its corresponding nearest points).")),
            bp::make_function(
                +[](DistanceRequest& self, const bool value) -> void {
                  self.enable_nearest_points = value;
                },
                deprecated_warning_policy<>(
                    "enable_nearest_points has been marked as deprecated. "
                    "Nearest points are always computed when computing "
                    "distance. They are the points of the shapes that achieve "
                    "a distance of "
                    "DistanceResult::min_distance.\n"
                    "Use `enable_signed_distance` if you want to compute a "
                    "signed minimum "
                    "distance (and thus its corresponding nearest points).")),
            doxygen::class_attrib_doc<DistanceRequest>("enable_nearest_points"))
        .DEF_RW_CLASS_ATTRIB(DistanceRequest, enable_signed_distance)
        .DEF_RW_CLASS_ATTRIB(DistanceRequest, rel_err)
        .DEF_RW_CLASS_ATTRIB(DistanceRequest, abs_err)
        .def(SerializableVisitor<DistanceRequest>());
  }
  COAL_COMPILER_DIAGNOSTIC_POP

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<DistanceRequest> >()) {
    class_<std::vector<DistanceRequest> >("StdVec_DistanceRequest")
        .def(vector_indexing_suite<std::vector<DistanceRequest> >());
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<DistanceResult>()) {
    class_<DistanceResult, bases<QueryResult> >(
        "DistanceResult", doxygen::class_doc<DistanceResult>(), no_init)
        .def(dv::init<DistanceResult>())
        .DEF_RW_CLASS_ATTRIB(DistanceResult, min_distance)
        .DEF_RW_CLASS_ATTRIB(DistanceResult, normal)
        //.def_readwrite ("nearest_points", &DistanceResult::nearest_points)
        .def("getNearestPoint1", &DistanceResultWrapper::getNearestPoint1,
             doxygen::class_attrib_doc<DistanceResult>("nearest_points"))
        .def("getNearestPoint2", &DistanceResultWrapper::getNearestPoint2,
             doxygen::class_attrib_doc<DistanceResult>("nearest_points"))
        .DEF_RO_CLASS_ATTRIB(DistanceResult, nearest_points)
        .DEF_RO_CLASS_ATTRIB(DistanceResult, o1)
        .DEF_RO_CLASS_ATTRIB(DistanceResult, o2)
        .DEF_RW_CLASS_ATTRIB(DistanceResult, b1)
        .DEF_RW_CLASS_ATTRIB(DistanceResult, b2)

        .def("clear", &DistanceResult::clear,
             doxygen::member_func_doc(&DistanceResult::clear))
        .def(SerializableVisitor<DistanceResult>());
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<DistanceResult> >()) {
    class_<std::vector<DistanceResult> >("StdVec_DistanceResult")
        .def(vector_indexing_suite<std::vector<DistanceResult> >());
  }

  doxygen::def(
      "distance",
      static_cast<Scalar (*)(const CollisionObject*, const CollisionObject*,
                             const DistanceRequest&, DistanceResult&)>(
          &distance));
  doxygen::def(
      "distance",
      static_cast<Scalar (*)(const CollisionGeometry*, const Transform3s&,
                             const CollisionGeometry*, const Transform3s&,
                             const DistanceRequest&, DistanceResult&)>(
          &distance));

  class_<ComputeDistance>("ComputeDistance",
                          doxygen::class_doc<ComputeDistance>(), no_init)
      .def(dv::init<ComputeDistance, const CollisionGeometry*,
                    const CollisionGeometry*>())
      .def("__call__",
           static_cast<Scalar (ComputeDistance::*)(
               const Transform3s&, const Transform3s&, const DistanceRequest&,
               DistanceResult&) const>(&ComputeDistance::operator()));
}
