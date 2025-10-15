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
#include <eigenpy/geometry.hpp>

#include "coal/fwd.hh"
#include "coal/math/transform.h"
#include "coal/serialization/transform.h"

#include "coal.hh"
#include "pickle.hh"
#include "serializable.hh"

#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/coal/math/transform.h"
#endif

using namespace boost::python;
using namespace coal;
using namespace coal::python;

namespace dv = doxygen::visitor;

struct TriangleWrapper {
  static Triangle::index_type getitem(const Triangle& t, int i) {
    if (i >= 3 || i <= -3)
      PyErr_SetString(PyExc_IndexError, "Index out of range");
    return t[static_cast<coal::Triangle::index_type>(i % 3)];
  }
  static void setitem(Triangle& t, int i, Triangle::index_type v) {
    if (i >= 3 || i <= -3)
      PyErr_SetString(PyExc_IndexError, "Index out of range");
    t[static_cast<coal::Triangle::index_type>(i % 3)] = v;
  }
};

void exposeMaths() {
  eigenpy::enableEigenPy();

  if (!eigenpy::register_symbolic_link_to_registered_type<Eigen::Quaterniond>())
    eigenpy::exposeQuaternion();
  if (!eigenpy::register_symbolic_link_to_registered_type<Eigen::AngleAxisd>())
    eigenpy::exposeAngleAxis();

  eigenpy::enableEigenPySpecific<Matrix3s>();
  eigenpy::enableEigenPySpecific<Vec3s>();

  class_<Transform3s>("Transform3s", doxygen::class_doc<Transform3s>(), no_init)
      .def(dv::init<Transform3s>())
      .def(dv::init<Transform3s, const Matrix3s::MatrixBase&,
                    const Vec3s::MatrixBase&>())
      .def(dv::init<Transform3s, const Quatf&, const Vec3s::MatrixBase&>())
      .def(dv::init<Transform3s, const Matrix3s&>())
      .def(dv::init<Transform3s, const Quatf&>())
      .def(dv::init<Transform3s, const Vec3s&>())
      .def(dv::init<Transform3s, const Transform3s&>())

      .def(dv::member_func("getQuatRotation", &Transform3s::getQuatRotation))
      .def("getTranslation", &Transform3s::getTranslation,
           doxygen::member_func_doc(&Transform3s::getTranslation),
           return_value_policy<copy_const_reference>())
      .def("getRotation", &Transform3s::getRotation,
           return_value_policy<copy_const_reference>())
      .def("isIdentity", &Transform3s::isIdentity,
           (bp::arg("self"),
            bp::arg("prec") = Eigen::NumTraits<CoalScalar>::dummy_precision()),
           doxygen::member_func_doc(&Transform3s::getTranslation))

      .def(dv::member_func("setQuatRotation", &Transform3s::setQuatRotation))
      .def("setTranslation", &Transform3s::setTranslation<Vec3s>)
      .def("setRotation", &Transform3s::setRotation<Matrix3s>)
      .def(dv::member_func("setTransform",
                           &Transform3s::setTransform<Matrix3s, Vec3s>))
      .def(dv::member_func(
          "setTransform",
          static_cast<void (Transform3s::*)(const Quatf&, const Vec3s&)>(
              &Transform3s::setTransform)))
      .def(dv::member_func("setIdentity", &Transform3s::setIdentity))
      .def(dv::member_func("Identity", &Transform3s::Identity))
      .staticmethod("Identity")

      .def(dv::member_func("setRandom", &Transform3s::setRandom))
      .def(dv::member_func("Random", &Transform3s::Random))
      .staticmethod("Random")

      .def(dv::member_func("transform", &Transform3s::transform<Vec3s>))
      .def("inverseInPlace", &Transform3s::inverseInPlace,
           return_internal_reference<>(),
           doxygen::member_func_doc(&Transform3s::inverseInPlace))
      .def(dv::member_func("inverse", &Transform3s::inverse))
      .def(dv::member_func("inverseTimes", &Transform3s::inverseTimes))

      .def(self * self)
      .def(self *= self)
      .def(self == self)
      .def(self != self)
      .def_pickle(PickleObject<Transform3s>())
      .def(SerializableVisitor<Transform3s>());

  class_<Triangle>("Triangle", no_init)
      .def(dv::init<Triangle>())
      .def(dv::init<Triangle, Triangle::index_type, Triangle::index_type,
                    Triangle::index_type>())
      .def("__getitem__", &TriangleWrapper::getitem)
      .def("__setitem__", &TriangleWrapper::setitem)
      .def(dv::member_func("set", &Triangle::set))
      .def(dv::member_func("size", &Triangle::size))
      .staticmethod("size")
      .def(self == self);

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<Vec3s> >()) {
    class_<std::vector<Vec3s> >("StdVec_Vec3s")
        .def(vector_indexing_suite<std::vector<Vec3s> >());
  }
  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<Triangle> >()) {
    class_<std::vector<Triangle> >("StdVec_Triangle")
        .def(vector_indexing_suite<std::vector<Triangle> >());
  }
}
