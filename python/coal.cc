//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2019-2022 CNRS-LAAS INRIA
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

#include "coal/fwd.hh"
#include "coal/shape/geometric_shapes.h"
#include "coal/BVH/BVH_model.h"

#include "coal/mesh_loader/loader.h"

#include "coal/collision.h"

#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/coal/mesh_loader/loader.h"
#endif

#include "doc/autodoc/doxygen.hh"
#include "doc/autodoc/doxygen-boost.hh"

using namespace coal;
namespace dv = doxygen::visitor;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(load_overloads, MeshLoader::load, 1, 2)
#pragma GCC diagnostic pop

void exposeMeshLoader() {
  using namespace boost::python;

  if (!eigenpy::register_symbolic_link_to_registered_type<MeshLoader>()) {
    class_<MeshLoader, shared_ptr<MeshLoader> >(
        "MeshLoader", doxygen::class_doc<MeshLoader>(),
        init<optional<NODE_TYPE> >(
            (arg("self"), arg("node_type")),
            doxygen::constructor_doc<MeshLoader, const NODE_TYPE&>()))
        .def("load", &MeshLoader::load,
             load_overloads((arg("self"), arg("filename"), arg("scale")),
                            doxygen::member_func_doc(&MeshLoader::load)))
        .def(dv::member_func("loadOctree", &MeshLoader::loadOctree));
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<CachedMeshLoader>()) {
    class_<CachedMeshLoader, bases<MeshLoader>, shared_ptr<CachedMeshLoader> >(
        "CachedMeshLoader", doxygen::class_doc<MeshLoader>(),
        init<optional<NODE_TYPE> >(
            (arg("self"), arg("node_type")),
            doxygen::constructor_doc<CachedMeshLoader, const NODE_TYPE&>()));
  }
}

BOOST_PYTHON_MODULE(coal_pywrap) {
  namespace bp = boost::python;

  PyImport_ImportModule("warnings");
  eigenpy::enableEigenPy();

  exposeVersion();
  exposeMaths();
  exposeCollisionGeometries();
  exposeCollisionObject();
  exposeMeshLoader();
  exposeCollisionAPI();
  exposeContactPatchAPI();
  exposeDistanceAPI();
  exposeGJK();
#ifdef COAL_HAS_OCTOMAP
  exposeOctree();
#endif
  exposeBroadPhase();
}
