/// Copyright 2025 INRIA

#include "coal/config.hh"
#include "coal/mesh_loader/loader.h"

#include "fwd.h"

using namespace nb::literals;

inline constexpr bool checkVersionAtLeast(int major, int minor, int patch) {
  return COAL_VERSION_AT_LEAST(major, minor, patch);
}

inline constexpr bool checkVersionAtMost(int major, int minor, int patch) {
  return COAL_VERSION_AT_MOST(major, minor, patch);
}

void exposeVersion(nb::module_ &m) {
  m.attr("__version__") = COAL_VERSION;
  m.attr("COAL_MAJOR_VERSION") = COAL_MAJOR_VERSION;
  m.attr("COAL_MINOR_VERSION") = COAL_MINOR_VERSION;
  m.attr("COAL_PATCH_VERSION") = COAL_PATCH_VERSION;

  m.attr("WITH_QHULL") =
#if COAL_HAS_QHULL
      true;
#else
      false;
#endif

  m.attr("WITH_OCTOMAP") =
#if COAL_HAS_OCTOMAP
      true;
#else
      false;
#endif

  m.def("checkVersionAtLeast", &checkVersionAtLeast, "major"_a, "minor"_a,
        "patch"_a,
        "Checks if the current version of coal is at least the version "
        "provided by the input arguments.");

  m.def("checkVersionAtMost", &checkVersionAtMost, "major"_a, "minor"_a,
        "patch"_a,
        "Checks if the current version of coal is at most the version provided "
        "by the input arguments.");
}

void exposeMeshLoader(nb::module_ &m) {
  using namespace coal;
  nb::handle cl_cur = nb::type<MeshLoader>();
  if (cl_cur.is_valid()) {
    return;
  }

  if (!nb::type<MeshLoader>().is_valid()) {
    nb::class_<MeshLoader>(m, "MeshLoader")
        .def(nb::init<NODE_TYPE>(), "node_type"_a = BV_OBBRSS)
        .def("load", &MeshLoader::load, "filename"_a, "scale"_a = Vec3s::Ones())
        .def("loadOctree", &MeshLoader::loadOctree, "filename"_a);
  }

  if (!nb::type<CachedMeshLoader>().is_valid()) {
    nb::class_<CachedMeshLoader, MeshLoader>(m, "CachedMeshLoader")
        .def(nb::init<NODE_TYPE>(), "node_type"_a = BV_OBBRSS);
  }
}

void exposeMaths(nb::module_ &m);
void exposeCollisionGeometries(nb::module_ &m);
void exposeCollisionObject(nb::module_ &m);
void exposeCollisionAPI(nb::module_ &m);
void exposeContactPatchAPI(nb::module_ &m);
void exposeDistanceAPI(nb::module_ &m);
void exposeGJK(nb::module_ &m);
#ifdef COAL_HAS_OCTOMAP
void exposeOctree(nb::module_ &m);
#endif
void exposeBroadPhase(nb::module_ &m);

NB_MODULE(COAL_PYTHON_LIBNAME, m) {
  exposeVersion(m);
  exposeMaths(m);
  exposeCollisionGeometries(m);
  exposeCollisionObject(m);
  exposeCollisionAPI(m);
  exposeContactPatchAPI(m);
  exposeDistanceAPI(m);
  exposeGJK(m);
  exposeMeshLoader(m);
#ifdef COAL_HAS_OCTOMAP
  exposeOctree(m);
#endif
  exposeBroadPhase(m);
}
