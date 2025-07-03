/// Copyright 2025 INRIA

#include "coal/BVH/BVH_model.h"
#include "coal/serialization/BVH_model.h"

#include "pickle.hh"
#include "serializable.hh"

#include "fwd.h"

using namespace coal;
using namespace nb::literals;

typedef std::vector<Vec3s> Vec3ss;
typedef std::vector<Triangle> Triangles;

template <typename BV>
void exposeBVHModel(nb::module_& m, const char* name) {
  using BVHType = BVHModel<BV>;
  nb::class_<BVHType, BVHModelBase>(m, name)
      .def(nb::init<>())
      .def(nb::init<const BVHType&>(), "other"_a)
      .DEF_CLASS_FUNC(BVHType, getNumBVs)
      .DEF_CLASS_FUNC(BVHType, makeParentRelative)
      .DEF_CLASS_FUNC(BVHType, memUsage)
      .def("clone", &BVHType::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<BVHType>())
      .def(python::v2::SerializableVisitor<BVHType>());
}

void exposeBVHModels(nb::module_& m) {
  using RowMatrixX3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
  using MapRowMatrixX3 = Eigen::Map<RowMatrixX3>;
  nb::class_<BVHModelBase, CollisionGeometry>(m, "BVHModelBase")
      .def("vertex",
           [](BVHModelBase& bvh, size_t i) -> Vec3s& {
             if (i >= bvh.num_vertices)
               throw nb::index_error("Vertex index out of range.");
             return (*bvh.vertices)[i];
           })
      .def(
          "vertices",
          [](BVHModelBase& bvh) {
            if (bvh.num_vertices > 0) {
              return MapRowMatrixX3{bvh.vertices->data()->data(),
                                    bvh.num_vertices, 3};
            } else {
              return MapRowMatrixX3{nullptr, 0, 3};
            }
          },
          "Retrieve all vertices", nb::rv_policy::reference_internal)
      .def(
          "tri_indices",
          [](const BVHModelBase& bvh, size_t i) {
            if (i >= bvh.num_tris)
              nb::index_error("Triangle index out of range.");

            return (*bvh.tri_indices)[i];
          },
          "index"_a, "Retrieve the triangle given by its index.")
      .def_ro("num_vertices", &BVHModelBase::num_vertices)
      .def_ro("num_tris", &BVHModelBase::num_tris)
      .def_ro("build_state", &BVHModelBase::build_state)

      .def_ro("convex", &BVHModelBase::convex)

      .def("buildConvexRepresentation",
           &BVHModelBase::buildConvexRepresentation, "share_memory"_a)
      .def("buildConvexHull", &BVHModelBase::buildConvexHull, "keepTriangle"_a,
           "qhullCommand"_a = NULL)

      .DEF_CLASS_FUNC(BVHModelBase, beginModel)
      .DEF_CLASS_FUNC(BVHModelBase, addVertex)
      .DEF_CLASS_FUNC(BVHModelBase, addVertices)
      .DEF_CLASS_FUNC(BVHModelBase, addTriangle)
      .DEF_CLASS_FUNC(BVHModelBase, addTriangles)
      .def("addSubModel",
           [](BVHModelBase& self, const Vec3ss& vec, const Triangles& tri) {
             return self.addSubModel(vec, tri);
           })
      .def("addSubModel",
           [](BVHModelBase& self, const Vec3ss& vec) {
             return self.addSubModel(vec);
           })
      .DEF_CLASS_FUNC(BVHModelBase, endModel)
      .DEF_CLASS_FUNC(BVHModelBase, beginReplaceModel)
      .DEF_CLASS_FUNC(BVHModelBase, replaceVertex)
      .DEF_CLASS_FUNC(BVHModelBase, replaceTriangle)
      .DEF_CLASS_FUNC(BVHModelBase, replaceSubModel)
      .DEF_CLASS_FUNC(BVHModelBase, endReplaceModel)
      .DEF_CLASS_FUNC(BVHModelBase, getModelType)

      ;

  exposeBVHModel<OBB>(m, "BVHModelOBB");
  exposeBVHModel<OBBRSS>(m, "BVHModelOBBRSS");
}