/// Copyright 2025 INRIA

#include "coal/fwd.hh"
#include "coal/shape/geometric_shapes.h"
#include "coal/shape/convex.h"
#include "coal/BVH/BVH_model.h"

#include "coal/serialization/geometric_shapes.h"
#include "coal/serialization/convex.h"

#include "pickle.hh"
#include "serializable.hh"

#include "fwd.h"

using namespace coal;
using namespace nb::literals;

typedef std::vector<Vec3s> Vec3ss;

typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor> RowMatrixX3;
typedef Eigen::Map<RowMatrixX3> MapRowMatrixX3;
typedef Eigen::Ref<RowMatrixX3> RefRowMatrixX3;
typedef Eigen::Map<VecXs> MapVecXs;
typedef Eigen::Ref<VecXs> RefVecXs;

template <typename IndexType>
void exposeConvexBase(nb::module_& m, const std::string& classname) {
  typedef ConvexBaseTpl<IndexType> ConvexBaseType;

  nb::class_<ConvexBaseType, ShapeBase>(m, classname.c_str())
      .DEF_RO_CLASS_ATTRIB(ConvexBaseType, center)
      .DEF_RO_CLASS_ATTRIB(ConvexBaseType, num_points)
      .DEF_RO_CLASS_ATTRIB(ConvexBaseType, num_normals_and_offsets)
      .def(
          "point",
          [](const ConvexBaseType& convex, unsigned int i) -> Vec3s& {
            if (i >= convex.num_points) {
              throw std::out_of_range("index is out of range");
            }
            return (*(convex.points))[i];
          },
          "index"_a, "Retrieve the point given by its index.",
          nb::rv_policy::reference_internal)
      .def(
          "points",
          [](const ConvexBaseType& convex, unsigned int i) -> Vec3s& {
            if (i >= convex.num_points) {
              throw std::out_of_range("index is out of range");
            }
            return (*(convex.points))[i];
          },
          "index"_a, "Retrieve the point given by its index.",
          nb::rv_policy::reference_internal)
      .def(
          "points",
          [](const ConvexBaseType& convex) -> RefRowMatrixX3 {
            return MapRowMatrixX3((*(convex.points))[0].data(),
                                  convex.num_points, 3);
          },
          "Retrieve all the points.", nb::rv_policy::reference_internal)
      .def(
          "normal",
          [](const ConvexBaseType& convex, unsigned int i) -> Vec3s& {
            if (i >= convex.num_normals_and_offsets) {
              throw std::out_of_range("index is out of range");
            }
            return (*(convex.normals))[i];
          },
          "index"_a, "Retrieve the normal given by its index.",
          nb::rv_policy::reference_internal)
      .def(
          "normals",
          [](const ConvexBaseType& convex) -> RefRowMatrixX3 {
            return MapRowMatrixX3((*(convex.normals))[0].data(),
                                  convex.num_normals_and_offsets, 3);
          },
          "Retrieve all the normals.", nb::rv_policy::reference_internal)
      .def(
          "offset",
          [](const ConvexBaseType& convex, unsigned int i) -> Scalar {
            if (i >= convex.num_normals_and_offsets) {
              throw std::out_of_range("index is out of range");
            }
            return (*(convex.offsets))[i];
          },
          "index"_a, "Retrieve the offset given by its index.")
      .def(
          "offsets",
          [](const ConvexBaseType& convex) -> RefVecXs {
            return MapVecXs(convex.offsets->data(),
                            convex.num_normals_and_offsets, 1);
          },
          "Retrieve all the offsets.", nb::rv_policy::reference_internal)
      .def("neighbors",
           [](const ConvexBaseType& convex, unsigned int i) -> nb::list {
             if (i >= convex.num_points) {
               throw std::out_of_range("index is out of range");
             }
             nb::list n;
             const std::vector<typename ConvexBaseType::Neighbors>& neighbors_ =
                 *(convex.neighbors);
             for (unsigned char j = 0; j < neighbors_[i].count; ++j) {
               n.append(convex.neighbor(IndexType(i), j));
             }
             return n;
           })
      .def_static(
          "convexHull",
          [](const Vec3ss& points, bool keepTri,
             nb::handle qhullCommand) -> ConvexBaseType* {
            const char* qhullCommand_a = nullptr;
            if (!qhullCommand.is_none()) {
              qhullCommand_a = nb::cast<std::string>(qhullCommand).c_str();
            }
            return ConvexBaseType::convexHull(points.data(),
                                              (unsigned int)points.size(),
                                              keepTri, qhullCommand_a);
          },
          "points"_a, "keepTri"_a, "qhullCommand"_a = nb::none(),
          nb::rv_policy::take_ownership)
      .def("clone", &ConvexBaseType::clone, nb::rv_policy::take_ownership);
}

template <typename PolygonT>
void exposeConvex(nb::module_& m, const std::string& classname) {
  typedef ConvexTpl<PolygonT> ConvexType;
  typedef ConvexBaseTpl<typename PolygonT::IndexType> ConvexBaseType;
  typedef typename PolygonT::IndexType IndexType;
  typedef TriangleTpl<IndexType> TriangleType;

  nb::class_<ConvexType, ConvexBaseType>(m, classname.c_str())
      .def(nb::init<>())
      .def("__init__",
           [](ConvexType* self, const Vec3ss& pts,
              const std::vector<TriangleType>& tri) {
             std::shared_ptr<Vec3ss> points(new Vec3ss(pts.size()));
             Vec3ss& points_ = *points;
             for (std::size_t i = 0; i < pts.size(); ++i) {
               points_[i] = pts[i];
             }

             std::shared_ptr<std::vector<TriangleType>> tris(
                 new std::vector<TriangleType>(tri.size()));
             std::vector<TriangleType>& tris_ = *tris;
             for (std::size_t i = 0; i < tri.size(); ++i) {
               tris_[i] = tri[i];
             }
             new (self) ConvexType(*(shared_ptr<ConvexType>(
                 new ConvexType(points, (unsigned int)pts.size(), tris,
                                (unsigned int)tri.size()))));
           })
      .def(nb::init<const ConvexType&>(), "other_"_a)
      .DEF_RO_CLASS_ATTRIB(ConvexType, num_polygons)
      .def(
          "polygons",
          [](const ConvexType& convex, unsigned int i) -> TriangleType {
            if (i >= convex.num_polygons) {
              throw std::out_of_range("index is out of range");
            }
            return (*convex.polygons)[i];
          },
          "index"_a, "Retrieve the polygon given by its index.")
      .def(python::v2::PickleVisitor<ConvexType>())
      .def(python::v2::SerializableVisitor<ConvexType>())
      .def(nanoeigenpy::IdVisitor());
}

void exposeShapes(nb::module_& m) {
  nb::class_<ShapeBase, CollisionGeometry>(m, "ShapeBase")
      .def("setSweptSphereRadius", &ShapeBase::setSweptSphereRadius)
      .def("getSweptSphereRadius", &ShapeBase::getSweptSphereRadius);

  nb::class_<Box, ShapeBase>(m, "Box")
      .def(nb::init<>())
      .def(nb::init<const Box&>(), "other"_a)
      .def(nb::init<Scalar, Scalar, Scalar>(), "x"_a, "y"_a, "z"_a)
      .def(nb::init<const Vec3s&>(), "side_"_a)
      .DEF_RW_CLASS_ATTRIB(Box, halfSide)
      .def("clone", &Box::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Box>())
      .def(python::v2::SerializableVisitor<Box>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<Capsule, ShapeBase>(m, "Capsule")
      .def(nb::init<>())
      .def(nb::init<Scalar, Scalar>(), "radius"_a, "lz_"_a)
      .def(nb::init<const Capsule&>(), "other_"_a)
      .DEF_RW_CLASS_ATTRIB(Capsule, radius)
      .DEF_RW_CLASS_ATTRIB(Capsule, halfLength)
      .def("clone", &Capsule::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Capsule>())
      .def(python::v2::SerializableVisitor<Capsule>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<Cone, ShapeBase>(m, "Cone")
      .def(nb::init<>())
      .def(nb::init<Scalar, Scalar>(), "radius"_a, "lz_"_a)
      .def(nb::init<const Cone&>(), "other_"_a)
      .DEF_RW_CLASS_ATTRIB(Cone, radius)
      .DEF_RW_CLASS_ATTRIB(Cone, halfLength)
      .def("clone", &Cone::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Cone>())
      .def(python::v2::SerializableVisitor<Cone>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<Cylinder, ShapeBase>(m, "Cylinder")
      .def(nb::init<>())
      .def(nb::init<Scalar, Scalar>(), "radius"_a, "lz_"_a)
      .def(nb::init<const Cylinder&>(), "other_"_a)
      .DEF_RW_CLASS_ATTRIB(Cylinder, radius)
      .DEF_RW_CLASS_ATTRIB(Cylinder, halfLength)
      .def("clone", &Cylinder::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Cylinder>())
      .def(python::v2::SerializableVisitor<Cylinder>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<Halfspace, ShapeBase>(m, "Halfspace")
      .def(nb::init<>())
      .def(nb::init<const Vec3s&, Scalar>(), "n_"_a, "d_"_a)
      .def(nb::init<const Halfspace&>(), "other_"_a)
      .def(nb::init<Scalar, Scalar, Scalar, Scalar>(), "a"_a, "b"_a, "c"_a,
           "d_"_a)
      .DEF_RW_CLASS_ATTRIB(Halfspace, n)
      .DEF_RW_CLASS_ATTRIB(Halfspace, d)
      .def("clone", &Halfspace::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Halfspace>())
      .def(python::v2::SerializableVisitor<Halfspace>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<Plane, ShapeBase>(m, "Plane")
      .def(nb::init<>())
      .def(nb::init<const Vec3s&, Scalar>(), "n_"_a, "d_"_a)
      .def(nb::init<const Plane&>(), "other_"_a)
      .def(nb::init<Scalar, Scalar, Scalar, Scalar>(), "a"_a, "b"_a, "c"_a,
           "d_"_a)
      .DEF_RW_CLASS_ATTRIB(Plane, n)
      .DEF_RW_CLASS_ATTRIB(Plane, d)
      .def("clone", &Plane::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Plane>())
      .def(python::v2::SerializableVisitor<Plane>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<Sphere, ShapeBase>(m, "Sphere")
      .def(nb::init<>())
      .def(nb::init<Scalar>(), "radius_"_a)
      .def(nb::init<const Sphere&>(), "other_"_a)
      .DEF_RW_CLASS_ATTRIB(Sphere, radius)
      .def("clone", &Sphere::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Sphere>())
      .def(python::v2::SerializableVisitor<Sphere>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<Ellipsoid, ShapeBase>(m, "Ellipsoid")
      .def(nb::init<>())
      .def(nb::init<Scalar, Scalar, Scalar>(), "rx"_a, "ry"_a, "rz"_a)
      .def(nb::init<Vec3s>(), "radii"_a)
      .def(nb::init<const Ellipsoid&>(), "other_"_a)
      .DEF_RW_CLASS_ATTRIB(Ellipsoid, radii)
      .def("clone", &Ellipsoid::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<Ellipsoid>())
      .def(python::v2::SerializableVisitor<Ellipsoid>())
      .def(nanoeigenpy::IdVisitor());

  nb::class_<TriangleP, ShapeBase>(m, "TriangleP")
      .def(nb::init<>())
      .def(nb::init<const Vec3s&, const Vec3s&, const Vec3s&>(), "a_"_a, "b_"_a,
           "c_"_a)
      .def(nb::init<const TriangleP&>(), "other_"_a)
      .DEF_RW_CLASS_ATTRIB(TriangleP, a)
      .DEF_RW_CLASS_ATTRIB(TriangleP, b)
      .DEF_RW_CLASS_ATTRIB(TriangleP, c)
      .def("clone", &TriangleP::clone, nb::rv_policy::take_ownership)
      .def(python::v2::PickleVisitor<TriangleP>())
      .def(python::v2::SerializableVisitor<TriangleP>())
      .def(nanoeigenpy::IdVisitor());

  exposeConvexBase<Triangle16::IndexType>(m, "ConvexBase16");
  exposeConvexBase<Triangle32::IndexType>(m, "ConvexBase32");
  exposeConvex<Triangle16>(m, "Convex16");
  exposeConvex<Triangle32>(m, "Convex32");
  m.attr("Convex") = m.attr("Convex32");
}