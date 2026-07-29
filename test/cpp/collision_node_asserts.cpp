#define BOOST_TEST_MODULE COAL_COLLISION_NODE_ASSERT

#include <boost/test/included/unit_test.hpp>
#include <boost/math/constants/constants.hpp>
#include "coal/BVH/BVH_model.h"
#include "coal/collision.h"

using namespace coal;

constexpr Scalar pi = boost::math::constants::pi<Scalar>();

Scalar DegToRad(const Scalar& deg) {
  static Scalar degToRad = pi / Scalar(180);
  return deg * degToRad;
}
std::vector<Vec3s> dirs{Vec3s::UnitZ(),  -Vec3s::UnitZ(), Vec3s::UnitY(),
                        -Vec3s::UnitY(), Vec3s::UnitX(),  -Vec3s::UnitX()};

BOOST_AUTO_TEST_CASE(TestTriangles) {
  std::vector<Vec3s> triVertices{Vec3s(1, 0, 0), Vec3s(1, 1, 0),
                                 Vec3s(0, 1, 0)};
  std::vector<Triangle32> triangle{{0, 1, 2}};

  BVHModel<OBBRSS> tri1{};
  BVHModel<OBBRSS> tri2{};

  tri1.beginModel();
  tri1.addSubModel(triVertices, triangle);
  tri1.endModel();

  tri2.beginModel();
  tri2.addSubModel(triVertices, triangle);
  tri2.endModel();

  CollisionRequest request(CONTACT | DISTANCE_LOWER_BOUND, 1);

  ComputeCollision compute(&tri1, &tri2);

  Transform3s tri1Tf{};
  Transform3s tri2Tf{};

  /// check some angles for two triangles
  for (int i = 0; i < 360; i += 30) {
    for (int j = 0; j < 180; j += 30) {
      for (int k = 0; k < 180; k += 30) {
        tri1Tf.setQuatRotation(
            Eigen::AngleAxis<Scalar>(0., Vec3s::UnitZ()) *
            Eigen::AngleAxis<Scalar>(DegToRad(Scalar(k)), Vec3s::UnitY()));
        tri2Tf.setQuatRotation(
            Eigen::AngleAxis<Scalar>(DegToRad(Scalar(i)), Vec3s::UnitZ()) *
            Eigen::AngleAxis<Scalar>(DegToRad(Scalar(j)), Vec3s::UnitY()));
        CollisionResult result;

        /// assertion: src/collision_node.cpp:58
        BOOST_CHECK_NO_THROW(compute(tri2Tf, tri1Tf, request, result));
      }
    }
  }
}
