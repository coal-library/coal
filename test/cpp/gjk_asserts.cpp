#define BOOST_TEST_MODULE COAL_GJK_ASSERTS

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

void CreateSphereMesh(BVHModel<OBBRSS>& model, const Scalar& radius) {
  size_t polarSteps{32};
  size_t azimuthSteps{32};

  const float PI = static_cast<float>(pi);

  const float polarStep = PI / (float)(polarSteps - 1);
  const float azimuthStep = 2.0f * PI / (float)(azimuthSteps - 1);
  std::vector<Vec3s> vertices;
  std::vector<Triangle32> triangles;

  for (size_t p = 0; p < polarSteps; ++p) {
    for (size_t a = 0; a < azimuthSteps; ++a) {
      const float x =
          std::sin((float)p * polarStep) * std::cos((float)a * azimuthStep);
      const float y =
          std::sin((float)p * polarStep) * std::sin((float)a * azimuthStep);
      const float z = std::cos((float)p * polarStep);
      vertices.emplace_back(radius * x, radius * y, radius * z);
    }
  }

  for (size_t p = 0; p < polarSteps - 1; ++p) {
    for (size_t a = 0; a < azimuthSteps - 1; ++a) {
      size_t p0 = p * azimuthSteps + a;
      size_t p1 = p * azimuthSteps + (a + 1);
      size_t p2 = (p + 1) * azimuthSteps + (a + 1);
      size_t p3 = (p + 1) * azimuthSteps + a;
      triangles.emplace_back(p0, p2, p1);
      triangles.emplace_back(p0, p3, p2);
    }
  }
  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

BOOST_AUTO_TEST_CASE(TestSpheres) {
  BVHModel<OBBRSS> sphere1{};
  BVHModel<OBBRSS> sphere2{};

  CreateSphereMesh(sphere1, 1.);
  CreateSphereMesh(sphere2, 2.);

  CollisionRequest request(CONTACT | DISTANCE_LOWER_BOUND, 1);

  ComputeCollision compute(&sphere2, &sphere1);

  Transform3s sphere1Tf = Transform3s::Identity();
  Transform3s sphere2Tf = Transform3s::Identity();

  for (int i = 0; i < 360; ++i) {
    for (int j = 0; j < 180; ++j) {
      if (
          /// assertion: src/narrowphase/gjk.cpp:331
          (i == 5 && j == 48) || (i == 64 && j == 151) ||
          (i == 98 && j == 47) || (i == 355 && j == 48) ||
          /// assertion: src/narrowphase/gjk.cpp:1263
          (i == 86 && j == 52) || (i == 89 && j == 17) ||
          (i == 89 && j == 58) || (i == 89 && j == 145)) {
        sphere2Tf.setQuatRotation(
            Eigen::AngleAxis<Scalar>(DegToRad(Scalar(i)), Vec3s::UnitZ()) *
            Eigen::AngleAxis<Scalar>(DegToRad(Scalar(j)), Vec3s::UnitY()));
        for (const Vec3s& dir : dirs) {
          sphere2Tf.setTranslation(dir);
          CollisionResult result;

          BOOST_CHECK_NO_THROW(compute(sphere2Tf, sphere1Tf, request, result));
        }
      }
    }
  }
}
