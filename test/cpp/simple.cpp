#define BOOST_TEST_MODULE COAL_SIMPLE
#include <boost/test/included/unit_test.hpp>

#include "coal/internal/intersect.h"
#include "coal/collision.h"
#include "coal/BVH/BVH_model.h"
#include "fcl_resources/config.h"
#include <sstream>

using namespace coal;

static Scalar epsilon = Scalar(1e-6);

static bool approx(Scalar x, Scalar y) { return std::abs(x - y) < epsilon; }

BOOST_AUTO_TEST_CASE(projection_test_line) {
  Vec3s v1(0, 0, 0);
  Vec3s v2(2, 0, 0);

  Vec3s p(1, 0, 0);
  Project<Scalar>::ProjectResult res = Project<Scalar>::projectLine(v1, v2, p);
  BOOST_CHECK(res.encode == 3);
  BOOST_CHECK(approx(res.sqr_distance, 0));
  BOOST_CHECK(approx(res.parameterization[0], 0.5));
  BOOST_CHECK(approx(res.parameterization[1], 0.5));

  p = Vec3s(-1, 0, 0);
  res = Project<Scalar>::projectLine(v1, v2, p);
  BOOST_CHECK(res.encode == 1);
  BOOST_CHECK(approx(res.sqr_distance, 1));
  BOOST_CHECK(approx(res.parameterization[0], 1));
  BOOST_CHECK(approx(res.parameterization[1], 0));

  p = Vec3s(3, 0, 0);
  res = Project<Scalar>::projectLine(v1, v2, p);
  BOOST_CHECK(res.encode == 2);
  BOOST_CHECK(approx(res.sqr_distance, 1));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 1));
}

BOOST_AUTO_TEST_CASE(projection_test_triangle) {
  Vec3s v1(0, 0, 1);
  Vec3s v2(0, 1, 0);
  Vec3s v3(1, 0, 0);

  Vec3s p(1, 1, 1);
  Project<Scalar>::ProjectResult res =
      Project<Scalar>::projectTriangle(v1, v2, v3, p);
  BOOST_CHECK(res.encode == 7);
  BOOST_CHECK(approx(res.sqr_distance, Scalar(4 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[0], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[1], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[2], Scalar(1 / 3.0)));

  p = Vec3s(0, 0, 1.5);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  BOOST_CHECK(res.encode == 1);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 1));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 0));

  p = Vec3s(1.5, 0, 0);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  BOOST_CHECK(res.encode == 4);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 1));

  p = Vec3s(0, 1.5, 0);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  BOOST_CHECK(res.encode == 2);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 1));
  BOOST_CHECK(approx(res.parameterization[2], 0));

  p = Vec3s(1, 1, 0);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  BOOST_CHECK(res.encode == 6);
  BOOST_CHECK(approx(res.sqr_distance, 0.5));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 0.5));
  BOOST_CHECK(approx(res.parameterization[2], 0.5));

  p = Vec3s(1, 0, 1);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  BOOST_CHECK(res.encode == 5);
  BOOST_CHECK(approx(res.sqr_distance, 0.5));
  BOOST_CHECK(approx(res.parameterization[0], 0.5));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 0.5));

  p = Vec3s(0, 1, 1);
  res = Project<Scalar>::projectTriangle(v1, v2, v3, p);
  BOOST_CHECK(res.encode == 3);
  BOOST_CHECK(approx(res.sqr_distance, 0.5));
  BOOST_CHECK(approx(res.parameterization[0], 0.5));
  BOOST_CHECK(approx(res.parameterization[1], 0.5));
  BOOST_CHECK(approx(res.parameterization[2], 0));
}

BOOST_AUTO_TEST_CASE(projection_test_tetrahedron) {
  Vec3s v1(0, 0, 1);
  Vec3s v2(0, 1, 0);
  Vec3s v3(1, 0, 0);
  Vec3s v4(1, 1, 1);

  Vec3s p(0.5, 0.5, 0.5);
  Project<Scalar>::ProjectResult res =
      Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 15);
  BOOST_CHECK(approx(res.sqr_distance, 0));
  BOOST_CHECK(approx(res.parameterization[0], 0.25));
  BOOST_CHECK(approx(res.parameterization[1], 0.25));
  BOOST_CHECK(approx(res.parameterization[2], 0.25));
  BOOST_CHECK(approx(res.parameterization[3], 0.25));

  p = Vec3s(0, 0, 0);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 7);
  BOOST_CHECK(approx(res.sqr_distance, Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[0], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[1], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[2], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[3], 0));

  p = Vec3s(0, 1, 1);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 11);
  BOOST_CHECK(approx(res.sqr_distance, Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[0], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[1], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[2], 0));
  BOOST_CHECK(approx(res.parameterization[3], Scalar(1 / 3.0)));

  p = Vec3s(1, 1, 0);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 14);
  BOOST_CHECK(approx(res.sqr_distance, Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[2], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[3], Scalar(1 / 3.0)));

  p = Vec3s(1, 0, 1);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 13);
  BOOST_CHECK(approx(res.sqr_distance, Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[0], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], Scalar(1 / 3.0)));
  BOOST_CHECK(approx(res.parameterization[3], Scalar(1 / 3.0)));

  p = Vec3s(1.5, 1.5, 1.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 8);
  BOOST_CHECK(approx(res.sqr_distance, 0.75));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 0));
  BOOST_CHECK(approx(res.parameterization[3], 1));

  p = Vec3s(1.5, -0.5, -0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 4);
  BOOST_CHECK(approx(res.sqr_distance, 0.75));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 1));
  BOOST_CHECK(approx(res.parameterization[3], 0));

  p = Vec3s(-0.5, -0.5, 1.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 1);
  BOOST_CHECK(approx(res.sqr_distance, 0.75));
  BOOST_CHECK(approx(res.parameterization[0], 1));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 0));
  BOOST_CHECK(approx(res.parameterization[3], 0));

  p = Vec3s(-0.5, 1.5, -0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 2);
  BOOST_CHECK(approx(res.sqr_distance, 0.75));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 1));
  BOOST_CHECK(approx(res.parameterization[2], 0));
  BOOST_CHECK(approx(res.parameterization[3], 0));

  p = Vec3s(0.5, -0.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 5);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0.5));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 0.5));
  BOOST_CHECK(approx(res.parameterization[3], 0));

  p = Vec3s(0.5, 1.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 10);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 0.5));
  BOOST_CHECK(approx(res.parameterization[2], 0));
  BOOST_CHECK(approx(res.parameterization[3], 0.5));

  p = Vec3s(1.5, 0.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 12);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 0.5));
  BOOST_CHECK(approx(res.parameterization[3], 0.5));

  p = Vec3s(-0.5, 0.5, 0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 3);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0.5));
  BOOST_CHECK(approx(res.parameterization[1], 0.5));
  BOOST_CHECK(approx(res.parameterization[2], 0));
  BOOST_CHECK(approx(res.parameterization[3], 0));

  p = Vec3s(0.5, 0.5, 1.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 9);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0.5));
  BOOST_CHECK(approx(res.parameterization[1], 0));
  BOOST_CHECK(approx(res.parameterization[2], 0));
  BOOST_CHECK(approx(res.parameterization[3], 0.5));

  p = Vec3s(0.5, 0.5, -0.5);
  res = Project<Scalar>::projectTetrahedra(v1, v2, v3, v4, p);
  BOOST_CHECK(res.encode == 6);
  BOOST_CHECK(approx(res.sqr_distance, 0.25));
  BOOST_CHECK(approx(res.parameterization[0], 0));
  BOOST_CHECK(approx(res.parameterization[1], 0.5));
  BOOST_CHECK(approx(res.parameterization[2], 0.5));
  BOOST_CHECK(approx(res.parameterization[3], 0));
}
