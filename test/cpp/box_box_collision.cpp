#define BOOST_TEST_MODULE COAL_BOX_BOX_COLLISION
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/tools.h"

#include "utility.h"

using coal::Box;
using coal::collide;
using coal::CollisionRequest;
using coal::CollisionResult;
using coal::ComputeCollision;
using coal::Scalar;
using coal::Transform3s;
using coal::Vec3s;

BOOST_AUTO_TEST_CASE(box_box_collision) {
  // Define boxes
  Box shape1(1, 1, 1);
  Box shape2(1, 1, 1);

  // Define transforms
  Transform3s T1 = Transform3s::Identity();
  Transform3s T2 = Transform3s::Identity();

  // Compute collision
  CollisionRequest req;
  req.enable_cached_gjk_guess = true;
  req.distance_upper_bound = Scalar(1e-6);
  CollisionResult res;
  ComputeCollision collide_functor(&shape1, &shape2);

  T1.setTranslation(Vec3s(0, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  T1.setTranslation(Vec3s(2, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);
}
