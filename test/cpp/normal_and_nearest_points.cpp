/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022-2024, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Louis Montaut */

#define BOOST_TEST_MODULE COAL_NORMAL_AND_NEAREST_POINTS
#include <boost/test/included/unit_test.hpp>

#include "coal/fwd.hh"
#include "coal/shape/geometric_shapes.h"
#include "coal/collision_data.h"
#include "coal/BV/OBBRSS.h"
#include "coal/BVH/BVH_model.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"

#include "utility.h"

using namespace coal;
typedef Eigen::Vector2d Vec2d;

// This test suite is designed to operate on any pair of primitive shapes:
// spheres, capsules, boxes, ellipsoids, cones, cylinders, planes, halfspaces,
// convex meshes. Do not use this file for BVH, octree etc. It would not make
// sense.

// This test is designed to check if the normal and the nearest points
// are properly handled as defined in DistanceResult::normal.
// Regardless of wether or not the two shapes are in intersection, regardless of
// whether `collide` or `distance` is called:
// --> we denote `dist` the (signed) distance that separates the two shapes
// --> we denote `p1` and `p2` their nearest_points (witness points)
// --> the `normal` should always point from shape 1 to shape 2, i.e we should
//     always have: | normal = sign(dist) * (p2 - p1).normalized()
//                  | p2 = p1 + dist * normal
// Thus:
// --> if o1 and o2 are not in collision, translating o2 by vector
//     `-(dist + eps) * normal` should bring them in collision (eps > 0).
// --> if o1 and o2 are in collision, translating o2 by vector
//    `-(dist - eps)) * normal` should separate them (eps > 0).
// --> finally, if abs(dist) > 0, we should have:
//       normal = sign(dist) * (p2 - p1).normalized()
//       p2 = p1 + dist * normal
template <typename ShapeType1, typename ShapeType2>
void test_normal_and_nearest_points(
    const ShapeType1& o1, const ShapeType2& o2,
    size_t gjk_max_iterations = GJK_DEFAULT_MAX_ITERATIONS,
    Scalar gjk_tolerance = GJK_DEFAULT_TOLERANCE,
    size_t epa_max_iterations = EPA_DEFAULT_MAX_ITERATIONS,
    Scalar epa_tolerance = EPA_DEFAULT_TOLERANCE) {
// Generate random poses for o2
#ifndef NDEBUG  // if debug mode
  std::size_t n = 10;
#else
  size_t n = 1000;
#endif
  // We want to make sure we generate poses that are in collision
  // so we take a relatively small extent for the random poses
  Scalar extents[] = {-1.5, -1.5, -1.5, 1.5, 1.5, 1.5};
  std::vector<Transform3s> transforms;
  generateRandomTransforms(extents, transforms, n);
  Transform3s tf1 = Transform3s::Identity();

  CollisionRequest colreq;
  colreq.distance_upper_bound = std::numeric_limits<Scalar>::max();
  // For strictly convex shapes, the default tolerance of EPA is way too low.
  // Because EPA is basically trying to fit a polytope to a strictly convex
  // surface, it might take it a lot of iterations to converge to a low
  // tolerance. A solution is to increase the number of iterations and the
  // tolerance (and/or increase the number of faces and vertices EPA is allowed
  // to work with).
  colreq.gjk_max_iterations = gjk_max_iterations;
  colreq.gjk_tolerance = gjk_tolerance;
  colreq.epa_max_iterations = epa_max_iterations;
  colreq.epa_tolerance = epa_tolerance;
  DistanceRequest distreq;
  distreq.gjk_max_iterations = gjk_max_iterations;
  distreq.gjk_tolerance = gjk_tolerance;
  distreq.epa_max_iterations = epa_max_iterations;
  distreq.epa_tolerance = epa_tolerance;

  for (size_t i = 0; i < n; i++) {
    // Run both `distance` and `collide`.
    // Since CollisionRequest::distance_lower_bound is set to infinity,
    // these functions should agree on the results regardless of collision or
    // not.
    Transform3s tf2 = transforms[i];
    CollisionResult colres;
    DistanceResult distres;
    size_t col = collide(&o1, tf1, &o2, tf2, colreq, colres);
    Scalar dist = distance(&o1, tf1, &o2, tf2, distreq, distres);

    const Scalar dummy_precision(100 * std::numeric_limits<Scalar>::epsilon());
    if (col) {
      BOOST_CHECK(dist <= 0.);
      BOOST_CHECK_CLOSE(dist, distres.min_distance, dummy_precision);
      Contact contact = colres.getContact(0);
      BOOST_CHECK_CLOSE(dist, contact.penetration_depth, dummy_precision);

      Vec3s cp1 = contact.nearest_points[0];
      EIGEN_VECTOR_IS_APPROX(cp1, distres.nearest_points[0], dummy_precision);

      Vec3s cp2 = contact.nearest_points[1];
      EIGEN_VECTOR_IS_APPROX(cp2, distres.nearest_points[1], dummy_precision);
      BOOST_CHECK_CLOSE(contact.penetration_depth, -(cp2 - cp1).norm(),
                        epa_tolerance);
      EIGEN_VECTOR_IS_APPROX(cp1, cp2 - dist * distres.normal, epa_tolerance);

      Vec3s separation_vector = contact.penetration_depth * contact.normal;
      EIGEN_VECTOR_IS_APPROX(separation_vector, cp2 - cp1, epa_tolerance);

      if (dist < 0) {
        EIGEN_VECTOR_IS_APPROX(contact.normal, -(cp2 - cp1).normalized(),
                               epa_tolerance);
      }

      // Separate the shapes
      Transform3s new_tf1 = tf1;
      Scalar eps = Scalar(1e-2);
      new_tf1.setTranslation(tf1.getTranslation() + separation_vector -
                             eps * contact.normal);
      CollisionResult new_colres;
      DistanceResult new_distres;
      size_t new_col = collide(&o1, new_tf1, &o2, tf2, colreq, new_colres);
      Scalar new_dist = distance(&o1, new_tf1, &o2, tf2, distreq, new_distres);
      BOOST_CHECK(new_dist > 0);
      BOOST_CHECK(!new_col);
      BOOST_CHECK(!new_colres.isCollision());
      BOOST_CHECK_CLOSE(new_colres.distance_lower_bound, new_dist,
                        epa_tolerance);
      Vec3s new_cp1 = new_distres.nearest_points[0];
      Vec3s new_cp2 = new_distres.nearest_points[1];
      BOOST_CHECK_CLOSE(new_dist, (new_cp1 - new_cp2).norm(), epa_tolerance);
      EIGEN_VECTOR_IS_APPROX(new_cp1, new_cp2 - new_dist * new_distres.normal,
                             epa_tolerance);

      Vec3s new_separation_vector = new_dist * new_distres.normal;
      EIGEN_VECTOR_IS_APPROX(new_separation_vector, new_cp2 - new_cp1,
                             epa_tolerance);

      if (new_dist > 0) {
        EIGEN_VECTOR_IS_APPROX(new_distres.normal,
                               (new_cp2 - new_cp1).normalized(), gjk_tolerance);
      }
    } else {
      BOOST_CHECK(dist >= 0.);
      BOOST_CHECK_CLOSE(distres.min_distance, dist, dummy_precision);
      BOOST_CHECK_CLOSE(dist, colres.distance_lower_bound, dummy_precision);

      Vec3s cp1 = distres.nearest_points[0];
      Vec3s cp2 = distres.nearest_points[1];
      BOOST_CHECK_CLOSE(dist, (cp1 - cp2).norm(), gjk_tolerance);
      EIGEN_VECTOR_IS_APPROX(cp1, cp2 - dist * distres.normal, gjk_tolerance);

      Vec3s separation_vector = dist * distres.normal;
      EIGEN_VECTOR_IS_APPROX(separation_vector, cp2 - cp1, gjk_tolerance);

      if (dist > 0) {
        EIGEN_VECTOR_IS_APPROX(distres.normal, (cp2 - cp1).normalized(),
                               gjk_tolerance);
      }

      // Bring the shapes in collision
      // We actually can't guarantee that the shapes will be in collision.
      // Suppose you have two disjoing cones, which witness points are the tips
      // of the cones.
      // If you translate one of the cones by the separation vector and it
      // happens to be parallel to the axis of the cone, the two shapes will
      // still be disjoint.
      Scalar eps = Scalar(1e-2);
      Transform3s new_tf1 = tf1;
      new_tf1.setTranslation(tf1.getTranslation() + separation_vector +
                             eps * distres.normal);
      CollisionResult new_colres;
      DistanceResult new_distres;
      collide(&o1, new_tf1, &o2, tf2, colreq, new_colres);
      Scalar new_dist = distance(&o1, new_tf1, &o2, tf2, distreq, new_distres);
      BOOST_CHECK(new_dist < dist);
      BOOST_CHECK_CLOSE(new_colres.distance_lower_bound, new_dist,
                        dummy_precision);
      // tolerance
      if (new_colres.isCollision()) {
        Contact contact = new_colres.getContact(0);
        Vec3s new_cp1 = contact.nearest_points[0];
        EIGEN_VECTOR_IS_APPROX(new_cp1, new_distres.nearest_points[0],
                               dummy_precision);

        Vec3s new_cp2 = contact.nearest_points[1];
        EIGEN_VECTOR_IS_APPROX(new_cp2, new_distres.nearest_points[1],
                               dummy_precision);
        BOOST_CHECK_CLOSE(contact.penetration_depth,
                          -(new_cp2 - new_cp1).norm(), epa_tolerance);
        EIGEN_VECTOR_IS_APPROX(new_cp1, new_cp2 - new_dist * new_distres.normal,
                               epa_tolerance);

        Vec3s new_separation_vector =
            contact.penetration_depth * contact.normal;
        EIGEN_VECTOR_IS_APPROX(new_separation_vector, new_cp2 - new_cp1,
                               epa_tolerance);

        if (new_dist < 0) {
          EIGEN_VECTOR_IS_APPROX(
              contact.normal, -(new_cp2 - new_cp1).normalized(), epa_tolerance);
        }
      }
    }
  }
}

template <int VecSize>
Eigen::Matrix<Scalar, VecSize, 1> generateRandomVector(Scalar min, Scalar max) {
  typedef Eigen::Matrix<Scalar, VecSize, 1> VecType;
  // Generate a random vector in the [min, max] range
  VecType v = VecType::Random() * (max - min) * 0.5 +
              VecType::Ones() * (max + min) * 0.5;
  return v;
}

Scalar generateRandomNumber(Scalar min, Scalar max) {
  Scalar r = static_cast<Scalar>(rand()) / static_cast<Scalar>(RAND_MAX);
  r = 2 * r - 1;
  const Scalar half(0.5);
  return r * (max - min) * half + (max + min) * half;
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_sphere) {
  for (size_t i = 0; i < 10; ++i) {
    Vec2s radii = generateRandomVector<2>(Scalar(0.05), 1);
    shared_ptr<Sphere> o1(new Sphere(radii(0)));
    shared_ptr<Sphere> o2(new Sphere(radii(1)));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_capsule) {
  for (size_t i = 0; i < 10; ++i) {
    Vec2s radii = generateRandomVector<2>(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Sphere> o1(new Sphere(radii(0)));
    shared_ptr<Capsule> o2(new Capsule(radii(1), h));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_box) {
  for (size_t i = 0; i < 10; ++i) {
    shared_ptr<Box> o1(new Box(generateRandomVector<3>(Scalar(0.05), 1)));
    shared_ptr<Sphere> o2(new Sphere(generateRandomNumber(Scalar(0.05), 1)));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_mesh) {
  for (size_t i = 0; i < 10; ++i) {
    ConvexTpl<Triangle32> o1_ = constructPolytopeFromEllipsoid(
        Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
    shared_ptr<ConvexTpl<Triangle32>> o1(new ConvexTpl<Triangle32>(
        o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
    ConvexTpl<Triangle32> o2_ = constructPolytopeFromEllipsoid(
        Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
    shared_ptr<ConvexTpl<Triangle32>> o2(new ConvexTpl<Triangle32>(
        o2_.points, o2_.num_points, o2_.polygons, o2_.num_polygons));

    size_t gjk_max_iterations = GJK_DEFAULT_MAX_ITERATIONS;
    Scalar gjk_tolerance = GJK_DEFAULT_TOLERANCE;
    size_t epa_max_iterations = EPA_DEFAULT_MAX_ITERATIONS;
    Scalar epa_tolerance = EPA_DEFAULT_TOLERANCE;
    test_normal_and_nearest_points(*o1.get(), *o2.get(), gjk_max_iterations,
                                   gjk_tolerance, epa_max_iterations,
                                   epa_tolerance);
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_box) {
  ConvexTpl<Triangle32> o1_ = constructPolytopeFromEllipsoid(
      Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
  shared_ptr<ConvexTpl<Triangle32>> o1(new ConvexTpl<Triangle32>(
      o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
  shared_ptr<Box> o2(new Box(generateRandomVector<3>(Scalar(0.05), 1)));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_ellipsoid) {
  for (size_t i = 0; i < 10; ++i) {
    ConvexTpl<Triangle32> o1_ = constructPolytopeFromEllipsoid(
        Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
    shared_ptr<ConvexTpl<Triangle32>> o1(new ConvexTpl<Triangle32>(
        o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
    shared_ptr<Ellipsoid> o2(
        new Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));

    Scalar gjk_tolerance = Scalar(1e-6);
    // With EPA's tolerance set at 1e-3, the precision on the normal, contact
    // points and penetration depth is on the order of the milimeter. However,
    // EPA (currently) cannot converge to lower tolerances on strictly convex
    // shapes in a reasonable amount of iterations.
    Scalar epa_tolerance = Scalar(1e-3);
    test_normal_and_nearest_points(*o1.get(), *o2.get(),
                                   GJK_DEFAULT_MAX_ITERATIONS, gjk_tolerance,
                                   EPA_DEFAULT_MAX_ITERATIONS, epa_tolerance);
    test_normal_and_nearest_points(*o2.get(), *o1.get(),
                                   GJK_DEFAULT_MAX_ITERATIONS, gjk_tolerance,
                                   EPA_DEFAULT_MAX_ITERATIONS, epa_tolerance);
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_ellipsoid_ellipsoid) {
  for (size_t i = 0; i < 10; ++i) {
    shared_ptr<Ellipsoid> o1(
        new Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1.0)));
    shared_ptr<Ellipsoid> o2(
        new Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1.0)));

    size_t gjk_max_iterations = GJK_DEFAULT_MAX_ITERATIONS;
    Scalar gjk_tolerance = Scalar(1e-6);
    // With EPA's tolerance set at 1e-3, the precision on the normal, contact
    // points and penetration depth is on the order of the milimeter. However,
    // EPA (currently) cannot converge to lower tolerances on strictly convex
    // shapes in a reasonable amount of iterations.
    size_t epa_max_iterations = 250;
    Scalar epa_tolerance = Scalar(1e-3);
    // For EPA on ellipsoids, we need to increase the number of iterations in
    // this test. This is simply because this test checks **a lot** of cases and
    // it can generate some of the worst cases for EPA. We don't want to
    // increase the tolerance too much because otherwise the test would not
    // work.
    test_normal_and_nearest_points(*o1.get(), *o2.get(), gjk_max_iterations,
                                   gjk_tolerance, epa_max_iterations,
                                   epa_tolerance);
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_box_plane) {
  for (size_t i = 0; i < 10; ++i) {
    shared_ptr<Box> o1(new Box(generateRandomVector<3>(Scalar(0.05), 1)));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Plane> o2(new Plane(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_box_halfspace) {
  for (size_t i = 0; i < 10; ++i) {
    shared_ptr<Box> o1(new Box(generateRandomVector<3>(Scalar(0.05), 1)));
    Scalar offset = Scalar(0.1);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Halfspace> o2(new Halfspace(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_capsule_halfspace) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar r = generateRandomNumber(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Capsule> o1(new Capsule(r, h));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Halfspace> o2(new Halfspace(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_halfspace) {
  for (size_t i = 0; i < 10; ++i) {
    shared_ptr<Sphere> o1(new Sphere(generateRandomNumber(Scalar(0.05), 1)));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Halfspace> o2(new Halfspace(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_plane) {
  for (size_t i = 0; i < 10; ++i) {
    shared_ptr<Sphere> o1(new Sphere(generateRandomNumber(Scalar(0.05), 1)));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Plane> o2(new Plane(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_halfspace) {
  for (size_t i = 0; i < 10; ++i) {
    ConvexTpl<Triangle32> o1_ = constructPolytopeFromEllipsoid(
        Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
    shared_ptr<ConvexTpl<Triangle32>> o1(new ConvexTpl<Triangle32>(
        o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Halfspace> o2(new Halfspace(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cone_cylinder) {
  for (size_t i = 0; i < 10; ++i) {
    Vec2s r = generateRandomVector<2>(Scalar(0.05), 1);
    Vec2s h = generateRandomVector<2>(Scalar(0.15), 1);
    shared_ptr<Cone> o1(new Cone(r(0), h(0)));
    shared_ptr<Cylinder> o2(new Cylinder(r(1), h(1)));

    size_t gjk_max_iterations = GJK_DEFAULT_MAX_ITERATIONS;
    Scalar gjk_tolerance = Scalar(1e-6);
    size_t epa_max_iterations = 250;
    Scalar epa_tolerance = Scalar(1e-3);
    test_normal_and_nearest_points(*o1.get(), *o2.get(), gjk_max_iterations,
                                   gjk_tolerance, epa_max_iterations,
                                   epa_tolerance);
    test_normal_and_nearest_points(*o2.get(), *o1.get(), gjk_max_iterations,
                                   gjk_tolerance, epa_max_iterations,
                                   epa_tolerance);
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cylinder_ellipsoid) {
  for (size_t i = 0; i < 10; ++i) {
    shared_ptr<Ellipsoid> o1(
        new Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
    Vec2s r = generateRandomVector<2>(Scalar(0.05), 1);
    Vec2s h = generateRandomVector<2>(Scalar(0.15), 1);
    shared_ptr<Cylinder> o2(new Cylinder(r(1), h(1)));

    size_t gjk_max_iterations = GJK_DEFAULT_MAX_ITERATIONS;
    Scalar gjk_tolerance = Scalar(1e-6);
    size_t epa_max_iterations = 250;
    Scalar epa_tolerance = Scalar(1e-3);
    test_normal_and_nearest_points(*o1.get(), *o2.get(), gjk_max_iterations,
                                   gjk_tolerance, epa_max_iterations,
                                   epa_tolerance);
    test_normal_and_nearest_points(*o2.get(), *o1.get(), gjk_max_iterations,
                                   gjk_tolerance, epa_max_iterations,
                                   epa_tolerance);
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cone_halfspace) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar r = generateRandomNumber(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Cone> o1(new Cone(r, h));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Halfspace> o2(new Halfspace(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cylinder_halfspace) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar r = generateRandomNumber(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Cylinder> o1(new Cylinder(r, h));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Halfspace> o2(new Halfspace(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cone_plane) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar r = generateRandomNumber(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Cone> o1(new Cone(r, h));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Plane> o2(new Plane(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cylinder_plane) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar r = generateRandomNumber(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Cylinder> o1(new Cylinder(r, h));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Plane> o2(new Plane(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_capsule_plane) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar r = generateRandomNumber(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Capsule> o1(new Capsule(r, h));
    const Scalar half(0.5);
    Scalar offset = generateRandomNumber(-half, half);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Plane> o2(new Plane(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_capsule_capsule) {
  for (size_t i = 0; i < 10; ++i) {
    Vec2s r = generateRandomVector<2>(Scalar(0.05), 1);
    Vec2s h = generateRandomVector<2>(Scalar(0.15), 1);
    shared_ptr<Capsule> o1(new Capsule(r(0), h(0)));
    shared_ptr<Capsule> o2(new Capsule(r(1), h(1)));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_cylinder) {
  for (size_t i = 0; i < 10; ++i) {
    Vec2s r = generateRandomVector<2>(Scalar(0.05), 1);
    Scalar h = generateRandomNumber(Scalar(0.15), 1);
    shared_ptr<Sphere> o1(new Sphere(r(0)));
    shared_ptr<Cylinder> o2(new Cylinder(r(1), h));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_ellipsoid_halfspace) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar offset = generateRandomNumber(Scalar(0.15), 1);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Ellipsoid> o1(
        new Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
    shared_ptr<Halfspace> o2(new Halfspace(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_ellipsoid_plane) {
  for (size_t i = 0; i < 10; ++i) {
    Scalar offset = generateRandomNumber(Scalar(0.15), 1);
    Vec3s n = Vec3s::Random();
    n.normalize();
    shared_ptr<Ellipsoid> o1(
        new Ellipsoid(generateRandomVector<3>(Scalar(0.05), 1)));
    shared_ptr<Plane> o2(new Plane(n, offset));

    test_normal_and_nearest_points(*o1.get(), *o2.get());
    test_normal_and_nearest_points(*o2.get(), *o1.get());
  }
}

void test_normal_and_nearest_points(const BVHModel<OBBRSS>& o1,
                                    const Halfspace& o2) {
  // Generate random poses for o2
#ifndef NDEBUG  // if debug mode
  std::size_t n = 1;
#else
  size_t n = 10000;
#endif
  Scalar extents[] = {-2., -2., -2., 2., 2., 2.};
  std::vector<Transform3s> transforms;
  generateRandomTransforms(extents, transforms, n);
  Transform3s tf1 = Transform3s::Identity();
  transforms[0] = Transform3s::Identity();

  CollisionRequest colreq;
  colreq.distance_upper_bound = std::numeric_limits<Scalar>::max();
  colreq.num_max_contacts = 100;
  CollisionResult colres;
  DistanceRequest distreq;
  DistanceResult distres;

  for (size_t i = 0; i < n; i++) {
    Transform3s tf2 = transforms[i];
    colres.clear();
    distres.clear();
    size_t col = collide(&o1, tf1, &o2, tf2, colreq, colres);
    Scalar dist = distance(&o1, tf1, &o2, tf2, distreq, distres);

    if (col) {
      BOOST_CHECK(dist <= 0.);
      BOOST_CHECK_CLOSE(dist, distres.min_distance, 1e-6);
      for (size_t c = 0; c < colres.numContacts(); c++) {
        Contact contact = colres.getContact(c);
        BOOST_CHECK(contact.penetration_depth <= 0);
        BOOST_CHECK(contact.penetration_depth >= colres.distance_lower_bound);

        Vec3s cp1 = contact.nearest_points[0];
        Vec3s cp2 = contact.nearest_points[1];
        BOOST_CHECK_CLOSE(contact.penetration_depth, -(cp2 - cp1).norm(), 1e-6);
        EIGEN_VECTOR_IS_APPROX(cp1,
                               cp2 - contact.penetration_depth * contact.normal,
                               Scalar(1e-6));

        Vec3s separation_vector = contact.penetration_depth * contact.normal;
        EIGEN_VECTOR_IS_APPROX(separation_vector, cp2 - cp1, Scalar(1e-6));

        if (dist < 0) {
          EIGEN_VECTOR_IS_APPROX(contact.normal, -(cp2 - cp1).normalized(),
                                 Scalar(1e-6));
        }
      }
    } else {
      BOOST_CHECK(dist >= 0.);
      BOOST_CHECK_CLOSE(distres.min_distance, dist, 1e-6);
      BOOST_CHECK_CLOSE(dist, colres.distance_lower_bound, 1e-6);
    }
  }
}

void test_normal_and_nearest_points(const Halfspace& o1,
                                    const BVHModel<OBBRSS>& o2) {
  test_normal_and_nearest_points(o2, o1);
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_bvh_halfspace) {
  Box* box_ptr = new coal::Box(1, 1, 1);
  coal::CollisionGeometryPtr_t b1(box_ptr);
  BVHModel<coal::OBBRSS> o1 = BVHModel<OBBRSS>();
  generateBVHModel(o1, *box_ptr, Transform3s());
  o1.buildConvexRepresentation(false);

  Scalar offset = Scalar(0.1);
  Vec3s n = Vec3s::Random();
  n.normalize();
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(o1, *o2.get());
  test_normal_and_nearest_points(*o2.get(), o1);
}
