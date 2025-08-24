/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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

#define BOOST_TEST_MODULE COAL_SWEPT_SPHERE_RADIUS
#include <boost/test/included/unit_test.hpp>

#include "coal/narrowphase/narrowphase.h"
#include "coal/collision_utility.h"

#include "coal/serialization/geometric_shapes.h"
#include "coal/serialization/convex.h"
#include "coal/serialization/transform.h"
#include "coal/serialization/archive.h"

#include "utility.h"

using namespace coal;

NODE_TYPE node1_type;
NODE_TYPE node2_type;
int line;

#define SET_LINE                     \
  node1_type = shape1.getNodeType(); \
  node2_type = shape2.getNodeType(); \
  line = __LINE__

#define COAL_CHECK(cond)                                                     \
  BOOST_CHECK_MESSAGE(                                                       \
      cond, "from line " << line << ", for collision pair: "                 \
                         << get_node_type_name(node1_type) << " - "          \
                         << get_node_type_name(node2_type)                   \
                         << " with ssr1 = " << shape1.getSweptSphereRadius() \
                         << ", ssr2 = " << shape2.getSweptSphereRadius()     \
                         << ": " #cond)

#define COAL_CHECK_VECTOR_CLOSE(v1, v2, tol) \
  EIGEN_VECTOR_IS_APPROX(v1, v2, tol);       \
  COAL_CHECK(((v1) - (v2)).isZero(tol))

#define COAL_CHECK_REAL_CLOSE(v1, v2, tol) \
  Scalar_IS_APPROX(v1, v2, tol);           \
  COAL_CHECK(std::abs((v1) - (v2)) < tol)

#define COAL_CHECK_CONDITION(cond) \
  BOOST_CHECK(cond);               \
  COAL_CHECK(cond)

// Preambule: swept sphere radius allows to virually convolve geometric shapes
// by a sphere with positive radius (Minkowski sum).
// Sweeping a shape by a sphere corresponds to doing a Minkowski addition of the
// shape with a sphere of radius r. Essentially, this rounds the shape's corners
// and edges, which can be useful to smooth collision detection algorithms.
//
// A nice mathematical property of GJK and EPA is that it is not
// necessary to take into account the swept sphere radius in the iterations of
// the algorithms. This is because the GJK and EPA algorithms are based on the
// Minkowski difference of the two objects.
// With spheres of radii r1 and r2 swept around the shapes s1 and s2 of a
// collision pair, the Minkowski difference is simply the Minkowski difference
// of s1 and s2, summed with a sphere of radius r1 + r2.
// This means that running GJK and EPA on the swept-sphere shapes is equivalent
// to running GJK and EPA on the original shapes, and then augmenting the
// distance by r1 + r2.
// This does not modify the normal returned by GJK and EPA.
// So we can also easily recover the witness points of the swept sphere shapes.
//
// This suite of test is designed to verify that property and generally test for
// swept-sphere radius support in Coal.
// Notes:
//   - not all collision pairs use GJK/EPA, so this test makes sure that
//     swept-sphere radius is taken into account even for specialized collision
//     algorithms.
//   - when manually taking swept-sphere radius into account in GJK/EPA, we
//     strongly deteriorate the convergence properties of the algorithms. This
//     is because certain parts of the shapes become locally strictly convex,
//     which GJK/EPA are not designed to handle. This becomes particularly the
//     bigger the swept-sphere radius. So don't be surprised if the tests fail
//     for large radii.

struct SweptSphereGJKSolver : public GJKSolver {
  template <typename S1, typename S2>
  Scalar shapeDistance(
      const S1& s1, const Transform3s& tf1, const S2& s2,
      const Transform3s& tf2, bool compute_penetration, Vec3s& p1, Vec3s& p2,
      Vec3s& normal, bool use_swept_sphere_radius_in_gjk_epa_iterations) const {
    if (use_swept_sphere_radius_in_gjk_epa_iterations) {
      Scalar distance;
      this->runGJKAndEPA<S1, S2, details::SupportOptions::WithSweptSphere>(
          s1, tf1, s2, tf2, compute_penetration, distance, p1, p2, normal);
      return distance;
    }

    // Default behavior of coal's GJKSolver
    Scalar distance;
    this->runGJKAndEPA<S1, S2, details::SupportOptions::NoSweptSphere>(
        s1, tf1, s2, tf2, compute_penetration, distance, p1, p2, normal);
    return distance;
  }
};

template <typename S1, typename S2>
void test_gjksolver_swept_sphere_radius(S1& shape1, S2& shape2) {
  SweptSphereGJKSolver solver;
  // The swept sphere radius is detrimental to the convergence of GJK
  // and EPA. This gets worse as the radius of the swept sphere increases.
  // So we need to increase the number of iterations to get a good result.
  const Scalar tol = Scalar(1e-6);
  solver.gjk_tolerance = tol;
  solver.epa_tolerance = tol;
  solver.epa_max_iterations = 1000;
  const bool compute_penetration = true;

  Scalar extents[] = {-2, -2, -2, 2, 2, 2};
  std::size_t n = 10;
  std::vector<Transform3s> tf1s;
  std::vector<Transform3s> tf2s;
  generateRandomTransforms(extents, tf1s, n);
  generateRandomTransforms(extents, tf2s, n);
  const std::array<Scalar, 4> swept_sphere_radius = {0, Scalar(0.1), 1, 10};

  for (const Scalar& ssr1 : swept_sphere_radius) {
    shape1.setSweptSphereRadius(ssr1);
    for (const Scalar& ssr2 : swept_sphere_radius) {
      shape2.setSweptSphereRadius(ssr2);
      for (std::size_t i = 0; i < n; ++i) {
        Transform3s tf1 = tf1s[i];
        Transform3s tf2 = tf2s[i];

        SET_LINE;

        std::array<Scalar, 2> distance;
        std::array<Vec3s, 2> p1;
        std::array<Vec3s, 2> p2;
        std::array<Vec3s, 2> normal;

        // Default coal behavior - Don't take swept sphere radius into account
        // during GJK/EPA iterations. Correct the solution afterwards.
        distance[0] =
            solver.shapeDistance(shape1, tf1, shape2, tf2, compute_penetration,
                                 p1[0], p2[0], normal[0], false);

        // Take swept sphere radius into account during GJK/EPA iterations
        distance[1] =
            solver.shapeDistance(shape1, tf1, shape2, tf2, compute_penetration,
                                 p1[1], p2[1], normal[1], true);

        // Precision is dependent on the swept-sphere radius.
        // The issue of precision does not come from the default behavior of
        // coal, but from the result in which we manually take the swept
        // sphere radius into account in GJK/EPA iterations.
        const Scalar precision =
            3 * sqrt(tol) +
            (1 / Scalar(100)) * std::max(shape1.getSweptSphereRadius(),
                                         shape2.getSweptSphereRadius());

        // Check that the distance is the same
        COAL_CHECK_REAL_CLOSE(distance[0], distance[1], precision);

        // Check that the normal is the same
        COAL_CHECK_CONDITION(normal[0].dot(normal[1]) > 0);
        COAL_CHECK_CONDITION(std::abs(1 - normal[0].dot(normal[1])) <
                             precision);

        // Check that the witness points are the same
        COAL_CHECK_VECTOR_CLOSE(p1[0], p1[1], precision);
        COAL_CHECK_VECTOR_CLOSE(p2[0], p2[1], precision);
      }
    }
  }
}

static const Scalar min_shape_size = Scalar(0.1);
static const Scalar max_shape_size = Scalar(0.5);

BOOST_AUTO_TEST_CASE(ssr_mesh_mesh) {
  ConvexTpl<Triangle32> shape1 =
      makeRandomConvex(min_shape_size, max_shape_size);
  ConvexTpl<Triangle32> shape2 =
      makeRandomConvex(min_shape_size, max_shape_size);
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_mesh_ellipsoid) {
  ConvexTpl<Triangle32> shape1 =
      makeRandomConvex(min_shape_size, max_shape_size);
  Ellipsoid shape2 = makeRandomEllipsoid(min_shape_size, max_shape_size);
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_box_box) {
  Box shape1 = makeRandomBox(min_shape_size, max_shape_size);
  Box shape2 = makeRandomBox(min_shape_size, max_shape_size);
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_ellipsoid_ellipsoid) {
  Ellipsoid shape1 = makeRandomEllipsoid(min_shape_size, max_shape_size);
  Ellipsoid shape2 = makeRandomEllipsoid(min_shape_size, max_shape_size);
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_ellipsoid_box) {
  Ellipsoid shape1 = makeRandomEllipsoid(min_shape_size, max_shape_size);
  Box shape2 = makeRandomBox(min_shape_size, max_shape_size);
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_cone_cone) {
  Cone shape1 = makeRandomCone({min_shape_size / 2, min_shape_size},
                               {max_shape_size, max_shape_size});
  Cone shape2 = makeRandomCone({min_shape_size / 2, min_shape_size},
                               {max_shape_size, max_shape_size});
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_cone_ellipsoid) {
  Cone shape1 = makeRandomCone({min_shape_size / 2, min_shape_size},
                               {max_shape_size, max_shape_size});
  Ellipsoid shape2 = makeRandomEllipsoid(min_shape_size, max_shape_size);
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_capsule_capsule) {
  Capsule shape1 = makeRandomCapsule({min_shape_size / 2, min_shape_size},
                                     {max_shape_size, max_shape_size});
  Capsule shape2 = makeRandomCapsule({min_shape_size / 2, min_shape_size},
                                     {max_shape_size, max_shape_size});
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_capsule_cone) {
  Capsule shape1 = makeRandomCapsule({min_shape_size / 2, min_shape_size},
                                     {max_shape_size, max_shape_size});
  Cone shape2 = makeRandomCone({min_shape_size / 2, min_shape_size},
                               {max_shape_size, max_shape_size});
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

BOOST_AUTO_TEST_CASE(ssr_cylinder_cylinder) {
  Cylinder shape1 = makeRandomCylinder({min_shape_size / 2, min_shape_size},
                                       {max_shape_size, max_shape_size});
  Cylinder shape2 = makeRandomCylinder({min_shape_size / 2, min_shape_size},
                                       {max_shape_size, max_shape_size});
  test_gjksolver_swept_sphere_radius(shape1, shape2);
}

template <typename S1, typename S2>
void test_collide_swept_sphere_radius(S1& shape1, S2& shape2) {
  std::cout << "Testing collision between "
            << std::string(get_node_type_name(shape1.getNodeType())) << " and "
            << std::string(get_node_type_name(shape2.getNodeType())) << '\n';

  Scalar extents[] = {-2, -2, -2, 2, 2, 2};
  std::size_t n = 1;
  std::vector<Transform3s> tf1s;
  std::vector<Transform3s> tf2s;
  generateRandomTransforms(extents, tf1s, n);
  generateRandomTransforms(extents, tf2s, n);

  const std::array<Scalar, 4> swept_sphere_radius = {0, Scalar(0.1), 1, 10};
  for (const Scalar& ssr1 : swept_sphere_radius) {
    shape1.setSweptSphereRadius(ssr1);
    for (const Scalar& ssr2 : swept_sphere_radius) {
      shape2.setSweptSphereRadius(ssr2);
      for (std::size_t i = 0; i < n; ++i) {
        Transform3s tf1 = tf1s[i];
        Transform3s tf2 = tf2s[i];

        SET_LINE;
        CollisionRequest request;
        request.enable_contact = true;
        // We make sure we get witness points by setting the security margin to
        // infinity. That way shape1 and shape2 will always be considered in
        // collision.
        request.security_margin = std::numeric_limits<Scalar>::max();
        const Scalar tol = Scalar(1e-6);
        request.gjk_tolerance = tol;
        request.epa_tolerance = tol;

        std::array<CollisionResult, 2> result;

        // Without swept sphere radius
        const Scalar ssr1 = shape1.getSweptSphereRadius();
        const Scalar ssr2 = shape2.getSweptSphereRadius();
        shape1.setSweptSphereRadius(0.);
        shape2.setSweptSphereRadius(0.);
        coal::collide(&shape1, tf1, &shape2, tf2, request, result[0]);

        // With swept sphere radius
        shape1.setSweptSphereRadius(ssr1);
        shape2.setSweptSphereRadius(ssr2);
        coal::collide(&shape1, tf1, &shape2, tf2, request, result[1]);

        BOOST_CHECK(result[0].isCollision());
        BOOST_CHECK(result[1].isCollision());
        if (result[0].isCollision() && result[1].isCollision()) {
          std::array<Contact, 2> contact;
          contact[0] = result[0].getContact(0);
          contact[1] = result[1].getContact(0);

          // Precision is dependent on the swept sphere radii.
          // The issue of precision does not come from the default behavior of
          // coal, but from the result in which we manually take the swept
          // sphere radius into account in GJK/EPA iterations.
          const Scalar precision =
              3 * sqrt(tol) + (1 / Scalar(100)) * std::max(ssr1, ssr2);
          const Scalar ssr = ssr1 + ssr2;

          // Check that the distance is the same
          COAL_CHECK_REAL_CLOSE(contact[0].penetration_depth - ssr,
                                contact[1].penetration_depth, precision);

          // Check that the normal is the same
          COAL_CHECK_CONDITION((contact[0].normal).dot(contact[1].normal) > 0);
          COAL_CHECK_CONDITION(
              std::abs(1 - (contact[0].normal).dot(contact[1].normal)) <
              precision);

          // Check that the witness points are the same
          COAL_CHECK_VECTOR_CLOSE(
              contact[0].nearest_points[0] + ssr1 * contact[0].normal,
              contact[1].nearest_points[0], precision);
          COAL_CHECK_VECTOR_CLOSE(
              contact[0].nearest_points[1] - ssr2 * contact[0].normal,
              contact[1].nearest_points[1], precision);
        }
      }
    }
  }
}

const std::vector<NODE_TYPE> tested_geometries = {
    GEOM_BOX,      GEOM_SPHERE, GEOM_ELLIPSOID, GEOM_CAPSULE,  GEOM_CONE,
    GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE,     GEOM_HALFSPACE};

BOOST_AUTO_TEST_CASE(ssr_geom_geom) {
  // Each possible geom pair is tested twice
  for (const NODE_TYPE& shape_type1 : tested_geometries) {
    for (const NODE_TYPE& shape_type2 : tested_geometries) {
      if (shape_type1 == GEOM_PLANE || shape_type1 == GEOM_HALFSPACE) {
        if (shape_type2 == GEOM_PLANE || shape_type2 == GEOM_HALFSPACE) {
          // TODO(louis): check plane-plane plane-halfspace etc. collisions
          continue;
        }
      }
      std::shared_ptr<ShapeBase> shape1 = makeRandomGeometry(shape_type1);
      std::shared_ptr<ShapeBase> shape2 = makeRandomGeometry(shape_type2);
      test_collide_swept_sphere_radius(*shape1, *shape2);
    }
  }
}
