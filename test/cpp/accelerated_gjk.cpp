/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, INRIA
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

#define BOOST_TEST_MODULE COAL_NESTEROV_GJK
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/tools.h"

#include "utility.h"

using coal::Box;
using coal::Capsule;
using coal::constructPolytopeFromEllipsoid;
using coal::ConvexTpl;
using coal::Ellipsoid;
using coal::GJKSolver;
using coal::GJKVariant;
using coal::Scalar;
using coal::ShapeBase;
using coal::SolverScalar;
using coal::support_func_guess_t;
using coal::Transform3s;
using coal::Triangle32;
using coal::Vec3ps;
using coal::Vec3s;
using coal::details::GJK;
using coal::details::MinkowskiDiff;
using coal::details::SupportOptions;
using std::size_t;

BOOST_AUTO_TEST_CASE(set_gjk_variant) {
  GJKSolver solver;
  GJK gjk(128, Scalar(1e-6));
  MinkowskiDiff shape;

  // Checking defaults
  BOOST_CHECK(solver.gjk.gjk_variant == GJKVariant::DefaultGJK);
  BOOST_CHECK(gjk.gjk_variant == GJKVariant::DefaultGJK);
  BOOST_CHECK(shape.normalize_support_direction == false);

  // Checking set
  solver.gjk.gjk_variant = GJKVariant::NesterovAcceleration;
  gjk.gjk_variant = GJKVariant::NesterovAcceleration;

  BOOST_CHECK(solver.gjk.gjk_variant == GJKVariant::NesterovAcceleration);
  BOOST_CHECK(gjk.gjk_variant == GJKVariant::NesterovAcceleration);

  solver.gjk.gjk_variant = GJKVariant::PolyakAcceleration;
  gjk.gjk_variant = GJKVariant::PolyakAcceleration;

  BOOST_CHECK(solver.gjk.gjk_variant == GJKVariant::PolyakAcceleration);
  BOOST_CHECK(gjk.gjk_variant == GJKVariant::PolyakAcceleration);
}

BOOST_AUTO_TEST_CASE(need_nesterov_normalize_support_direction) {
  Ellipsoid ellipsoid = Ellipsoid(1, 1, 1);
  Box box = Box(1, 1, 1);
  ConvexTpl<Triangle32> cvx;

  MinkowskiDiff mink_diff1;
  mink_diff1.set<SupportOptions::NoSweptSphere>(&ellipsoid, &ellipsoid);
  BOOST_CHECK(mink_diff1.normalize_support_direction == false);

  MinkowskiDiff mink_diff2;
  mink_diff2.set<SupportOptions::NoSweptSphere>(&ellipsoid, &box);
  BOOST_CHECK(mink_diff2.normalize_support_direction == false);

  MinkowskiDiff mink_diff3;
  mink_diff3.set<SupportOptions::NoSweptSphere>(&cvx, &cvx);
  BOOST_CHECK(mink_diff3.normalize_support_direction == true);
}

void test_accelerated_gjk(const ShapeBase& shape0, const ShapeBase& shape1) {
  // Solvers
  unsigned int max_iterations = 128;
  Scalar tolerance = Scalar(1e-6);
  GJK gjk(max_iterations, tolerance);
  GJK gjk_nesterov(max_iterations, tolerance);
  gjk_nesterov.gjk_variant = GJKVariant::NesterovAcceleration;
  GJK gjk_polyak(max_iterations, tolerance);
  gjk_polyak.gjk_variant = GJKVariant::PolyakAcceleration;

  // Minkowski difference
  MinkowskiDiff mink_diff;

  // Generate random transforms
  size_t n = 1000;
  Scalar extents[] = {-3., -3., 0, 3., 3., 3.};
  std::vector<Transform3s> transforms;
  generateRandomTransforms(extents, transforms, n);
  Transform3s identity = Transform3s::Identity();

  // Same init for both solvers
  Vec3s init_guess = Vec3s(1, 0, 0);
  support_func_guess_t init_support_guess;
  init_support_guess.setZero();

  for (size_t i = 0; i < n; ++i) {
    // No need to take into account swept-sphere radius in supports computation
    // when using GJK/EPA; after they have converged, these algos will correctly
    // handle the swept-sphere radius of the shapes.
    mink_diff.set<SupportOptions::NoSweptSphere>(&shape0, &shape1, identity,
                                                 transforms[i]);

    // Evaluate both solvers twice, make sure they give the same solution
    GJK::Status res_gjk_1 = gjk.evaluate(
        mink_diff, init_guess.cast<SolverScalar>(), init_support_guess);
    Vec3ps ray_gjk = gjk.ray;
    GJK::Status res_gjk_2 = gjk.evaluate(
        mink_diff, init_guess.cast<SolverScalar>(), init_support_guess);
    BOOST_CHECK(res_gjk_1 == res_gjk_2);
    EIGEN_VECTOR_IS_APPROX(ray_gjk, gjk.ray, SolverScalar(1e-8));

    // --------------
    // -- Nesterov --
    // --------------
    GJK::Status res_nesterov_gjk_1 = gjk_nesterov.evaluate(
        mink_diff, init_guess.cast<SolverScalar>(), init_support_guess);
    Vec3ps ray_nesterov = gjk_nesterov.ray;
    GJK::Status res_nesterov_gjk_2 = gjk_nesterov.evaluate(
        mink_diff, init_guess.cast<SolverScalar>(), init_support_guess);
    BOOST_CHECK(res_nesterov_gjk_1 == res_nesterov_gjk_2);
    EIGEN_VECTOR_IS_APPROX(ray_nesterov, gjk_nesterov.ray, SolverScalar(1e-8));

    // Make sure GJK and Nesterov accelerated GJK find the same distance between
    // the shapes
    BOOST_CHECK(res_nesterov_gjk_1 == res_gjk_1);
    BOOST_CHECK_SMALL(fabs(ray_gjk.norm() - ray_nesterov.norm()),
                      SolverScalar(1e-4));

    // Make sure GJK and Nesterov accelerated GJK converges in a reasonable
    // amount of iterations
    BOOST_CHECK(gjk.getNumIterations() < max_iterations);
    BOOST_CHECK(gjk_nesterov.getNumIterations() < max_iterations);

    // ------------
    // -- Polyak --
    // ------------
    GJK::Status res_polyak_gjk_1 = gjk_polyak.evaluate(
        mink_diff, init_guess.cast<SolverScalar>(), init_support_guess);
    Vec3ps ray_polyak = gjk_polyak.ray;
    GJK::Status res_polyak_gjk_2 = gjk_polyak.evaluate(
        mink_diff, init_guess.cast<SolverScalar>(), init_support_guess);
    BOOST_CHECK(res_polyak_gjk_1 == res_polyak_gjk_2);
    EIGEN_VECTOR_IS_APPROX(ray_polyak, gjk_polyak.ray, SolverScalar(1e-8));

    // Make sure GJK and Polyak accelerated GJK find the same distance between
    // the shapes
    BOOST_CHECK(res_polyak_gjk_1 == res_gjk_1);
    BOOST_CHECK_SMALL(fabs(ray_gjk.norm() - ray_polyak.norm()),
                      SolverScalar(1e-4));

    // Make sure GJK and Polyak accelerated GJK converges in a reasonable
    // amount of iterations
    BOOST_CHECK(gjk.getNumIterations() < max_iterations);
    BOOST_CHECK(gjk_polyak.getNumIterations() < max_iterations);
  }
}

BOOST_AUTO_TEST_CASE(ellipsoid_ellipsoid) {
  Ellipsoid ellipsoid0 = Ellipsoid(Scalar(0.3), Scalar(0.4), Scalar(0.5));
  Ellipsoid ellipsoid1 = Ellipsoid(Scalar(1.5), Scalar(1.4), Scalar(1.3));

  test_accelerated_gjk(ellipsoid0, ellipsoid1);
  test_accelerated_gjk(ellipsoid0, ellipsoid1);
}

BOOST_AUTO_TEST_CASE(ellipsoid_capsule) {
  Ellipsoid ellipsoid0 = Ellipsoid(Scalar(0.5), Scalar(0.4), Scalar(0.3));
  Ellipsoid ellipsoid1 = Ellipsoid(Scalar(1.5), Scalar(1.4), Scalar(1.3));
  Capsule capsule0 = Capsule(Scalar(0.1), Scalar(0.3));
  Capsule capsule1 = Capsule(Scalar(1.1), Scalar(1.3));

  test_accelerated_gjk(ellipsoid0, capsule0);
  test_accelerated_gjk(ellipsoid0, capsule1);
  test_accelerated_gjk(ellipsoid1, capsule0);
  test_accelerated_gjk(ellipsoid1, capsule1);
}

BOOST_AUTO_TEST_CASE(ellipsoid_box) {
  Ellipsoid ellipsoid0 = Ellipsoid(Scalar(0.5), Scalar(0.4), Scalar(0.3));
  Ellipsoid ellipsoid1 = Ellipsoid(Scalar(1.5), Scalar(1.4), Scalar(1.3));
  Box box0 = Box(Scalar(0.1), Scalar(0.2), Scalar(0.3));
  Box box1 = Box(Scalar(1.1), Scalar(1.2), Scalar(1.3));

  test_accelerated_gjk(ellipsoid0, box0);
  test_accelerated_gjk(ellipsoid0, box1);
  test_accelerated_gjk(ellipsoid1, box0);
  test_accelerated_gjk(ellipsoid1, box1);
}

BOOST_AUTO_TEST_CASE(ellipsoid_mesh) {
  Ellipsoid ellipsoid0 = Ellipsoid(Scalar(0.5), Scalar(0.4), Scalar(0.3));
  Ellipsoid ellipsoid1 = Ellipsoid(Scalar(1.5), Scalar(1.4), Scalar(1.3));
  ConvexTpl<Triangle32> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  ConvexTpl<Triangle32> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);

  test_accelerated_gjk(ellipsoid0, cvx0);
  test_accelerated_gjk(ellipsoid0, cvx1);
  test_accelerated_gjk(ellipsoid1, cvx0);
  test_accelerated_gjk(ellipsoid1, cvx1);
}

BOOST_AUTO_TEST_CASE(capsule_mesh) {
  Ellipsoid ellipsoid0 = Ellipsoid(Scalar(0.5), Scalar(0.4), Scalar(0.3));
  Ellipsoid ellipsoid1 = Ellipsoid(Scalar(1.5), Scalar(1.4), Scalar(1.3));
  ConvexTpl<Triangle32> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  ConvexTpl<Triangle32> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);
  Capsule capsule0 = Capsule(Scalar(0.1), Scalar(0.3));
  Capsule capsule1 = Capsule(Scalar(1.1), Scalar(1.3));

  test_accelerated_gjk(capsule0, cvx0);
  test_accelerated_gjk(capsule0, cvx1);
  test_accelerated_gjk(capsule1, cvx0);
  test_accelerated_gjk(capsule1, cvx1);
}

BOOST_AUTO_TEST_CASE(capsule_capsule) {
  Capsule capsule0 = Capsule(Scalar(0.1), Scalar(0.3));
  Capsule capsule1 = Capsule(Scalar(1.1), Scalar(1.3));

  test_accelerated_gjk(capsule0, capsule0);
  test_accelerated_gjk(capsule1, capsule1);
  test_accelerated_gjk(capsule0, capsule1);
}

BOOST_AUTO_TEST_CASE(box_box) {
  Box box0 = Box(Scalar(0.1), Scalar(0.2), Scalar(0.3));
  Box box1 = Box(Scalar(1.1), Scalar(1.2), Scalar(1.3));
  test_accelerated_gjk(box0, box0);
  test_accelerated_gjk(box0, box1);
  test_accelerated_gjk(box1, box1);
}

BOOST_AUTO_TEST_CASE(box_mesh) {
  Box box0 = Box(Scalar(0.1), Scalar(0.2), Scalar(0.3));
  Box box1 = Box(Scalar(1.1), Scalar(1.2), Scalar(1.3));
  Ellipsoid ellipsoid0 = Ellipsoid(Scalar(0.5), Scalar(0.4), Scalar(0.3));
  Ellipsoid ellipsoid1 = Ellipsoid(Scalar(1.5), Scalar(1.4), Scalar(1.3));
  ConvexTpl<Triangle32> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  ConvexTpl<Triangle32> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);

  test_accelerated_gjk(box0, cvx0);
  test_accelerated_gjk(box0, cvx1);
  test_accelerated_gjk(box1, cvx0);
  test_accelerated_gjk(box1, cvx1);
}

BOOST_AUTO_TEST_CASE(mesh_mesh) {
  Ellipsoid ellipsoid0 = Ellipsoid(Scalar(0.5), Scalar(0.4), Scalar(0.3));
  Ellipsoid ellipsoid1 = Ellipsoid(Scalar(1.5), Scalar(1.4), Scalar(1.3));
  ConvexTpl<Triangle32> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  ConvexTpl<Triangle32> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);

  test_accelerated_gjk(cvx0, cvx0);
  test_accelerated_gjk(cvx0, cvx1);
  test_accelerated_gjk(cvx1, cvx1);
}
