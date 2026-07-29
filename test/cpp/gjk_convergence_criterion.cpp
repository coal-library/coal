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
#include <boost/test/tools/old/interface.hpp>

#include <Eigen/Geometry>
#include "coal/data_types.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/tools.h"

#include "utility.h"

using coal::Box;
using coal::GJKConvergenceCriterion;
using coal::GJKConvergenceCriterionType;
using coal::GJKSolver;
using coal::Scalar;
using coal::ShapeBase;
using coal::SolverScalar;
using coal::support_func_guess_t;
using coal::Transform3s;
using coal::Vec3ps;
using coal::Vec3s;
using coal::details::GJK;
using coal::details::MinkowskiDiff;
using std::size_t;

BOOST_AUTO_TEST_CASE(set_cv_criterion) {
  GJKSolver solver;
  GJK gjk(128, Scalar(1e-6));

  // Checking defaults
  BOOST_CHECK(solver.gjk.convergence_criterion ==
              GJKConvergenceCriterion::Default);
  BOOST_CHECK(solver.gjk.convergence_criterion_type ==
              GJKConvergenceCriterionType::Relative);

  BOOST_CHECK(gjk.convergence_criterion == GJKConvergenceCriterion::Default);
  BOOST_CHECK(gjk.convergence_criterion_type ==
              GJKConvergenceCriterionType::Relative);

  // Checking set
  solver.gjk.convergence_criterion = GJKConvergenceCriterion::DualityGap;
  gjk.convergence_criterion = GJKConvergenceCriterion::DualityGap;
  solver.gjk.convergence_criterion_type = GJKConvergenceCriterionType::Absolute;
  gjk.convergence_criterion_type = GJKConvergenceCriterionType::Absolute;

  BOOST_CHECK(solver.gjk.convergence_criterion ==
              GJKConvergenceCriterion::DualityGap);
  BOOST_CHECK(solver.gjk.convergence_criterion_type ==
              GJKConvergenceCriterionType::Absolute);

  BOOST_CHECK(gjk.convergence_criterion == GJKConvergenceCriterion::DualityGap);
  BOOST_CHECK(gjk.convergence_criterion_type ==
              GJKConvergenceCriterionType::Absolute);
}

void test_gjk_cv_criterion(const ShapeBase& shape0, const ShapeBase& shape1,
                           const GJKConvergenceCriterionType cv_type) {
  // Solvers
  unsigned int max_iterations = 128;
  // by default GJK uses the VDB convergence criterion, which is relative.
  GJK gjk1(max_iterations, Scalar(1e-6));

  Scalar tol;
  switch (cv_type) {
    // need to lower the tolerance when absolute
    case GJKConvergenceCriterionType::Absolute:
      tol = Scalar(1e-8);
      break;
    case GJKConvergenceCriterionType::Relative:
      tol = Scalar(1e-6);
      break;
    default:
      throw std::logic_error("Wrong convergence criterion type");
  }

  GJK gjk2(max_iterations, tol);
  gjk2.convergence_criterion = GJKConvergenceCriterion::DualityGap;
  gjk2.convergence_criterion_type = cv_type;

  GJK gjk3(max_iterations, tol);
  gjk3.convergence_criterion = GJKConvergenceCriterion::Hybrid;
  gjk3.convergence_criterion_type = cv_type;

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

  // Run for 3 different cv criterions
  for (size_t i = 0; i < n; ++i) {
    mink_diff.set<false>(&shape0, &shape1, identity, transforms[i]);

    GJK::Status res1 = gjk1.evaluate(mink_diff, init_guess.cast<SolverScalar>(),
                                     init_support_guess);
    BOOST_CHECK(gjk1.getNumIterations() <= max_iterations);
    Vec3ps ray1 = gjk1.ray;
    res1 = gjk1.evaluate(mink_diff, init_guess.cast<SolverScalar>(),
                         init_support_guess);
    BOOST_CHECK(res1 != GJK::Status::Failed);
    EIGEN_VECTOR_IS_APPROX(gjk1.ray, ray1, SolverScalar(1e-8));

    GJK::Status res2 = gjk2.evaluate(mink_diff, init_guess.cast<SolverScalar>(),
                                     init_support_guess);
    BOOST_CHECK(gjk2.getNumIterations() <= max_iterations);
    Vec3ps ray2 = gjk2.ray;
    res2 = gjk2.evaluate(mink_diff, init_guess.cast<SolverScalar>(),
                         init_support_guess);
    BOOST_CHECK(res2 != GJK::Status::Failed);
    EIGEN_VECTOR_IS_APPROX(gjk2.ray, ray2, SolverScalar(1e-8));

    GJK::Status res3 = gjk3.evaluate(mink_diff, init_guess.cast<SolverScalar>(),
                                     init_support_guess);
    BOOST_CHECK(gjk3.getNumIterations() <= max_iterations);
    Vec3ps ray3 = gjk3.ray;
    res3 = gjk3.evaluate(mink_diff, init_guess.cast<SolverScalar>(),
                         init_support_guess);
    BOOST_CHECK(res3 != GJK::Status::Failed);
    EIGEN_VECTOR_IS_APPROX(gjk3.ray, ray3, SolverScalar(1e-8));

    // check that solutions are close enough
    EIGEN_VECTOR_IS_APPROX(gjk1.ray, gjk2.ray, SolverScalar(1e-4));
    EIGEN_VECTOR_IS_APPROX(gjk1.ray, gjk3.ray, SolverScalar(1e-4));
  }
}

BOOST_AUTO_TEST_CASE(cv_criterion_same_solution) {
  Box box0 = Box(Scalar(0.1), Scalar(0.2), Scalar(0.3));
  Box box1 = Box(Scalar(1.1), Scalar(1.2), Scalar(1.3));
  test_gjk_cv_criterion(box0, box1, GJKConvergenceCriterionType::Absolute);
  test_gjk_cv_criterion(box0, box1, GJKConvergenceCriterionType::Relative);
}
