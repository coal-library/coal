/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Toyota Research Institute.
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

#define BOOST_TEST_MODULE COAL_SUPPORT_FUNCTION_STATICS
#include <boost/test/included/unit_test.hpp>

#include "coal/data_types.h"
#include "coal/narrowphase/support_functions.h"
#include "coal/shape/geometric_shapes.h"

// Test getShapeSupport functions.
// This file will contains non regression test.

using namespace coal;

// Historically, the support function for boxes had a static local variable
// that would cause a set of invocations to change their answers based on the
// order they were evaluated. This minimal test offers a test against regression
// to that state by evaluating three directions in the pattern A, B, A. A and B
// are spelled in a way that would latch the static local variable with opposing
// values. By evaluating each in sequence, we confirm that each gets the correct
// answer, regardless of what has been evaluated before it.
BOOST_AUTO_TEST_CASE(box_support_function_inflate_regression_test) {
  // Note: This test becomes a tautology for any Scalar type for which 1 cannot
  // be distinguished from 1 + 1e-10 (e.g., float).
  // Also, if the inflation value (1-e10) changes in support_functions.cpp, it
  // will also need to be updated here.
  const Box box(2, 2, 2);
  int hint = 0;

  const Vec3s directions[] = {Vec3s(-1, -1, -1), Vec3s(-1, 0, 0),
                              Vec3s(-1, -1, -1)};
  const Vec3s expected[] = {
      Vec3s(-1, -1, -1), Vec3s(-(Scalar(1) + 1e-10), 0, 0), Vec3s(-1, -1, -1)};

  for (int i = 0; i < 3; ++i) {
    const Vec3s support = details::getSupport(&box, directions[i], hint);
    for (int axis = 0; axis < 3; ++axis) {
      BOOST_CHECK_EQUAL(support[axis], expected[i][axis]);
    }
  }
}
