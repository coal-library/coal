/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026 Coal Library
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the conditions of the BSD
 *  3-Clause License are satisfied.
 */

#define _USE_MATH_DEFINES
#include <cmath>

#define BOOST_TEST_MODULE COAL_RSS_DISTANCE
#include <boost/test/included/unit_test.hpp>

#include "coal/BV/RSS.h"
#include "coal/math/transform.h"
#include "utility.h"

using namespace coal;

// Locks down the dispatch contract introduced by the rectDistance output
// specialization (`rectDistanceImpl<true>` / `rectDistanceImpl<false>`):
//
//   - call shapes that select the no-points path (P==NULL or Q==NULL or both)
//     must return the same distance as the with-points path
//   - the with-points path must return a (P, Q) pair that lies on the two
//     rectangles, with |Q-P| equal to the reported distance
//
// Without this gate, a regression in either templated path would only show up
// indirectly through mesh-distance tests, which are noisy and hard to bisect.
BOOST_AUTO_TEST_SUITE(rss_distance)

namespace {

RSS makeRSS(const Vec3s& Tr, const Matrix3s& axes, Scalar a0, Scalar a1) {
  RSS r;
  r.Tr = Tr;
  r.axes = axes;
  r.length[0] = a0;
  r.length[1] = a1;
  r.radius = Scalar(0);
  return r;
}

// Builds the rotation matrix for an angle-axis rotation (avoids pulling in
// Eigen::AngleAxis just for the test).
Matrix3s rotZ(Scalar theta) {
  Matrix3s R;
  R << std::cos(theta), -std::sin(theta), 0, std::sin(theta), std::cos(theta),
      0, 0, 0, 1;
  return R;
}

Matrix3s rotX(Scalar theta) {
  Matrix3s R;
  R << 1, 0, 0, 0, std::cos(theta), -std::sin(theta), 0, std::sin(theta),
      std::cos(theta);
  return R;
}

}  // namespace

BOOST_AUTO_TEST_CASE(specialization_consistency) {
  struct Case {
    RSS r1, r2;
    const char* name;
  };

  const Matrix3s I = Matrix3s::Identity();
  std::vector<Case> cases = {
      {makeRSS(Vec3s(0, 0, 0), I, 1, 1), makeRSS(Vec3s(5, 0, 0), I, 1, 1),
       "axis_aligned_separated"},
      {makeRSS(Vec3s(0, 0, 0), I, 2, 1),
       makeRSS(Vec3s(0, 5, 0), rotZ(Scalar(M_PI) / 4), 1, 2),
       "skewed_separated"},
      {makeRSS(Vec3s(0, 0, 0), I, 1, 2),
       makeRSS(Vec3s(0, 0, 3), rotX(Scalar(M_PI) / 3), 2, 1),
       "rotated_separated"},
      {makeRSS(Vec3s(0, 0, 0), I, 1, 1),
       makeRSS(Vec3s(Scalar(0.5), 0, Scalar(0.1)), I, 1, 1), "overlapping"},
  };

  for (const Case& c : cases) {
    BOOST_TEST_CHECKPOINT(c.name);

    const Vec3s sentinel(Scalar(123.0), Scalar(456.0), Scalar(789.0));
    Vec3s P = sentinel;
    Vec3s Q = sentinel;

    // 1. Both points requested (rectDistanceImpl<true>)
    const Scalar d_full = c.r1.distance(c.r2, &P, &Q);

    // 2. No points requested (rectDistanceImpl<false>)
    const Scalar d_dist_only = c.r1.distance(c.r2, nullptr, nullptr);

    // 3. P only / 4. Q only — dispatcher falls through to <false>; sentinel
    // values must remain untouched.
    Vec3s P_alt = sentinel;
    const Scalar d_P_only = c.r1.distance(c.r2, &P_alt, nullptr);
    Vec3s Q_alt = sentinel;
    const Scalar d_Q_only = c.r1.distance(c.r2, nullptr, &Q_alt);

    BOOST_CHECK_CLOSE(d_full, d_dist_only, Scalar(1e-10));
    BOOST_CHECK_CLOSE(d_full, d_P_only, Scalar(1e-10));
    BOOST_CHECK_CLOSE(d_full, d_Q_only, Scalar(1e-10));

    BOOST_CHECK(P_alt.isApprox(sentinel));
    BOOST_CHECK(Q_alt.isApprox(sentinel));

    // |Q - P| should equal the reported distance.
    BOOST_CHECK_CLOSE((Q - P).norm(), d_full, Scalar(1e-9));

    // P sits on rect 1: bring back to rect-1 local frame.
    const Vec3s P_local = c.r1.axes.transpose() * (P - c.r1.Tr);
    BOOST_CHECK_LE(std::abs(P_local[0]), c.r1.length[0] + Scalar(1e-9));
    BOOST_CHECK_LE(std::abs(P_local[1]), c.r1.length[1] + Scalar(1e-9));
    BOOST_CHECK_LE(std::abs(P_local[2]), Scalar(1e-9));

    // Q sits on rect 2: bring back to rect-2 local frame.
    const Vec3s Q_local = c.r2.axes.transpose() * (Q - c.r2.Tr);
    BOOST_CHECK_LE(std::abs(Q_local[0]), c.r2.length[0] + Scalar(1e-9));
    BOOST_CHECK_LE(std::abs(Q_local[1]), c.r2.length[1] + Scalar(1e-9));
    BOOST_CHECK_LE(std::abs(Q_local[2]), Scalar(1e-9));
  }
}

BOOST_AUTO_TEST_SUITE_END()
