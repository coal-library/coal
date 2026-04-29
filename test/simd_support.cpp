/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Asensus Surgical
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of Asensus Surgical nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#define BOOST_TEST_MODULE COAL_SIMD_SUPPORT
#include <boost/test/included/unit_test.hpp>

#include "coal/internal/simd_support.h"

#include <cmath>
#include <random>
#include <vector>

namespace {

int scalarMaxDot(const std::vector<coal::Vec3s>& points, const coal::Vec3s& dir,
                 coal::Scalar& maxdot) {
  int best = 0;
  maxdot = points[0].dot(dir);
  for (int i = 1; i < static_cast<int>(points.size()); ++i) {
    const coal::Scalar dot = points[static_cast<size_t>(i)].dot(dir);
    if (dot > maxdot) {
      maxdot = dot;
      best = i;
    }
  }
  return best;
}

void splitPoints(const std::vector<coal::Vec3s>& points,
                 std::vector<coal::Scalar>& xs, std::vector<coal::Scalar>& ys,
                 std::vector<coal::Scalar>& zs) {
  xs.resize(points.size());
  ys.resize(points.size());
  zs.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    xs[i] = points[i][0];
    ys[i] = points[i][1];
    zs[i] = points[i][2];
  }
}

}  // namespace

BOOST_AUTO_TEST_CASE(max_dot_matches_scalar_reference) {
  std::mt19937 rng(1234);
  std::uniform_real_distribution<coal::Scalar> dist(coal::Scalar(-10),
                                                    coal::Scalar(10));

  std::vector<coal::Vec3s> points;
  points.reserve(257);
  for (int i = 0; i < 257; ++i) {
    points.emplace_back(dist(rng), dist(rng), dist(rng));
  }

  for (int i = 0; i < 200; ++i) {
    const coal::Vec3s dir(dist(rng), dist(rng), dist(rng));
    coal::Scalar scalar_dot;
    const int scalar_hint = scalarMaxDot(points, dir, scalar_dot);

    coal::Scalar simd_dot;
    const int simd_hint = coal::details::simd::maxDot(
        points.data(), static_cast<int>(points.size()), dir, simd_dot);

    BOOST_CHECK_EQUAL(simd_hint, scalar_hint);
    BOOST_CHECK_SMALL(std::abs(simd_dot - scalar_dot), coal::Scalar(1e-4));

    std::vector<coal::Scalar> xs, ys, zs;
    splitPoints(points, xs, ys, zs);
    coal::Scalar soa_dot;
    const int soa_hint = coal::details::simd::maxDotSoA(
        xs.data(), ys.data(), zs.data(), static_cast<int>(points.size()), dir,
        soa_dot);

    BOOST_CHECK_EQUAL(soa_hint, scalar_hint);
    BOOST_CHECK_SMALL(std::abs(soa_dot - scalar_dot), coal::Scalar(1e-4));
  }
}

BOOST_AUTO_TEST_CASE(max_dot_preserves_first_index_tie_breaking) {
  const std::vector<coal::Vec3s> points = {
      coal::Vec3s(1, 0, 0), coal::Vec3s(2, 0, 0), coal::Vec3s(2, 0, 0),
      coal::Vec3s(0, 4, 0), coal::Vec3s(2, 0, 0), coal::Vec3s(-1, 0, 0)};
  const coal::Vec3s dir(1, 0, 0);

  coal::Scalar maxdot;
  const int hint = coal::details::simd::maxDot(
      points.data(), static_cast<int>(points.size()), dir, maxdot);

  BOOST_CHECK_EQUAL(hint, 1);
  BOOST_CHECK_EQUAL(maxdot, coal::Scalar(2));

  std::vector<coal::Scalar> xs, ys, zs;
  splitPoints(points, xs, ys, zs);
  coal::Scalar soa_maxdot;
  const int soa_hint = coal::details::simd::maxDotSoA(
      xs.data(), ys.data(), zs.data(), static_cast<int>(points.size()), dir,
      soa_maxdot);

  BOOST_CHECK_EQUAL(soa_hint, 1);
  BOOST_CHECK_EQUAL(soa_maxdot, coal::Scalar(2));
}
