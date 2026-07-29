/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, INRIA
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

#define _USE_MATH_DEFINES
#include <cmath>

#define BOOST_TEST_MODULE COAL_POLYGON_CONVEX_HULL
#include <boost/test/included/unit_test.hpp>

#include "coal/contact_patch/polygon_convex_hull.h"

using namespace coal;

/// @brief Generate a random cloud of 2D points.
std::vector<Vec2s> generateRandomCloud(std::size_t n) {
  std::vector<Vec2s> cloud;
  cloud.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    cloud.emplace_back(Vec2s::Random());
  }
  return cloud;
}

/// @brief Computes the convex hull of a cloud of 2D points by computing
/// the support point in directions taken on the unit circle.
void computePolygonConvexHullNaive(const std::vector<Vec2s>& cloud,
                                   std::vector<Vec2s>& cvx_hull) {
  std::size_t n = cloud.size() * 1000;

  // lambda to compute the support point of `cloud` in a given direction.
  const auto compute_support_point = [&](const Vec2s& dir) {
    Vec2s best_support = cloud[0];
    Scalar best_support_val = best_support.dot(dir);
    for (const auto& pt : cloud) {
      const Scalar support_val = pt.dot(dir);
      if (support_val > best_support_val) {
        best_support_val = support_val;
        best_support = pt;
      }
    }
    return best_support;
  };

  // lambda to check if a given point is already in `cvx_hull`.
  const auto is_in_cvx_hull = [&](const Vec2s& pt) {
    bool found = false;
    for (const auto& hull_pt : cvx_hull) {
      if ((pt - hull_pt).norm() < Eigen::NumTraits<Scalar>::dummy_precision()) {
        found = true;
        break;
      }
    }
    return found;
  };

  // Starting from direction (1, 0), we take n directions along the unit circle
  // and compute the support point of `cloud` in this direction. If the point is
  // not already in `cvx_hull`, we add it.
  cvx_hull.clear();
  const Scalar angle_step = 2 * M_PI / static_cast<Scalar>(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Scalar theta = static_cast<Scalar>(i) * angle_step;
    const Vec2s dir(std::cos(theta), std::sin(theta));
    const Vec2s support_pt = compute_support_point(dir);
    if (!is_in_cvx_hull(support_pt)) {
      cvx_hull.push_back(support_pt);
    }
  }
}

BOOST_AUTO_TEST_CASE(test_compute_polygon_convex_hull) {
  for (std::size_t i = 0; i < 10; ++i) {
    std::vector<Vec2s> cloud = generateRandomCloud(20);

    std::vector<Vec2s> expected_cvx_hull;
    computePolygonConvexHullNaive(cloud, expected_cvx_hull);

    std::vector<Vec2s> cvx_hull;
    computePolygonConvexHull(cloud, cvx_hull);

    std::cout << "cvx_hull.size()          = " << cvx_hull.size() << "\n";
    std::cout << "expected_cvx_hull.size() = " << expected_cvx_hull.size()
              << "\n";
    BOOST_CHECK(cvx_hull.size() == expected_cvx_hull.size());
    if (cvx_hull.size() == expected_cvx_hull.size()) {
      // Check that the two convex hulls match
      bool same_cvx_hull = true;
      for (const Vec2s& pt : cvx_hull) {
        bool found = false;
        for (const Vec2s& other_pt : expected_cvx_hull) {
          if (pt == other_pt) {
            found = true;
            break;
          }
        }
        if (!found) {
          same_cvx_hull = false;
          break;
        }
      }
      BOOST_CHECK(same_cvx_hull);
    }
  }
}
