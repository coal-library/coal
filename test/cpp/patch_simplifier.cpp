
//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE COAL_PATCH_SIMPLIFIER
#include <boost/test/unit_test.hpp>

#include "coal/contact_patch/polygon_convex_hull.h"
#include "coal/contact_patch/contact_patch_simplifier.h"

using namespace coal;

/// @brief Generate a random ContactPatch by creating a random polygon
/// and computing its convex hull.
ContactPatch generateRandomPatch(std::size_t max_num_points = 20) {
  // Generate a random cloud of 2D points.
  std::vector<Vec2s> cloud;
  cloud.reserve(max_num_points);
  for (std::size_t i = 0; i < max_num_points; ++i) {
    cloud.emplace_back(Vec2s::Random());
  }

  // Compute the convex hull of the cloud.
  std::vector<Vec2s> cvx_hull;
  computePolygonConvexHull(cloud, cvx_hull);

  // Create a ContactPatch and set its points to the convex hull.
  ContactPatch patch;
  patch.points() = cvx_hull;
  return patch;
}

/// @brief Compute the barycenter of a contact patch.
Vec2s computeBarycenter(const ContactPatch& patch) {
  Vec2s barycenter = Vec2s::Zero();
  for (std::size_t i = 0; i < patch.size(); ++i) {
    barycenter += patch.point(i);
  }
  barycenter /= static_cast<Scalar>(patch.size());
  return barycenter;
}

/// @brief Find the index of the point farthest from a reference point.
std::size_t findFarthestPoint(const ContactPatch& patch,
                              const Vec2s& reference) {
  std::size_t farthest_idx = 0;
  Scalar max_dist_sq = 0;
  for (std::size_t i = 0; i < patch.size(); ++i) {
    const Scalar dist_sq = (patch.point(i) - reference).squaredNorm();
    if (dist_sq > max_dist_sq) {
      max_dist_sq = dist_sq;
      farthest_idx = i;
    }
  }
  return farthest_idx;
}

BOOST_AUTO_TEST_CASE(patch_simplifier_barycenter) {
  const std::size_t desired_size = 1;
  for (std::size_t i = 0; i < 10; ++i) {
    ContactPatch patch = generateRandomPatch(20);

    ContactPatch patch_naive = patch;
    ContactPatchSimplifierNaive naive_simplifier;
    naive_simplifier.simplify(patch_naive, desired_size);

    ContactPatch patch_max_area = patch;
    ContactPatchSimplifierMaxArea max_area_simplifier;
    max_area_simplifier.simplify(patch_max_area, desired_size);

    // Check that both simplified patches contain one point.
    BOOST_CHECK_EQUAL(patch_naive.size(), 1);
    BOOST_CHECK_EQUAL(patch_max_area.size(), 1);

    // Check that this point is the barycenter of the points in `patch`.
    const Vec2s expected_barycenter = computeBarycenter(patch);
    const Scalar tol = Eigen::NumTraits<Scalar>::dummy_precision();
    BOOST_CHECK((patch_naive.point(0) - expected_barycenter).norm() < tol);
    BOOST_CHECK((patch_max_area.point(0) - expected_barycenter).norm() < tol);
  }
}

BOOST_AUTO_TEST_CASE(patch_simplifier_two_points) {
  const std::size_t desired_size = 2;
  for (std::size_t i = 0; i < 10; ++i) {
    ContactPatch patch = generateRandomPatch(20);

    ContactPatch patch_naive = patch;
    ContactPatchSimplifierNaive naive_simplifier;
    naive_simplifier.simplify(patch_naive, desired_size);

    ContactPatch patch_max_area = patch;
    ContactPatchSimplifierMaxArea max_area_simplifier;
    max_area_simplifier.simplify(patch_max_area, desired_size);

    // Check that both simplified patches contain two points.
    BOOST_CHECK_EQUAL(patch_naive.size(), 2);
    BOOST_CHECK_EQUAL(patch_max_area.size(), 2);

    // Check that these points are the first point of `patch` and the point
    // farthest from it.
    const Vec2s first_point = patch.point(0);
    const std::size_t farthest_idx = findFarthestPoint(patch, first_point);
    const Vec2s farthest_point = patch.point(farthest_idx);

    const Scalar tol = Eigen::NumTraits<Scalar>::dummy_precision();

    // For naive simplifier
    BOOST_CHECK((patch_naive.point(0) - first_point).norm() < tol);
    BOOST_CHECK((patch_naive.point(1) - farthest_point).norm() < tol);

    // For max area simplifier
    BOOST_CHECK((patch_max_area.point(0) - first_point).norm() < tol);
    BOOST_CHECK((patch_max_area.point(1) - farthest_point).norm() < tol);
  }
}

BOOST_AUTO_TEST_CASE(patch_simplifier) {
  const std::vector<std::size_t> desired_sizes = {3, 4, 5, 6, 30};
  for (const std::size_t desired_size : desired_sizes) {
    for (std::size_t i = 0; i < 10; ++i) {
      ContactPatch patch = generateRandomPatch(20);
      const Scalar original_area = patch.computeArea();

      ContactPatch patch_naive = patch;
      ContactPatchSimplifierNaive naive_simplifier;
      naive_simplifier.simplify(patch_naive, desired_size);

      ContactPatch patch_max_area = patch;
      ContactPatchSimplifierMaxArea max_area_simplifier;
      max_area_simplifier.simplify(patch_max_area, desired_size);

      // Check that both simplified patches have the expected size.
      const std::size_t expected_size = std::min(desired_size, patch.size());
      BOOST_CHECK_EQUAL(patch_naive.size(), expected_size);
      BOOST_CHECK_EQUAL(patch_max_area.size(), expected_size);

      // Compute areas of simplified patches.
      const Scalar naive_area = patch_naive.computeArea();
      const Scalar max_area_area = patch_max_area.computeArea();

      // Check that both simplified patches have the same area (both algorithms
      // should find the maximum area subset).
      const Scalar tol = Eigen::NumTraits<Scalar>::dummy_precision();
      BOOST_CHECK_CLOSE(naive_area, max_area_area, tol);

      // Check that this area is smaller than or equal to the original patch's
      // area.
      BOOST_CHECK_LE(naive_area, original_area + tol);
      BOOST_CHECK_LE(max_area_area, original_area + tol);
    }
  }
}
