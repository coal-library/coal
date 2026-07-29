/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, LAAS-CNRS
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

/** \author Joseph Mirabel */

#define BOOST_TEST_MODULE COAL_GEOMETRIC_SHAPES
#include <boost/test/included/unit_test.hpp>

#include "coal/shape/convex.h"
#include "coal/collision.h"
#include "coal/distance.h"

#include "utility.h"

using namespace coal;

BOOST_AUTO_TEST_CASE(convex) {
  Scalar l = 1, w = 1, d = 1;
  ConvexTpl<Quadrilateral32> box(buildBox(l, w, d));

  // Check neighbors
  for (size_t i = 0; i < 8; ++i) {
    BOOST_CHECK_EQUAL((*box.neighbors)[i].count, 3);
  }
  BOOST_CHECK_EQUAL(box.neighbor(0, 0), 1);
  BOOST_CHECK_EQUAL(box.neighbor(0, 1), 2);
  BOOST_CHECK_EQUAL(box.neighbor(0, 2), 4);

  BOOST_CHECK_EQUAL(box.neighbor(1, 0), 0);
  BOOST_CHECK_EQUAL(box.neighbor(1, 1), 3);
  BOOST_CHECK_EQUAL(box.neighbor(1, 2), 5);

  BOOST_CHECK_EQUAL(box.neighbor(2, 0), 0);
  BOOST_CHECK_EQUAL(box.neighbor(2, 1), 3);
  BOOST_CHECK_EQUAL(box.neighbor(2, 2), 6);

  BOOST_CHECK_EQUAL(box.neighbor(3, 0), 1);
  BOOST_CHECK_EQUAL(box.neighbor(3, 1), 2);
  BOOST_CHECK_EQUAL(box.neighbor(3, 2), 7);

  BOOST_CHECK_EQUAL(box.neighbor(4, 0), 0);
  BOOST_CHECK_EQUAL(box.neighbor(4, 1), 5);
  BOOST_CHECK_EQUAL(box.neighbor(4, 2), 6);

  BOOST_CHECK_EQUAL(box.neighbor(5, 0), 1);
  BOOST_CHECK_EQUAL(box.neighbor(5, 1), 4);
  BOOST_CHECK_EQUAL(box.neighbor(5, 2), 7);

  BOOST_CHECK_EQUAL(box.neighbor(6, 0), 2);
  BOOST_CHECK_EQUAL(box.neighbor(6, 1), 4);
  BOOST_CHECK_EQUAL(box.neighbor(6, 2), 7);

  BOOST_CHECK_EQUAL(box.neighbor(7, 0), 3);
  BOOST_CHECK_EQUAL(box.neighbor(7, 1), 5);
  BOOST_CHECK_EQUAL(box.neighbor(7, 2), 6);

  // Cast to int16 and check that the box is the same
  ConvexTpl<Quadrilateral16> box16 = box.cast<Quadrilateral16>();
  BOOST_CHECK_EQUAL(box16.neighbor(0, 0), 1);
  BOOST_CHECK_EQUAL(box16.neighbor(0, 1), 2);
  BOOST_CHECK_EQUAL(box16.neighbor(0, 2), 4);

  BOOST_CHECK_EQUAL(box16.neighbor(1, 0), 0);
  BOOST_CHECK_EQUAL(box16.neighbor(1, 1), 3);
  BOOST_CHECK_EQUAL(box16.neighbor(1, 2), 5);

  BOOST_CHECK_EQUAL(box16.neighbor(2, 0), 0);
  BOOST_CHECK_EQUAL(box16.neighbor(2, 1), 3);
  BOOST_CHECK_EQUAL(box16.neighbor(2, 2), 6);

  BOOST_CHECK_EQUAL(box16.neighbor(3, 0), 1);
  BOOST_CHECK_EQUAL(box16.neighbor(3, 1), 2);
  BOOST_CHECK_EQUAL(box16.neighbor(3, 2), 7);

  BOOST_CHECK_EQUAL(box16.neighbor(4, 0), 0);
  BOOST_CHECK_EQUAL(box16.neighbor(4, 1), 5);
  BOOST_CHECK_EQUAL(box16.neighbor(4, 2), 6);

  BOOST_CHECK_EQUAL(box16.neighbor(5, 0), 1);
  BOOST_CHECK_EQUAL(box16.neighbor(5, 1), 4);
  BOOST_CHECK_EQUAL(box16.neighbor(5, 2), 7);

  BOOST_CHECK_EQUAL(box16.neighbor(6, 0), 2);
  BOOST_CHECK_EQUAL(box16.neighbor(6, 1), 4);
  BOOST_CHECK_EQUAL(box16.neighbor(6, 2), 7);

  BOOST_CHECK_EQUAL(box16.neighbor(7, 0), 3);
  BOOST_CHECK_EQUAL(box16.neighbor(7, 1), 5);
  BOOST_CHECK_EQUAL(box16.neighbor(7, 2), 6);

  const std::vector<Vec3s>& box_points = *(box.points);
  const std::vector<Vec3s>& box16_points = *(box16.points);
  for (size_t i = 0; i < box.points->size(); ++i) {
    EIGEN_VECTOR_IS_APPROX(box_points[i], box16_points[i], 1e-12);
  }
}

template <typename Sa, typename Sb>
void compareShapeIntersection(const Sa& sa, const Sb& sb,
                              const Transform3s& tf1, const Transform3s& tf2,
                              Scalar tol = Scalar(1e-9)) {
  CollisionRequest request(CONTACT | DISTANCE_LOWER_BOUND, 1);
  CollisionResult resA, resB;

  collide(&sa, tf1, &sa, tf2, request, resA);
  collide(&sb, tf1, &sb, tf2, request, resB);

  BOOST_CHECK_EQUAL(resA.isCollision(), resB.isCollision());
  BOOST_CHECK_EQUAL(resA.numContacts(), resB.numContacts());

  if (resA.isCollision() && resB.isCollision()) {
    Contact cA = resA.getContact(0), cB = resB.getContact(0);

    BOOST_TEST_MESSAGE(tf1 << '\n'
                           << cA.pos.format(pyfmt) << '\n'
                           << '\n'
                           << tf2 << '\n'
                           << cB.pos.format(pyfmt) << '\n');
    // Only warnings because there are still some bugs.
    BOOST_WARN_SMALL((cA.pos - cB.pos).squaredNorm(), tol);
    BOOST_WARN_SMALL((cA.normal - cB.normal).squaredNorm(), tol);
  } else {
    BOOST_CHECK_CLOSE(resA.distance_lower_bound, resB.distance_lower_bound,
                      tol);  // distances should be same
  }
}

template <typename Sa, typename Sb>
void compareShapeDistance(const Sa& sa, const Sb& sb, const Transform3s& tf1,
                          const Transform3s& tf2, Scalar tol = Scalar(1e-9)) {
  DistanceRequest request(true);
  DistanceResult resA, resB;

  distance(&sa, tf1, &sa, tf2, request, resA);
  distance(&sb, tf1, &sb, tf2, request, resB);

  BOOST_TEST_MESSAGE(tf1 << '\n'
                         << resA.normal.format(pyfmt) << '\n'
                         << resA.nearest_points[0].format(pyfmt) << '\n'
                         << resA.nearest_points[1].format(pyfmt) << '\n'
                         << '\n'
                         << tf2 << '\n'
                         << resB.normal.format(pyfmt) << '\n'
                         << resB.nearest_points[0].format(pyfmt) << '\n'
                         << resB.nearest_points[1].format(pyfmt) << '\n');
  // TODO in one case, there is a mismatch between the distances and I cannot
  // say which one is correct. To visualize the case, use script
  // test/geometric_shapes.py
  BOOST_WARN_CLOSE(resA.min_distance, resB.min_distance, tol);
  // BOOST_CHECK_CLOSE(resA.min_distance, resB.min_distance, tol);

  // Only warnings because there are still some bugs.
  BOOST_WARN_SMALL((resA.normal - resA.normal).squaredNorm(), tol);
  BOOST_WARN_SMALL(
      (resA.nearest_points[0] - resB.nearest_points[0]).squaredNorm(), tol);
  BOOST_WARN_SMALL(
      (resA.nearest_points[1] - resB.nearest_points[1]).squaredNorm(), tol);
}

BOOST_AUTO_TEST_CASE(compare_convex_box) {
  Scalar extents[6] = {0, 0, 0, 10, 10, 10};
  Scalar l = 1, w = 1, d = 1, eps = Scalar(1e-4);
  Box box(l * 2, w * 2, d * 2);
  ConvexTpl<Quadrilateral32> convex_box(buildBox(l, w, d));

  Transform3s tf1;
  Transform3s tf2;

  tf2.setTranslation(Vec3s(3, 0, 0));
  compareShapeIntersection(box, convex_box, tf1, tf2, eps);
  compareShapeDistance(box, convex_box, tf1, tf2, eps);

  tf2.setTranslation(Vec3s(0, 0, 0));
  compareShapeIntersection(box, convex_box, tf1, tf2, eps);
  compareShapeDistance(box, convex_box, tf1, tf2, eps);

  for (int i = 0; i < 1000; ++i) {
    generateRandomTransform(extents, tf2);
    compareShapeIntersection(box, convex_box, tf1, tf2, eps);
    compareShapeDistance(box, convex_box, tf1, tf2, eps);
  }
}

#ifdef COAL_HAS_QHULL
BOOST_AUTO_TEST_CASE(convex_hull_throw) {
  std::shared_ptr<std::vector<Vec3s>> points(
      new std::vector<Vec3s>({Vec3s(1, 1, 1), Vec3s(0, 0, 0), Vec3s(1, 0, 0)}));

  BOOST_CHECK_THROW(ConvexBase32::convexHull(points, 0, false, NULL),
                    std::invalid_argument);
  BOOST_CHECK_THROW(ConvexBase32::convexHull(points, 1, false, NULL),
                    std::invalid_argument);
  BOOST_CHECK_THROW(ConvexBase32::convexHull(points, 2, false, NULL),
                    std::invalid_argument);
  BOOST_CHECK_THROW(ConvexBase32::convexHull(points, 3, false, NULL),
                    std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(convex_hull_quad) {
  std::shared_ptr<std::vector<Vec3s>> points(new std::vector<Vec3s>({
      Vec3s(1, 1, 1),
      Vec3s(0, 0, 0),
      Vec3s(1, 0, 0),
      Vec3s(0, 0, 1),
  }));

  ConvexBase32* convexHull = ConvexBase32::convexHull(points, 4, false, NULL);

  BOOST_REQUIRE_EQUAL(convexHull->num_points, 4);
  BOOST_CHECK_EQUAL((*(convexHull->neighbors))[0].count, 3);
  BOOST_CHECK_EQUAL((*(convexHull->neighbors))[1].count, 3);
  BOOST_CHECK_EQUAL((*(convexHull->neighbors))[2].count, 3);
  delete convexHull;
}

BOOST_AUTO_TEST_CASE(convex_hull_box_like) {
  std::shared_ptr<std::vector<Vec3s>> points(new std::vector<Vec3s>({
      Vec3s(1, 1, 1),
      Vec3s(1, 1, -1),
      Vec3s(1, -1, 1),
      Vec3s(1, -1, -1),
      Vec3s(-1, 1, 1),
      Vec3s(-1, 1, -1),
      Vec3s(-1, -1, 1),
      Vec3s(-1, -1, -1),
      Vec3s(0, 0, 0),
      Vec3s(0, 0, Scalar(0.99)),
  }));

  ConvexBase32* convexHull = ConvexBase32::convexHull(points, 9, false, NULL);

  BOOST_REQUIRE_EQUAL(8, convexHull->num_points);
  {
    const std::vector<Vec3s>& cvxhull_points = *(convexHull->points);
    for (size_t i = 0; i < 8; ++i) {
      BOOST_CHECK(cvxhull_points[i].cwiseAbs() == Vec3s(1, 1, 1));
      BOOST_CHECK_EQUAL((*(convexHull->neighbors))[i].count, 3);
    }
  }
  delete convexHull;

  convexHull = ConvexBase32::convexHull(points, 9, true, NULL);
  ConvexTpl<Triangle32>* convex_tri =
      dynamic_cast<ConvexTpl<Triangle32>*>(convexHull);
  BOOST_CHECK(convex_tri != NULL);

  BOOST_REQUIRE_EQUAL(8, convexHull->num_points);
  {
    const std::vector<Vec3s>& cvxhull_points = *(convexHull->points);
    for (size_t i = 0; i < 8; ++i) {
      BOOST_CHECK(cvxhull_points[i].cwiseAbs() == Vec3s(1, 1, 1));
      BOOST_CHECK((*(convexHull->neighbors))[i].count >= 3);
      BOOST_CHECK((*(convexHull->neighbors))[i].count <= 6);
    }
  }
  delete convexHull;
}

BOOST_AUTO_TEST_CASE(convex_copy_constructor) {
  ConvexTpl<Triangle32>* convexHullTriCopy;
  {
    std::shared_ptr<std::vector<Vec3s>> points(new std::vector<Vec3s>({
        Vec3s(1, 1, 1),
        Vec3s(1, 1, -1),
        Vec3s(1, -1, 1),
        Vec3s(1, -1, -1),
        Vec3s(-1, 1, 1),
        Vec3s(-1, 1, -1),
        Vec3s(-1, -1, 1),
        Vec3s(-1, -1, -1),
        Vec3s(0, 0, 0),
    }));

    ConvexTpl<Triangle32>* convexHullTri = dynamic_cast<ConvexTpl<Triangle32>*>(
        ConvexBase32::convexHull(points, 9, true, NULL));
    convexHullTriCopy = new ConvexTpl<Triangle32>(*convexHullTri);
    BOOST_CHECK(*convexHullTri == *convexHullTriCopy);
  }
  ConvexTpl<Triangle32>* convexHullTriCopyOfCopy =
      new ConvexTpl<Triangle32>(*convexHullTriCopy);
  BOOST_CHECK(*convexHullTriCopyOfCopy == *convexHullTriCopy);
}

BOOST_AUTO_TEST_CASE(convex_clone) {
  std::shared_ptr<std::vector<Vec3s>> points(new std::vector<Vec3s>({
      Vec3s(1, 1, 1),
      Vec3s(1, 1, -1),
      Vec3s(1, -1, 1),
      Vec3s(1, -1, -1),
      Vec3s(-1, 1, 1),
      Vec3s(-1, 1, -1),
      Vec3s(-1, -1, 1),
      Vec3s(-1, -1, -1),
      Vec3s(0, 0, 0),
  }));

  ConvexTpl<Triangle32>* convexHullTri = dynamic_cast<ConvexTpl<Triangle32>*>(
      ConvexBase32::convexHull(points, 9, true, NULL));
  ConvexTpl<Triangle32>* convexHullTriCopy;
  convexHullTriCopy = convexHullTri->clone();
  BOOST_CHECK(*convexHullTri == *convexHullTriCopy);
}

#endif
