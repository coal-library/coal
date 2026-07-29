/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2015, CNRS-LAAS and AIST
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/** \author Florent Lamiraux */

#define BOOST_TEST_MODULE COAL_GEOMETRIC_SHAPES
#include <boost/test/included/unit_test.hpp>

#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE((x + 1.0), (1.0), (eps))

#include "utility.h"

#include <cmath>
#include "coal/distance.h"
#include "coal/math/transform.h"
#include "coal/collision.h"
#include "coal/collision_object.h"
#include "coal/shape/geometric_shapes.h"

using coal::Scalar;

BOOST_AUTO_TEST_CASE(distance_capsule_box) {
  typedef coal::shared_ptr<coal::CollisionGeometry> CollisionGeometryPtr_t;
  // Capsule of radius 2 and of height 4
  CollisionGeometryPtr_t capsuleGeometry(new coal::Capsule(2., 4.));
  // Box of size 1 by 2 by 4
  CollisionGeometryPtr_t boxGeometry(new coal::Box(1., 2., 4.));

  // Enable computation of nearest points
  coal::DistanceRequest distanceRequest(true, 0, 0);
  coal::DistanceResult distanceResult;

  // Rotate capsule around y axis by pi/2 and move it behind box
  coal::Transform3s tf1(
      coal::makeQuat(sqrt(Scalar(2)) / 2, 0, sqrt(Scalar(2)) / 2, 0),
      coal::Vec3s(Scalar(-10.), Scalar(0.8), Scalar(1.5)));
  coal::Transform3s tf2;
  coal::CollisionObject capsule(capsuleGeometry, tf1);
  coal::CollisionObject box(boxGeometry, tf2);

  // test distance
  distanceResult.clear();
  coal::distance(&capsule, &box, distanceRequest, distanceResult);
  coal::Vec3s o1 = distanceResult.nearest_points[0];
  coal::Vec3s o2 = distanceResult.nearest_points[1];

  BOOST_CHECK_CLOSE(distanceResult.min_distance, 5.5, 1e-2);
  BOOST_CHECK_CLOSE(o1[0], -6, 1e-2);
  BOOST_CHECK_CLOSE(o1[1], 0.8, 1e-1);
  BOOST_CHECK_CLOSE(o1[2], 1.5, 1e-2);
  BOOST_CHECK_CLOSE(o2[0], -0.5, 1e-2);
  BOOST_CHECK_CLOSE(o2[1], 0.8, 1e-1);
  BOOST_CHECK_CLOSE(o2[2], 1.5, 1e-2);
}
