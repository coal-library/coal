/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, INRIA
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

/** \author Justin Carpentier */

#define BOOST_TEST_MODULE COAL_HEIGHT_FIELDS
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "fcl_resources/config.h"

#include "coal/collision.h"
#include "coal/hfield.h"
#include "coal/math/transform.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/mesh_loader/assimp.h"
#include "coal/mesh_loader/loader.h"

#include "coal/collision.h"
#include "coal/internal/traversal_node_hfield_shape.h"

#include "utility.h"
#include <iostream>

using namespace coal;
using RowVector = Eigen::RowVector<Scalar, Eigen::Dynamic>;

template <typename BV>
void test_constant_hfields(const Eigen::DenseIndex nx,
                           const Eigen::DenseIndex ny,
                           const Scalar min_altitude,
                           const Scalar max_altitude) {
  const Scalar x_dim = 1., y_dim = 2.;
  const MatrixXs heights = MatrixXs::Constant(ny, nx, max_altitude);

  HeightField<BV> hfield(x_dim, y_dim, heights, min_altitude);

  BOOST_CHECK(hfield.getXDim() == x_dim);
  BOOST_CHECK(hfield.getYDim() == y_dim);

  const VecXs& x_grid = hfield.getXGrid();
  BOOST_CHECK(x_grid[0] == -x_dim / 2.);
  BOOST_CHECK(x_grid[nx - 1] == x_dim / 2.);

  const VecXs& y_grid = hfield.getYGrid();
  BOOST_CHECK(y_grid[0] == y_dim / 2.);
  BOOST_CHECK(y_grid[ny - 1] == -y_dim / 2.);

  // Test local AABB
  hfield.computeLocalAABB();

  for (Eigen::DenseIndex i = 0; i < nx; ++i) {
    for (Eigen::DenseIndex j = 0; j < ny; ++j) {
      Vec3s point(x_grid[i], y_grid[j], heights(j, i));
      BOOST_CHECK(hfield.aabb_local.contain(point));
    }
  }

  // Test clone
  {
    HeightField<BV>* hfield_clone = hfield.clone();
    hfield_clone->computeLocalAABB();
    BOOST_CHECK(*hfield_clone == hfield);

    delete hfield_clone;
  }

  // Build equivalent object
  const Box equivalent_box(x_dim, y_dim, max_altitude - min_altitude);
  const Transform3s box_placement(
      Matrix3s::Identity(),
      Vec3s(0., 0., (max_altitude + min_altitude) / Scalar(2)));

  // Test collision
  const Sphere sphere(1.);
  static const Transform3s IdTransform;

  const Box box(Vec3s::Ones());

  Transform3s M_sphere, M_box;

  // No collision case
  {
    const Scalar eps_no_collision =
        +Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(!result.isCollision());

    CollisionResult result_check_sphere;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check_sphere);

    BOOST_CHECK(!result_check_sphere.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(!result_check_box.isCollision());
  }

  // Collision case
  {
    const Scalar eps_collision = -Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_collision));
    CollisionRequest
        request;  //(CONTACT | DISTANCE_LOWER_BOUND, (size_t)((nx-1)*(ny-1)));

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(result.isCollision());

    CollisionResult result_check;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check);

    BOOST_CHECK(result_check.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(result_check_box.isCollision());
  }

  // Update height
  hfield.updateHeights(
      MatrixXs::Constant(ny, nx, max_altitude / 2));  // We change nothing

  // No collision case
  {
    const Scalar eps_no_collision =
        +Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(!result.isCollision());

    CollisionResult result_check_sphere;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check_sphere);

    BOOST_CHECK(!result_check_sphere.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(!result_check_box.isCollision());
  }

  // Collision case
  {
    const Scalar eps_collision = -Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_collision));
    CollisionRequest
        request;  //(CONTACT | DISTANCE_LOWER_BOUND, (size_t)((nx-1)*(ny-1)));

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(!result.isCollision());

    CollisionResult result_check;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check);

    BOOST_CHECK(result_check.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(result_check_box.isCollision());
  }

  // Restore height
  hfield.updateHeights(
      MatrixXs::Constant(ny, nx, max_altitude));  // We change nothing

  // Collision case
  {
    const Scalar eps_collision = -Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_collision));
    CollisionRequest
        request;  //(CONTACT | DISTANCE_LOWER_BOUND, (size_t)((nx-1)*(ny-1)));

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(result.isCollision());

    CollisionResult result_check;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check);

    BOOST_CHECK(result_check.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(result_check_box.isCollision());
  }
}

BOOST_AUTO_TEST_CASE(building_constant_hfields) {
  const Scalar max_altitude = 1., min_altitude = 0.;

  test_constant_hfields<OBBRSS>(2, 2, min_altitude,
                                max_altitude);  // Simple case
  test_constant_hfields<OBBRSS>(20, 2, min_altitude, max_altitude);
  test_constant_hfields<OBBRSS>(100, 100, min_altitude, max_altitude);
  //  test_constant_hfields<OBBRSS>(1000,1000,min_altitude,max_altitude);

  test_constant_hfields<AABB>(2, 2, min_altitude, max_altitude);  // Simple case
  test_constant_hfields<AABB>(20, 2, min_altitude, max_altitude);
  test_constant_hfields<AABB>(100, 100, min_altitude, max_altitude);
}

template <typename BV>
void test_negative_security_margin(const Eigen::DenseIndex nx,
                                   const Eigen::DenseIndex ny,
                                   const Scalar min_altitude,
                                   const Scalar max_altitude) {
  const Scalar x_dim = 1., y_dim = 2.;
  const MatrixXs heights = MatrixXs::Constant(ny, nx, max_altitude);

  HeightField<BV> hfield(x_dim, y_dim, heights, min_altitude);

  // Build equivalent object
  const Box equivalent_box(x_dim, y_dim, max_altitude - min_altitude);
  const Transform3s box_placement(
      Matrix3s::Identity(),
      Vec3s(0, 0, (max_altitude + min_altitude) / Scalar(2)));

  // Test collision
  const Sphere sphere(1.);
  static const Transform3s IdTransform;

  const Box box(Vec3s::Ones());

  Transform3s M_sphere, M_box;

  // No collision case
  {
    const Scalar eps_no_collision =
        +Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(!result.isCollision());

    CollisionResult result_check_sphere;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check_sphere);

    BOOST_CHECK(!result_check_sphere.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(!result_check_box.isCollision());
  }

  // Collision case - positive security_margin
  {
    const Scalar eps_no_collision =
        +Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;
    request.security_margin = eps_no_collision + Scalar(1e-6);

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(result.isCollision());

    CollisionResult result_check_sphere;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check_sphere);

    BOOST_CHECK(result_check_sphere.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(result_check_box.isCollision());
  }

  // Collision case
  {
    const Scalar eps_no_collision =
        -Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(result.isCollision());

    CollisionResult result_check_sphere;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check_sphere);

    BOOST_CHECK(result_check_sphere.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(result_check_box.isCollision());
  }

  // No collision case - negative security_margin
  {
    const Scalar eps_no_collision =
        -Scalar(0.1) * (max_altitude - min_altitude);
    M_sphere.setTranslation(
        Vec3s(0., 0., max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(
        Vec3s(0., 0., max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;
    request.security_margin = eps_no_collision - Scalar(1e-4);

    CollisionResult result;
    collide(&hfield, IdTransform, &sphere, M_sphere, request, result);

    BOOST_CHECK(!result.isCollision());

    CollisionResult result_check_sphere;
    collide(&equivalent_box, IdTransform * box_placement, &sphere, M_sphere,
            request, result_check_sphere);

    BOOST_CHECK(!result_check_sphere.isCollision());

    CollisionResult result_check_box;
    collide(&equivalent_box, IdTransform * box_placement, &box, M_box, request,
            result_check_box);

    BOOST_CHECK(!result_check_box.isCollision());
  }
}

BOOST_AUTO_TEST_CASE(negative_security_margin) {
  const Scalar max_altitude = 1., min_altitude = 0.;

  //  test_negative_security_margin<OBBRSS>(100, 100, min_altitude,
  //  max_altitude);
  test_negative_security_margin<AABB>(100, 100, min_altitude, max_altitude);
}

BOOST_AUTO_TEST_CASE(hfield_with_square_hole) {
  const Eigen::DenseIndex nx = 100, ny = 100;

  typedef AABB BV;
  const MatrixXs X = RowVector::LinSpaced(nx, -1., 1.).replicate(ny, 1);
  const MatrixXs Y = VecXs::LinSpaced(ny, 1., -1.).replicate(1, nx);

  const Scalar dim_square = 0.5;

  const Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> hole =
      (X.array().abs() < dim_square) && (Y.array().abs() < dim_square);

  const MatrixXs heights =
      MatrixXs::Ones(ny, nx) - hole.cast<Scalar>().matrix();

  const HeightField<BV> hfield(2., 2., heights, -10.);

  Sphere sphere(Scalar(0.48));
  const Transform3s sphere_pos(Vec3s(0, 0, Scalar(0.5)));
  const Transform3s hfield_pos;

  const CollisionRequest request;

  CollisionResult result;
  {
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }

  sphere.radius = Scalar(0.51);

  {
    CollisionResult result;
    const Sphere sphere2(Scalar(0.51));
    collide(&hfield, hfield_pos, &sphere2, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());
  }
}

BOOST_AUTO_TEST_CASE(hfield_with_circular_hole) {
  const Eigen::DenseIndex nx = 100, ny = 100;

  //  typedef OBBRSS BV; TODO(jcarpent): OBBRSS does not work (compile in Debug
  //  mode), as the overlap of OBBRSS is not satisfactory yet.
  typedef AABB BV;
  const MatrixXs X = RowVector::LinSpaced(nx, -1., 1.).replicate(ny, 1);
  const MatrixXs Y = VecXs::LinSpaced(ny, 1., -1.).replicate(1, nx);

  const Scalar dim_hole = 1;

  const Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> hole =
      (X.array().square() + Y.array().square() <= dim_hole);

  const MatrixXs heights =
      MatrixXs::Ones(ny, nx) - hole.cast<Scalar>().matrix();

  const HeightField<BV> hfield(2., 2., heights, -10.);

  BOOST_CHECK(hfield.getXGrid()[0] == -1.);
  BOOST_CHECK(hfield.getXGrid()[nx - 1] == +1.);

  BOOST_CHECK(hfield.getYGrid()[0] == +1.);
  BOOST_CHECK(hfield.getYGrid()[ny - 1] == -1.);

  Sphere sphere(Scalar(0.975));
  const Transform3s sphere_pos(Vec3s(0, 0, 1));
  const Transform3s hfield_pos;

  const Scalar thresholds[3] = {0, Scalar(0.01), Scalar(-0.005)};

  for (int i = 0; i < 3; ++i) {
    CollisionResult result;
    CollisionRequest request;
    request.security_margin = thresholds[i];
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }

  // Increase the size of the sphere to force the collision
  sphere.radius = Scalar(1.01);
  for (int i = 0; i < 3; ++i) {
    CollisionResult result;
    CollisionRequest request;
    request.security_margin = thresholds[i];
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());
  }

  {
    CollisionResult result;
    CollisionRequest request;
    request.security_margin = Scalar(-0.02);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }
}

bool isApprox(const Scalar v1, const Scalar v2,
              const Scalar tol = Scalar(1e-6)) {
  return std::fabs(v1 - v2) <= tol;
}

Vec3s computeFaceNormal(const Triangle32& triangle,
                        const std::vector<Vec3s>& points) {
  const Vec3s pointA = points[triangle[0]];
  const Vec3s pointB = points[triangle[1]];
  const Vec3s pointC = points[triangle[2]];

  return (pointB - pointA).cross(pointC - pointA).normalized();
}

BOOST_AUTO_TEST_CASE(test_hfield_bin_face_normal_orientation) {
  const Scalar sphere_radius = 1.;
  Sphere sphere(sphere_radius);
  MatrixXs altitutes(2, 2);
  Scalar altitude_value = 1.;
  altitutes.fill(altitude_value);

  typedef AABB BV;
  HeightField<BV> hfield(1., 1., altitutes, 0.);

  const HeightField<BV>::BVS& nodes = hfield.getNodes();
  BOOST_CHECK(nodes.size() == 1);
  const HeightField<BV>::Node& node = nodes[0];

  typedef HFNodeBase::FaceOrientation FaceOrientation;
  BOOST_CHECK((node.contact_active_faces & FaceOrientation::BOTTOM) ==
              int(FaceOrientation::BOTTOM));
  BOOST_CHECK((node.contact_active_faces & FaceOrientation::TOP) ==
              int(FaceOrientation::TOP));
  BOOST_CHECK((node.contact_active_faces & FaceOrientation::NORTH) ==
              int(FaceOrientation::NORTH));
  BOOST_CHECK((node.contact_active_faces & FaceOrientation::SOUTH) ==
              int(FaceOrientation::SOUTH));
  BOOST_CHECK((node.contact_active_faces & FaceOrientation::EAST) ==
              int(FaceOrientation::EAST));
  BOOST_CHECK((node.contact_active_faces & FaceOrientation::WEST) ==
              int(FaceOrientation::WEST));

  ConvexTpl<Triangle32> convex1, convex2;
  int convex1_active_faces, convex2_active_faces;
  details::buildConvexTriangles(node, hfield, convex1, convex1_active_faces,
                                convex2, convex2_active_faces);

  // Check face normals for convex1
  {
    const std::vector<Vec3s>& points = *(convex1.points);
    // BOTTOM
    {
      const Triangle32& triangle = (*(convex1.polygons))[0];

      BOOST_CHECK(
          computeFaceNormal(triangle, points).isApprox(-Vec3s::UnitZ()));
    }

    // TOP
    {
      const Triangle32& triangle = (*(convex1.polygons))[1];

      BOOST_CHECK(computeFaceNormal(triangle, points).isApprox(Vec3s::UnitZ()));
    }

    // WEST sides
    {
      const Triangle32& triangle1 = (*(convex1.polygons))[2];
      const Triangle32& triangle2 = (*(convex1.polygons))[3];

      BOOST_CHECK(
          computeFaceNormal(triangle1, points).isApprox(-Vec3s::UnitX()));
      BOOST_CHECK(
          computeFaceNormal(triangle2, points).isApprox(-Vec3s::UnitX()));
    }

    // SOUTH-EAST sides
    {
      const Vec3s south_east_normal = Vec3s(1., -1., 0).normalized();

      const Triangle32& triangle1 = (*(convex1.polygons))[4];
      const Triangle32& triangle2 = (*(convex1.polygons))[5];

      BOOST_CHECK(
          computeFaceNormal(triangle1, points).isApprox(south_east_normal));
      BOOST_CHECK(
          computeFaceNormal(triangle2, points).isApprox(south_east_normal));
    }

    // NORTH sides
    {
      const Triangle32& triangle1 = (*(convex1.polygons))[6];
      const Triangle32& triangle2 = (*(convex1.polygons))[7];

      std::cout << "computeFaceNormal(triangle1,points): "
                << computeFaceNormal(triangle1, points).transpose()
                << std::endl;
      BOOST_CHECK(
          computeFaceNormal(triangle1, points).isApprox(Vec3s::UnitY()));
      BOOST_CHECK(
          computeFaceNormal(triangle2, points).isApprox(Vec3s::UnitY()));
    }
  }

  // Check face normals for convex2
  {
    const std::vector<Vec3s>& points = *(convex2.points);

    // BOTTOM
    {
      const Triangle32& triangle = (*(convex2.polygons))[0];

      BOOST_CHECK(
          computeFaceNormal(triangle, points).isApprox(-Vec3s::UnitZ()));
    }

    // TOP
    {
      const Triangle32& triangle = (*(convex2.polygons))[1];

      BOOST_CHECK(computeFaceNormal(triangle, points).isApprox(Vec3s::UnitZ()));
    }

    // SOUTH sides
    {
      const Triangle32& triangle1 = (*(convex2.polygons))[2];
      const Triangle32& triangle2 = (*(convex2.polygons))[3];

      BOOST_CHECK(
          computeFaceNormal(triangle1, points).isApprox(-Vec3s::UnitY()));
      BOOST_CHECK(
          computeFaceNormal(triangle2, points).isApprox(-Vec3s::UnitY()));
    }

    // NORTH-WEST sides
    {
      const Vec3s north_west_normal = Vec3s(-1., 1., 0).normalized();

      const Triangle32& triangle1 = (*(convex2.polygons))[4];
      const Triangle32& triangle2 = (*(convex2.polygons))[5];

      BOOST_CHECK(
          computeFaceNormal(triangle1, points).isApprox(north_west_normal));
      BOOST_CHECK(
          computeFaceNormal(triangle2, points).isApprox(north_west_normal));
    }

    // EAST sides
    {
      const Triangle32& triangle1 = (*(convex2.polygons))[6];
      const Triangle32& triangle2 = (*(convex2.polygons))[7];

      BOOST_CHECK(
          computeFaceNormal(triangle1, points).isApprox(Vec3s::UnitX()));
      BOOST_CHECK(
          computeFaceNormal(triangle2, points).isApprox(Vec3s::UnitX()));
    }
  }
}

BOOST_AUTO_TEST_CASE(test_hfield_bin_active_faces) {
  typedef HFNodeBase::FaceOrientation FaceOrientation;
  const Scalar sphere_radius = 1.;
  Sphere sphere(sphere_radius);
  MatrixXs altitutes(3, 3);
  Scalar altitude_value = 1.;
  altitutes.fill(altitude_value);

  typedef AABB BV;
  HeightField<BV> hfield(1., 1., altitutes, 0.);

  const HeightField<BV>::BVS& nodes = hfield.getNodes();
  BOOST_CHECK(nodes.size() == 7);

  for (const auto& node : nodes) {
    if (node.isLeaf()) {
      BOOST_CHECK((node.contact_active_faces & FaceOrientation::BOTTOM) ==
                  int(FaceOrientation::BOTTOM));
      BOOST_CHECK((node.contact_active_faces & FaceOrientation::TOP) ==
                  int(FaceOrientation::TOP));

      if (node.x_id == 0)
        BOOST_CHECK((node.contact_active_faces & FaceOrientation::WEST) ==
                    int(FaceOrientation::WEST));
      if (node.y_id == 0)
        BOOST_CHECK((node.contact_active_faces & FaceOrientation::NORTH) ==
                    int(FaceOrientation::NORTH));

      if (node.x_id == 1)
        BOOST_CHECK((node.contact_active_faces & FaceOrientation::EAST) ==
                    int(FaceOrientation::EAST));
      if (node.y_id == 1)
        BOOST_CHECK((node.contact_active_faces & FaceOrientation::SOUTH) ==
                    int(FaceOrientation::SOUTH));
    }
  }
}

BOOST_AUTO_TEST_CASE(test_hfield_single_bin) {
  const Scalar sphere_radius = 1.;
  Sphere sphere(sphere_radius);
  MatrixXs altitutes(2, 2);
  Scalar altitude_value = 1.;
  altitutes.fill(altitude_value);

  typedef AABB BV;
  HeightField<BV> hfield(1., 1., altitutes, 0.);

  const HeightField<BV>::BVS& nodes = hfield.getNodes();
  BOOST_CHECK(nodes.size() == 1);

  // Collision from the TOP
  {
    const Transform3s sphere_pos(Vec3s(0., 0., 2.));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = -Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
    BOOST_CHECK(
        isApprox(result.distance_lower_bound, -request.security_margin));
  }

  // Same, but with a positive margin.
  {
    const Transform3s sphere_pos(Vec3s(0., 0., 2.));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = +Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());
    if (result.isCollision()) {
      const Contact& contact = result.getContact(0);
      BOOST_CHECK(contact.normal.isApprox(Vec3s::UnitZ()));
      std::cout << "contact.penetration_depth: " << contact.penetration_depth
                << std::endl;
      BOOST_CHECK(isApprox(contact.penetration_depth, 0.));
    }
  }

  // Collision from the BOTTOM
  {
    const Transform3s sphere_pos(Vec3s(0., 0., -1.));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = -Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }

  {
    const Transform3s sphere_pos(Vec3s(0., 0., -1.));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = +Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());
    {
      const Contact& contact = result.getContact(0);
      BOOST_CHECK(contact.normal.isApprox(-Vec3s::UnitZ()));
      std::cout << "contact.penetration_depth: " << contact.penetration_depth
                << std::endl;
      BOOST_CHECK(isApprox(contact.penetration_depth, 0.));
    }
  }

  // Collision from the WEST
  {
    const Transform3s sphere_pos(
        Vec3s(hfield.getXGrid()[0] - sphere_radius, 0., 0.5));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = -Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }

  {
    const Transform3s sphere_pos(
        Vec3s(hfield.getXGrid()[0] - sphere_radius, 0., 0.5));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = +Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());
    if (result.isCollision()) {
      const Contact& contact = result.getContact(0);
      BOOST_CHECK(contact.normal.isApprox(-Vec3s::UnitX()));
      BOOST_CHECK(isApprox(contact.penetration_depth, 0.));
    }
  }

  // Collision from the EAST
  {
    const Transform3s sphere_pos(
        Vec3s(hfield.getXGrid()[1] + sphere_radius, 0., 0.5));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = -Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }

  {
    const Transform3s sphere_pos(
        Vec3s(hfield.getXGrid()[1] + sphere_radius, 0., 0.5));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = +Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());

    if (result.isCollision()) {
      const Contact& contact = result.getContact(0);
      BOOST_CHECK(contact.normal.isApprox(Vec3s::UnitX()));
      BOOST_CHECK(isApprox(contact.penetration_depth, 0.));
    }
  }

  // Collision from the NORTH
  {
    const Transform3s sphere_pos(
        Vec3s(0., hfield.getYGrid()[0] + sphere_radius, 0.5));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = -Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }

  {
    const Transform3s sphere_pos(
        Vec3s(0, hfield.getYGrid()[0] + sphere_radius, Scalar(0.5)));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = +Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());

    if (result.isCollision()) {
      const Contact& contact = result.getContact(0);
      BOOST_CHECK(contact.normal.isApprox(Vec3s::UnitY()));
      BOOST_CHECK(isApprox(contact.penetration_depth, 0.));
    }
  }

  // Collision from the SOUTH
  {
    const Transform3s sphere_pos(
        Vec3s(0., hfield.getYGrid()[1] - sphere_radius, 0.5));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = -Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(!result.isCollision());
  }

  {
    const Transform3s sphere_pos(
        Vec3s(0., hfield.getYGrid()[1] - sphere_radius, Scalar(0.5)));
    const Transform3s hfield_pos;

    CollisionResult result;
    CollisionRequest request;
    request.security_margin = +Scalar(0.005);
    collide(&hfield, hfield_pos, &sphere, sphere_pos, request, result);

    BOOST_CHECK(result.isCollision());

    if (result.isCollision()) {
      const Contact& contact = result.getContact(0);
      BOOST_CHECK(contact.normal.isApprox(-Vec3s::UnitY()));
      BOOST_CHECK(isApprox(contact.penetration_depth, 0.));
    }
  }
}
