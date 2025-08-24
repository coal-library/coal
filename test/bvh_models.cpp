/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Open Source Robotics Foundation
 *  Copyright (c) 2020, INRIA
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

/** \author Jeongseok Lee */

#define BOOST_TEST_MODULE COAL_BVH_MODELS
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "fcl_resources/config.h"

#include "coal/collision.h"
#include "coal/BVH/BVH_model.h"
#include "coal/BVH/BVH_utility.h"
#include "coal/math/transform.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/mesh_loader/assimp.h"
#include "coal/mesh_loader/loader.h"
#include "utility.h"
#include <iostream>

using namespace coal;

template <typename BV>
void testBVHModelPointCloud() {
  Box box(Vec3s::Ones());
  Scalar a = box.halfSide[0];
  Scalar b = box.halfSide[1];
  Scalar c = box.halfSide[2];
  std::vector<Vec3s> points(8);
  points[0] << a, -b, c;
  points[1] << a, b, c;
  points[2] << -a, b, c;
  points[3] << -a, -b, c;
  points[4] << a, -b, -c;
  points[5] << a, b, -c;
  points[6] << -a, b, -c;
  points[7] << -a, -b, -c;

  {
    shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);

    if (model->getNodeType() != BV_AABB && model->getNodeType() != BV_KDOP16 &&
        model->getNodeType() != BV_KDOP18 &&
        model->getNodeType() != BV_KDOP24) {
      std::cout << "Abort test since '" << getNodeTypeName(model->getNodeType())
                << "' does not support point cloud model. "
                << "Please see issue #67." << std::endl;
      return;
    }

    int result;

    result = model->beginModel();
    BOOST_CHECK_EQUAL(result, BVH_OK);

    for (std::size_t i = 0; i < points.size(); ++i) {
      result = model->addVertex(points[i]);
      BOOST_CHECK_EQUAL(result, BVH_OK);
    }

    result = model->endModel();
    BOOST_CHECK_EQUAL(result, BVH_OK);

    model->computeLocalAABB();

    BOOST_CHECK_EQUAL(model->num_vertices, 8);
    BOOST_CHECK_EQUAL(model->num_tris, 0);
    BOOST_CHECK_EQUAL(model->build_state, BVH_BUILD_STATE_PROCESSED);
  }

  {
    shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);

    if (model->getNodeType() != BV_AABB && model->getNodeType() != BV_KDOP16 &&
        model->getNodeType() != BV_KDOP18 &&
        model->getNodeType() != BV_KDOP24) {
      std::cout << "Abort test since '" << getNodeTypeName(model->getNodeType())
                << "' does not support point cloud model. "
                << "Please see issue #67." << std::endl;
      return;
    }

    MatrixX3s all_points((Eigen::DenseIndex)points.size(), 3);
    for (size_t k = 0; k < points.size(); ++k)
      all_points.row((Eigen::DenseIndex)k) = points[k].transpose();

    int result;

    result = model->beginModel();
    BOOST_CHECK_EQUAL(result, BVH_OK);

    result = model->addVertices(all_points);

    result = model->endModel();
    BOOST_CHECK_EQUAL(result, BVH_OK);

    model->computeLocalAABB();

    BOOST_CHECK_EQUAL(model->num_vertices, 8);
    BOOST_CHECK_EQUAL(model->num_tris, 0);
    BOOST_CHECK_EQUAL(model->build_state, BVH_BUILD_STATE_PROCESSED);
  }
}

template <typename BV>
void testBVHModelTriangles() {
  shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Box box(Vec3s::Ones());
  AABB aabb(Vec3s(-1, 0, -1), Vec3s(1, 1, 1));

  Scalar a = box.halfSide[0];
  Scalar b = box.halfSide[1];
  Scalar c = box.halfSide[2];
  std::vector<Vec3s> points(8);
  std::vector<Triangle32> tri_indices(12);
  points[0] << a, -b, c;
  points[1] << a, b, c;
  points[2] << -a, b, c;
  points[3] << -a, -b, c;
  points[4] << a, -b, -c;
  points[5] << a, b, -c;
  points[6] << -a, b, -c;
  points[7] << -a, -b, -c;

  tri_indices[0].set(0, 4, 1);
  tri_indices[1].set(1, 4, 5);
  tri_indices[2].set(2, 6, 3);
  tri_indices[3].set(3, 6, 7);
  tri_indices[4].set(3, 0, 2);
  tri_indices[5].set(2, 0, 1);
  tri_indices[6].set(6, 5, 7);
  tri_indices[7].set(7, 5, 4);
  tri_indices[8].set(1, 5, 2);
  tri_indices[9].set(2, 5, 6);
  tri_indices[10].set(3, 7, 0);
  tri_indices[11].set(0, 7, 4);

  int result;

  result = model->beginModel();
  BOOST_CHECK_EQUAL(result, BVH_OK);

  for (std::size_t i = 0; i < tri_indices.size(); ++i) {
    result =
        model->addTriangle(points[tri_indices[i][0]], points[tri_indices[i][1]],
                           points[tri_indices[i][2]]);
    BOOST_CHECK_EQUAL(result, BVH_OK);
  }

  result = model->endModel();
  BOOST_CHECK_EQUAL(result, BVH_OK);

  model->computeLocalAABB();

  BOOST_CHECK_EQUAL(model->num_vertices, 12 * 3);
  BOOST_CHECK_EQUAL(model->num_tris, 12);
  BOOST_CHECK_EQUAL(model->build_state, BVH_BUILD_STATE_PROCESSED);

  Transform3s pose;
  shared_ptr<BVHModel<BV> > cropped(BVHExtract(*model, pose, aabb));
  BOOST_REQUIRE(cropped);
  BOOST_CHECK(cropped->build_state == BVH_BUILD_STATE_PROCESSED);
  BOOST_CHECK_EQUAL(cropped->num_vertices, model->num_vertices - 6);
  BOOST_CHECK_EQUAL(cropped->num_tris, model->num_tris - 2);

  pose.setTranslation(Vec3s(0, 1, 0));
  cropped.reset(BVHExtract(*model, pose, aabb));
  BOOST_REQUIRE(cropped);
  BOOST_CHECK(cropped->build_state == BVH_BUILD_STATE_PROCESSED);
  BOOST_CHECK_EQUAL(cropped->num_vertices, model->num_vertices - 6);
  BOOST_CHECK_EQUAL(cropped->num_tris, model->num_tris - 2);

  pose.setTranslation(Vec3s(0, 0, 0));
  Scalar sqrt2_2 = std::sqrt(Scalar(2)) / 2;
  pose.setQuatRotation(Quats(sqrt2_2, sqrt2_2, 0, 0));
  cropped.reset(BVHExtract(*model, pose, aabb));
  BOOST_REQUIRE(cropped);
  BOOST_CHECK(cropped->build_state == BVH_BUILD_STATE_PROCESSED);
  BOOST_CHECK_EQUAL(cropped->num_vertices, model->num_vertices - 6);
  BOOST_CHECK_EQUAL(cropped->num_tris, model->num_tris - 2);

  pose.setTranslation(-Vec3s(1, 1, 1));
  pose.setQuatRotation(Quats::Identity());
  cropped.reset(BVHExtract(*model, pose, aabb));
  BOOST_CHECK(!cropped);

  aabb = AABB(Vec3s(Scalar(-0.1), Scalar(-0.1), Scalar(-0.1)),
              Vec3s(Scalar(0.1), Scalar(0.1), Scalar(0.1)));
  pose.setTranslation(Vec3s(-0.5, -0.5, 0));
  cropped.reset(BVHExtract(*model, pose, aabb));
  BOOST_REQUIRE(cropped);
  BOOST_CHECK_EQUAL(cropped->num_tris, 2);
  BOOST_CHECK_EQUAL(cropped->num_vertices, 6);
}

template <typename BV>
void testBVHModelSubModel() {
  shared_ptr<BVHModel<BV> > model(new BVHModel<BV>);
  Box box(Vec3s::Ones());

  Scalar a = box.halfSide[0];
  Scalar b = box.halfSide[1];
  Scalar c = box.halfSide[2];
  std::vector<Vec3s> points(8);
  std::vector<Triangle32> tri_indices(12);
  points[0] << a, -b, c;
  points[1] << a, b, c;
  points[2] << -a, b, c;
  points[3] << -a, -b, c;
  points[4] << a, -b, -c;
  points[5] << a, b, -c;
  points[6] << -a, b, -c;
  points[7] << -a, -b, -c;

  tri_indices[0].set(0, 4, 1);
  tri_indices[1].set(1, 4, 5);
  tri_indices[2].set(2, 6, 3);
  tri_indices[3].set(3, 6, 7);
  tri_indices[4].set(3, 0, 2);
  tri_indices[5].set(2, 0, 1);
  tri_indices[6].set(6, 5, 7);
  tri_indices[7].set(7, 5, 4);
  tri_indices[8].set(1, 5, 2);
  tri_indices[9].set(2, 5, 6);
  tri_indices[10].set(3, 7, 0);
  tri_indices[11].set(0, 7, 4);

  int result;

  result = model->beginModel();
  BOOST_CHECK_EQUAL(result, BVH_OK);

  result = model->addSubModel(points, tri_indices);
  BOOST_CHECK_EQUAL(result, BVH_OK);

  result = model->endModel();
  BOOST_CHECK_EQUAL(result, BVH_OK);

  model->computeLocalAABB();

  BOOST_CHECK_EQUAL(model->num_vertices, 8);
  BOOST_CHECK_EQUAL(model->num_tris, 12);
  BOOST_CHECK_EQUAL(model->build_state, BVH_BUILD_STATE_PROCESSED);
}

template <typename BV>
void testBVHModel() {
  testBVHModelTriangles<BV>();
  testBVHModelPointCloud<BV>();
  testBVHModelSubModel<BV>();
}

BOOST_AUTO_TEST_CASE(building_bvh_models) {
  testBVHModel<AABB>();
  testBVHModel<OBB>();
  testBVHModel<RSS>();
  testBVHModel<kIOS>();
  testBVHModel<OBBRSS>();
  testBVHModel<KDOP<16> >();
  testBVHModel<KDOP<18> >();
  testBVHModel<KDOP<24> >();
}

template <class BoundingVolume>
void testLoadPolyhedron() {
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  std::string env = (path / "env.obj").string(),
              rob = (path / "rob.obj").string();

  typedef BVHModel<BoundingVolume> Polyhedron_t;
  typedef shared_ptr<Polyhedron_t> PolyhedronPtr_t;
  PolyhedronPtr_t P1(new Polyhedron_t), P2;

  Vec3s scale;
  scale.setConstant(1);
  loadPolyhedronFromResource(env, scale, P1);

  scale.setConstant(-1);
  CachedMeshLoader loader(P1->getNodeType());
  CollisionGeometryPtr_t geom = loader.load(env, scale);
  P2 = dynamic_pointer_cast<Polyhedron_t>(geom);
  BOOST_REQUIRE(P2);

  BOOST_CHECK_EQUAL(P1->num_tris, P2->num_tris);
  BOOST_CHECK_EQUAL(P1->num_vertices, P2->num_vertices);
  BOOST_CHECK_EQUAL(P1->getNumBVs(), P2->getNumBVs());

  CollisionGeometryPtr_t geom2 = loader.load(env, scale);
  BOOST_CHECK_EQUAL(geom, geom2);
}

template <class BoundingVolume>
void testLoadGerardBauzil() {
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  std::string env = (path / "staircases_koroibot_hr.dae").string();

  typedef BVHModel<BoundingVolume> Polyhedron_t;
  typedef shared_ptr<Polyhedron_t> PolyhedronPtr_t;
  PolyhedronPtr_t P1(new Polyhedron_t), P2;

  Vec3s scale;
  scale.setConstant(1);
  loadPolyhedronFromResource(env, scale, P1);
  CollisionGeometryPtr_t cylinder(new Cylinder(Scalar(.27), Scalar(.27)));
  Transform3s pos(Vec3s(Scalar(-1.33), Scalar(1.36), Scalar(.14)));
  CollisionObject obj(cylinder, pos);
  CollisionObject stairs(P1);

  CollisionRequest request;
  CollisionResult result;

  collide(&stairs, &obj, request, result);
  BOOST_CHECK(result.isCollision());
}

BOOST_AUTO_TEST_CASE(load_polyhedron) {
  testLoadPolyhedron<AABB>();
  testLoadPolyhedron<OBB>();
  testLoadPolyhedron<RSS>();
  testLoadPolyhedron<kIOS>();
  testLoadPolyhedron<OBBRSS>();
  testLoadPolyhedron<KDOP<16> >();
  testLoadPolyhedron<KDOP<18> >();
  testLoadPolyhedron<KDOP<24> >();
}

BOOST_AUTO_TEST_CASE(gerard_bauzil) {
  testLoadGerardBauzil<OBB>();
  testLoadGerardBauzil<RSS>();
  testLoadGerardBauzil<kIOS>();
  testLoadGerardBauzil<OBBRSS>();
}

BOOST_AUTO_TEST_CASE(load_illformated_mesh) {
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  const std::string filename = (path / "illformated_mesh.dae").string();

  MeshLoader loader;
  BOOST_CHECK_NO_THROW(loader.load(filename));
}

BOOST_AUTO_TEST_CASE(test_convex) {
  Box* box_ptr = new coal::Box(1, 1, 1);
  CollisionGeometryPtr_t b1(box_ptr);
  BVHModel<OBBRSS> box_bvh_model = BVHModel<OBBRSS>();
  generateBVHModel(box_bvh_model, *box_ptr, Transform3s());
  box_bvh_model.buildConvexRepresentation(false);

  box_bvh_model.convex->computeLocalAABB();
  std::shared_ptr<ConvexBase32> convex_copy(box_bvh_model.convex->clone());
  BOOST_CHECK(*convex_copy.get() == *box_bvh_model.convex.get());
}
