/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CNRS-LAAS.
 *  Copyright (c) 2023, INRIA.
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

/** \author Florent Lamiraux <florent@laas.fr> */

#define BOOST_TEST_MODULE COAL_OCTREE
#include <fstream>
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "coal/BVH/BVH_model.h"
#include "coal/collision.h"
#include "coal/distance.h"
#include "coal/hfield.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/shape/geometric_shapes_utility.h"
#include "coal/internal/BV_splitter.h"

#include "utility.h"
#include "fcl_resources/config.h"

namespace utf = boost::unit_test::framework;

using namespace coal;

void makeMesh(const std::vector<Vec3s>& vertices,
              const std::vector<Triangle32>& triangles,
              BVHModel<OBBRSS>& model) {
  coal::SplitMethodType split_method(coal::SPLIT_METHOD_MEAN);
  model.bv_splitter.reset(new BVSplitter<OBBRSS>(split_method));
  model.bv_splitter.reset(new BVSplitter<OBBRSS>(split_method));

  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

coal::OcTree makeOctree(const BVHModel<OBBRSS>& mesh,
                        const Scalar& resolution) {
  Vec3s m(mesh.aabb_local.min_);
  Vec3s M(mesh.aabb_local.max_);
  coal::Box box(resolution, resolution, resolution);
  CollisionRequest request(coal::CONTACT | coal::DISTANCE_LOWER_BOUND, 1);
  CollisionResult result;
  Transform3s tfBox;
  octomap::OcTreePtr_t octree(new octomap::OcTree(resolution));

  for (Scalar x = resolution * floor(m[0] / resolution); x <= M[0];
       x += resolution) {
    for (Scalar y = resolution * floor(m[1] / resolution); y <= M[1];
         y += resolution) {
      for (Scalar z = resolution * floor(m[2] / resolution); z <= M[2];
           z += resolution) {
        const Scalar half = Scalar(0.5);
        Vec3s center(x + half * resolution, y + half * resolution,
                     z + half * resolution);
        tfBox.setTranslation(center);
        coal::collide(&box, tfBox, &mesh, Transform3s(), request, result);
        if (result.isCollision()) {
          octomap::point3d p((float)center[0], (float)center[1],
                             (float)center[2]);
          octree->updateNode(p, true);
          result.clear();
        }
      }
    }
  }

  octree->updateInnerOccupancy();
  octree->writeBinary("./env.octree");
  return OcTree(octree);
}

BOOST_AUTO_TEST_CASE(octree_aabb) {
  // Create a simple octree with 8 nodes arranged in a 2x2x2 cube
  Scalar resolution = 1.0;
  octomap::OcTreePtr_t octree_ptr(new octomap::OcTree(resolution));
  std::vector<octomap::point3d> points = {
      octomap::point3d(0.5, 0.5, 0.5), octomap::point3d(1.5, 0.5, 0.5),
      octomap::point3d(0.5, 1.5, 0.5), octomap::point3d(1.5, 1.5, 0.5),
      octomap::point3d(0.5, 0.5, 1.5), octomap::point3d(1.5, 0.5, 1.5),
      octomap::point3d(0.5, 1.5, 1.5), octomap::point3d(1.5, 1.5, 1.5)};
  for (const auto& point : points) {
    octree_ptr->updateNode(point, true);
  }
  octree_ptr->updateInnerOccupancy();
  // Make sure the octree is not pruned
  octree_ptr->expand();
  // Check initial size
  BOOST_CHECK_EQUAL(octree_ptr->getNumLeafNodes(), 8);

  // Create coal::OcTree and compute AABB
  coal::OcTree octree(octree_ptr);
  octree.computeLocalAABB();

  // The AABB should encompass the 2x2x2 cube from (0,0,0) to (2,2,2)
  BOOST_CHECK(octree.aabb_local.min_.isApprox(Vec3s(0, 0, 0), 1e-6));
  BOOST_CHECK(octree.aabb_local.max_.isApprox(Vec3s(2, 2, 2), 1e-6));

  // Create a copy and prune it
  octomap::OcTreePtr_t octree_pruned_ptr(new octomap::OcTree(*octree_ptr));
  octree_pruned_ptr->prune();
  // Check pruned size, the 8 leaf nodes should have been merged into 1
  BOOST_CHECK_EQUAL(octree_pruned_ptr->getNumLeafNodes(), 1);

  // Create coal::OcTree and compute AABB
  coal::OcTree octree_pruned(octree_pruned_ptr);
  octree_pruned.computeLocalAABB();

  // The AABB should encompass the 2x2x2 cube from (0,0,0) to (2,2,2)
  BOOST_CHECK(octree_pruned.aabb_local.min_.isApprox(Vec3s(0, 0, 0), 1e-6));
  BOOST_CHECK(octree_pruned.aabb_local.max_.isApprox(Vec3s(2, 2, 2), 1e-6));

  // The AABB should remain the same after pruning
  BOOST_CHECK(octree.aabb_local.min_.isApprox(octree_pruned.aabb_local.min_));
  BOOST_CHECK(octree.aabb_local.max_.isApprox(octree_pruned.aabb_local.max_));
}

BOOST_AUTO_TEST_CASE(octree_mesh) {
  Eigen::IOFormat tuple(Eigen::FullPrecision, Eigen::DontAlignCols, "", ", ",
                        "", "", "(", ")");
  Scalar resolution(10.);
  std::vector<Vec3s> pRob, pEnv;
  std::vector<Triangle32> tRob, tEnv;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "rob.obj").string().c_str(), pRob, tRob);
  loadOBJFile((path / "env.obj").string().c_str(), pEnv, tEnv);

  BVHModel<OBBRSS> robMesh, envMesh;
  // Build meshes with robot and environment
  makeMesh(pRob, tRob, robMesh);
  makeMesh(pEnv, tEnv, envMesh);
  // Build octomap with environment
  envMesh.computeLocalAABB();
  // Load octree built from envMesh by makeOctree(envMesh, resolution)
  OcTree envOctree(
      coal::loadOctreeFile((path / "env.octree").string(), resolution));

  std::cout << "Finished loading octree." << std::endl;

  // Test operator==
  {
    BOOST_CHECK(envOctree == envOctree);
    BOOST_CHECK(envOctree == OcTree(envOctree));

    const OcTree envOctree_from_tree(envOctree.getTree());
    BOOST_CHECK(envOctree == envOctree_from_tree);
  }

  // Test tobytes()
  {
    const std::vector<uint8_t> bytes = envOctree.tobytes();
    BOOST_CHECK(bytes.size() > 0 && bytes.size() <= envOctree.toBoxes().size() *
                                                        3 * sizeof(Scalar));
  }

  std::vector<Transform3s> transforms;
  Scalar extents[] = {-2000, -2000, 0, 2000, 2000, 2000};
#ifndef NDEBUG  // if debug mode
  std::size_t N = 100;
#else
  std::size_t N = 10000;
#endif
  N = coal::getNbRun(utf::master_test_suite().argc,
                     utf::master_test_suite().argv, N);

  generateRandomTransforms(extents, transforms, 2 * N);

  CollisionRequest request(coal::CONTACT | coal::DISTANCE_LOWER_BOUND, 1);
  for (std::size_t i = 0; i < N; ++i) {
    CollisionResult resultMesh;
    CollisionResult resultOctree;
    Transform3s tf1(transforms[2 * i]);
    Transform3s tf2(transforms[2 * i + 1]);
    ;
    // Test collision between meshes with random transform for robot.
    coal::collide(&robMesh, tf1, &envMesh, tf2, request, resultMesh);
    // Test collision between mesh and octree for the same transform.
    coal::collide(&robMesh, tf1, &envOctree, tf2, request, resultOctree);
    bool resMesh(resultMesh.isCollision());
    bool resOctree(resultOctree.isCollision());
    BOOST_CHECK(!resMesh || resOctree);
    if (!resMesh && resOctree) {
      coal::DistanceRequest dreq;
      coal::DistanceResult dres;
      coal::distance(&robMesh, tf1, &envMesh, tf2, dreq, dres);
      std::cout << "distance mesh mesh: " << dres.min_distance << std::endl;
      BOOST_CHECK(dres.min_distance < sqrt(2.) * resolution);
    }
  }
}

BOOST_AUTO_TEST_CASE(octree_height_field) {
  Eigen::IOFormat tuple(Eigen::FullPrecision, Eigen::DontAlignCols, "", ", ",
                        "", "", "(", ")");
  Scalar resolution(10.);
  std::vector<Vec3s> pEnv;
  std::vector<Triangle32> tEnv;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), pEnv, tEnv);

  BVHModel<OBBRSS> envMesh;
  // Build meshes with robot and environment
  makeMesh(pEnv, tEnv, envMesh);
  // Build octomap with environment
  envMesh.computeLocalAABB();
  // Load octree built from envMesh by makeOctree(envMesh, resolution)
  OcTree envOctree(
      coal::loadOctreeFile((path / "env.octree").string(), resolution));

  std::cout << "Finished loading octree." << std::endl;

  // Building hfield
  const Scalar x_dim = 10, y_dim = 20;
  const int nx = 100, ny = 100;
  const Scalar max_altitude = 1., min_altitude = 0.;
  const MatrixXs heights = MatrixXs::Constant(ny, nx, max_altitude);

  HeightField<AABB> hfield(x_dim, y_dim, heights, min_altitude);
  hfield.computeLocalAABB();

  std::vector<Transform3s> transforms;
  Scalar extents[] = {-2000, -2000, 0, 2000, 2000, 2000};
#ifndef NDEBUG  // if debug mode
  std::size_t N = 1000;
#else
  std::size_t N = 100000;
#endif
  N = coal::getNbRun(utf::master_test_suite().argc,
                     utf::master_test_suite().argv, N);

  generateRandomTransforms(extents, transforms, 2 * N);

  CollisionRequest request(coal::CONTACT | coal::DISTANCE_LOWER_BOUND, 1);
  for (std::size_t i = 0; i < N; ++i) {
    CollisionResult resultBox;
    CollisionResult resultHfield1, resultHfield2;
    Transform3s tf1(transforms[2 * i]);
    Transform3s tf2(transforms[2 * i + 1]);

    Box box;
    Transform3s box_tf;
    constructBox(hfield.aabb_local, tf2, box, box_tf);

    // Test collision between octree and equivalent box.
    coal::collide(&envOctree, tf1, &box, box_tf, request, resultBox);
    // Test collision between octree and hfield.
    coal::collide(&envOctree, tf1, &hfield, tf2, request, resultHfield1);
    coal::collide(&hfield, tf2, &envOctree, tf1, request, resultHfield2);

    bool resBox(resultBox.isCollision());
    bool resHfield(resultHfield1.isCollision());
    BOOST_CHECK(resBox == resHfield);
    BOOST_CHECK(resultHfield1.isCollision() == resultHfield2.isCollision());
    if (!resBox && resHfield) {
      coal::DistanceRequest dreq;
      coal::DistanceResult dres;
      coal::distance(&envMesh, tf1, &box, box_tf, dreq, dres);
      std::cout << "distance mesh box: " << dres.min_distance << std::endl;
      BOOST_CHECK(dres.min_distance < sqrt(2.) * resolution);
    }
  }
}
