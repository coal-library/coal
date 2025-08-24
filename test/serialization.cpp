/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-2023 INRIA.
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

#define BOOST_TEST_MODULE COAL_SERIALIZATION
#include <fstream>
#include <boost/test/included/unit_test.hpp>

#include "coal/fwd.hh"

COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS

#include "coal/collision.h"

#include "coal/contact_patch.h"
#include "coal/distance.h"
#include "coal/BV/OBBRSS.h"
#include "coal/BVH/BVH_model.h"

#include "coal/serialization/collision_data.h"
#include "coal/serialization/contact_patch.h"
#include "coal/serialization/AABB.h"
#include "coal/serialization/BVH_model.h"
#include "coal/serialization/hfield.h"
#include "coal/serialization/transform.h"
#include "coal/serialization/geometric_shapes.h"
#include "coal/serialization/convex.h"
#include "coal/serialization/archive.h"
#include "coal/serialization/memory.h"

#ifdef COAL_HAS_OCTOMAP
#include "coal/serialization/octree.h"
#endif

#include "utility.h"
#include "fcl_resources/config.h"

#include <boost/archive/tmpdir.hpp>
#include <boost/filesystem.hpp>

namespace utf = boost::unit_test::framework;

using namespace coal;

template <typename T>
bool check(const T& value, const T& other) {
  return value == other;
}

template <typename T>
bool check_ptr(const T* value, const T* other) {
  return *value == *other;
}

enum SerializationMode { TXT = 1, XML = 2, BIN = 4, STREAM = 8 };

template <typename T>
void test_serialization(const T* value, T& other_value,
                        const int mode = TXT | XML | BIN | STREAM) {
  test_serialization(*value, other_value, mode);
}

template <typename T,
          bool is_base = std::is_base_of<T, CollisionGeometry>::value>
struct test_pointer_serialization_impl {
  static void run(const T&, T&, const int) {}
};

template <typename T>
struct test_pointer_serialization_impl<T, true> {
  static void run(const T& value, T& other_value, const int mode) {
    const CollisionGeometry* ptr = &value;
    CollisionGeometry* other_ptr = &other_value;

    const boost::filesystem::path tmp_path(boost::archive::tmpdir());
    const boost::filesystem::path txt_path("file.txt");
    const boost::filesystem::path txt_ptr_path("ptr_file.txt");
    const boost::filesystem::path xml_path("file.xml");
    const boost::filesystem::path bin_path("file.bin");
    const boost::filesystem::path txt_filename(tmp_path / txt_path);
    const boost::filesystem::path xml_filename(tmp_path / xml_path);
    const boost::filesystem::path bin_filename(tmp_path / bin_path);

    // TXT
    if (mode & 0x1) {
      {
        std::ofstream ofs(txt_filename.c_str());

        boost::archive::text_oarchive oa(ofs);
        oa << ptr;
      }
      BOOST_CHECK(check(*reinterpret_cast<const CollisionGeometry*>(ptr),
                        *reinterpret_cast<const CollisionGeometry*>(ptr)));

      {
        std::ifstream ifs(txt_filename.c_str());
        boost::archive::text_iarchive ia(ifs);

        ia >> other_ptr;
      }
      BOOST_CHECK(
          check(*reinterpret_cast<const CollisionGeometry*>(ptr),
                *reinterpret_cast<const CollisionGeometry*>(other_ptr)));
    }
  }
};

template <typename T>
void test_pointer_serialization(const T& value, T& other_value,
                                const int mode = TXT | XML | BIN | STREAM) {
  test_pointer_serialization_impl<T>::run(value, other_value, mode);
}

template <typename T>
void test_serialization(const T& value, T& other_value,
                        const int mode = TXT | XML | BIN | STREAM) {
  const boost::filesystem::path tmp_path(boost::archive::tmpdir());
  const boost::filesystem::path txt_path("file.txt");
  const boost::filesystem::path txt_ptr_path("ptr_file.txt");
  const boost::filesystem::path xml_path("file.xml");
  const boost::filesystem::path bin_path("file.bin");
  const boost::filesystem::path txt_filename(tmp_path / txt_path);
  const boost::filesystem::path xml_filename(tmp_path / xml_path);
  const boost::filesystem::path bin_filename(tmp_path / bin_path);

  // TXT
  if (mode & 0x1) {
    // -- TXT
    {
      const std::string filename = txt_filename.string();
      coal::serialization::saveToText(value, filename);
      BOOST_CHECK(check(value, value));

      coal::serialization::loadFromText(other_value, filename);
      BOOST_CHECK(check(value, other_value));
    }

    // -- String stream (TXT format)
    {
      std::stringstream ss_out;
      coal::serialization::saveToStringStream(value, ss_out);
      BOOST_CHECK(check(value, value));

      std::istringstream ss_in(ss_out.str());
      coal::serialization::loadFromStringStream(other_value, ss_in);
      BOOST_CHECK(check(value, other_value));
    }

    // -- String
    {
      const std::string str_out = coal::serialization::saveToString(value);
      BOOST_CHECK(check(value, value));

      const std::string str_in(str_out);
      coal::serialization::loadFromString(other_value, str_in);
      BOOST_CHECK(check(value, other_value));
    }
  }

  // XML
  if (mode & 0x2) {
    {
      const std::string filename = xml_filename.string();
      const std::string xml_tag = "value";
      coal::serialization::saveToXML(value, filename, xml_tag);
      BOOST_CHECK(check(value, value));

      coal::serialization::loadFromXML(other_value, filename, xml_tag);
      BOOST_CHECK(check(value, other_value));
    }
  }

  // BIN
  if (mode & 0x4) {
    {
      const std::string filename = bin_filename.string();
      coal::serialization::saveToBinary(value, filename);
      BOOST_CHECK(check(value, value));

      coal::serialization::loadFromBinary(other_value, filename);
      BOOST_CHECK(check(value, other_value));
    }
  }

  // Stream Buffer
  if (mode & 0x8) {
    {
      boost::asio::streambuf buffer;
      coal::serialization::saveToBuffer(value, buffer);
      BOOST_CHECK(check(value, value));

      coal::serialization::loadFromBuffer(other_value, buffer);
      BOOST_CHECK(check(value, other_value));
    }
  }

  // Test std::shared_ptr<T>
  {
    const boost::filesystem::path txt_ptr_filename(tmp_path / txt_ptr_path);
    std::shared_ptr<T> ptr = std::make_shared<T>(value);

    const std::string filename = txt_ptr_filename.string();
    coal::serialization::saveToText(ptr, filename);
    BOOST_CHECK(check_ptr(ptr.get(), ptr.get()));

    std::shared_ptr<T> other_ptr = nullptr;
    coal::serialization::loadFromText(other_ptr, filename);
    BOOST_CHECK(check_ptr(ptr.get(), other_ptr.get()));
  }

  test_pointer_serialization(value, other_value);
}

template <typename T>
void test_serialization(const T& value,
                        const int mode = TXT | XML | BIN | STREAM) {
  T other_value;
  test_serialization(value, other_value, mode);
}

BOOST_AUTO_TEST_CASE(test_aabb) {
  AABB aabb(-Vec3s::Ones(), Vec3s::Ones());
  test_serialization(aabb);
}

BOOST_AUTO_TEST_CASE(test_collision_data) {
  Contact contact(NULL, NULL, 1, 2, Vec3s::Ones(), Vec3s::Zero(), -10.);
  test_serialization(contact);

  CollisionRequest collision_request(CONTACT, 10);
  test_serialization(collision_request);

  CollisionResult collision_result;
  collision_result.addContact(contact);
  collision_result.addContact(contact);
  collision_result.distance_lower_bound = Scalar(0.1);
  collision_result.normal.setOnes();
  collision_result.nearest_points[0].setRandom();
  collision_result.nearest_points[1].setRandom();
  test_serialization(collision_result);

  DistanceRequest distance_request(true, 1., 2.);
  test_serialization(distance_request);

  DistanceResult distance_result;
  distance_result.normal.setOnes();
  distance_result.nearest_points[0].setRandom();
  distance_result.nearest_points[1].setRandom();
  test_serialization(distance_result);

  {
    // Serializing contact patches.
    const Halfspace hspace(0, 0, 1, 0);
    const Scalar radius = 0.25;
    const Scalar height = 1.;
    const Cylinder cylinder(radius, height);

    const Transform3s tf1;
    Transform3s tf2;
    // set translation to have a collision
    const Scalar offset = Scalar(0.001);
    tf2.setTranslation(Vec3s(0, 0, height / 2 - offset));

    const size_t num_max_contact = 1;
    const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                   num_max_contact);
    CollisionResult col_res;
    coal::collide(&hspace, tf1, &cylinder, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    if (col_res.isCollision()) {
      ContactPatchRequest patch_req;
      ContactPatchResult patch_res(patch_req);
      coal::computeContactPatch(&hspace, tf1, &cylinder, tf2, col_res,
                                patch_req, patch_res);
      BOOST_CHECK(patch_res.numContactPatches() == 1);

      // Serialize patch request, result and the patch itself
      test_serialization(patch_req);
      test_serialization(patch_res);
      if (patch_res.numContactPatches() > 0) {
        test_serialization(patch_res.getContactPatch(0));
      }
    }
  }
}

template <typename T>
void checkEqualStdVector(const std::vector<T>& v1, const std::vector<T>& v2) {
  BOOST_CHECK(v1.size() == v2.size());
  if (v1.size() == v2.size()) {
    for (size_t i = 0; i < v1.size(); i++) {
      BOOST_CHECK(v1[i] == v2[i]);
    }
  }
}

BOOST_AUTO_TEST_CASE(test_BVHModel) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle32> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  BVHModel<OBBRSS> m1, m2;

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();
  BOOST_CHECK(m1.num_vertices == p1.size());
  BOOST_CHECK(m1.num_tris == t1.size());
  checkEqualStdVector(*m1.vertices, p1);
  checkEqualStdVector(*m1.tri_indices, t1);
  BOOST_CHECK(m1 == m1);

  m2.beginModel();
  m2.addSubModel(p2, t2);
  m2.endModel();
  BOOST_CHECK(m2.num_vertices == p2.size());
  BOOST_CHECK(m2.num_tris == t2.size());
  checkEqualStdVector(*m2.vertices, p2);
  checkEqualStdVector(*m2.tri_indices, t2);
  BOOST_CHECK(m2 == m2);
  BOOST_CHECK(m1 != m2);

  // Test BVHModel
  {
    BVHModel<OBBRSS> m1_copy;
    test_serialization(m1, m1_copy);
  }
  {
    BVHModel<OBBRSS> m1_copy;
    test_serialization(m1, m1_copy, STREAM);
  }
}

#ifdef COAL_HAS_QHULL
BOOST_AUTO_TEST_CASE(test_Convex) {
  std::vector<Vec3s> p1;
  std::vector<Triangle32> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  BVHModel<OBBRSS> m1;

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();

  m1.buildConvexHull(true);

  ConvexTpl<Triangle32>& convex =
      static_cast<ConvexTpl<Triangle32>&>(*m1.convex.get());

  // Test Convex
  {
    ConvexTpl<Triangle32> convex_copy;
    test_serialization(convex, convex_copy);
  }

  // Test std::shared_ptr<CollisionGeometry>
  {
    const boost::filesystem::path tmp_dir(boost::archive::tmpdir());
    // TODO(louis): understand why serializing a shared_ptr<CollisionGeometry>
    // in TXT format fails only on MacOS + -O0.
    // const boost::filesystem::path txt_filename = tmp_dir / "file.txt";
    // const boost::filesystem::path bin_filename = tmp_dir / "file.bin";
    const boost::filesystem::path xml_filename = tmp_dir / "file.xml";
    ConvexTpl<Triangle32> convex_copy;

    std::shared_ptr<CollisionGeometry> ptr =
        std::make_shared<ConvexTpl<Triangle32>>(convex);
    BOOST_CHECK(ptr.get());
    const std::string filename = xml_filename.string();
    const std::string tag_name = "CollisionGeometry";
    coal::serialization::saveToXML(ptr, filename, tag_name);
    BOOST_CHECK(
        check(*reinterpret_cast<ConvexTpl<Triangle32>*>(ptr.get()), convex));

    std::shared_ptr<CollisionGeometry> other_ptr = nullptr;
    BOOST_CHECK(!other_ptr.get());
    coal::serialization::loadFromXML(other_ptr, filename, tag_name);
    BOOST_CHECK(check(
        convex, *reinterpret_cast<ConvexTpl<Triangle32>*>(other_ptr.get())));
  }
}
#endif

BOOST_AUTO_TEST_CASE(test_HeightField) {
  const Scalar min_altitude = -1.;
  const Scalar x_dim = 1., y_dim = 2.;
  const Eigen::DenseIndex nx = 100, ny = 200;
  const MatrixXs heights = MatrixXs::Random(ny, nx);

  HeightField<OBBRSS> hfield(x_dim, y_dim, heights, min_altitude);

  // Test HeightField
  {
    HeightField<OBBRSS> hfield_copy;
    test_serialization(hfield, hfield_copy);
  }
  {
    HeightField<OBBRSS> hfield_copy;
    test_serialization(hfield, hfield_copy, STREAM);
  }
}

BOOST_AUTO_TEST_CASE(test_transform) {
  Transform3s T;
  T.setQuatRotation(Quaternion3f::UnitRandom());
  T.setTranslation(Vec3s::Random());

  Transform3s T_copy;
  test_serialization(T, T_copy);
}

BOOST_AUTO_TEST_CASE(test_shapes) {
  {
    TriangleP triangle(Vec3s::UnitX(), Vec3s::UnitY(), Vec3s::UnitZ());
    triangle.setSweptSphereRadius(1.);
    triangle.computeLocalAABB();
    TriangleP triangle_copy(Vec3s::Random(), Vec3s::Random(), Vec3s::Random());
    test_serialization(triangle, triangle_copy);
  }

  {
    Box box(Vec3s::UnitX()), box_copy(Vec3s::Random());
    box.setSweptSphereRadius(1.);
    box.computeLocalAABB();
    test_serialization(box, box_copy);
  }

  {
    Sphere sphere(1.), sphere_copy(2.);
    sphere.setSweptSphereRadius(1.);
    sphere.computeLocalAABB();
    test_serialization(sphere, sphere_copy);
  }

  {
    Ellipsoid ellipsoid(1., 2., 3.), ellipsoid_copy(0., 0., 0.);
    ellipsoid.setSweptSphereRadius(1.);
    ellipsoid.computeLocalAABB();
    test_serialization(ellipsoid, ellipsoid_copy);
  }

  {
    Capsule capsule(1., 2.), capsule_copy(10., 10.);
    capsule.setSweptSphereRadius(1.);
    capsule.computeLocalAABB();
    test_serialization(capsule, capsule_copy);
  }

  {
    Cone cone(1., 2.), cone_copy(10., 10.);
    cone.setSweptSphereRadius(1.);
    cone.computeLocalAABB();
    test_serialization(cone, cone_copy);
  }

  {
    Cylinder cylinder(1., 2.), cylinder_copy(10., 10.);
    cylinder.setSweptSphereRadius(1.);
    cylinder.computeLocalAABB();
    test_serialization(cylinder, cylinder_copy);
  }

  {
    Halfspace hs(Vec3s::Random(), 1.), hs_copy(Vec3s::Zero(), 0.);
    hs.setSweptSphereRadius(1.);
    hs.computeLocalAABB();
    test_serialization(hs, hs_copy);
  }

  {
    Plane plane(Vec3s::Random(), 1.), plane_copy(Vec3s::Zero(), 0.);
    plane.setSweptSphereRadius(1.);
    plane.computeLocalAABB();
    test_serialization(plane, plane_copy);
  }

#ifdef COAL_HAS_QHULL
  {
    const size_t num_points = 500;
    std::shared_ptr<std::vector<Vec3s>> points =
        std::make_shared<std::vector<Vec3s>>();
    points->reserve(num_points);
    for (size_t i = 0; i < num_points; i++) {
      points->emplace_back(Vec3s::Random());
    }
    using Convex = ConvexTpl<Triangle32>;
    std::unique_ptr<Convex> convex =
        std::unique_ptr<Convex>(static_cast<Convex*>(ConvexBase32::convexHull(
            points, static_cast<unsigned int>(points->size()), true)));
    convex->setSweptSphereRadius(1.);
    convex->computeLocalAABB();

    Convex convex_copy;
    test_serialization(*convex, convex_copy);
  }
#endif
}

#ifdef COAL_HAS_OCTOMAP
BOOST_AUTO_TEST_CASE(test_octree) {
  const Scalar resolution = Scalar(1e-2);
  const MatrixX3s points = MatrixX3s::Random(1000, 3);
  OcTreePtr_t octree_ptr = makeOctree(points, resolution);
  const OcTree& octree = *octree_ptr.get();

  const boost::filesystem::path tmp_dir(boost::archive::tmpdir());
  const boost::filesystem::path txt_filename = tmp_dir / "file.txt";
  const boost::filesystem::path bin_filename = tmp_dir / "file.bin";

  {
    std::ofstream ofs(bin_filename.c_str(), std::ios::binary);
    boost::archive::binary_oarchive oa(ofs);
    oa << octree;
  }

  OcTree octree_value(1.);
  {
    std::ifstream ifs(bin_filename.c_str(),
                      std::fstream::binary | std::fstream::in);
    boost::archive::binary_iarchive ia(ifs);

    ia >> octree_value;
  }

  BOOST_CHECK(octree.getTree() == octree.getTree());
  BOOST_CHECK(octree_value.getTree() == octree_value.getTree());
  //  BOOST_CHECK(octree.getTree() == octree_value.getTree());
  BOOST_CHECK(octree.getResolution() == octree_value.getResolution());
  BOOST_CHECK(octree.getTree()->size() == octree_value.getTree()->size());
  BOOST_CHECK(octree.toBoxes().size() == octree_value.toBoxes().size());
  BOOST_CHECK(octree == octree_value);

  OcTree octree_copy(1.);
  test_serialization(octree, octree_copy);
}
#endif

BOOST_AUTO_TEST_CASE(test_memory_footprint) {
  Sphere sphere(1.);
  BOOST_CHECK(sizeof(Sphere) == computeMemoryFootprint(sphere));

  std::vector<Vec3s> p1;
  std::vector<Triangle32> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  BVHModel<OBBRSS> m1;

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();

  std::cout << "computeMemoryFootprint(m1): " << computeMemoryFootprint(m1)
            << std::endl;
  BOOST_CHECK(sizeof(BVHModel<OBBRSS>) < computeMemoryFootprint(m1));
  BOOST_CHECK(static_cast<size_t>(m1.memUsage(false)) ==
              computeMemoryFootprint(m1));
}

COAL_COMPILER_DIAGNOSTIC_POP
