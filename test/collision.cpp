/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

#include <boost/mpl/vector.hpp>

#define BOOST_TEST_MODULE COAL_COLLISION
#include <boost/test/included/unit_test.hpp>

#include <boost/assign/list_of.hpp>
#include <fstream>

#include "coal/fwd.hh"

COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS

#include "coal/collision.h"

COAL_COMPILER_DIAGNOSTIC_POP

#include "coal/BV/BV.h"
#include "coal/mesh_loader/assimp.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"

#include "../src/collision_node.h"
#include "coal/internal/BV_splitter.h"
#include "coal/internal/traversal_node_bvhs.h"
#include "coal/internal/traversal_node_setup.h"

#include "coal/timings.h"

#include "fcl_resources/config.h"
#include "utility.h"

using namespace coal;
namespace utf = boost::unit_test::framework;

int num_max_contacts = (std::numeric_limits<int>::max)();

BOOST_AUTO_TEST_CASE(OBB_Box_test) {
  Scalar r_extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::vector<Transform3s> rotate_transform;
  generateRandomTransforms(r_extents, rotate_transform, 1);

  AABB aabb1;
  aabb1.min_ = Vec3s(-600, -600, -600);
  aabb1.max_ = Vec3s(600, 600, 600);

  OBB obb1;
  convertBV(aabb1, rotate_transform[0], obb1);
  Box box1;
  Transform3s box1_tf;
  constructBox(aabb1, rotate_transform[0], box1, box1_tf);

  Scalar extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  std::vector<Transform3s> transforms;
  generateRandomTransforms(extents, transforms, n);

  for (std::size_t i = 0; i < transforms.size(); ++i) {
    AABB aabb;
    aabb.min_ = aabb1.min_ * 0.5;
    aabb.max_ = aabb1.max_ * 0.5;

    OBB obb2;
    convertBV(aabb, transforms[i], obb2);

    Box box2;
    Transform3s box2_tf;
    constructBox(aabb, transforms[i], box2, box2_tf);

    GJKSolver solver;

    bool overlap_obb = obb1.overlap(obb2);
    CollisionRequest request;
    CollisionResult result;
    // Convention: when doing primitive-primitive collision, either use
    // `collide` or `ShapeShapeCollide`.
    // **DO NOT** use methods of the GJKSolver directly if you don't know what
    // you are doing.
    ShapeShapeCollide<Box, Box>(&box1, box1_tf, &box2, box2_tf, &solver,
                                request, result);
    bool overlap_box = result.isCollision();

    BOOST_CHECK(overlap_obb == overlap_box);
  }
}

BOOST_AUTO_TEST_CASE(OBB_shape_test) {
  Scalar r_extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::vector<Transform3s> rotate_transform;
  generateRandomTransforms(r_extents, rotate_transform, 1);

  AABB aabb1;
  aabb1.min_ = Vec3s(-600, -600, -600);
  aabb1.max_ = Vec3s(600, 600, 600);

  OBB obb1;
  convertBV(aabb1, rotate_transform[0], obb1);
  Box box1;
  Transform3s box1_tf;
  constructBox(aabb1, rotate_transform[0], box1, box1_tf);

  Scalar extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  std::vector<Transform3s> transforms;
  generateRandomTransforms(extents, transforms, n);

  for (std::size_t i = 0; i < transforms.size(); ++i) {
    Scalar len = (aabb1.max_[0] - aabb1.min_[0]) * Scalar(0.5);
    OBB obb2;
    GJKSolver solver;

    {
      Sphere sphere(len);
      computeBV(sphere, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      CollisionRequest request;
      CollisionResult result;
      ShapeShapeCollide<Box, Sphere>(&box1, box1_tf, &sphere, transforms[i],
                                     &solver, request, result);
      bool overlap_sphere = result.isCollision();
      // The sphere is contained inside obb2. So if the sphere overlaps with
      // obb1, then necessarily obb2 overlaps with obb1.
      BOOST_CHECK(overlap_obb >= overlap_sphere);
    }

    {
      Capsule capsule(len, 2 * len);
      computeBV(capsule, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);

      CollisionRequest request;
      CollisionResult result;
      ShapeShapeCollide<Box, Capsule>(&box1, box1_tf, &capsule, transforms[i],
                                      &solver, request, result);
      bool overlap_capsule = result.isCollision();
      BOOST_CHECK(overlap_obb >= overlap_capsule);
    }

    {
      Cone cone(len, 2 * len);
      computeBV(cone, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      CollisionRequest request;
      CollisionResult result;
      ShapeShapeCollide<Box, Cone>(&box1, box1_tf, &cone, transforms[i],
                                   &solver, request, result);
      bool overlap_cone = result.isCollision();
      BOOST_CHECK(overlap_obb >= overlap_cone);
    }

    {
      Cylinder cylinder(len, 2 * len);
      computeBV(cylinder, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      CollisionRequest request;
      CollisionResult result;
      ShapeShapeCollide<Box, Cylinder>(&box1, box1_tf, &cylinder, transforms[i],
                                       &solver, request, result);
      bool overlap_cylinder = result.isCollision();
      BOOST_CHECK(overlap_obb >= overlap_cylinder);
    }
  }
}

BOOST_AUTO_TEST_CASE(OBB_AABB_test) {
  Scalar extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  std::vector<Transform3s> transforms;
  generateRandomTransforms(extents, transforms, n);

  AABB aabb1;
  aabb1.min_ = Vec3s(-600, -600, -600);
  aabb1.max_ = Vec3s(600, 600, 600);

  OBB obb1;
  convertBV(aabb1, Transform3s(), obb1);

  for (std::size_t i = 0; i < transforms.size(); ++i) {
    AABB aabb;
    aabb.min_ = aabb1.min_ * 0.5;
    aabb.max_ = aabb1.max_ * 0.5;

    AABB aabb2 = translate(aabb, transforms[i].getTranslation());

    OBB obb2;
    convertBV(aabb, Transform3s(transforms[i].getTranslation()), obb2);

    bool overlap_aabb = aabb1.overlap(aabb2);
    bool overlap_obb = obb1.overlap(obb2);
    if (overlap_aabb != overlap_obb) {
      std::cout << aabb1.min_ << " " << aabb1.max_ << std::endl;
      std::cout << aabb2.min_ << " " << aabb2.max_ << std::endl;
      std::cout << obb1.To << " " << obb1.extent << " " << obb1.axes
                << std::endl;
      std::cout << obb2.To << " " << obb2.extent << " " << obb2.axes
                << std::endl;
    }

    BOOST_CHECK(overlap_aabb == overlap_obb);
  }
  std::cout << std::endl;
}

std::ostream *bench_stream = NULL;
bool bs_nl = true;
bool bs_hp = false;
#define BENCHMARK(stream)                           \
  if (bench_stream != NULL) {                       \
    *bench_stream << (bs_nl ? "" : ", ") << stream; \
    bs_nl = false;                                  \
  }
#define BENCHMARK_HEADER(stream) \
  if (!bs_hp) {                  \
    BENCHMARK(stream)            \
  }
#define BENCHMARK_NEXT()                \
  if (bench_stream != NULL && !bs_nl) { \
    *bench_stream << '\n';              \
    bs_nl = true;                       \
    bs_hp = true;                       \
  }

typedef std::vector<Contact> Contacts_t;
typedef boost::mpl::vector<OBB, RSS, KDOP<24>, KDOP<18>, KDOP<16>, kIOS, OBBRSS>
    BVs_t;
std::vector<SplitMethodType> splitMethods = boost::assign::list_of(
    SPLIT_METHOD_MEAN)(SPLIT_METHOD_MEDIAN)(SPLIT_METHOD_BV_CENTER);

#define BV_STR_SPECIALIZATION(bv) \
  template <>                     \
  const char *str<bv>() {         \
    return #bv;                   \
  }
template <typename BV>
const char *str();
BV_STR_SPECIALIZATION(AABB)
BV_STR_SPECIALIZATION(OBB)
BV_STR_SPECIALIZATION(RSS)
BV_STR_SPECIALIZATION(KDOP<24>)
BV_STR_SPECIALIZATION(KDOP<18>)
BV_STR_SPECIALIZATION(KDOP<16>)
BV_STR_SPECIALIZATION(kIOS)
BV_STR_SPECIALIZATION(OBBRSS)

template <typename T>
struct wrap {};

struct base_traits {
  enum { IS_IMPLEMENTED = true };
};

enum {
  Oriented = true,
  NonOriented = false,
  Recursive = true,
  NonRecursive = false
};

template <typename BV, bool Oriented, bool recursive>
struct traits : base_traits {};

template <short N, bool recursive>
struct traits<KDOP<N>, Oriented, recursive> : base_traits {
  enum { IS_IMPLEMENTED = false };
};

COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS

struct mesh_mesh_run_test {
  mesh_mesh_run_test(const std::vector<Transform3s> &_transforms,
                     const CollisionRequest _request)
      : transforms(_transforms),
        request(_request),
        enable_statistics(false),
        benchmark(false),
        isInit(false),
        indent(0) {}

  const std::vector<Transform3s> &transforms;
  const CollisionRequest request;
  bool enable_statistics, benchmark;
  std::vector<Contacts_t> contacts;
  std::vector<Contacts_t> contacts_ref;
  bool isInit;

  int indent;

  COAL_COMPILER_DIAGNOSTIC_POP

  const char *getindent() {
    assert(indent < 9);
    static const char *t[] = {"",
                              "\t",
                              "\t\t",
                              "\t\t\t",
                              "\t\t\t\t",
                              "\t\t\t\t\t",
                              "\t\t\t\t\t\t",
                              "\t\t\t\t\t\t\t",
                              "\t\t\t\t\t\t\t\t"};
    return t[indent];
  }

  template <typename BV>
  void query(const std::vector<Transform3s> &transforms,
             SplitMethodType splitMethod, const CollisionRequest request,
             std::vector<Contacts_t> &contacts) {
    BENCHMARK_HEADER("BV");
    BENCHMARK_HEADER("oriented");
    BENCHMARK_HEADER("Split method");
    if (enable_statistics) {
      BENCHMARK_HEADER("num_bv_tests");
      BENCHMARK_HEADER("num_leaf_tests");
    }
    BENCHMARK_HEADER("numContacts");
    BENCHMARK_HEADER("distance_lower_bound");
    BENCHMARK_HEADER("time");
    BENCHMARK_NEXT();

    typedef BVHModel<BV> BVH_t;
    typedef shared_ptr<BVH_t> BVHPtr_t;

    BVHPtr_t model1(new BVH_t), model2(new BVH_t);
    model1->bv_splitter.reset(new BVSplitter<BV>(splitMethod));
    model2->bv_splitter.reset(new BVSplitter<BV>(splitMethod));

    loadPolyhedronFromResource(TEST_RESOURCES_DIR "/env.obj", Vec3s::Ones(),
                               model1);
    loadPolyhedronFromResource(TEST_RESOURCES_DIR "/rob.obj", Vec3s::Ones(),
                               model2);

    Timer timer(false);
    const Transform3s tf2;
    const std::size_t N = transforms.size();

    contacts.resize(3 * N);

    if (traits<BV, Oriented, Recursive>::IS_IMPLEMENTED) {
      BOOST_TEST_MESSAGE(getindent() << "BV: " << str<BV>() << " oriented");
      ++indent;

      for (std::size_t i = 0; i < transforms.size(); ++i) {
        const Transform3s &tf1 = transforms[i];
        timer.start();

        CollisionResult local_result;
        MeshCollisionTraversalNode<BV, 0> node(request);
        node.enable_statistics = enable_statistics;

        bool success =
            initialize(node, *model1, tf1, *model2, tf2, local_result);
        BOOST_REQUIRE(success);

        collide(&node, request, local_result);

        timer.stop();

        BENCHMARK(str<BV>());
        BENCHMARK(1);
        BENCHMARK(splitMethod);
        if (enable_statistics) {
          BOOST_TEST_MESSAGE(getindent() << "statistics: " << node.num_bv_tests
                                         << " " << node.num_leaf_tests);
          BOOST_TEST_MESSAGE(getindent()
                             << "nb contacts: " << local_result.numContacts());
          BENCHMARK(node.num_bv_tests);
          BENCHMARK(node.num_leaf_tests);
        }
        BENCHMARK(local_result.numContacts());
        BENCHMARK(local_result.distance_lower_bound);
        BENCHMARK(timer.duration().count());
        BENCHMARK_NEXT();

        if (local_result.numContacts() > 0) {
          local_result.getContacts(contacts[i]);
          std::sort(contacts[i].begin(), contacts[i].end());
        }
      }
      --indent;
    }

    if (traits<BV, NonOriented, Recursive>::IS_IMPLEMENTED) {
      BOOST_TEST_MESSAGE(getindent() << "BV: " << str<BV>());
      ++indent;

      for (std::size_t i = 0; i < transforms.size(); ++i) {
        const Transform3s tf1 = transforms[i];

        timer.start();
        CollisionResult local_result;
        MeshCollisionTraversalNode<BV, RelativeTransformationIsIdentity> node(
            request);
        node.enable_statistics = enable_statistics;

        BVH_t *model1_tmp = new BVH_t(*model1);
        Transform3s tf1_tmp = tf1;
        BVH_t *model2_tmp = new BVH_t(*model2);
        Transform3s tf2_tmp = tf2;

        bool success = initialize(node, *model1_tmp, tf1_tmp, *model2_tmp,
                                  tf2_tmp, local_result, true, true);
        BOOST_REQUIRE(success);

        collide(&node, request, local_result);
        delete model1_tmp;
        delete model2_tmp;

        timer.stop();
        BENCHMARK(str<BV>());
        BENCHMARK(2);
        BENCHMARK(splitMethod);
        if (enable_statistics) {
          BOOST_TEST_MESSAGE(getindent() << "statistics: " << node.num_bv_tests
                                         << " " << node.num_leaf_tests);
          BOOST_TEST_MESSAGE(getindent()
                             << "nb contacts: " << local_result.numContacts());
          BENCHMARK(node.num_bv_tests);
          BENCHMARK(node.num_leaf_tests);
        }
        BENCHMARK(local_result.numContacts());
        BENCHMARK(local_result.distance_lower_bound);
        BENCHMARK(timer.duration().count());
        BENCHMARK_NEXT();

        if (local_result.numContacts() > 0) {
          local_result.getContacts(contacts[i + N]);
          std::sort(contacts[i + N].begin(), contacts[i + N].end());
        }
      }
      --indent;
    }

    if (traits<BV, Oriented, NonRecursive>::IS_IMPLEMENTED) {
      BOOST_TEST_MESSAGE(getindent()
                         << "BV: " << str<BV>() << " oriented non-recursive");
      ++indent;

      for (std::size_t i = 0; i < transforms.size(); ++i) {
        timer.start();
        const Transform3s tf1 = transforms[i];

        CollisionResult local_result;
        MeshCollisionTraversalNode<BV, 0> node(request);
        node.enable_statistics = enable_statistics;

        bool success =
            initialize(node, *model1, tf1, *model2, tf2, local_result);
        BOOST_REQUIRE(success);

        collide(&node, request, local_result, NULL, false);

        timer.stop();
        BENCHMARK(str<BV>());
        BENCHMARK(0);
        BENCHMARK(splitMethod);
        if (enable_statistics) {
          BOOST_TEST_MESSAGE(getindent() << "statistics: " << node.num_bv_tests
                                         << " " << node.num_leaf_tests);
          BOOST_TEST_MESSAGE(getindent()
                             << "nb contacts: " << local_result.numContacts());
          BENCHMARK(node.num_bv_tests);
          BENCHMARK(node.num_leaf_tests);
        }
        BENCHMARK(local_result.numContacts());
        BENCHMARK(local_result.distance_lower_bound);
        BENCHMARK(timer.duration().count());
        BENCHMARK_NEXT();

        if (local_result.numContacts() > 0) {
          local_result.getContacts(contacts[i + 2 * N]);
          std::sort(contacts[i + 2 * N].begin(), contacts[i + 2 * N].end());
        }
      }
      --indent;
    }
  }

  void check_contacts(std::size_t i0, std::size_t i1, bool warn) {
    for (std::size_t i = i0; i < i1; ++i) {
      Contacts_t in_ref_but_not_in_i;
      std::set_difference(
          contacts_ref[i].begin(), contacts_ref[i].end(), contacts[i].begin(),
          contacts[i].end(),
          std::inserter(in_ref_but_not_in_i, in_ref_but_not_in_i.begin()));
      if (!in_ref_but_not_in_i.empty()) {
        for (std::size_t j = 0; j < in_ref_but_not_in_i.size(); ++j) {
          if (warn) {
            BOOST_WARN_MESSAGE(
                false, "Missed contacts: " << in_ref_but_not_in_i[j].b1 << ", "
                                           << in_ref_but_not_in_i[j].b2);
          } else {
            BOOST_CHECK_MESSAGE(
                false, "Missed contacts: " << in_ref_but_not_in_i[j].b1 << ", "
                                           << in_ref_but_not_in_i[j].b2);
          }
        }
      }

      Contacts_t in_i_but_not_in_ref;
      std::set_difference(
          contacts[i].begin(), contacts[i].end(), contacts_ref[i].begin(),
          contacts_ref[i].end(),
          std::inserter(in_i_but_not_in_ref, in_i_but_not_in_ref.begin()));

      if (!in_i_but_not_in_ref.empty()) {
        for (std::size_t j = 0; j < in_i_but_not_in_ref.size(); ++j) {
          if (warn) {
            BOOST_WARN_MESSAGE(
                false, "False contacts: " << in_i_but_not_in_ref[j].b1 << ", "
                                          << in_i_but_not_in_ref[j].b2);
          } else {
            BOOST_CHECK_MESSAGE(
                false, "False contacts: " << in_i_but_not_in_ref[j].b1 << ", "
                                          << in_i_but_not_in_ref[j].b2);
          }
        }
      }
    }
  }

  template <typename BV>
  void check() {
    if (benchmark) return;
    const std::size_t N = transforms.size();

    BOOST_REQUIRE_EQUAL(contacts.size(), 3 * N);
    BOOST_REQUIRE_EQUAL(contacts.size(), contacts_ref.size());

    if (traits<BV, Oriented, Recursive>::IS_IMPLEMENTED) {
      BOOST_TEST_MESSAGE(getindent() << "BV: " << str<BV>() << " oriented");
      ++indent;
      check_contacts(0, N, false);
      --indent;
    }
    if (traits<BV, NonOriented, Recursive>::IS_IMPLEMENTED) {
      BOOST_TEST_MESSAGE(getindent() << "BV: " << str<BV>());
      ++indent;
      check_contacts(N, 2 * N, true);
      --indent;
    }
    if (traits<BV, Oriented, NonRecursive>::IS_IMPLEMENTED) {
      BOOST_TEST_MESSAGE(getindent()
                         << "BV: " << str<BV>() << " oriented non-recursive");
      ++indent;
      check_contacts(2 * N, 3 * N, false);
      --indent;
    }
  }

  template <typename BV>
  void operator()(wrap<BV>) {
    for (std::size_t i = 0; i < splitMethods.size(); ++i) {
      BOOST_TEST_MESSAGE(getindent() << "splitMethod: " << splitMethods[i]);
      ++indent;
      query<BV>(transforms, splitMethods[i], request,
                (isInit ? contacts : contacts_ref));
      if (isInit) check<BV>();
      isInit = true;
      --indent;
    }
  }
};

// This test
//  1. load two objects "env.obj" and "rob.obj" from directory
//     fcl_resources,
//  2. generates n random transformation and for each of them denote tf,
//    2.1 performs a collision test where object 1 is in pose tf. All
//        the contacts are stored in vector global_pairs.
//    2.2 performs a series of collision tests with the same object and
//        the same poses using various methods and various types of bounding
//        volumes. Each time the contacts are stored in vector global_pairs_now.
//
// The methods used to test collision are
//  - collide_Test that calls function collide with tf for object1 pose and
//      identity for the second object pose,
//  - collide_Test2 that moves all vertices of object1 in pose tf and that
//      calls function collide with identity for both object poses,
//
BOOST_AUTO_TEST_CASE(mesh_mesh) {
  std::vector<Transform3s> transforms;
  Scalar extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
#ifndef NDEBUG  // if debug mode
  std::size_t n = 1;
#else
  std::size_t n = 10;
#endif
  n = getNbRun(utf::master_test_suite().argc, utf::master_test_suite().argv, n);

  generateRandomTransforms(extents, transforms, n);

  Eigen::IOFormat f(Eigen::FullPrecision, 0, ", ", ",", "", "", "(", ")");
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    BOOST_TEST_MESSAGE(
        "q" << i << "=" << transforms[i].getTranslation().format(f) << "+"
            << transforms[i].getQuatRotation().coeffs().format(f));
  }

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS

  // Request all contacts and check that all methods give the same result.
  mesh_mesh_run_test runner(
      transforms, CollisionRequest(CONTACT, (size_t)num_max_contacts));
  runner.enable_statistics = true;
  boost::mpl::for_each<BVs_t, wrap<boost::mpl::placeholders::_1>>(runner);

  COAL_COMPILER_DIAGNOSTIC_POP
}

BOOST_AUTO_TEST_CASE(mesh_mesh_benchmark) {
  std::vector<Transform3s> transforms;
  Scalar extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
#ifndef NDEBUG
  std::size_t n = 0;
#else
  std::size_t n = 10;
#endif
  n = getNbRun(utf::master_test_suite().argc, utf::master_test_suite().argv, n);

  generateRandomTransforms(extents, transforms, n);

  Eigen::IOFormat f(Eigen::FullPrecision, 0, ", ", ",", "", "", "(", ")");
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    BOOST_TEST_MESSAGE(
        "q" << i << "=" << transforms[i].getTranslation().format(f) << "+"
            << transforms[i].getQuatRotation().coeffs().format(f));
  }

  // Request all contacts and check that all methods give the same result.
  typedef boost::mpl::vector<OBB, RSS, AABB, KDOP<24>, KDOP<18>, KDOP<16>, kIOS,
                             OBBRSS>
      BVs_t;

  std::ofstream ofs("./collision.benchmark.csv", std::ofstream::out);
  bench_stream = &ofs;

  // without lower bound.
  mesh_mesh_run_test runner1(transforms, CollisionRequest());
  runner1.enable_statistics = false;
  runner1.benchmark = true;
  boost::mpl::for_each<BVs_t, wrap<boost::mpl::placeholders::_1>>(runner1);

  // with lower bound.
  mesh_mesh_run_test runner2(transforms,
                             CollisionRequest(DISTANCE_LOWER_BOUND, 1));
  runner2.enable_statistics = false;
  runner2.benchmark = true;
  boost::mpl::for_each<BVs_t, wrap<boost::mpl::placeholders::_1>>(runner2);

  bench_stream = NULL;
  ofs.close();
}
