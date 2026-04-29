// Devirtualized variant of benchmark.cpp: calls the templated
// collisionRecurseT / distanceRecurseT directly with the concrete
// traversal-node type, so the compiler can resolve all virtual calls
// statically. Direct A/B against benchmark.cpp.

#include <boost/filesystem.hpp>

#include "coal/internal/traversal_node_setup.h"
#include "coal/internal/traversal_node_bvhs.h"
#include "coal/internal/traversal_recurse.h"
#include "../src/collision_node.h"
#include "coal/internal/BV_splitter.h"

#include "utility.h"
#include "fcl_resources/config.h"

#define RUN_CASE(BV, tf, models, split) \
  run<BV>(tf, models, split, #BV " - " #split ":\t")

using namespace coal;

bool verbose = false;
Scalar DELTA = Scalar(0.001);

template <typename BV>
void makeModel(const std::vector<Vec3s>& vertices,
               const std::vector<Triangle32>& triangles,
               SplitMethodType split_method, BVHModel<BV>& model) {
  model.bv_splitter.reset(new BVSplitter<BV>(split_method));
  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

template <typename BV>
struct traits {};

template <>
struct traits<RSS> {
  typedef MeshCollisionTraversalNodeRSS CollisionTraversalNode;
  typedef MeshDistanceTraversalNodeRSS DistanceTraversalNode;
};
template <>
struct traits<kIOS> {
  typedef MeshCollisionTraversalNodekIOS CollisionTraversalNode;
  typedef MeshDistanceTraversalNodekIOS DistanceTraversalNode;
};
template <>
struct traits<OBB> {
  typedef MeshCollisionTraversalNodeOBB CollisionTraversalNode;
};
template <>
struct traits<OBBRSS> {
  typedef MeshCollisionTraversalNodeOBBRSS CollisionTraversalNode;
  typedef MeshDistanceTraversalNodeOBBRSS DistanceTraversalNode;
};

template <typename BV, typename TraversalNode>
double distanceT(const std::vector<Transform3s>& tf, const BVHModel<BV>& m1,
                 const BVHModel<BV>& m2) {
  Transform3s pose2;
  DistanceResult local_result;
  DistanceRequest request(true);
  TraversalNode node;
  node.enable_statistics = false;

  BenchTimer timer;
  timer.start();
  for (std::size_t i = 0; i < tf.size(); ++i) {
    if (!initialize(node, m1, tf[i], m2, pose2, request, local_result))
      std::cout << "init err\n";
    node.preprocess();
    distanceRecurseT(&node, 0u, 0u, /*front_list*/ nullptr);
    node.postprocess();
  }
  return timer.getElapsedTimeInMicroSec();
}

template <typename BV, typename TraversalNode>
double collideT(const std::vector<Transform3s>& tf, const BVHModel<BV>& m1,
                const BVHModel<BV>& m2) {
  Transform3s pose2;
  CollisionResult local_result;
  CollisionRequest request;
  TraversalNode node(request);
  node.enable_statistics = false;

  BenchTimer timer;
  timer.start();
  for (std::size_t i = 0; i < tf.size(); ++i) {
    bool success(initialize(node, m1, tf[i], m2, pose2, local_result));
    (void)success;
    assert(success);
    CollisionResult result;
    Scalar sqrDistLowerBound = 0;
    collisionRecurseT(&node, 0u, 0u, /*front_list*/ nullptr, sqrDistLowerBound);
  }
  return timer.getElapsedTimeInMicroSec();
}

template <typename BV>
double run(const std::vector<Transform3s>& tf,
           const BVHModel<BV> (&models)[2][3], int split_method,
           const char* prefix) {
  double col = collideT<BV, typename traits<BV>::CollisionTraversalNode>(
      tf, models[0][split_method], models[1][split_method]);
  double dist = distanceT<BV, typename traits<BV>::DistanceTraversalNode>(
      tf, models[0][split_method], models[1][split_method]);
  std::cout << prefix << " (" << col << ", " << dist << ")\n";
  return col + dist;
}

template <>
double run<OBB>(const std::vector<Transform3s>& tf,
                const BVHModel<OBB> (&models)[2][3], int split_method,
                const char* prefix) {
  double col = collideT<OBB, traits<OBB>::CollisionTraversalNode>(
      tf, models[0][split_method], models[1][split_method]);
  std::cout << prefix << " (\t" << col << ", \tNaN)\n";
  return col;
}

int main(int, char*[]) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle32> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  BVHModel<RSS> ms_rss[2][3];
  for (int s = 0; s < 3; ++s) {
    makeModel(p1, t1, (SplitMethodType)s, ms_rss[0][s]);
    makeModel(p2, t2, (SplitMethodType)s, ms_rss[1][s]);
  }
  BVHModel<kIOS> ms_kios[2][3];
  for (int s = 0; s < 3; ++s) {
    makeModel(p1, t1, (SplitMethodType)s, ms_kios[0][s]);
    makeModel(p2, t2, (SplitMethodType)s, ms_kios[1][s]);
  }
  BVHModel<OBB> ms_obb[2][3];
  for (int s = 0; s < 3; ++s) {
    makeModel(p1, t1, (SplitMethodType)s, ms_obb[0][s]);
    makeModel(p2, t2, (SplitMethodType)s, ms_obb[1][s]);
  }
  BVHModel<OBBRSS> ms_obbrss[2][3];
  for (int s = 0; s < 3; ++s) {
    makeModel(p1, t1, (SplitMethodType)s, ms_obbrss[0][s]);
    makeModel(p2, t2, (SplitMethodType)s, ms_obbrss[1][s]);
  }

  std::vector<Transform3s> transforms;
  Scalar extents[] = {-3000, -3000, -3000, 3000, 3000, 3000};
  std::size_t n = 10000;
  generateRandomTransforms(extents, transforms, n);
  double total_time = 0;

  total_time += RUN_CASE(RSS, transforms, ms_rss, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(RSS, transforms, ms_rss, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(RSS, transforms, ms_rss, SPLIT_METHOD_MEDIAN);

  total_time += RUN_CASE(kIOS, transforms, ms_kios, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(kIOS, transforms, ms_kios, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(kIOS, transforms, ms_kios, SPLIT_METHOD_MEDIAN);

  total_time += RUN_CASE(OBB, transforms, ms_obb, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(OBB, transforms, ms_obb, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(OBB, transforms, ms_obb, SPLIT_METHOD_MEDIAN);

  total_time += RUN_CASE(OBBRSS, transforms, ms_obbrss, SPLIT_METHOD_MEAN);
  total_time += RUN_CASE(OBBRSS, transforms, ms_obbrss, SPLIT_METHOD_BV_CENTER);
  total_time += RUN_CASE(OBBRSS, transforms, ms_obbrss, SPLIT_METHOD_MEDIAN);

  std::cout << "\n\nTotal time: " << total_time << std::endl;
  return 0;
}
