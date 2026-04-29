// Robot-vs-robot collision benchmark: forces leaf-level triangle-triangle
// tests to fire by placing two copies of rob.obj near each other across a
// range of translation extents (deep overlap, near miss, mostly disjoint).
//
// Reports per-cell: query time, num_bv_tests/query, num_leaf_tests/query,
// collision rate. Collision-only (no distance), matching real robot-vs-robot
// motion-validation usage.

#include <boost/filesystem.hpp>
#include <iostream>
#include <iomanip>

#include "coal/internal/traversal_node_setup.h"
#include "coal/internal/traversal_node_bvhs.h"
#include "../src/collision_node.h"
#include "coal/internal/BV_splitter.h"

#include "utility.h"
#include "fcl_resources/config.h"

using namespace coal;

template <typename BV>
void makeModel(const std::vector<Vec3s>& vertices,
               const std::vector<Triangle32>& triangles,
               SplitMethodType split_method, BVHModel<BV>& model) {
  model.bv_splitter.reset(new BVSplitter<BV>(split_method));
  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

template <typename BV, typename TraversalNode>
void runCell(const char* label, const std::vector<Transform3s>& tfs,
             const BVHModel<BV>& m1, const BVHModel<BV>& m2) {
  CollisionRequest request;
  request.num_max_contacts = 1;    // typical motion-validation usage
  request.enable_contact = false;  // boolean overlap only
  TraversalNode node(request);
  node.enable_statistics = true;

  Transform3s identity;

  long long total_bv = 0, total_leaf = 0;
  int colliding = 0;

  BenchTimer timer;
  timer.start();

  for (size_t i = 0; i < tfs.size(); ++i) {
    CollisionResult result;
    bool ok = initialize(node, m1, identity, m2, tfs[i], result);
    (void)ok;
    node.num_bv_tests = 0;
    node.num_leaf_tests = 0;

    collide(&node, request, result);

    total_bv += node.num_bv_tests;
    total_leaf += node.num_leaf_tests;
    if (result.numContacts() > 0) ++colliding;
  }
  double us = timer.getElapsedTimeInMicroSec();
  double per_query_us = us / double(tfs.size());

  std::cout << std::left << std::setw(28) << label << " | " << std::setw(9)
            << std::fixed << std::setprecision(2) << per_query_us << " us/q | "
            << std::setw(8) << (total_bv / (long long)tfs.size()) << " bv | "
            << std::setw(7) << (total_leaf / (long long)tfs.size())
            << " leaf | " << colliding << "/" << tfs.size() << " colliding\n";
}

template <typename BV, typename TraversalNode>
void runAllCells(const char* bv_name, const BVHModel<BV>& m1,
                 const BVHModel<BV>& m2) {
  // Each cell: rob-vs-rob with rotation + translation in given extent.
  // rob bbox is ~1000x1200x500; pick extents to span overlap regimes.
  struct {
    const char* name;
    Scalar extent;
  } cells[] = {
      {"deep_overlap (e=200)", Scalar(200)},
      {"medium_overlap (e=500)", Scalar(500)},
      {"near_miss (e=1000)", Scalar(1000)},
      {"mostly_disjoint (e=2000)", Scalar(2000)},
  };

  std::cout << "\n--- " << bv_name << " ---\n";
  for (const auto& c : cells) {
    std::vector<Transform3s> tfs;
    Scalar ext[6] = {-c.extent, -c.extent, -c.extent,
                     c.extent,  c.extent,  c.extent};
    generateRandomTransforms(ext, tfs, 2000);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s/%s", bv_name, c.name);
    runCell<BV, TraversalNode>(buf, tfs, m1, m2);
  }
}

int main(int, char*[]) {
  std::vector<Vec3s> p;
  std::vector<Triangle32> t;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "rob.obj").string().c_str(), p, t);

  std::cout << "rob mesh: " << p.size() << " verts, " << t.size()
            << " tris\n\n";
  std::cout << "Cell                          | Time      | BV tests | "
               "Leaf    | Outcome\n";
  std::cout
      << "------------------------------+-----------+----------+---------+"
         "----------\n";

  BVHModel<RSS> m_rss[2];
  makeModel(p, t, SPLIT_METHOD_MEAN, m_rss[0]);
  makeModel(p, t, SPLIT_METHOD_MEAN, m_rss[1]);
  runAllCells<RSS, MeshCollisionTraversalNodeRSS>("RSS", m_rss[0], m_rss[1]);

  BVHModel<OBB> m_obb[2];
  makeModel(p, t, SPLIT_METHOD_MEAN, m_obb[0]);
  makeModel(p, t, SPLIT_METHOD_MEAN, m_obb[1]);
  runAllCells<OBB, MeshCollisionTraversalNodeOBB>("OBB", m_obb[0], m_obb[1]);

  BVHModel<OBBRSS> m_obbrss[2];
  makeModel(p, t, SPLIT_METHOD_MEAN, m_obbrss[0]);
  makeModel(p, t, SPLIT_METHOD_MEAN, m_obbrss[1]);
  runAllCells<OBBRSS, MeshCollisionTraversalNodeOBBRSS>("OBBRSS", m_obbrss[0],
                                                        m_obbrss[1]);

  BVHModel<kIOS> m_kios[2];
  makeModel(p, t, SPLIT_METHOD_MEAN, m_kios[0]);
  makeModel(p, t, SPLIT_METHOD_MEAN, m_kios[1]);
  runAllCells<kIOS, MeshCollisionTraversalNodekIOS>("kIOS", m_kios[0],
                                                    m_kios[1]);

  return 0;
}
