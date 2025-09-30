// associated header
#include "icosahedron.hh"

#include <algorithm>
#include <ostream>

namespace coal {
namespace bench {
namespace utils {

namespace {

static Eigen::Vector3d toSphere(const Eigen::Vector3d& point, double scale) {
  return scale / point.norm() * point;
}

struct key_hash {
  using Key = std::tuple<std::size_t, std::size_t>;
  std::size_t operator()(const Key& k) const {
    return std::get<0>(k) ^ std::get<1>(k);
  }
};

struct MiddlePointCache {
  using Key = std::tuple<std::size_t, std::size_t>;

  MiddlePointCache(std::vector<Eigen::Vector3d>* points) : points(points) {}

  std::size_t getMiddlePoint(std::size_t point1_index,
                             std::size_t point2_index) {
    const auto [min, max] = std::minmax(point1_index, point2_index);
    const auto [it, emplaced] =
        index_from_segment.try_emplace({min, max}, points->size());
    if (emplaced) {
      const Eigen::Vector3d& p1 = (*points)[point1_index];
      const Eigen::Vector3d& p2 = (*points)[point2_index];
      points->push_back(toSphere((p1 + p2) * 0.5, 1.));
    }
    return it->second;
  }

  void clear() { index_from_segment.clear(); }

  std::unordered_map<Key, std::size_t, key_hash> index_from_segment;
  std::vector<Eigen::Vector3d>* points;
};

}  // namespace

Icosahedron Icosahedron::construct(double scale) {
  const double PHI = (1 + std::sqrt(5)) / 2;
  Icosahedron ico;
  ico.points.reserve(12);
  ico.points.push_back(toSphere(Eigen::Vector3d(-1, PHI, 0), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(1, PHI, 0), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(-1, -PHI, 0), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(1, -PHI, 0), scale));

  ico.points.push_back(toSphere(Eigen::Vector3d(0, -1, PHI), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(0, 1, PHI), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(0, -1, -PHI), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(0, 1, -PHI), scale));

  ico.points.push_back(toSphere(Eigen::Vector3d(PHI, 0, -1), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(PHI, 0, 1), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(-PHI, 0, -1), scale));
  ico.points.push_back(toSphere(Eigen::Vector3d(-PHI, 0, 1), scale));

  ico.triangles.reserve(20);
  ico.triangles.push_back({0, 11, 5});
  ico.triangles.push_back({0, 5, 1});
  ico.triangles.push_back({0, 1, 7});
  ico.triangles.push_back({0, 7, 10});
  ico.triangles.push_back({0, 10, 11});

  ico.triangles.push_back({1, 5, 9});
  ico.triangles.push_back({5, 11, 4});
  ico.triangles.push_back({11, 10, 2});
  ico.triangles.push_back({10, 7, 6});
  ico.triangles.push_back({7, 1, 8});

  ico.triangles.push_back({3, 9, 4});
  ico.triangles.push_back({3, 4, 2});
  ico.triangles.push_back({3, 2, 6});
  ico.triangles.push_back({3, 6, 8});
  ico.triangles.push_back({3, 8, 9});

  ico.triangles.push_back({4, 9, 5});
  ico.triangles.push_back({2, 4, 11});
  ico.triangles.push_back({6, 2, 10});
  ico.triangles.push_back({8, 6, 7});
  ico.triangles.push_back({9, 8, 1});

  ico.constructNeighbors();
  return ico;
}

Icosahedron Icosahedron::subdivide(std::size_t num_subdiv) const {
  Icosahedron ico(*this);

  // Temporary buffer
  std::vector<TriangleIndex> new_triangles;

  // Reserve memory to avoid reallocation
  std::size_t total_points_count = points.size();
  std::size_t triangle_pow = 1;
  for (std::size_t i = 0; i < num_subdiv; ++i) {
    // 6 new points per 4 triangles
    total_points_count += 6 * (triangle_pow * triangles.size()) / 4;
    triangle_pow *= 4;
  }
  ico.points.reserve(total_points_count);
  std::size_t total_triangle_count = triangle_pow * triangles.size();
  new_triangles.reserve(total_triangle_count);
  ico.triangles.reserve(total_triangle_count);

  MiddlePointCache cache(&ico.points);

  for (std::size_t step = 0; step < num_subdiv; ++step) {
    new_triangles.clear();
    for (const auto& tri : ico.triangles) {
      std::size_t v1_index = cache.getMiddlePoint(tri[0], tri[1]);
      std::size_t v2_index = cache.getMiddlePoint(tri[1], tri[2]);
      std::size_t v3_index = cache.getMiddlePoint(tri[2], tri[0]);

      new_triangles.push_back({tri[0], v1_index, v3_index});
      new_triangles.push_back({tri[1], v2_index, v1_index});
      new_triangles.push_back({tri[2], v3_index, v2_index});
      new_triangles.push_back({v1_index, v2_index, v3_index});
    }
    std::swap(ico.triangles, new_triangles);
  }
  ico.constructNeighbors();
  return ico;
}

void Icosahedron::toSTL(std::ostream& stream) const {
  stream << "solid icosahedron\n";
  for (const auto& tri : triangles) {
    const auto& v1 = points[tri[0]];
    const auto& v2 = points[tri[1]];
    const auto& v3 = points[tri[2]];
    stream << "  facet normal 0 0 0\n";
    stream << "    outer loop\n";
    stream << "      vertex " << v1.x() << " " << v1.y() << " " << v1.z()
           << "\n";
    stream << "      vertex " << v2.x() << " " << v2.y() << " " << v2.z()
           << "\n";
    stream << "      vertex " << v3.x() << " " << v3.y() << " " << v3.z()
           << "\n";
    stream << "    endloop\n";
    stream << "  endfacet\n";
  }
  stream << "endsolid icosahedron\n";
}

Eigen::Vector3d Icosahedron::middlePoint(std::size_t point1_index,
                                         std::size_t point2_index) const {
  const Eigen::Vector3d& p1 = points[point1_index];
  const Eigen::Vector3d& p2 = points[point2_index];
  return toSphere((p1 + p2) * 0.5, 1.);
}

void Icosahedron::constructNeighbors() {
  neighbors.clear();
  neighbors.resize(points.size());

  auto push_if_not_exist = [&](std::size_t vertex_index,
                               std::size_t neighbor_index) {
    auto& vertex_neighbors = neighbors[vertex_index];
    auto it = std::find(vertex_neighbors.begin(), vertex_neighbors.end(),
                        neighbor_index);
    if (it == vertex_neighbors.end()) {
      vertex_neighbors.push_back(neighbor_index);
    }
  };
  for (const auto& tri : triangles) {
    push_if_not_exist(tri[0], tri[1]);
    push_if_not_exist(tri[0], tri[2]);

    push_if_not_exist(tri[1], tri[0]);
    push_if_not_exist(tri[1], tri[2]);

    push_if_not_exist(tri[2], tri[0]);
    push_if_not_exist(tri[2], tri[1]);
  }
}

const Icosahedron& IcosahedronDatabase::get(std::size_t num_subdiv) {
  auto it = icosahedrons.find(num_subdiv);
  if (it != icosahedrons.end()) {
    return it->second;
  }
  return icosahedrons
      .insert({num_subdiv, Icosahedron::construct(1.).subdivide(num_subdiv)})
      .first->second;
}
std::unordered_map<std::size_t, Icosahedron> IcosahedronDatabase::icosahedrons;

}  // namespace utils
}  // namespace bench
}  // namespace coal
