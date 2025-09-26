// associated header
#include "icosahedron.hh"

#include <ostream>

namespace coal {
namespace bench {
namespace utils {

static Eigen::Vector3d toSphere(const Eigen::Vector3d& point, double scale) {
  return scale / point.norm() * point;
}

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
  ico.triangles.emplace_back(0, 11, 5);
  ico.triangles.emplace_back(0, 5, 1);
  ico.triangles.emplace_back(0, 1, 7);
  ico.triangles.emplace_back(0, 7, 10);
  ico.triangles.emplace_back(0, 10, 11);

  ico.triangles.emplace_back(1, 5, 9);
  ico.triangles.emplace_back(5, 11, 4);
  ico.triangles.emplace_back(11, 10, 2);
  ico.triangles.emplace_back(10, 7, 6);
  ico.triangles.emplace_back(7, 1, 8);

  ico.triangles.emplace_back(3, 9, 4);
  ico.triangles.emplace_back(3, 4, 2);
  ico.triangles.emplace_back(3, 2, 6);
  ico.triangles.emplace_back(3, 6, 8);
  ico.triangles.emplace_back(3, 8, 9);

  ico.triangles.emplace_back(4, 9, 5);
  ico.triangles.emplace_back(2, 4, 11);
  ico.triangles.emplace_back(6, 2, 10);
  ico.triangles.emplace_back(8, 6, 7);
  ico.triangles.emplace_back(9, 8, 1);
  return ico;
}

Icosahedron Icosahedron::subdivide(std::size_t num_subdiv) const {
  Icosahedron ico(*this);

  // Temporary buffer
  std::vector<TriangleIndex> new_triangles;

  // Reserve memory to avoid realocation
  std::size_t total_points_count = points.size();
  std::size_t triangle_pow = 1;
  for (std::size_t i = 0; i < num_subdiv; ++i) {
    total_points_count += 3 * triangle_pow * triangles.size();
    triangle_pow *= 4;
  }
  ico.points.reserve(total_points_count);
  std::size_t total_triangle_count = triangle_pow * triangles.size();
  new_triangles.reserve(total_triangle_count);
  ico.triangles.reserve(total_triangle_count);

  for (std::size_t step = 0; step < num_subdiv; ++step) {
    new_triangles.clear();
    for (const auto& tri : ico.triangles) {
      Eigen::Vector3d v1 = ico.middlePoint(std::get<0>(tri), std::get<1>(tri));
      Eigen::Vector3d v2 = ico.middlePoint(std::get<1>(tri), std::get<2>(tri));
      Eigen::Vector3d v3 = ico.middlePoint(std::get<2>(tri), std::get<0>(tri));

      std::size_t v1_index = ico.points.size();
      ico.points.push_back(v1);
      std::size_t v2_index = ico.points.size();
      ico.points.push_back(v2);
      std::size_t v3_index = ico.points.size();
      ico.points.push_back(v3);

      new_triangles.emplace_back(std::get<0>(tri), v1_index, v3_index);
      new_triangles.emplace_back(std::get<1>(tri), v2_index, v1_index);
      new_triangles.emplace_back(std::get<2>(tri), v3_index, v2_index);
      new_triangles.emplace_back(v1_index, v2_index, v3_index);
    }
    std::swap(ico.triangles, new_triangles);
  }
  return ico;
}

void Icosahedron::toSTL(std::ostream& stream) const {
  stream << "solid icosahedron\n";
  for (const auto& tri : triangles) {
    const auto& v1 = points[std::get<0>(tri)];
    const auto& v2 = points[std::get<1>(tri)];
    const auto& v3 = points[std::get<2>(tri)];
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
