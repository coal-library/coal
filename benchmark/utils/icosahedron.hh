#ifndef COAL_BENCH_UTILS_ICOSAHEDRON_HH
#define COAL_BENCH_UTILS_ICOSAHEDRON_HH

#include <Eigen/Core>

#include <array>
#include <vector>
#include <cstddef>
#include <unordered_map>

namespace coal {
namespace bench {
namespace utils {

class Icosahedron {
 public:
  using TriangleIndex = std::array<std::size_t, 3>;

  /// Construct a icosahedron.
  /// For explanation, see:
  /// https://sinestesia.co/blog/tutorials/python-icospheres/
  static Icosahedron construct(double scale);

  Icosahedron subdivide(std::size_t num_subdiv) const;
  void toSTL(std::ostream& stream) const;

 protected:
  Eigen::Vector3d middlePoint(std::size_t point1_index,
                              std::size_t point2_index) const;

 public:
  std::vector<Eigen::Vector3d> points;
  std::vector<TriangleIndex> triangles;
};

class IcosahedronDatabase {
 public:
  static const Icosahedron& get(std::size_t num_subdiv);

 protected:
  static std::unordered_map<std::size_t, Icosahedron> icosahedrons;
};

}  // namespace utils
}  // namespace bench
}  // namespace coal

#endif  // ifndef COAL_BENCH_UTILS_ICOSAHEDRON_HH
