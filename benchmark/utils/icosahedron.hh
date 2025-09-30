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

 public:
  std::vector<Eigen::Vector3d> points;
  std::vector<TriangleIndex> triangles;
};

class IcosahedronWithNeighbors {
 public:
  using NeighborIndexes = std::vector<std::size_t>;

  IcosahedronWithNeighbors(Icosahedron ico) : icosahedron(ico) {
    constructNeighbors();
  }

 protected:
  void constructNeighbors();

 public:
  Icosahedron icosahedron;
  std::vector<NeighborIndexes> neighbors;
};

class IcosahedronDatabase {
 public:
  static const Icosahedron& get(std::size_t num_subdiv);

 protected:
  static std::unordered_map<std::size_t, Icosahedron> icosahedrons;
};

class IcosahedronWithNeighborsDatabase {
 public:
  static const IcosahedronWithNeighbors& get(std::size_t num_subdiv);

 protected:
  static std::unordered_map<std::size_t, IcosahedronWithNeighbors> icosahedrons;
};

}  // namespace utils
}  // namespace bench
}  // namespace coal

#endif  // ifndef COAL_BENCH_UTILS_ICOSAHEDRON_HH
