// TODO
// code support function with SOA + highway

#include <hwy/targets.h>
#include <hwy/aligned_allocator.h>

#include <benchmark/benchmark.h>

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <iostream>

static Eigen::Vector3d toSphere(const Eigen::Vector3d& point, double scale) {
  return scale / point.norm() * point;
}

struct Icosahedron {
  using TriangleIndex = std::tuple<std::size_t, std::size_t, std::size_t>;

  /// Construct a icosahedron.
  /// For explanation, see:
  /// https://sinestesia.co/blog/tutorials/python-icospheres/
  static Icosahedron construct(double scale) {
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

  Icosahedron subdivide(std::size_t num_subdiv) const {
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
        Eigen::Vector3d v1 =
            ico.middlePoint(std::get<0>(tri), std::get<1>(tri));
        Eigen::Vector3d v2 =
            ico.middlePoint(std::get<1>(tri), std::get<2>(tri));
        Eigen::Vector3d v3 =
            ico.middlePoint(std::get<2>(tri), std::get<0>(tri));

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

  Eigen::Vector3d middlePoint(std::size_t point1_index,
                              std::size_t point2_index) const {
    const Eigen::Vector3d& p1 = points[point1_index];
    const Eigen::Vector3d& p2 = points[point2_index];
    return toSphere((p1 + p2) * 0.5, 1.);
  }

  void toSTL(std::ostream& stream) const {
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

  std::vector<Eigen::Vector3d> points;
  std::vector<TriangleIndex> triangles;
};

struct IcosahedronDatabase {
  static std::unordered_map<std::size_t, Icosahedron> icosahedrons;

  static const Icosahedron& get(std::size_t num_subdiv) {
    auto it = icosahedrons.find(num_subdiv);
    if (it != icosahedrons.end()) {
      return it->second;
    }
    return icosahedrons
        .insert({num_subdiv, Icosahedron::construct(1.).subdivide(num_subdiv)})
        .first->second;
  }
};
std::unordered_map<std::size_t, Icosahedron> IcosahedronDatabase::icosahedrons;

template <typename _Scalar>
struct LegacyAlgorithm {
  using Scalar = _Scalar;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = LegacyAlgorithm;

  static Algorithm fromIcosahedron(const Icosahedron& ico) {
    Algorithm algo;
    algo.points.reserve(ico.points.size());
    for (std::size_t i = 0; i < ico.points.size(); ++i) {
      algo.points.push_back(ico.points[i].cast<Scalar>());
    }
    return algo;
  }

  Scalar support(const Vec3& dir) const {
    Scalar max_dot = points[0].dot(dir);
    for (std::size_t i = 1; i < points.size(); ++i) {
      Scalar dot = points[i].dot(dir);
      if (dot > max_dot) {
        max_dot = dot;
      }
    }
    return max_dot;
  }

  std::vector<Vec3> points;
};

struct SOAFloatEigenAlgorithm {
  using Scalar = float;
  using Vec = Eigen::Vector<Scalar, Eigen::Dynamic>;
  using Array4 = Eigen::Array<Scalar, 4, 1>;
  using Mask4 = Eigen::Array<bool, 4, 1>;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = SOAFloatEigenAlgorithm;

  static Algorithm fromIcosahedron(const Icosahedron& ico) {
    Algorithm algo;
    std::size_t i = 0;
    for (; (i + 4) < ico.points.size(); i += 4) {
      Array4 x, y, z;
      x << static_cast<Scalar>(ico.points[i].x()),
          static_cast<Scalar>(ico.points[i + 1].x()),
          static_cast<Scalar>(ico.points[i + 2].x()),
          static_cast<Scalar>(ico.points[i + 3].x());
      y << static_cast<Scalar>(ico.points[i].y()),
          static_cast<Scalar>(ico.points[i + 1].y()),
          static_cast<Scalar>(ico.points[i + 2].y()),
          static_cast<Scalar>(ico.points[i + 3].y());
      z << static_cast<Scalar>(ico.points[i].z()),
          static_cast<Scalar>(ico.points[i + 1].z()),
          static_cast<Scalar>(ico.points[i + 2].z()),
          static_cast<Scalar>(ico.points[i + 3].z());
      algo.x.push_back(x);
      algo.y.push_back(y);
      algo.z.push_back(z);
    }
    for (; i < ico.points.size(); ++i) {
      algo.remainder.push_back(ico.points[i].cast<Scalar>());
    }
    return algo;
  }

  Scalar support(const Vec3& dir) {
    assert(x.size() == y.size() == z.size());

    Array4 x_dir(dir.x(), dir.x(), dir.x(), dir.x());
    Array4 y_dir(dir.y(), dir.y(), dir.y(), dir.y());
    Array4 z_dir(dir.z(), dir.z(), dir.z(), dir.z());

    constexpr Scalar min_scalar = std::numeric_limits<Scalar>::min();
    Array4 max_dot(min_scalar, min_scalar, min_scalar, min_scalar);

    for (size_t i = 0; i < x.size(); ++i) {
      Array4 dot_x = x[i] * x_dir;
      Array4 dot_y = y[i] * y_dir;
      Array4 dot_z = z[i] * z_dir;
      Array4 dot = dot_x + dot_y + dot_z;

      Mask4 mask = max_dot > dot;
      max_dot = mask.select(max_dot, dot);
    }

    Scalar max_dot_scalar = max_dot.maxCoeff();
    for (size_t i = 0; i < remainder.size(); ++i) {
      Scalar dot = remainder[i].dot(dir);
      if (dot > max_dot_scalar) {
        max_dot_scalar = dot;
      }
    }

    return max_dot_scalar;
  }

  std::vector<Array4> x, y, z;
  std::vector<Vec3> remainder;
};

namespace coal {
float support(const float* HWY_RESTRICT x, const float* HWY_RESTRICT y,
              const float* HWY_RESTRICT z, float x_dir, float y_dir,
              float z_dir, std::size_t count);
float support_with_target(const float* HWY_RESTRICT x,
                          const float* HWY_RESTRICT y,
                          const float* HWY_RESTRICT z, float x_dir, float y_dir,
                          float z_dir, std::size_t count, std::int64_t target);
}  // namespace coal

struct SOAFloatHighwayAlgorithm {
  using Scalar = float;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = SOAFloatHighwayAlgorithm;

  static Algorithm fromIcosahedron(const Icosahedron& ico) {
    Algorithm algo;
    algo.x = hwy::AllocateAligned<Scalar>(ico.points.size());
    algo.y = hwy::AllocateAligned<Scalar>(ico.points.size());
    algo.z = hwy::AllocateAligned<Scalar>(ico.points.size());
    algo.count = ico.points.size();
    for (std::size_t i = 0; i < ico.points.size(); ++i) {
      algo.x[i] = static_cast<Scalar>(ico.points[i].x());
      algo.y[i] = static_cast<Scalar>(ico.points[i].y());
      algo.z[i] = static_cast<Scalar>(ico.points[i].z());
    }
    return algo;
  }

  Scalar support(const Vec3& dir, std::int64_t target) {
    return coal::support_with_target(x.get(), y.get(), z.get(), dir.x(),
                                     dir.y(), dir.z(), count, target);
  }

  hwy::AlignedFreeUniquePtr<Scalar[]> x, y, z;
  std::size_t count;
};

template <typename Scalar>
static void legacyAlgorithmBench(benchmark::State& state) {
  using Algorithm = LegacyAlgorithm<Scalar>;
  auto ico = IcosahedronDatabase::get(static_cast<std::size_t>(state.range(0)));
  auto algo = Algorithm::fromIcosahedron(ico);
  auto vec = Algorithm::Vec3::UnitX();
  for (auto _ : state) {
    auto res = algo.support(vec);
    benchmark::DoNotOptimize(res);
  }
}

static void SOAFloatEigenAlgorithmBench(benchmark::State& state) {
  using Algorithm = SOAFloatEigenAlgorithm;
  auto ico = IcosahedronDatabase::get(static_cast<std::size_t>(state.range(0)));
  auto algo = Algorithm::fromIcosahedron(ico);
  auto vec = Algorithm::Vec3::UnitX();
  for (auto _ : state) {
    auto res = algo.support(vec);
    benchmark::DoNotOptimize(res);
  }
}

static void SOAFloatHighwayAlgorithmBench(benchmark::State& state) {
  using Algorithm = SOAFloatHighwayAlgorithm;
  auto ico = IcosahedronDatabase::get(static_cast<std::size_t>(state.range(1)));
  auto algo = Algorithm::fromIcosahedron(ico);
  auto vec = Algorithm::Vec3::UnitX();
  for (auto _ : state) {
    auto res = algo.support(vec, state.range(0));
    benchmark::DoNotOptimize(res);
  }
}

static void CustomArguments(benchmark::internal::Benchmark* b) {
  // 4 subdivide doesn't fit into L1 cache 5112 points > 48KB
  b->Arg(0)->Arg(1)->Arg(2)->Arg(3)->Arg(4);
}
static void CustomArgumentsHighway(benchmark::internal::Benchmark* b) {
  for (std::int64_t target : hwy::SupportedAndGeneratedTargets()) {
    // 4 subdivide doesn't fit into L1 cache 5112 points > 48KB
    b->Args({target, 0})
        ->Args({target, 1})
        ->Args({target, 2})
        ->Args({target, 3})
        ->Args({target, 4});
  }
}

BENCHMARK(legacyAlgorithmBench<float>)->Apply(CustomArguments);
BENCHMARK(legacyAlgorithmBench<double>)->Apply(CustomArguments);
BENCHMARK(SOAFloatEigenAlgorithmBench)->Apply(CustomArguments);
BENCHMARK(SOAFloatHighwayAlgorithmBench)->Apply(CustomArgumentsHighway);

BENCHMARK_MAIN();
