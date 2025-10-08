#include "convex_support_highway.hh"
#include "utils/icosahedron.hh"

#include <hwy/targets.h>
#include <hwy/aligned_allocator.h>

#include <benchmark/benchmark.h>

#include <Eigen/Core>

#include <boost/math/constants/constants.hpp>

#include <limits>
#include <vector>
#include <cmath>
#include <random>

namespace coal {
namespace bench {

template <typename Scalar>
Eigen::Vector<Scalar, 3> fromSpherical(Scalar horiz, Scalar vert) {
  const auto sin_horiz = std::sin(horiz);
  const auto cos_horiz = std::cos(horiz);
  const auto sin_vert = std::sin(vert);
  const auto cos_vert = std::cos(vert);
  return Eigen::Vector<Scalar, 3>(cos_horiz * cos_vert, sin_vert,
                                  sin_horiz * cos_vert);
}

struct WarmStartMesh {
  static WarmStartMesh construct(std::size_t points_horizontal,
                                 std::size_t points_vertical) {
    const auto half_pi = boost::math::constants::half_pi<double>();
    const auto pi = boost::math::constants::pi<double>();
    const auto two_pi = boost::math::constants::two_pi<double>();
    WarmStartMesh ws;
    ws.points.reserve(points_horizontal * points_vertical);
    for (std::size_t x = 0; x < points_horizontal; ++x) {
      for (std::size_t y = 0; y < points_vertical; ++y) {
        const double horiz =
            (static_cast<double>(x) / static_cast<double>(points_horizontal)) *
            two_pi;
        const double vert =
            -half_pi + (static_cast<double>(y + 1) /
                        static_cast<double>(points_vertical + 1)) *
                           pi;
        ws.points.push_back(fromSpherical(horiz, vert));
      }
    }
    ws.points.push_back(Eigen::Vector3d::UnitY());
    ws.points.push_back(-Eigen::Vector3d::UnitY());
    return ws;
  }

  std::vector<Eigen::Vector3d> points;
};

template <typename _Scalar>
struct LegacyLinearAlgorithm {
  using Scalar = _Scalar;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = LegacyLinearAlgorithm;

  static Algorithm fromPoints(const std::vector<Eigen::Vector3d>& points) {
    Algorithm algo;
    algo.points.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      algo.points.push_back(points[i].cast<Scalar>());
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

  std::tuple<Scalar, std::size_t> supportWithIndex(const Vec3& dir) const {
    std::size_t max_index = 0;
    Scalar max_dot = points[0].dot(dir);
    for (std::size_t i = 1; i < points.size(); ++i) {
      Scalar dot = points[i].dot(dir);
      if (dot > max_dot) {
        max_dot = dot;
        max_index = i;
      }
    }
    return std::make_tuple(max_dot, max_index);
  }

  std::vector<Vec3> points;
};

template <typename _Scalar>
struct LegacyLogWarmStart {
  using Scalar = _Scalar;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using SearchAlgorithm = LegacyLinearAlgorithm<Scalar>;

  static LegacyLogWarmStart fromPoints(
      const std::vector<Eigen::Vector3d>& points) {
    LegacyLogWarmStart warm_start;
    auto warm_start_mesh = WarmStartMesh::construct(4, 3);
    auto init_search_algo = SearchAlgorithm::fromPoints(points);
    warm_start.search_algo =
        SearchAlgorithm::fromPoints(warm_start_mesh.points);
    warm_start.closest_index.reserve(warm_start_mesh.points.size());

    // Match warm_start_mesh points with geometry points
    for (const auto& p : warm_start_mesh.points) {
      auto [_, index] =
          init_search_algo.supportWithIndex(p.template cast<Scalar>());
      warm_start.closest_index.push_back(index);
    }

    return warm_start;
  }

  std::size_t hint(const Vec3& dir) const {
    auto [__, index] = search_algo.supportWithIndex(dir);
    return closest_index[index];
  }

  SearchAlgorithm search_algo;
  std::vector<std::size_t> closest_index;
};

template <typename _Scalar>
struct LegacyLogAlgorithm {
  using Scalar = _Scalar;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using NeighborIndexes = std::vector<std::size_t>;
  using WarmStart = LegacyLogWarmStart<Scalar>;
  using Algorithm = LegacyLogAlgorithm;

  static Algorithm fromPointsAndNeighbors(
      const std::vector<Eigen::Vector3d>& points,
      std::vector<NeighborIndexes> neighbors) {
    Algorithm algo;

    algo.warm_start = WarmStart::fromPoints(points);
    algo.points.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      algo.points.push_back(points[i].cast<Scalar>());
    }
    algo.visited.resize(points.size());
    algo.neighbors = std::move(neighbors);
    return algo;
  }

  std::tuple<Scalar, std::size_t> supportWithIndex(const Vec3& dir,
                                                   std::size_t hint) const {
    std::fill(visited.begin(), visited.end(), false);

    // Compute initial hint if dir is too far from last_dir
    const Scalar use_warm_start_threshold = Scalar(0.9);
    Vec3 dir_normalized = dir.normalized();
    if (last_dir.isZero() ||
        last_dir.dot(dir_normalized) < use_warm_start_threshold) {
      hint = warm_start.hint(dir);
    }
    last_dir = dir_normalized;

    bool found = true;
    bool loose_check = true;
    std::size_t current_vertex_index = hint;

    Scalar max_dot = points[current_vertex_index].dot(dir);
    while (found) {
      const NeighborIndexes& n = neighbors[current_vertex_index];
      found = false;
      for (const auto neighbor_index : n) {
        if (visited[neighbor_index]) continue;

        visited[neighbor_index] = true;
        const Scalar dot = points[neighbor_index].dot(dir);
        bool better = false;
        if (dot > max_dot) {
          better = true;
          loose_check = false;
        } else if (loose_check && dot == max_dot) {
          better = true;
        }

        if (better) {
          max_dot = dot;
          current_vertex_index = neighbor_index;
          found = true;
        }
      }
    }
    return std::make_tuple(max_dot, current_vertex_index);
  }

  std::vector<Vec3> points;
  std::vector<NeighborIndexes> neighbors;
  WarmStart warm_start;
  mutable std::vector<int8_t> visited;
  mutable Vec3 last_dir = Vec3::Zero();
};

struct SOAFloatEigenLinearAlgorithm {
  using Scalar = float;
  using Vec = Eigen::Vector<Scalar, Eigen::Dynamic>;
  using Array4 = Eigen::Array<Scalar, 4, 1>;
  using Mask4 = Eigen::Array<bool, 4, 1>;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = SOAFloatEigenLinearAlgorithm;

  static Algorithm fromPoints(const std::vector<Eigen::Vector3d>& points) {
    Algorithm algo;
    std::size_t i = 0;
    for (; (i + 4) < points.size(); i += 4) {
      Array4 x, y, z;
      x << static_cast<Scalar>(points[i].x()),
          static_cast<Scalar>(points[i + 1].x()),
          static_cast<Scalar>(points[i + 2].x()),
          static_cast<Scalar>(points[i + 3].x());
      y << static_cast<Scalar>(points[i].y()),
          static_cast<Scalar>(points[i + 1].y()),
          static_cast<Scalar>(points[i + 2].y()),
          static_cast<Scalar>(points[i + 3].y());
      z << static_cast<Scalar>(points[i].z()),
          static_cast<Scalar>(points[i + 1].z()),
          static_cast<Scalar>(points[i + 2].z()),
          static_cast<Scalar>(points[i + 3].z());
      algo.x.push_back(x);
      algo.y.push_back(y);
      algo.z.push_back(z);
    }
    for (; i < points.size(); ++i) {
      algo.remainder.push_back(points[i].cast<Scalar>());
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

template <typename Scalar>
static void legacyLinearAlgorithmBench(benchmark::State& state) {
  using Algorithm = LegacyLinearAlgorithm<Scalar>;
  auto ico =
      utils::IcosahedronDatabase::get(static_cast<std::size_t>(state.range(0)));
  auto algo = Algorithm::fromPoints(ico.points);
  auto vec = Algorithm::Vec3::UnitX();
  for (auto _ : state) {
    auto res = algo.support(vec);
    benchmark::DoNotOptimize(res);
  }
}

static void SOAFloatEigenLinearAlgorithmBench(benchmark::State& state) {
  using Algorithm = SOAFloatEigenLinearAlgorithm;
  auto ico =
      utils::IcosahedronDatabase::get(static_cast<std::size_t>(state.range(0)));
  auto algo = Algorithm::fromPoints(ico.points);
  auto vec = Algorithm::Vec3::UnitX();
  for (auto _ : state) {
    auto res = algo.support(vec);
    benchmark::DoNotOptimize(res);
  }
}

template <typename Scalar>
static void SOAHighwayLinearAlgorithmBench(benchmark::State& state) {
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  const auto target = state.range(0);
  const auto num_subdiv = state.range(1);

  auto ico =
      utils::IcosahedronDatabase::get(static_cast<std::size_t>(num_subdiv));
  auto vec = Algorithm::Vec3::UnitX();
  hwy::SetSupportedTargetsForTest(target);
  auto algo = Algorithm::fromPoints(ico.points);
  for (auto _ : state) {
    auto res = algo.support(vec);
    benchmark::DoNotOptimize(res);
  }
  hwy::SetSupportedTargetsForTest(0);
}

template <typename Scalar>
static void SOAHighwayLinearWithIndexAlgorithmBench(benchmark::State& state) {
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  const auto target = state.range(0);
  const auto num_subdiv = state.range(1);

  auto ico =
      utils::IcosahedronDatabase::get(static_cast<std::size_t>(num_subdiv));
  auto vec = Algorithm::Vec3::UnitX();
  hwy::SetSupportedTargetsForTest(target);
  auto algo = Algorithm::fromPoints(ico.points);
  for (auto _ : state) {
    auto res = algo.supportWithIndex(vec);
    benchmark::DoNotOptimize(res);
  }
  hwy::SetSupportedTargetsForTest(0);
}

template <typename Scalar>
EIGEN_DONT_INLINE static std::vector<Eigen::Vector<Scalar, 3>> sampleVector(
    double horiz, double vert, double std_dev) {
  using Vec3 = Eigen::Vector<Scalar, 3>;

  std::mt19937 gen{1234};
  std::normal_distribution d{0.0, std_dev};

  constexpr std::size_t N = 50;
  std::vector<Vec3> ret;
  ret.reserve(N);

  for (std::size_t i = 0; i < N; ++i) {
    Scalar rand_horiz = static_cast<Scalar>(d(gen));
    Scalar rand_vert = static_cast<Scalar>(d(gen));
    ret.push_back(fromSpherical(static_cast<Scalar>(horiz) + rand_horiz,
                                static_cast<Scalar>(vert) + rand_vert));
  }
  return ret;
}

template <typename Scalar>
static void legacyLogWarmStartAlgorithmBench(benchmark::State& state) {
  using Algorithm = LegacyLogAlgorithm<Scalar>;

  const auto std_dev = static_cast<double>(state.range(0)) / 100.;
  const auto num_subdiv = state.range(1);

  auto ico = utils::IcosahedronWithNeighborsDatabase::get(
      static_cast<std::size_t>(num_subdiv));
  auto algo = Algorithm::fromPointsAndNeighbors(ico.points, ico.neighbors);

  auto vecs = sampleVector<Scalar>(2., 2., std_dev);

  std::size_t hint = 0;
  std::size_t i = 0;
  for (auto _ : state) {
    auto res_hint = algo.supportWithIndex(vecs[i % vecs.size()], hint);
    benchmark::DoNotOptimize(res_hint);
    hint = std::get<1>(res_hint);
    ++i;
  }
}

static void LinearCustomArguments(benchmark::internal::Benchmark* b) {
  // 5 subdivide doesn't fit into L1 cache (10242 points > 48KB)
  b->Arg(0)->Arg(1)->Arg(2)->Arg(3)->Arg(4)->Arg(5);
}
static void LinearCustomArgumentsHighway(benchmark::internal::Benchmark* b) {
  for (std::int64_t target : hwy::SupportedAndGeneratedTargets()) {
    if (target != HWY_SSSE3 && target != HWY_SSE4 && target != HWY_NEON_BF16) {
      // 5 subdivide doesn't fit into L1 cache (10242 points > 48KB)
      b->Args({target, 0})
          ->Args({target, 1})
          ->Args({target, 2})
          ->Args({target, 3})
          ->Args({target, 4})
          ->Args({target, 5});
    }
  }
}
static void LogCustomArguments(benchmark::internal::Benchmark* b) {
  // 5 subdivide doesn't fit into L1 cache (10242 points > 48KB)
  b->ArgsProduct({{0, 10, 1000}, {0, 1, 2, 3, 4, 5}});
}

BENCHMARK(legacyLinearAlgorithmBench<float>)->Apply(LinearCustomArguments);
BENCHMARK(legacyLinearAlgorithmBench<double>)->Apply(LinearCustomArguments);
BENCHMARK(SOAFloatEigenLinearAlgorithmBench)->Apply(LinearCustomArguments);
BENCHMARK(SOAHighwayLinearAlgorithmBench<float>)
    ->Apply(LinearCustomArgumentsHighway);
BENCHMARK(SOAHighwayLinearAlgorithmBench<double>)
    ->Apply(LinearCustomArgumentsHighway);
BENCHMARK(SOAHighwayLinearWithIndexAlgorithmBench<float>)
    ->Apply(LinearCustomArgumentsHighway);
BENCHMARK(SOAHighwayLinearWithIndexAlgorithmBench<double>)
    ->Apply(LinearCustomArgumentsHighway);
BENCHMARK(legacyLogWarmStartAlgorithmBench<float>)->Apply(LogCustomArguments);
BENCHMARK(legacyLogWarmStartAlgorithmBench<double>)->Apply(LogCustomArguments);

}  // namespace bench
}  // namespace coal

BENCHMARK_MAIN();
