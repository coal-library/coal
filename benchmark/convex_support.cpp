// TODO
// code support function with SOA + highway

#include <hwy/targets.h>
#include <hwy/aligned_allocator.h>

#include <benchmark/benchmark.h>

#include <Eigen/Core>

#include <limits>
#include <vector>
#include <cmath>

#include "utils/icosahedron.hh"

namespace coal {
namespace bench {

template <typename _Scalar>
struct LegacyAlgorithm {
  using Scalar = _Scalar;
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = LegacyAlgorithm;

  static Algorithm fromIcosahedron(const utils::Icosahedron& ico) {
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

  static Algorithm fromIcosahedron(const utils::Icosahedron& ico) {
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

float support(const float* HWY_RESTRICT x, const float* HWY_RESTRICT y,
              const float* HWY_RESTRICT z, float x_dir, float y_dir,
              float z_dir, std::size_t count);
float support_with_target(const float* HWY_RESTRICT x,
                          const float* HWY_RESTRICT y,
                          const float* HWY_RESTRICT z, float x_dir, float y_dir,
                          float z_dir, std::size_t count, std::int64_t target);
double support(const double* HWY_RESTRICT x, const double* HWY_RESTRICT y,
               const double* HWY_RESTRICT z, double x_dir, double y_dir,
               double z_dir, std::size_t count);
double support_with_target(const double* HWY_RESTRICT x,
                           const double* HWY_RESTRICT y,
                           const double* HWY_RESTRICT z, double x_dir,
                           double y_dir, double z_dir, std::size_t count,
                           std::int64_t target);

template <typename Scalar>
struct SOAHighwayAlgorithm {
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  static Algorithm fromIcosahedron(const utils::Icosahedron& ico) {
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
    return support_with_target(x.get(), y.get(), z.get(), dir.x(), dir.y(),
                               dir.z(), count, target);
  }

  hwy::AlignedFreeUniquePtr<Scalar[]> x, y, z;
  std::size_t count;
};

template <typename Scalar>
static void legacyAlgorithmBench(benchmark::State& state) {
  using Algorithm = LegacyAlgorithm<Scalar>;
  auto ico =
      utils::IcosahedronDatabase::get(static_cast<std::size_t>(state.range(0)));
  auto algo = Algorithm::fromIcosahedron(ico);
  auto vec = Algorithm::Vec3::UnitX();
  for (auto _ : state) {
    auto res = algo.support(vec);
    benchmark::DoNotOptimize(res);
  }
}

static void SOAFloatEigenAlgorithmBench(benchmark::State& state) {
  using Algorithm = SOAFloatEigenAlgorithm;
  auto ico =
      utils::IcosahedronDatabase::get(static_cast<std::size_t>(state.range(0)));
  auto algo = Algorithm::fromIcosahedron(ico);
  auto vec = Algorithm::Vec3::UnitX();
  for (auto _ : state) {
    auto res = algo.support(vec);
    benchmark::DoNotOptimize(res);
  }
}

template <typename Scalar>
static void SOAHighwayAlgorithmBench(benchmark::State& state) {
  using Algorithm = SOAHighwayAlgorithm<Scalar>;
  auto ico =
      utils::IcosahedronDatabase::get(static_cast<std::size_t>(state.range(1)));
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
BENCHMARK(SOAHighwayAlgorithmBench<float>)->Apply(CustomArgumentsHighway);
BENCHMARK(SOAHighwayAlgorithmBench<double>)->Apply(CustomArgumentsHighway);

}  // namespace bench
}  // namespace coal

BENCHMARK_MAIN();
