#ifndef COAL_BENCH_CONVEX_SUPPORT_HIGHWAY_HH
#define COAL_BENCH_CONVEX_SUPPORT_HIGHWAY_HH

#include "utils/icosahedron.hh"

#include <hwy/aligned_allocator.h>

#include <Eigen/Core>

#include <cstdint>

namespace coal {
namespace bench {

float support(const float* HWY_RESTRICT x, const float* HWY_RESTRICT y,
              const float* HWY_RESTRICT z, float x_dir, float y_dir,
              float z_dir, std::size_t count);
float supportWithTarget(const float* HWY_RESTRICT x,
                        const float* HWY_RESTRICT y,
                        const float* HWY_RESTRICT z, float x_dir, float y_dir,
                        float z_dir, std::size_t count, std::int64_t target);
double support(const double* HWY_RESTRICT x, const double* HWY_RESTRICT y,
               const double* HWY_RESTRICT z, double x_dir, double y_dir,
               double z_dir, std::size_t count);
double supportWithTarget(const double* HWY_RESTRICT x,
                         const double* HWY_RESTRICT y,
                         const double* HWY_RESTRICT z, double x_dir,
                         double y_dir, double z_dir, std::size_t count,
                         std::int64_t target);

template <typename Scalar>
struct SOAHighwayAlgorithm {
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  static Algorithm fromIcosahedron(const utils::Icosahedron& ico);
  Scalar support(const Vec3& dir, std::int64_t target) {
    return supportWithTarget(x.get(), y.get(), z.get(), dir.x(), dir.y(),
                             dir.z(), count, target);
  }

  hwy::AlignedFreeUniquePtr<Scalar[]> x, y, z;
  std::size_t count;
};

}  // namespace bench
}  // namespace coal

#endif  // ifndef COAL_BENCH_CONVEX_SUPPORT_HIGHWAY_HH
