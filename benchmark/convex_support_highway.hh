#ifndef COAL_BENCH_CONVEX_SUPPORT_HIGHWAY_HH
#define COAL_BENCH_CONVEX_SUPPORT_HIGHWAY_HH

#include <hwy/aligned_allocator.h>

#include <Eigen/Core>

namespace coal {
namespace bench {

float support(const float* HWY_RESTRICT x, const float* HWY_RESTRICT y,
              const float* HWY_RESTRICT z, float x_dir, float y_dir,
              float z_dir, std::size_t count);
double support(const double* HWY_RESTRICT x, const double* HWY_RESTRICT y,
               const double* HWY_RESTRICT z, double x_dir, double y_dir,
               double z_dir, std::size_t count);

template <typename Scalar>
struct SOAHighwayAlgorithm {
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  static Algorithm fromPoints(const std::vector<Eigen::Vector3d>& points);
  Scalar support(const Vec3& dir) {
    return coal::bench::support(x.get(), y.get(), z.get(), dir.x(), dir.y(),
                                dir.z(), count);
  }

  hwy::AlignedFreeUniquePtr<Scalar[]> x, y, z;
  std::size_t count;
};

}  // namespace bench
}  // namespace coal

#endif  // ifndef COAL_BENCH_CONVEX_SUPPORT_HIGHWAY_HH
