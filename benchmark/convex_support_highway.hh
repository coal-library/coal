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
std::tuple<float, std::size_t> supportWithIndex(const float* HWY_RESTRICT x,
                                                const float* HWY_RESTRICT y,
                                                const float* HWY_RESTRICT z,
                                                float x_dir, float y_dir,
                                                float z_dir, std::size_t count);
std::tuple<double, std::size_t> supportWithIndex(const double* HWY_RESTRICT x,
                                                 const double* HWY_RESTRICT y,
                                                 const double* HWY_RESTRICT z,
                                                 double x_dir, double y_dir,
                                                 double z_dir,
                                                 std::size_t count);

template <typename Scalar>
struct SOAHighwayAlgorithm {
  using Vec3 = Eigen::Vector<Scalar, 3>;
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  static Algorithm fromPoints(const std::vector<Eigen::Vector3d>& points);
  Scalar support(const Vec3& dir) const {
    return coal::bench::support(x.get(), y, z, dir.x(), dir.y(), dir.z(),
                                count);
  }
  std::tuple<Scalar, std::size_t> supportWithIndex(const Vec3& dir) const {
    return coal::bench::supportWithIndex(x.get(), y, z, dir.x(), dir.y(),
                                         dir.z(), count);
  }

  // Allocate all in data in x. y and z are pointing inside x buffer.
  // Only work if count is a multiple of Lanes (misalignment or load split)
  hwy::AlignedFreeUniquePtr<Scalar[]> x;
  Scalar *y, *z;
  std::size_t count;
};

}  // namespace bench
}  // namespace coal

#endif  // ifndef COAL_BENCH_CONVEX_SUPPORT_HIGHWAY_HH
