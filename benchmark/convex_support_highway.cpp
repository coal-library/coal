#include <cstdint>
#include <limits>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "convex_support_highway.cpp"
#include <hwy/foreach_target.h>

#include <hwy/highway.h>
#include <hwy/aligned_allocator.h>

HWY_BEFORE_NAMESPACE();
namespace coal {
namespace bench {
namespace HWY_NAMESPACE {
namespace {

namespace hn = hwy::HWY_NAMESPACE;

template <typename Scalar>
Scalar _support(const Scalar* HWY_RESTRICT x, const Scalar* HWY_RESTRICT y,
                const Scalar* HWY_RESTRICT z, Scalar x_dir, Scalar y_dir,
                Scalar z_dir, std::size_t count) {
  const hn::ScalableTag<Scalar> d;
  const size_t N = hn::Lanes(d);
  using V = decltype(hn::Zero(d));

  V x_dir_v = hn::Set(d, x_dir);
  V y_dir_v = hn::Set(d, y_dir);
  V z_dir_v = hn::Set(d, z_dir);

  auto func = [&](std::size_t i) -> V {
    const V x_v_ilp1 = hn::Load(d, x + i);
    const V dot_v1_ilp1 = x_v_ilp1 * x_dir_v;
    const V y_v_ilp1 = hn::Load(d, y + i);
    const V dot_v2_ilp1 = hn::MulAdd(y_v_ilp1, y_dir_v, dot_v1_ilp1);
    const V z_v_ilp1 = hn::Load(d, z + i);
    return hn::MulAdd(z_v_ilp1, z_dir_v, dot_v2_ilp1);
  };

  V max_dot_v_ilp1 = hn::Set(d, std::numeric_limits<Scalar>::min());
  V max_dot_v_ilp2 = hn::Set(d, std::numeric_limits<Scalar>::min());
  size_t i = 0;
  for (; i + N * 2 <= count; i += N * 2) {
    const V dot_v_ilp1 = func(i);
    max_dot_v_ilp1 = hn::Max(max_dot_v_ilp1, dot_v_ilp1);

    const V dot_v_ilp2 = func(i + 1 * N);
    max_dot_v_ilp2 = hn::Max(max_dot_v_ilp2, dot_v_ilp2);
  }
  V max_dot_v = hn::Max(max_dot_v_ilp1, max_dot_v_ilp2);

  for (; i + N <= count; i += N) {
    const V x_v = hn::Load(d, x + i);
    const V dot_v1 = x_v * x_dir_v;
    const V y_v = hn::Load(d, y + i);
    const V dot_v2 = hn::MulAdd(y_v, y_dir_v, dot_v1);
    const V z_v = hn::Load(d, z + i);
    const V dot_v = hn::MulAdd(z_v, z_dir_v, dot_v2);

    max_dot_v = hn::Max(max_dot_v, dot_v);
  }

  Scalar max_dot = hn::ReduceMax(d, max_dot_v);
  for (; i < count; ++i) {
    const Scalar dot = x[i] * x_dir + y[i] * y_dir + z[i] * z_dir;
    max_dot = std::max(max_dot, dot);
  }

  return max_dot;
}

float _support_float(const float* HWY_RESTRICT x, const float* HWY_RESTRICT y,
                     const float* HWY_RESTRICT z, float x_dir, float y_dir,
                     float z_dir, std::size_t count) {
  return _support(x, y, z, x_dir, y_dir, z_dir, count);
}
double _support_double(const double* HWY_RESTRICT x,
                       const double* HWY_RESTRICT y,
                       const double* HWY_RESTRICT z, double x_dir, double y_dir,
                       double z_dir, std::size_t count) {
  return _support(x, y, z, x_dir, y_dir, z_dir, count);
}

}  // namespace
}  // namespace HWY_NAMESPACE
}  // namespace bench
}  // namespace coal
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace coal {
namespace bench {

HWY_EXPORT(_support_float);
HWY_EXPORT(_support_double);

float support(const float* HWY_RESTRICT x, const float* HWY_RESTRICT y,
              const float* HWY_RESTRICT z, float x_dir, float y_dir,
              float z_dir, std::size_t count) {
  return HWY_DYNAMIC_DISPATCH(_support_float)(x, y, z, x_dir, y_dir, z_dir,
                                              count);
}
float support_with_target(const float* HWY_RESTRICT x,
                          const float* HWY_RESTRICT y,
                          const float* HWY_RESTRICT z, float x_dir, float y_dir,
                          float z_dir, std::size_t count, std::int64_t target) {
  hwy::SetSupportedTargetsForTest(target);
  return HWY_DYNAMIC_DISPATCH(_support_float)(x, y, z, x_dir, y_dir, z_dir,
                                              count);
  hwy::SetSupportedTargetsForTest(0);
}
double support(const double* HWY_RESTRICT x, const double* HWY_RESTRICT y,
               const double* HWY_RESTRICT z, double x_dir, double y_dir,
               double z_dir, std::size_t count) {
  return HWY_DYNAMIC_DISPATCH(_support_double)(x, y, z, x_dir, y_dir, z_dir,
                                               count);
}
double support_with_target(const double* HWY_RESTRICT x,
                           const double* HWY_RESTRICT y,
                           const double* HWY_RESTRICT z, double x_dir,
                           double y_dir, double z_dir, std::size_t count,
                           std::int64_t target) {
  hwy::SetSupportedTargetsForTest(target);
  return HWY_DYNAMIC_DISPATCH(_support_double)(x, y, z, x_dir, y_dir, z_dir,
                                               count);
  hwy::SetSupportedTargetsForTest(0);
}
}  // namespace bench
}  // namespace coal

#endif  // if HWY_ONCE
