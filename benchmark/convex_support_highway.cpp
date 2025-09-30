#include "convex_support_highway.hh"

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
SOAHighwayAlgorithm<Scalar> _fromIcosahedron(const utils::Icosahedron& ico) {
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  const hn::ScalableTag<Scalar> d;
  const std::size_t N = hn::Lanes(d);
  const std::size_t remainder = ico.points.size() % N;
  const std::size_t padded_size = ico.points.size() + remainder;

  Algorithm algo;
  algo.x = hwy::AllocateAligned<Scalar>(padded_size);
  algo.y = hwy::AllocateAligned<Scalar>(padded_size);
  algo.z = hwy::AllocateAligned<Scalar>(padded_size);
  algo.count = padded_size;

  for (std::size_t i = 0; i < ico.points.size(); ++i) {
    algo.x[i] = static_cast<Scalar>(ico.points[i].x());
    algo.y[i] = static_cast<Scalar>(ico.points[i].y());
    algo.z[i] = static_cast<Scalar>(ico.points[i].z());
  }
  for (std::size_t i = ico.points.size(); i < padded_size; ++i) {
    algo.x[i] = static_cast<Scalar>(ico.points.back().x());
    algo.y[i] = static_cast<Scalar>(ico.points.back().y());
    algo.z[i] = static_cast<Scalar>(ico.points.back().z());
  }
  return algo;
}

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
    const V x_v_ = hn::Load(d, x + i);
    const V dot_v1_ = x_v_ * x_dir_v;
    const V y_v_ = hn::Load(d, y + i);
    const V dot_v2_ = hn::MulAdd(y_v_, y_dir_v, dot_v1_);
    const V z_v_ = hn::Load(d, z + i);
    return hn::MulAdd(z_v_, z_dir_v, dot_v2_);
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
    const V dot_v = func(i);
    max_dot_v = hn::Max(max_dot_v, dot_v);
  }

  return hn::ReduceMax(d, max_dot_v);
}

}  // namespace
}  // namespace HWY_NAMESPACE
}  // namespace bench
}  // namespace coal
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace coal {
namespace bench {

namespace {

HWY_EXPORT_T(_supportFloat, _support<float>);
HWY_EXPORT_T(_supportDouble, _support<double>);
HWY_EXPORT_T(_fromIcosahedronFloat, _fromIcosahedron<float>);
HWY_EXPORT_T(_fromIcosahedronDouble, _fromIcosahedron<double>);

}  // namespace

float support(const float* HWY_RESTRICT x, const float* HWY_RESTRICT y,
              const float* HWY_RESTRICT z, float x_dir, float y_dir,
              float z_dir, std::size_t count) {
  return HWY_DYNAMIC_DISPATCH_T(_supportFloat)(x, y, z, x_dir, y_dir, z_dir,
                                               count);
}
double support(const double* HWY_RESTRICT x, const double* HWY_RESTRICT y,
               const double* HWY_RESTRICT z, double x_dir, double y_dir,
               double z_dir, std::size_t count) {
  return HWY_DYNAMIC_DISPATCH_T(_supportDouble)(x, y, z, x_dir, y_dir, z_dir,
                                                count);
}

template <>
SOAHighwayAlgorithm<float> SOAHighwayAlgorithm<float>::fromIcosahedron(
    const utils::Icosahedron& ico) {
  return HWY_DYNAMIC_DISPATCH_T(_fromIcosahedronFloat)(ico);
}

template <>
SOAHighwayAlgorithm<double> SOAHighwayAlgorithm<double>::fromIcosahedron(
    const utils::Icosahedron& ico) {
  return HWY_DYNAMIC_DISPATCH_T(_fromIcosahedronDouble)(ico);
}

}  // namespace bench
}  // namespace coal

#endif  // if HWY_ONCE
