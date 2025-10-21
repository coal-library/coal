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
SOAHighwayAlgorithm<Scalar> _fromPoints(
    const std::vector<Eigen::Vector3d>& points) {
  using Algorithm = SOAHighwayAlgorithm<Scalar>;

  const hn::ScalableTag<Scalar> d;
  const std::size_t N = hn::Lanes(d);
  const std::size_t remainder = points.size() % N;
  const std::size_t padded_size = points.size() + (N - remainder);

  Algorithm algo;
  algo.x = hwy::AllocateAligned<Scalar>(padded_size * 3);
  algo.y = algo.x.get() + padded_size;
  algo.z = algo.x.get() + 2 * padded_size;
  algo.count = padded_size;

  for (std::size_t i = 0; i < points.size(); ++i) {
    algo.x[i] = static_cast<Scalar>(points[i].x());
    algo.y[i] = static_cast<Scalar>(points[i].y());
    algo.z[i] = static_cast<Scalar>(points[i].z());
  }
  for (std::size_t i = points.size(); i < padded_size; ++i) {
    algo.x[i] = static_cast<Scalar>(points.back().x());
    algo.y[i] = static_cast<Scalar>(points.back().y());
    algo.z[i] = static_cast<Scalar>(points.back().z());
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

template <typename Scalar>
std::tuple<Scalar, std::size_t> _supportWithIndex(const Scalar* HWY_RESTRICT x,
                                                  const Scalar* HWY_RESTRICT y,
                                                  const Scalar* HWY_RESTRICT z,
                                                  Scalar x_dir, Scalar y_dir,
                                                  Scalar z_dir,
                                                  std::size_t count) {
  using DF = hn::ScalableTag<Scalar>;
  const DF d;
  const size_t N = hn::Lanes(d);
  using V = decltype(hn::Zero(d));

  // Create unsigned integer Vector type with same number
  // of lanes than DF
  const hn::RebindToUnsigned<DF> dui;
  using VUI = decltype(hn::Zero(dui));

  const V x_dir_v = hn::Set(d, x_dir);
  const V y_dir_v = hn::Set(d, y_dir);
  const V z_dir_v = hn::Set(d, z_dir);

  const VUI indices_increment_v = hn::Set(dui, 2 * N);

  // Compute dot product
  auto dot_product = [&](std::size_t i) -> V {
    const V x_v_ = hn::Load(d, x + i);
    const V dot_v1_ = x_v_ * x_dir_v;
    const V y_v_ = hn::Load(d, y + i);
    const V dot_v2_ = hn::MulAdd(y_v_, y_dir_v, dot_v1_);
    const V z_v_ = hn::Load(d, z + i);
    return hn::MulAdd(z_v_, z_dir_v, dot_v2_);
  };

  // Extract max dot product and corresponding max indices
  auto max_dot_and_indices =
      [&dui](const V& dot_v, const VUI& indices_v, const V& max_dot_v,
             const VUI& max_indices_v) -> std::tuple<V, VUI> {
    const auto mask_v = max_dot_v > dot_v;
    // Max give better perf than using the mask
    const V max_dot_v_tmp = hn::Max(max_dot_v, dot_v);
    const VUI max_indices_v_tmp =
        hn::IfThenElse(hn::RebindMask(dui, mask_v), max_indices_v, indices_v);
    return std::make_tuple(max_dot_v_tmp, max_indices_v_tmp);
  };

  V max_dot_v_ilp1 = hn::Set(d, std::numeric_limits<Scalar>::min());
  V max_dot_v_ilp2 = hn::Set(d, std::numeric_limits<Scalar>::min());
  // Initialize indices with right index (0, 1, 2, ...)
  VUI indices_v_ilp1 = hn::Iota(dui, 0);
  VUI indices_v_ilp2 = hn::Iota(dui, N);
  VUI max_indices_v_ilp1 = hn::Set(dui, 0);
  VUI max_indices_v_ilp2 = hn::Set(dui, 0);
  size_t i = 0;
  // First pass with 2 ILP
  for (; i + N * 2 <= count; i += N * 2) {
    const V dot_v_ilp1 = dot_product(i);
    std::tie(max_dot_v_ilp1, max_indices_v_ilp1) = max_dot_and_indices(
        dot_v_ilp1, indices_v_ilp1, max_dot_v_ilp1, max_indices_v_ilp1);
    indices_v_ilp1 += indices_increment_v;

    const V dot_v_ilp2 = dot_product(i + 1 * N);
    std::tie(max_dot_v_ilp2, max_indices_v_ilp2) = max_dot_and_indices(
        dot_v_ilp2, indices_v_ilp2, max_dot_v_ilp2, max_indices_v_ilp2);
    indices_v_ilp2 += indices_increment_v;
  }
  // Merge first  pass result
  auto [max_dot_v, max_indices_v] = max_dot_and_indices(
      max_dot_v_ilp1, max_indices_v_ilp1, max_dot_v_ilp2, max_indices_v_ilp2);

  // Second pass with the remaining batch
  if (i + N <= count) {
    const V dot_v = dot_product(i);
    std::tie(max_dot_v, max_indices_v) =
        max_dot_and_indices(dot_v, indices_v_ilp1, max_dot_v, max_indices_v);
  }

  // Extract max scalar dot product and corresponding max indices
  Scalar max_dot = hn::ReduceMax(d, max_dot_v);
  const auto max_scalar_mask_v = max_dot_v == hn::Set(d, max_dot);
  std::size_t best_index = hn::FindKnownFirstTrue(d, max_scalar_mask_v);
  return std::make_tuple(max_dot, hn::ExtractLane(max_indices_v, best_index));
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
HWY_EXPORT_T(_supportWithIndexFloat, _supportWithIndex<float>);
HWY_EXPORT_T(_supportWithIndexDouble, _supportWithIndex<double>);
HWY_EXPORT_T(_fromPointsFloat, _fromPoints<float>);
HWY_EXPORT_T(_fromPointsDouble, _fromPoints<double>);

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

std::tuple<float, std::size_t> supportWithIndex(const float* HWY_RESTRICT x,
                                                const float* HWY_RESTRICT y,
                                                const float* HWY_RESTRICT z,
                                                float x_dir, float y_dir,
                                                float z_dir,
                                                std::size_t count) {
  return HWY_DYNAMIC_DISPATCH_T(_supportWithIndexFloat)(x, y, z, x_dir, y_dir,
                                                        z_dir, count);
}
std::tuple<double, std::size_t> supportWithIndex(const double* HWY_RESTRICT x,
                                                 const double* HWY_RESTRICT y,
                                                 const double* HWY_RESTRICT z,
                                                 double x_dir, double y_dir,
                                                 double z_dir,
                                                 std::size_t count) {
  return HWY_DYNAMIC_DISPATCH_T(_supportWithIndexDouble)(x, y, z, x_dir, y_dir,
                                                         z_dir, count);
}

template <>
SOAHighwayAlgorithm<float> SOAHighwayAlgorithm<float>::fromPoints(
    const std::vector<Eigen::Vector3d>& points) {
  return HWY_DYNAMIC_DISPATCH_T(_fromPointsFloat)(points);
}

template <>
SOAHighwayAlgorithm<double> SOAHighwayAlgorithm<double>::fromPoints(
    const std::vector<Eigen::Vector3d>& points) {
  return HWY_DYNAMIC_DISPATCH_T(_fromPointsDouble)(points);
}

}  // namespace bench
}  // namespace coal

#endif  // if HWY_ONCE
