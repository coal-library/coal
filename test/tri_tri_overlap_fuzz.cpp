// Fuzz Möller's tri-tri overlap test against coal's GJK-based ground truth
// across a wide range of randomly generated triangle pairs. The intent is to
// catch sign-handling and interval-computation bugs before the fast path is
// trusted for production motion-validation queries.

#define BOOST_TEST_MODULE COAL_TRI_TRI_OVERLAP_FUZZ
#include <boost/test/included/unit_test.hpp>

#include <random>
#include <iomanip>
#include <iostream>

#include "coal/data_types.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/shape_shape_func.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/internal/tri_tri_overlap.h"

using namespace coal;

namespace {

Vec3s randomVec(std::mt19937_64& rng, Scalar range) {
  std::uniform_real_distribution<Scalar> u(-range, range);
  return Vec3s(u(rng), u(rng), u(rng));
}

bool gjkOverlap(const Vec3s& V0, const Vec3s& V1, const Vec3s& V2,
                const Vec3s& U0, const Vec3s& U1, const Vec3s& U2) {
  TriangleP t1(V0, V1, V2);
  TriangleP t2(U0, U1, U2);
  GJKSolver solver;
  solver.distance_upper_bound = 0;  // boolean mode
  Vec3s p1, p2, normal;
  Scalar d = internal::ShapeShapeDistance<TriangleP, TriangleP>(
      &t1, Transform3s::Identity(), &t2, Transform3s::Identity(), &solver,
      /*compute_penetration=*/false, p1, p2, normal);
  return d <= Scalar(0);
}

}  // namespace

BOOST_AUTO_TEST_CASE(tri_tri_overlap_random_pairs) {
  std::mt19937_64 rng(0x1337beefULL);
  const std::size_t N = 50000;
  std::size_t agree = 0, disagree = 0, coplanar = 0;

  for (std::size_t i = 0; i < N; ++i) {
    // Mix of overlapping and disjoint regimes:
    //  - overlapping: both triangles around origin, range ~1
    //  - near miss: one triangle offset by ~1
    //  - disjoint: one triangle offset by ~5
    Scalar range = Scalar(1);
    Vec3s offset = Vec3s::Zero();
    int regime = static_cast<int>(i % 3);
    if (regime == 1) offset = randomVec(rng, Scalar(1));
    if (regime == 2) offset = randomVec(rng, Scalar(5));

    Vec3s V0 = randomVec(rng, range);
    Vec3s V1 = randomVec(rng, range);
    Vec3s V2 = randomVec(rng, range);
    Vec3s U0 = randomVec(rng, range) + offset;
    Vec3s U1 = randomVec(rng, range) + offset;
    Vec3s U2 = randomVec(rng, range) + offset;

    bool cop = false;
    bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, cop);
    if (cop) {
      ++coplanar;
      continue;
    }

    bool gjk = gjkOverlap(V0, V1, V2, U0, U1, U2);
    if (moller == gjk) {
      ++agree;
    } else {
      ++disagree;
      if (disagree <= 5) {
        std::cerr << std::setprecision(17) << "DISAGREE: moller=" << moller
                  << " gjk=" << gjk << "\n  V0=" << V0.transpose()
                  << "\n  V1=" << V1.transpose() << "\n  V2=" << V2.transpose()
                  << "\n  U0=" << U0.transpose() << "\n  U1=" << U1.transpose()
                  << "\n  U2=" << U2.transpose() << "\n";
      }
    }
  }

  std::cerr << "tri_tri_overlap fuzz: " << agree << " agree / " << disagree
            << " disagree / " << coplanar << " coplanar (out of " << N << ")\n";

  // Coplanar / degenerate cases are expected to fall back; allow them to
  // skip. The signal we care about is that Möller and GJK never disagree on
  // non-degenerate inputs.
  BOOST_CHECK_EQUAL(disagree, 0u);
}
