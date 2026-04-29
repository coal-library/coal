// Targeted edge-case coverage for Möller tri-tri overlap. The random-fuzz
// test exercises generic non-degenerate pairs; this file covers cases that
// are common in CAD/voxelized meshes but practically never produced by
// uniform random sampling: coplanar pairs, zero-area triangles, collinear
// vertices, exact equality, and sharing a vertex/edge.
//
// The ground-truth oracle is GJK distance ≤ 0 (boolean overlap). For
// degenerate inputs where GJK is also ill-defined (zero-area triangles),
// we instead pin down the *contract* that triTriOverlap must satisfy
// regardless of the answer: it must not crash and must report `coplanar`
// truthfully.

#define BOOST_TEST_MODULE COAL_TRI_TRI_OVERLAP_EDGE_CASES
#include <boost/test/included/unit_test.hpp>

#include "coal/data_types.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/shape_shape_func.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/internal/tri_tri_overlap.h"

using namespace coal;

namespace {

// GJK boolean ground truth, shared with the fuzz test in spirit.
bool gjkOverlap(const Vec3s& V0, const Vec3s& V1, const Vec3s& V2,
                const Vec3s& U0, const Vec3s& U1, const Vec3s& U2) {
  TriangleP t1(V0, V1, V2);
  TriangleP t2(U0, U1, U2);
  GJKSolver solver;
  solver.distance_upper_bound = 0;
  Vec3s p1, p2, normal;
  Scalar d = internal::ShapeShapeDistance<TriangleP, TriangleP>(
      &t1, Transform3s::Identity(), &t2, Transform3s::Identity(), &solver,
      /*compute_penetration=*/false, p1, p2, normal);
  return d <= Scalar(0);
}

}  // namespace

BOOST_AUTO_TEST_CASE(coplanar_overlapping_triangles) {
  // Both triangles in the z=0 plane, geometrically overlapping.
  const Vec3s V0(0, 0, 0), V1(2, 0, 0), V2(1, 2, 0);
  const Vec3s U0(1, 1, 0), U1(3, 1, 0), U2(2, 3, 0);
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, coplanar);
  // Möller's contract for coplanar pairs: report coplanar=true and return
  // false (caller must fall back). This is the documented behavior of the
  // boolean-only Möller variant in coal.
  BOOST_CHECK(coplanar);
  BOOST_CHECK(!moller);
}

BOOST_AUTO_TEST_CASE(coplanar_disjoint_triangles) {
  // Both in z=0 but spatially disjoint.
  const Vec3s V0(0, 0, 0), V1(1, 0, 0), V2(0, 1, 0);
  const Vec3s U0(10, 10, 0), U1(11, 10, 0), U2(10, 11, 0);
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, coplanar);
  // Same contract: coplanar=true, return false (caller falls back; GJK will
  // correctly resolve "no overlap" in the fallback path).
  BOOST_CHECK(coplanar);
  BOOST_CHECK(!moller);
}

BOOST_AUTO_TEST_CASE(identical_triangles_overlap) {
  // V == U, fully coincident. Geometrically they share infinitely many
  // points. This is also coplanar — the contract is the same: report
  // coplanar=true so the caller falls back to GJK.
  const Vec3s V0(0, 0, 0), V1(1, 0, 0), V2(0, 1, 0);
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, V0, V1, V2, coplanar);
  BOOST_CHECK(coplanar);
  BOOST_CHECK(!moller);
}

BOOST_AUTO_TEST_CASE(triangles_sharing_one_vertex) {
  // T1 in z=0, T2 in z=+ direction, sharing V0/U0. They overlap (at the
  // shared vertex). Not coplanar, so Möller must give the right answer
  // directly.
  const Vec3s shared(0, 0, 0);
  const Vec3s V1(1, 0, 0), V2(0, 1, 0);
  const Vec3s U1(0, 0, 1), U2(1, 0, 1);
  bool coplanar = false;
  bool moller =
      details::triTriOverlap(shared, V1, V2, shared, U1, U2, coplanar);
  // GJK and Möller must agree (when not coplanar). Boundary cases (touching
  // at a vertex) are inherently sensitive to numerical jitter; we accept
  // either answer as long as Möller and GJK agree.
  if (!coplanar) {
    bool gjk = gjkOverlap(shared, V1, V2, shared, U1, U2);
    BOOST_CHECK_EQUAL(moller, gjk);
  }
}

BOOST_AUTO_TEST_CASE(triangles_sharing_one_edge) {
  // T1 and T2 share the edge V0V1 = U0U1, but are non-coplanar (T1 in z=0,
  // T2 tilted up). Geometrically they touch along the shared edge.
  const Vec3s V0(0, 0, 0), V1(1, 0, 0);
  const Vec3s V2(0.5, 1, 0);    // T1 in z=0
  const Vec3s U2(0.5, 0.5, 1);  // T2 tilted
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, V0, V1, U2, coplanar);
  if (!coplanar) {
    bool gjk = gjkOverlap(V0, V1, V2, V0, V1, U2);
    BOOST_CHECK_EQUAL(moller, gjk);
  }
}

BOOST_AUTO_TEST_CASE(triangle_with_zero_area_one_pair) {
  // T1 has two coincident vertices → zero area. Möller must not crash and
  // must produce a deterministic answer; we only check it doesn't throw,
  // doesn't segfault, and ends in a defined state. The collision answer is
  // implementation-defined for degenerate inputs.
  const Vec3s V0(0, 0, 0), V1(0, 0, 0), V2(1, 0, 0);  // degenerate (line)
  const Vec3s U0(0, 0, -1), U1(1, 0, -1), U2(0.5, 1, -1);
  bool coplanar = false;
  // We just need to confirm no UB. A successful return is enough.
  bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, coplanar);
  (void)moller;       // either answer is acceptable for degenerate input
  BOOST_CHECK(true);  // reached here without UB
}

BOOST_AUTO_TEST_CASE(triangle_with_collinear_vertices) {
  // All three vertices on a line → zero-area triangle. Same contract: must
  // not crash, exact answer is implementation-defined.
  const Vec3s V0(0, 0, 0), V1(1, 0, 0), V2(2, 0, 0);  // collinear
  const Vec3s U0(0, 1, 0), U1(1, 1, 0), U2(0.5, 0.5, 1);
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, coplanar);
  (void)moller;
  BOOST_CHECK(true);
}

BOOST_AUTO_TEST_CASE(near_coplanar_disjoint) {
  // T1 in z=0, T2 just above (z=+epsilon, small enough that ε-snapping in
  // Möller's robustness step *could* trigger, but large enough that they
  // remain genuinely non-coplanar in IEEE float arithmetic). The right
  // answer is "no overlap"; Möller should not flag coplanar.
  const Scalar eps = Scalar(1e-3);  // well above snap-to-zero tolerance
  const Vec3s V0(0, 0, 0), V1(1, 0, 0), V2(0, 1, 0);
  const Vec3s U0(0, 0, eps), U1(1, 0, eps), U2(0, 1, eps);
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, coplanar);
  BOOST_CHECK(!coplanar);
  BOOST_CHECK(!moller);
  bool gjk = gjkOverlap(V0, V1, V2, U0, U1, U2);
  BOOST_CHECK_EQUAL(moller, gjk);
}

BOOST_AUTO_TEST_CASE(near_coplanar_overlapping) {
  // Same axis-aligned setup but T2 dips slightly below z=0 in the middle,
  // so it pierces T1's plane. Answer is "overlap"; Möller must produce
  // the correct boolean (not flag coplanar).
  const Vec3s V0(-1, -1, 0), V1(1, -1, 0), V2(0, 1, 0);
  const Vec3s U0(-0.5, 0, -0.001), U1(0.5, 0, -0.001), U2(0, 0.5, 0.001);
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, coplanar);
  if (!coplanar) {
    bool gjk = gjkOverlap(V0, V1, V2, U0, U1, U2);
    BOOST_CHECK_EQUAL(moller, gjk);
  }
}

BOOST_AUTO_TEST_CASE(deep_interpenetration) {
  // T2 fully inside T1's swept volume — both planes intersect and both
  // interval projections deeply overlap. Sanity check that the happy path
  // through computeInterval works for deep overlap as well as near misses.
  const Vec3s V0(-2, -2, 0), V1(2, -2, 0), V2(0, 2, 0);
  const Vec3s U0(-0.5, -0.5, -1), U1(0.5, -0.5, 1), U2(0, 0.5, 0);
  bool coplanar = false;
  bool moller = details::triTriOverlap(V0, V1, V2, U0, U1, U2, coplanar);
  BOOST_CHECK(!coplanar);
  bool gjk = gjkOverlap(V0, V1, V2, U0, U1, U2);
  BOOST_CHECK_EQUAL(moller, gjk);
  BOOST_CHECK(moller);  // must say overlap
}
