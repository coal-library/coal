#define BOOST_TEST_MODULE COAL_CAPSULE_BOX_DISTANCE
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <random>

#include "coal/collision.h"
#include "coal/internal/shape_shape_func.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"

#include "utility.h"

using namespace coal;
using Vec4s = Eigen::Matrix<Scalar, 4, 1>;

namespace {

const Scalar tol = Scalar(100) * std::sqrt(Eigen::NumTraits<Scalar>::epsilon());
// Normals of the GJK/EPA reference are less accurate than its distances.
const Scalar normal_tol =
    Scalar(20) * std::cbrt(Eigen::NumTraits<Scalar>::epsilon());
// Checks that do not depend on GJK/EPA are tight in both precisions. In float,
// GJK/EPA is off by up to 1e-2 on deep penetrations, hence the loose tol above.
const Scalar witness_tol = Scalar(400) * Eigen::NumTraits<Scalar>::epsilon();

struct Witness {
  Scalar distance;
  Vec3s p1, p2, normal;
};

// Specialized capsule-box distance.
template <typename S1, typename S2>
Witness specialized(const S1& s1, const Transform3s& tf1, const S2& s2,
                    const Transform3s& tf2) {
  Witness w;
  w.distance = internal::ShapeShapeDistance<S1, S2>(&s1, tf1, &s2, tf2, nullptr,
                                                    true, w.p1, w.p2, w.normal);
  return w;
}

// Signed distance from a point to the surface of a box, in the box frame.
Scalar boxSignedDistance(const Box& box, const Vec3s& p) {
  const Vec3s q = p.cwiseAbs() - box.halfSide;
  return q.cwiseMax(0).norm() + (std::min)(q.maxCoeff(), Scalar(0));
}

// Distance from a point to the segment of a capsule, in the capsule frame.
Scalar distanceToCapsuleSegment(const Capsule& capsule, const Vec3s& p) {
  const Scalar z =
      (std::max)(-capsule.halfLength, (std::min)(capsule.halfLength, p[2]));
  return (p - Vec3s(0, 0, z)).norm();
}

void checkWitness(const Box& box, const Transform3s& tf_box,
                  const Capsule& capsule, const Transform3s& tf_capsule,
                  const Witness& w) {
  // w.p1 is on the box, w.p2 on the capsule, normal from box to capsule.
  BOOST_CHECK_SMALL(w.normal.norm() - 1, witness_tol);
  BOOST_CHECK(((w.p2 - w.p1) - w.distance * w.normal).isZero(witness_tol));
  BOOST_CHECK_SMALL(boxSignedDistance(box, tf_box.inverseTransform(w.p1)),
                    witness_tol);
  BOOST_CHECK_SMALL(
      distanceToCapsuleSegment(capsule, tf_capsule.inverseTransform(w.p2)) -
          capsule.radius,
      witness_tol);

  // The normal separates the shapes by the returned distance: the gap between
  // their supports along it equals the distance. As the signed distance is the
  // largest such gap over all directions, this bounds it from below, and, when
  // separated, the witness points bound it from above.
  const Vec3s& n = w.normal;
  const Scalar box_support =
      n.dot(tf_box.getTranslation()) +
      (tf_box.getRotation().transpose() * n).cwiseAbs().dot(box.halfSide);
  const Scalar capsule_support =
      n.dot(tf_capsule.getTranslation()) -
      capsule.halfLength * std::abs(n.dot(tf_capsule.getRotation().col(2))) -
      capsule.radius;
  BOOST_CHECK_SMALL(capsule_support - box_support - w.distance, witness_tol);
}

}  // namespace

BOOST_AUTO_TEST_CASE(capsule_box_matches_gjk_epa) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> U(0, 1);

  // GJK/EPA with tight tolerances, used as reference.
  GJKSolver solver;
  solver.gjk_tolerance = Scalar(1e-10);
  solver.epa_tolerance = Scalar(1e-10);
  solver.gjk_max_iterations = 256;
  solver.epa_max_iterations = 256;

  const int num_tests = 20000;
  int num_penetrating = 0;
  for (int i = 0; i < num_tests; ++i) {
    const Box box(Scalar(0.1 + U(rng)), Scalar(0.1 + U(rng)),
                  Scalar(0.1 + U(rng)));
    const Capsule capsule(Scalar(0.01 + 0.2 * U(rng)), Scalar(1.5 * U(rng)));
    const Transform3s tf_box(
        Quats(Vec4s(Vec4s::Random()).normalized()).toRotationMatrix(),
        Vec3s::Random());
    const Transform3s tf_capsule(
        Quats(Vec4s(Vec4s::Random()).normalized()).toRotationMatrix(),
        tf_box.getTranslation() + Vec3s::Random());

    const Witness w = specialized(box, tf_box, capsule, tf_capsule);
    checkWitness(box, tf_box, capsule, tf_capsule, w);

    Witness ref;
    ref.distance = solver.shapeDistance(box, tf_box, capsule, tf_capsule, true,
                                        ref.p1, ref.p2, ref.normal);
    if (solver.gjk.status == details::GJK::Failed ||
        (solver.epa.status != details::EPA::DidNotRun &&
         !(solver.epa.status & details::EPA::Valid)))
      continue;

    BOOST_CHECK_SMALL(w.distance - ref.distance, tol);
    // The normal is unique unless the shapes are (nearly) touching.
    if (std::abs(w.distance) > tol)
      BOOST_CHECK_SMALL((w.normal - ref.normal).norm(), normal_tol);
    if (w.distance < 0) ++num_penetrating;

    // The reversed pair returns the same witness points, swapped, and the
    // opposite normal.
    const Witness w_rev = specialized(capsule, tf_capsule, box, tf_box);
    BOOST_CHECK_EQUAL(w_rev.distance, w.distance);
    BOOST_CHECK(w_rev.p1.isApprox(w.p2));
    BOOST_CHECK(w_rev.p2.isApprox(w.p1));
    BOOST_CHECK(w_rev.normal.isApprox(-w.normal));
  }
  // Both separated and penetrating configurations must have been tested.
  BOOST_CHECK(num_penetrating > num_tests / 10);
  BOOST_CHECK(num_penetrating < num_tests - num_tests / 10);
}

BOOST_AUTO_TEST_CASE(capsule_box_collide_uses_specialization) {
  const Box box(2, 2, 2);
  const Capsule capsule(Scalar(0.5), 2);
  // Capsule standing on the top face of the box, penetrating by 0.1.
  const Transform3s tf_capsule(Vec3s(Scalar(0.3), Scalar(-0.2), Scalar(2.4)));

  CollisionRequest request;
  CollisionResult result;
  BOOST_CHECK_EQUAL(
      collide(&box, Transform3s(), &capsule, tf_capsule, request, result), 1);
  const Contact& contact = result.getContact(0);
  BOOST_CHECK_SMALL(contact.penetration_depth + Scalar(0.1), tol);
  BOOST_CHECK(contact.normal.isApprox(Vec3s::UnitZ()));
  BOOST_CHECK(contact.nearest_points[0].isApprox(
      Vec3s(Scalar(0.3), Scalar(-0.2), Scalar(1))));
  BOOST_CHECK(contact.nearest_points[1].isApprox(
      Vec3s(Scalar(0.3), Scalar(-0.2), Scalar(0.9))));
}

BOOST_AUTO_TEST_CASE(capsule_box_segment_parallel_to_face) {
  const Box box(2, 2, 2);
  const Capsule capsule(Scalar(0.5), 4);
  // Capsule lying along x across the top face, its segment 0.2 below the face:
  // the witness points lie in the middle of the part of the segment that is
  // above the face.
  const Transform3s tf_capsule(
      makeQuat(std::sqrt(Scalar(2)) / 2, 0, std::sqrt(Scalar(2)) / 2, 0),
      Vec3s(Scalar(1.5), Scalar(0.4), Scalar(0.8)));

  const Witness w = specialized(box, Transform3s(), capsule, tf_capsule);
  checkWitness(box, Transform3s(), capsule, tf_capsule, w);
  BOOST_CHECK_SMALL(w.distance + Scalar(0.7), tol);
  BOOST_CHECK(w.normal.isApprox(Vec3s::UnitZ()));
  BOOST_CHECK(w.p1.isApprox(Vec3s(Scalar(0.25), Scalar(0.4), Scalar(1))));
}

BOOST_AUTO_TEST_CASE(capsule_box_edge_separating_axis) {
  const Box box(2, 2, 2);
  const Capsule capsule(Scalar(0.1), 10);
  // Segment in the plane x + z = 1.8, crossing the box edge {x = 1, z = 1}
  // orthogonally: the separating axis of minimal overlap is (1, 0, 1)/sqrt(2).
  const Transform3s tf_capsule(makeQuat(std::cos(Scalar(3 * M_PI / 8)), 0,
                                        std::sin(Scalar(3 * M_PI / 8)), 0),
                               Vec3s(Scalar(0.9), Scalar(0.3), Scalar(0.9)));

  const Witness w = specialized(box, Transform3s(), capsule, tf_capsule);
  checkWitness(box, Transform3s(), capsule, tf_capsule, w);
  BOOST_CHECK_SMALL(
      w.distance + Scalar(0.2) / std::sqrt(Scalar(2)) + capsule.radius, tol);
  BOOST_CHECK(w.normal.isApprox(Vec3s(1, 0, 1).normalized()));
  BOOST_CHECK(w.p1.isApprox(Vec3s(1, Scalar(0.3), 1)));
}

BOOST_AUTO_TEST_CASE(capsule_box_zero_length_matches_sphere) {
  const Box box(Scalar(0.4), Scalar(0.6), Scalar(0.8));
  const Capsule capsule(Scalar(0.2), 0);
  const Sphere sphere(Scalar(0.2));
  for (int i = 0; i < 1000; ++i) {
    const Transform3s tf_box(
        Quats(Vec4s(Vec4s::Random()).normalized()).toRotationMatrix(),
        Vec3s::Random());
    const Transform3s tf_capsule(
        Quats(Vec4s(Vec4s::Random()).normalized()).toRotationMatrix(),
        tf_box.getTranslation() + Scalar(0.6) * Vec3s::Random());

    const Witness w = specialized(box, tf_box, capsule, tf_capsule);
    const Witness ref = specialized(box, tf_box, sphere, tf_capsule);
    BOOST_CHECK_SMALL(w.distance - ref.distance, tol);
    BOOST_CHECK(w.normal.isApprox(ref.normal, tol));
    BOOST_CHECK(w.p1.isApprox(ref.p1, tol));
    BOOST_CHECK(w.p2.isApprox(ref.p2, tol));
  }
}

BOOST_AUTO_TEST_CASE(capsule_box_swept_sphere_radius) {
  Box box(Scalar(0.4), Scalar(0.6), Scalar(0.8));
  Capsule capsule(Scalar(0.1), Scalar(0.5));
  const Transform3s tf_box(
      Quats(Vec4s(1, 2, 3, 4).normalized()).toRotationMatrix(), Vec3s::Zero());
  const Transform3s tf_capsule(
      Quats(Vec4s(4, 3, 2, 1).normalized()).toRotationMatrix(),
      Vec3s(Scalar(0.3), Scalar(0.4), Scalar(0.5)));

  const Witness w = specialized(box, tf_box, capsule, tf_capsule);
  box.setSweptSphereRadius(Scalar(0.03));
  capsule.setSweptSphereRadius(Scalar(0.02));
  const Witness w_inflated = specialized(box, tf_box, capsule, tf_capsule);

  BOOST_CHECK_SMALL(w_inflated.distance - (w.distance - Scalar(0.05)), tol);
  BOOST_CHECK(w_inflated.normal.isApprox(w.normal));
  BOOST_CHECK(w_inflated.p1.isApprox(w.p1 + Scalar(0.03) * w.normal));
  BOOST_CHECK(w_inflated.p2.isApprox(w.p2 - Scalar(0.02) * w.normal));
}
