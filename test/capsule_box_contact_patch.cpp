#define BOOST_TEST_MODULE COAL_CAPSULE_BOX_CONTACT_PATCH
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <random>

#include "coal/collision.h"
#include "coal/contact_patch.h"
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

template <typename S1, typename S2>
ContactPatch computePatch(const S1& s1, const Transform3s& tf1, const S2& s2,
                          const Transform3s& tf2,
                          const CollisionResult& result) {
  const ContactPatchRequest request;
  ContactPatchResult patch_result(request);
  computeContactPatch(&s1, tf1, &s2, tf2, result, request, patch_result);
  BOOST_REQUIRE_EQUAL(patch_result.numContactPatches(), 1);
  return patch_result.getContactPatch(0);
}

bool hasPoint(const ContactPatch& patch, const Vec3s& p, Scalar precision) {
  for (size_t i = 0; i < patch.size(); ++i)
    if ((patch.getPoint(i) - p).norm() <= precision) return true;
  return false;
}

}  // namespace

BOOST_AUTO_TEST_CASE(capsule_lying_on_face) {
  const Box box(2, 2, 2);
  const Capsule capsule(Scalar(0.25), 1);
  // Capsule along x, over the top face of the box and penetrating it by 0.01:
  // its two ends are in the patch.
  const Scalar z = Scalar(1) + Scalar(0.25) - Scalar(0.01);
  const Transform3s tf_capsule(
      makeQuat(std::sqrt(Scalar(2)) / 2, 0, std::sqrt(Scalar(2)) / 2, 0),
      Vec3s(Scalar(0.1), Scalar(0.2), z));

  for (const bool box_first : {true, false}) {
    CollisionRequest request(CollisionRequestFlag::CONTACT, 1);
    CollisionResult result;
    if (box_first)
      collide(&box, Transform3s(), &capsule, tf_capsule, request, result);
    else
      collide(&capsule, tf_capsule, &box, Transform3s(), request, result);
    BOOST_REQUIRE(result.isCollision());

    const ContactPatchRequest patch_request;
    ContactPatchResult patch_result(patch_request);
    if (box_first)
      computeContactPatch(&box, Transform3s(), &capsule, tf_capsule, result,
                          patch_request, patch_result);
    else
      computeContactPatch(&capsule, tf_capsule, &box, Transform3s(), result,
                          patch_request, patch_result);
    BOOST_REQUIRE_EQUAL(patch_result.numContactPatches(), 1);
    const ContactPatch& patch = patch_result.getContactPatch(0);
    BOOST_CHECK_EQUAL(patch.size(), 2);
    BOOST_CHECK(patch.getNormal().cwiseAbs().isApprox(Vec3s::UnitZ(), tol));

    // The two points are the ends of the capsule segment, in the contact plane
    // halfway between the face and the capsule surface (penetration 0.01).
    for (const Scalar sign : {Scalar(-1), Scalar(1)}) {
      const Vec3s end(Scalar(0.1) + sign * Scalar(0.5), Scalar(0.2),
                      Scalar(0.995));
      BOOST_CHECK(hasPoint(patch, end, tol));
    }
  }
}

BOOST_AUTO_TEST_CASE(patch_matches_gjk_epa_normal) {
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> U(0, 1);

  // GJK/EPA with tight tolerances, used as reference.
  GJKSolver solver;
  solver.gjk_tolerance = Scalar(1e-10);
  solver.epa_tolerance = Scalar(1e-10);
  solver.gjk_max_iterations = 256;
  solver.epa_max_iterations = 256;

  const int num_tests = 5000;
  int num_compared = 0;
  int num_two_points = 0;
  int num_deep = 0;
  for (int i = 0; i < num_tests; ++i) {
    const Box box(Scalar(0.5 + U(rng)), Scalar(0.5 + U(rng)),
                  Scalar(0.5 + U(rng)));
    const Capsule capsule(Scalar(0.05 + 0.2 * U(rng)), Scalar(1.5 * U(rng)));
    const Transform3s tf_box(
        Quats(Vec4s(Vec4s::Random()).normalized()).toRotationMatrix(),
        Vec3s::Random());
    const Transform3s tf_capsule(
        Quats(Vec4s(Vec4s::Random()).normalized()).toRotationMatrix(),
        tf_box.getTranslation() + Scalar(0.7) * Vec3s::Random());

    Vec3s p1, p2, normal;
    const Scalar distance = solver.shapeDistance(
        box, tf_box, capsule, tf_capsule, true, p1, p2, normal);
    if (solver.gjk.status == details::GJK::Failed ||
        !(solver.epa.status & details::EPA::Valid))
      continue;
    // Skip configurations where the normal is not unique: shapes nearly
    // touching, or segment touching the box (where, in float, GJK may also
    // stop without running EPA on a penetrating segment).
    if (distance > -tol || std::abs(distance + capsule.radius) < tol) continue;
    ++num_compared;
    if (distance < -capsule.radius) ++num_deep;

    // The patch depends on the order of the shapes: compare in both orders.
    for (const bool box_first : {true, false}) {
      CollisionRequest request(CollisionRequestFlag::CONTACT, 1);
      CollisionResult result;
      if (box_first)
        collide(&box, tf_box, &capsule, tf_capsule, request, result);
      else
        collide(&capsule, tf_capsule, &box, tf_box, request, result);
      BOOST_REQUIRE(result.isCollision());

      // Same contact, with the normal and witness points of GJK/EPA.
      CollisionResult reference;
      Contact contact = result.getContact(0);
      contact.normal = box_first ? normal : Vec3s(-normal);
      contact.nearest_points[0] = box_first ? p1 : p2;
      contact.nearest_points[1] = box_first ? p2 : p1;
      contact.pos = Scalar(0.5) * (p1 + p2);
      contact.penetration_depth = distance;
      reference.addContact(contact);
      reference.distance_lower_bound = distance;

      const ContactPatch patch =
          box_first ? computePatch(box, tf_box, capsule, tf_capsule, result)
                    : computePatch(capsule, tf_capsule, box, tf_box, result);
      const ContactPatch expected =
          box_first ? computePatch(box, tf_box, capsule, tf_capsule, reference)
                    : computePatch(capsule, tf_capsule, box, tf_box, reference);

      BOOST_CHECK_EQUAL(patch.size(), expected.size());
      BOOST_CHECK(patch.getNormal().isApprox(expected.getNormal(), normal_tol));
      BOOST_CHECK_SMALL(patch.penetration_depth - expected.penetration_depth,
                        tol);
      for (size_t j = 0; j < expected.size(); ++j)
        BOOST_CHECK(hasPoint(patch, expected.getPoint(j), normal_tol));
      if (patch.size() == 2) ++num_two_points;
    }
  }
  BOOST_CHECK(num_compared > num_tests / 10);
  BOOST_CHECK(num_two_points > 0);
  BOOST_CHECK(num_deep > 0);
}
