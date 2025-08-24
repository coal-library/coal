/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of INRIA nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Louis Montaut */

#define BOOST_TEST_MODULE COAL_CONTACT_PATCH
#include <boost/test/included/unit_test.hpp>

#include "coal/contact_patch.h"

#include "utility.h"

using namespace coal;

BOOST_AUTO_TEST_CASE(box_box_no_collision) {
  const Scalar halfside = Scalar(0.5);
  const Box box1(2 * halfside, 2 * halfside, 2 * halfside);
  const Box box2(2 * halfside, 2 * halfside, 2 * halfside);

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to separate the shapes
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, 2 * halfside + offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  coal::collide(&box1, tf1, &box2, tf2, col_req, col_res);

  BOOST_CHECK(!col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);
  coal::computeContactPatch(&box1, tf1, &box2, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 0);
}

BOOST_AUTO_TEST_CASE(box_sphere) {
  const Scalar halfside = Scalar(0.5);
  const Box box(2 * halfside, 2 * halfside, 2 * halfside);
  const Sphere sphere(halfside);

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to have a collision
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, 2 * halfside - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  coal::collide(&box, tf1, &sphere, tf2, col_req, col_res);

  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);
  coal::computeContactPatch(&box, tf1, &sphere, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);
  if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(contact_patch.size() == 1);
    const Scalar tol = Scalar(1e-8);
    EIGEN_VECTOR_IS_APPROX(contact_patch.getPoint(0), contact.pos, tol);
    EIGEN_VECTOR_IS_APPROX(contact_patch.tf.translation(), contact.pos, tol);
    EIGEN_VECTOR_IS_APPROX(contact_patch.getNormal(), contact.normal, tol);
    BOOST_CHECK(std::abs(contact_patch.penetration_depth -
                         contact.penetration_depth) < tol);
  }
}

BOOST_AUTO_TEST_CASE(box_box) {
  const Scalar halfside = 0.5;
  const Box box1(2 * halfside, 2 * halfside, 2 * halfside);
  const Box box2(2 * halfside, 2 * halfside, 2 * halfside);

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to have a collision
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, 2 * halfside - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  coal::collide(&box1, tf1, &box2, tf2, col_req, col_res);

  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res1(patch_req);
  ContactPatchResult patch_res2(patch_req);
  coal::computeContactPatch(&box1, tf1, &box2, tf2, col_res, patch_req,
                            patch_res1);
  coal::computeContactPatch(&box1, tf1, &box2, tf2, col_res, patch_req,
                            patch_res2);
  BOOST_CHECK(patch_res1.numContactPatches() == 1);
  BOOST_CHECK(patch_res2.numContactPatches() == 1);

  if (patch_res1.numContactPatches() > 0 &&
      patch_res2.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);
    EIGEN_VECTOR_IS_APPROX(contact.normal, Vec3s(0, 0, 1), tol);

    const size_t expected_size = 4;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const std::array<Vec3s, 4> corners = {
        Vec3s(halfside, halfside, halfside),
        Vec3s(halfside, -halfside, halfside),
        Vec3s(-halfside, -halfside, halfside),
        Vec3s(-halfside, halfside, halfside),
    };
    for (size_t i = 0; i < expected_size; ++i) {
      expected.addPoint(corners[i] +
                        (contact.penetration_depth * contact.normal) / 2);
    }

    BOOST_CHECK(patch_res1.getContactPatch(0).isSame(expected, tol));
    BOOST_CHECK(patch_res2.getContactPatch(0).isSame(expected, tol));
  }
}

BOOST_AUTO_TEST_CASE(halfspace_box) {
  const Halfspace hspace(0, 0, 1, 0);
  const Scalar halfside = Scalar(0.5);
  const Box box(2 * halfside, 2 * halfside, 2 * halfside);

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to have a collision
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, halfside - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  coal::collide(&hspace, tf1, &box, tf2, col_req, col_res);

  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res1(patch_req);
  ContactPatchResult patch_res2(patch_req);
  coal::computeContactPatch(&hspace, tf1, &box, tf2, col_res, patch_req,
                            patch_res1);
  coal::computeContactPatch(&hspace, tf1, &box, tf2, col_res, patch_req,
                            patch_res2);
  BOOST_CHECK(patch_res1.numContactPatches() == 1);
  BOOST_CHECK(patch_res2.numContactPatches() == 1);

  if (patch_res1.numContactPatches() > 0 &&
      patch_res2.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);
    EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);
    EIGEN_VECTOR_IS_APPROX(hspace.n, Vec3s(0, 0, 1), tol);

    const size_t expected_size = 4;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const std::array<Vec3s, 4> corners = {
        tf2.transform(Vec3s(halfside, halfside, -halfside)),
        tf2.transform(Vec3s(halfside, -halfside, -halfside)),
        tf2.transform(Vec3s(-halfside, -halfside, -halfside)),
        tf2.transform(Vec3s(-halfside, halfside, -halfside)),
    };
    for (size_t i = 0; i < expected_size; ++i) {
      expected.addPoint(corners[i] -
                        (contact.penetration_depth * contact.normal) / 2);
    }

    BOOST_CHECK(patch_res1.getContactPatch(0).isSame(expected, tol));
    BOOST_CHECK(patch_res2.getContactPatch(0).isSame(expected, tol));
  }
}

BOOST_AUTO_TEST_CASE(halfspace_capsule) {
  const Halfspace hspace(0, 0, 1, 0);
  const Scalar radius = Scalar(0.25);
  const Scalar height = 1.;
  const Capsule capsule(radius, height);

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to have a collision
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, height / 2 - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;
  coal::collide(&hspace, tf1, &capsule, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  BOOST_CHECK(patch_req.getNumSamplesCurvedShapes() ==
              ContactPatch::default_preallocated_size);
  ContactPatchResult patch_res(patch_req);
  coal::computeContactPatch(&hspace, tf1, &capsule, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);

  if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);
    EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);

    const size_t expected_size = 1;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const Vec3s capsule_end(0, 0, -capsule.halfLength);
    expected.addPoint(tf2.transform(capsule_end));

    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(expected.tf == contact_patch.tf);
    BOOST_CHECK(expected.isSame(contact_patch, tol));
  }

  // Rotate capsule 180 degrees around y-axis
  // Should only have one contact.
  tf2.rotation().col(0) << -1, 0, 0;
  tf2.rotation().col(1) << 0, 1, 0;
  tf2.rotation().col(2) << 0, 0, -1;
  col_res.clear();
  coal::collide(&hspace, tf1, &capsule, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());
  patch_res.clear();
  coal::computeContactPatch(&hspace, tf1, &capsule, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);
  if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);
    EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);

    const size_t expected_size = 1;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const Vec3s capsule_end(0, 0, capsule.halfLength);
    expected.addPoint(tf2.transform(capsule_end));

    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(expected.tf == contact_patch.tf);
    BOOST_CHECK(expected.isSame(contact_patch, tol));
  }

  // Rotate cone 90 degrees around y-axis
  // Should only have two contacts.
  tf2.rotation().col(0) << 0, 0, 1;
  tf2.rotation().col(1) << 0, 1, 0;
  tf2.rotation().col(2) << -1, 0, 0;
  tf2.translation() << 0, 0, capsule.radius - offset;
  col_res.clear();
  coal::collide(&hspace, tf1, &capsule, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());
  patch_res.clear();
  coal::computeContactPatch(&hspace, tf1, &capsule, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);
  if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);
    EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);

    const size_t expected_size = 2;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const Vec3s p1(-capsule.radius, 0, capsule.halfLength);
    const Vec3s p2(-capsule.radius, 0, -capsule.halfLength);
    expected.addPoint(tf2.transform(p1));
    expected.addPoint(tf2.transform(p2));

    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(expected.tf == contact_patch.tf);
    BOOST_CHECK(expected.isSame(contact_patch, tol));
  }
}

BOOST_AUTO_TEST_CASE(halfspace_cone) {
  const Halfspace hspace(0, 0, 1, 0);
  const Scalar radius = 0.25;
  const Scalar height = 1.;
  const Cone cone(radius, height);

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to have a collision
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, height / 2 - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;
  coal::collide(&hspace, tf1, &cone, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  BOOST_CHECK(patch_req.getNumSamplesCurvedShapes() ==
              ContactPatch::default_preallocated_size);
  ContactPatchResult patch_res(patch_req);
  coal::computeContactPatch(&hspace, tf1, &cone, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);

  if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);
    EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);

    const size_t expected_size = ContactPatch::default_preallocated_size;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    std::array<Vec3s, ContactPatch::default_preallocated_size> points;
    const Scalar angle_increment = 2.0 * (Scalar)(EIGEN_PI) / ((Scalar)(6));
    for (size_t i = 0; i < ContactPatch::default_preallocated_size; ++i) {
      const Scalar theta = (Scalar)(i)*angle_increment;
      Vec3s point_on_cone_base(std::cos(theta) * cone.radius,
                               std::sin(theta) * cone.radius, -cone.halfLength);
      expected.addPoint(tf2.transform(point_on_cone_base));
    }

    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(expected.tf == contact_patch.tf);
    BOOST_CHECK(expected.isSame(contact_patch, tol));
  }

  // Rotate cone 180 degrees around y-axis
  // Should only have one contact, due to cone-tip/halfspace collision.
  tf2.rotation().col(0) << -1, 0, 0;
  tf2.rotation().col(1) << 0, 1, 0;
  tf2.rotation().col(2) << 0, 0, -1;
  col_res.clear();
  coal::collide(&hspace, tf1, &cone, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());
  patch_res.clear();
  coal::computeContactPatch(&hspace, tf1, &cone, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);
  if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(contact_patch.size() == 1);
    const Scalar tol = Scalar(1e-8);
    EIGEN_VECTOR_IS_APPROX(contact_patch.getPoint(0), contact.pos, tol);
    EIGEN_VECTOR_IS_APPROX(contact_patch.tf.translation(), contact.pos, tol);
    EIGEN_VECTOR_IS_APPROX(contact_patch.getNormal(), contact.normal, tol);
    BOOST_CHECK(std::abs(contact_patch.penetration_depth -
                         contact.penetration_depth) < tol);

    const size_t expected_size = 1;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const Vec3s cone_tip(0, 0, cone.halfLength);
    expected.addPoint(tf2.transform(cone_tip));

    BOOST_CHECK(contact_patch.isSame(expected, tol));
  }

  // Rotate cone 90 degrees around y-axis
  // Should only have one contact, on cone circle basis.
  tf2.rotation().col(0) << 0, 0, 1;
  tf2.rotation().col(1) << 0, 1, 0;
  tf2.rotation().col(2) << -1, 0, 0;
  tf2.translation() << 0, 0, cone.radius - offset;
  col_res.clear();
  coal::collide(&hspace, tf1, &cone, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());
  patch_res.clear();
  coal::computeContactPatch(&hspace, tf1, &cone, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);
  if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(contact_patch.size() == 1);
    const Scalar tol = Scalar(1e-8);
    EIGEN_VECTOR_IS_APPROX(contact_patch.getPoint(0), contact.pos, tol);
    EIGEN_VECTOR_IS_APPROX(contact_patch.tf.translation(), contact.pos, tol);
    EIGEN_VECTOR_IS_APPROX(contact_patch.getNormal(), contact.normal, tol);
    BOOST_CHECK(std::abs(contact_patch.penetration_depth -
                         contact.penetration_depth) < tol);

    const size_t expected_size = 1;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const Vec3s point_on_circle_basis(-cone.radius, 0, -cone.halfLength);
    expected.addPoint(tf2.transform(point_on_circle_basis));

    BOOST_CHECK(contact_patch.isSame(expected, tol));
  }
}

BOOST_AUTO_TEST_CASE(halfspace_cylinder) {
  const Halfspace hspace(0, 0, 1, 0);
  const Scalar radius = Scalar(0.25);
  const Scalar height = 1;
  const Cylinder cylinder(radius, height);

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to have a collision
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, height / 2 - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;
  coal::collide(&hspace, tf1, &cylinder, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());

  if (col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const size_t expected_size = ContactPatch::default_preallocated_size;
    const Scalar tol = Scalar(1e-6);
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    std::array<Vec3s, ContactPatch::default_preallocated_size> points;
    const Scalar angle_increment = 2.0 * (Scalar)(EIGEN_PI) / ((Scalar)(6));
    for (size_t i = 0; i < ContactPatch::default_preallocated_size; ++i) {
      const Scalar theta = (Scalar)(i)*angle_increment;
      Vec3s point_on_cone_base(std::cos(theta) * cylinder.radius,
                               std::sin(theta) * cylinder.radius,
                               -cylinder.halfLength);
      expected.addPoint(tf2.transform(point_on_cone_base));
    }

    const ContactPatchRequest patch_req;
    BOOST_CHECK(patch_req.getNumSamplesCurvedShapes() ==
                ContactPatch::default_preallocated_size);
    ContactPatchResult patch_res(patch_req);
    coal::computeContactPatch(&hspace, tf1, &cylinder, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);
      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.tf == contact_patch.tf);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }

    // Rotate cylinder 180 degrees around y-axis.
    // Should only have the same contact-patch, due to cylinder symmetry.
    tf2.rotation().col(0) << -1, 0, 0;
    tf2.rotation().col(1) << 0, 1, 0;
    tf2.rotation().col(2) << 0, 0, -1;
    col_res.clear();
    coal::collide(&hspace, tf1, &cylinder, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&hspace, tf1, &cylinder, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);
    if (patch_res.numContactPatches() > 0 && col_res.isCollision()) {
      EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);
      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.tf == contact_patch.tf);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }

  // Rotate cylinder 90 degrees around y-axis.
  // Should have 2 contact points.
  tf2.rotation().col(0) << 0, 0, 1;
  tf2.rotation().col(1) << 0, 1, 0;
  tf2.rotation().col(2) << -1, 0, 0;
  tf2.translation() << 0, 0, cylinder.radius - offset;

  col_res.clear();
  coal::collide(&hspace, tf1, &cylinder, tf2, col_req, col_res);
  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);
  coal::computeContactPatch(&hspace, tf1, &cylinder, tf2, col_res, patch_req,
                            patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);
  if (col_res.isCollision() && patch_res.numContactPatches() > 0) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);

    const size_t expected_size = 2;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    expected.addPoint(
        tf2.transform(Vec3s(cylinder.radius, 0, cylinder.halfLength)));
    expected.addPoint(
        tf2.transform(Vec3s(cylinder.radius, 0, -cylinder.halfLength)));

    const ContactPatch &contact_patch = patch_res.getContactPatch(0);
    BOOST_CHECK(expected.isSame(contact_patch, tol));
  }
}

BOOST_AUTO_TEST_CASE(convex_convex) {
  const Scalar halfside = 0.5;
  const ConvexTpl<Quadrilateral32> box1(buildBox(halfside, halfside, halfside));
  const ConvexTpl<Quadrilateral32> box2(buildBox(halfside, halfside, halfside));

  const Transform3s tf1;
  Transform3s tf2;
  // set translation to have a collision
  const Scalar offset = Scalar(0.001);
  tf2.setTranslation(Vec3s(0, 0, 2 * halfside - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  coal::collide(&box1, tf1, &box2, tf2, col_req, col_res);

  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res1(patch_req);
  ContactPatchResult patch_res2(patch_req);
  coal::computeContactPatch(&box1, tf1, &box2, tf2, col_res, patch_req,
                            patch_res1);
  coal::computeContactPatch(&box1, tf1, &box2, tf2, col_res, patch_req,
                            patch_res2);
  BOOST_CHECK(patch_res1.numContactPatches() == 1);
  BOOST_CHECK(patch_res2.numContactPatches() == 1);

  if (patch_res1.numContactPatches() > 0 &&
      patch_res2.numContactPatches() > 0 && col_res.isCollision()) {
    const Contact &contact = col_res.getContact(0);
    const Scalar tol = Scalar(1e-6);
    EIGEN_VECTOR_IS_APPROX(contact.normal, Vec3s(0, 0, 1), tol);

    const size_t expected_size = 4;
    ContactPatch expected(expected_size);
    expected.tf.rotation() =
        constructOrthonormalBasisFromVector(contact.normal);
    expected.tf.translation() = contact.pos;
    expected.penetration_depth = contact.penetration_depth;
    const std::array<Vec3s, 4> corners = {
        Vec3s(halfside, halfside, halfside),
        Vec3s(halfside, -halfside, halfside),
        Vec3s(-halfside, -halfside, halfside),
        Vec3s(-halfside, halfside, halfside),
    };
    for (size_t i = 0; i < expected_size; ++i) {
      expected.addPoint(corners[i] +
                        (contact.penetration_depth * contact.normal) / 2);
    }

    BOOST_CHECK(patch_res1.getContactPatch(0).isSame(expected, tol));
    BOOST_CHECK(patch_res2.getContactPatch(0).isSame(expected, tol));
  }
}

BOOST_AUTO_TEST_CASE(edge_case_segment_segment) {
  // This case covers the segment-segment edge case of contact patches.
  // Two tetrahedrons make contact on one of their edge.

  const size_t expected_size = 2;
  const Vec3s expected_cp1(0, 0.5, 0);
  const Vec3s expected_cp2(0, 1, 0);

  const Transform3s tf1;  // identity
  const Transform3s tf2;  // identity

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;
  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);

  {
    // Case 1 - Face-Face contact
    std::shared_ptr<std::vector<Vec3s>> pts1(new std::vector<Vec3s>({
        Vec3s(-1, 0, 0),
        Vec3s(0, 0, 0),
        Vec3s(0, 1, 0),
        Vec3s(-1, -1, -1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris1(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra1(pts1, 4, tris1, 4);

    std::shared_ptr<std::vector<Vec3s>> pts2(new std::vector<Vec3s>({
        Vec3s(0, 0.5, 0),
        Vec3s(0, 1.5, 0),
        Vec3s(1, 0.5, 0),
        Vec3s(1, 1, 1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris2(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra2(pts2, 4, tris2, 4);

    col_res.clear();
    coal::collide(&tetra1, tf1, &tetra2, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&tetra1, tf1, &tetra2, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      const Contact &contact = col_res.getContact(0);
      const Scalar tol = Scalar(1e-6);

      ContactPatch expected(expected_size);
      // GJK/EPA can return any normal which is in the dual cone
      // of the cone {(-1, 0, 0)}.
      expected.tf.rotation() =
          constructOrthonormalBasisFromVector(contact.normal);
      expected.tf.translation() = contact.pos;
      expected.penetration_depth = contact.penetration_depth;
      expected.addPoint(expected_cp1);
      expected.addPoint(expected_cp2);

      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }

  {
    // Case 2 - Face-Segment contact
    std::shared_ptr<std::vector<Vec3s>> pts1(new std::vector<Vec3s>({
        Vec3s(-1, 0, Scalar(-0.2)),
        Vec3s(0, 0, 0),
        Vec3s(0, 1, 0),
        Vec3s(-1, -1, -1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris1(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra1(pts1, 4, tris1, 4);

    std::shared_ptr<std::vector<Vec3s>> pts2(new std::vector<Vec3s>({
        Vec3s(0, 0.5, 0),
        Vec3s(0, 1.5, 0),
        Vec3s(1, 0.5, 0),
        Vec3s(1, 1, 1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris2(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra2(pts2, 4, tris2, 4);

    col_res.clear();
    coal::collide(&tetra1, tf1, &tetra2, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&tetra1, tf1, &tetra2, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      const Contact &contact = col_res.getContact(0);
      const Scalar tol = Scalar(1e-6);

      ContactPatch expected(expected_size);
      expected.tf.rotation() =
          constructOrthonormalBasisFromVector(contact.normal);
      expected.tf.translation() = contact.pos;
      expected.penetration_depth = contact.penetration_depth;
      expected.addPoint(expected_cp1);
      expected.addPoint(expected_cp2);

      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }

  {
    // Case 3 - Segment-Segment contact
    std::shared_ptr<std::vector<Vec3s>> pts1(new std::vector<Vec3s>({
        Vec3s(-1, 0, Scalar(-0.2)),
        Vec3s(0, 0, 0),
        Vec3s(0, 1, 0),
        Vec3s(-1, -1, -1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris1(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra1(pts1, 4, tris1, 4);

    std::shared_ptr<std::vector<Vec3s>> pts2(new std::vector<Vec3s>({
        Vec3s(0, 0.5, 0),
        Vec3s(0, 1.5, 0),
        Vec3s(1, 0.5, 0.5),
        Vec3s(1, 1, 1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris2(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra2(pts2, 4, tris2, 4);

    col_res.clear();
    coal::collide(&tetra1, tf1, &tetra2, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&tetra1, tf1, &tetra2, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      const Contact &contact = col_res.getContact(0);
      const Scalar tol = Scalar(1e-6);

      ContactPatch expected(expected_size);
      expected.tf.rotation() =
          constructOrthonormalBasisFromVector(contact.normal);
      expected.tf.translation() = contact.pos;
      expected.penetration_depth = contact.penetration_depth;
      expected.addPoint(expected_cp1);
      expected.addPoint(expected_cp2);

      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }
}

BOOST_AUTO_TEST_CASE(edge_case_vertex_vertex) {
  // This case covers the vertex-vertex edge case of contact patches.
  // Two tetrahedrons make contact on one of their vertex.
  const size_t expected_size = 1;
  const Vec3s expected_cp(0, 0, 0);

  const Transform3s tf1;  // identity
  const Transform3s tf2;  // identity

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;
  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);

  {
    // Case 1 - Face-Face contact
    std::shared_ptr<std::vector<Vec3s>> pts1(new std::vector<Vec3s>({
        Vec3s(-1, 0, 0),
        Vec3s(0, 0, 0),
        Vec3s(0, 1, 0),
        Vec3s(-1, -1, -1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris1(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra1(pts1, 4, tris1, 4);

    std::shared_ptr<std::vector<Vec3s>> pts2(new std::vector<Vec3s>({
        Vec3s(1, 0, 0),
        Vec3s(0, 0, 0),
        Vec3s(0, -1, 0),
        Vec3s(1, 1, 1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris2(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra2(pts2, 4, tris2, 4);

    col_res.clear();
    coal::collide(&tetra1, tf1, &tetra2, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&tetra1, tf1, &tetra2, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      const Contact &contact = col_res.getContact(0);
      const Scalar tol = Scalar(1e-6);

      ContactPatch expected(expected_size);
      expected.tf.rotation() =
          constructOrthonormalBasisFromVector(contact.normal);
      expected.tf.translation() = contact.pos;
      expected.penetration_depth = contact.penetration_depth;
      expected.addPoint(expected_cp);

      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }

  {
    // Case 2 - Segment-Face contact
    std::shared_ptr<std::vector<Vec3s>> pts1(new std::vector<Vec3s>({
        Vec3s(-1, 0, -0.5),
        Vec3s(0, 0, 0),
        Vec3s(0, 1, 0),
        Vec3s(-1, -1, -1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris1(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra1(pts1, 4, tris1, 4);

    std::shared_ptr<std::vector<Vec3s>> pts2(new std::vector<Vec3s>({
        Vec3s(1, 0, 0),
        Vec3s(0, 0, 0),
        Vec3s(0, -1, 0),
        Vec3s(1, 1, 1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris2(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra2(pts2, 4, tris2, 4);

    col_res.clear();
    coal::collide(&tetra1, tf1, &tetra2, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&tetra1, tf1, &tetra2, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      const Contact &contact = col_res.getContact(0);
      const Scalar tol = Scalar(1e-6);

      ContactPatch expected(expected_size);
      expected.tf.rotation() =
          constructOrthonormalBasisFromVector(contact.normal);
      expected.tf.translation() = contact.pos;
      expected.penetration_depth = contact.penetration_depth;
      expected.addPoint(expected_cp);

      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }

  {
    // Case 2 - Segment-Segment contact
    std::shared_ptr<std::vector<Vec3s>> pts1(new std::vector<Vec3s>({
        Vec3s(-1, 0, Scalar(-0.2)),
        Vec3s(0, 0, 0),
        Vec3s(0, 1, 0),
        Vec3s(-1, -1, -1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris1(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra1(pts1, 4, tris1, 4);

    std::shared_ptr<std::vector<Vec3s>> pts2(new std::vector<Vec3s>({
        Vec3s(1, 0, 0),
        Vec3s(0, 0, 0),
        Vec3s(0, -1, 0.5),
        Vec3s(1, 1, 1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris2(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra2(pts2, 4, tris2, 4);

    col_res.clear();
    coal::collide(&tetra1, tf1, &tetra2, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&tetra1, tf1, &tetra2, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      const Contact &contact = col_res.getContact(0);
      const Scalar tol = Scalar(1e-6);

      ContactPatch expected(expected_size);
      expected.tf.rotation() =
          constructOrthonormalBasisFromVector(contact.normal);
      expected.tf.translation() = contact.pos;
      expected.penetration_depth = contact.penetration_depth;
      expected.addPoint(expected_cp);

      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }
}

BOOST_AUTO_TEST_CASE(edge_case_segment_face) {
  // This case covers the segment-face edge case of contact patches.
  // Two tetrahedrons make contact on one of their segment/face respectively.
  const size_t expected_size = 2;
  const Vec3s expected_cp1(0, 0, 0);
  const Vec3s expected_cp2(-0.5, 0.5, 0);

  const Transform3s tf1;  // identity
  const Transform3s tf2;  // identity

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;
  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);

  {
    std::shared_ptr<std::vector<Vec3s>> pts1(new std::vector<Vec3s>({
        Vec3s(-1, 0, -0),
        Vec3s(0, 0, 0),
        Vec3s(0, 1, 0),
        Vec3s(-1, -1, -1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris1(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra1(pts1, 4, tris1, 4);

    std::shared_ptr<std::vector<Vec3s>> pts2(new std::vector<Vec3s>({
        Vec3s(-0.5, 0.5, 0),
        Vec3s(0.5, -0.5, 0),
        Vec3s(1, 0.5, 0.5),
        Vec3s(1, 1, 1),
    }));
    std::shared_ptr<std::vector<Triangle32>> tris2(new std::vector<Triangle32>(
        {Triangle32(0, 1, 2), Triangle32(0, 2, 3), Triangle32(0, 3, 1),
         Triangle32(2, 1, 3)}));
    ConvexTpl<Triangle32> tetra2(pts2, 4, tris2, 4);

    col_res.clear();
    coal::collide(&tetra1, tf1, &tetra2, tf2, col_req, col_res);
    BOOST_CHECK(col_res.isCollision());
    patch_res.clear();
    coal::computeContactPatch(&tetra1, tf1, &tetra2, tf2, col_res, patch_req,
                              patch_res);
    BOOST_CHECK(patch_res.numContactPatches() == 1);

    if (patch_res.numContactPatches() > 0) {
      const Contact &contact = col_res.getContact(0);
      const Scalar tol = Scalar(1e-6);

      ContactPatch expected(expected_size);
      expected.tf.rotation() =
          constructOrthonormalBasisFromVector(contact.normal);
      expected.tf.translation() = contact.pos;
      expected.penetration_depth = contact.penetration_depth;
      expected.addPoint(expected_cp1);
      expected.addPoint(expected_cp2);

      const ContactPatch &contact_patch = patch_res.getContactPatch(0);
      BOOST_CHECK(expected.isSame(contact_patch, tol));
    }
  }
}
