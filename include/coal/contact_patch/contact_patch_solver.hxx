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

#ifndef COAL_CONTACT_PATCH_SOLVER_HXX
#define COAL_CONTACT_PATCH_SOLVER_HXX

#include "coal/data_types.h"
#include "coal/shape/geometric_shapes_traits.h"

#include "coal/tracy.hh"

namespace coal {

// ============================================================================
inline void ContactPatchSolver::set(const ContactPatchRequest& request) {
  // Note: it's important for the number of pre-allocated Vec3s in
  // `m_clipping_sets` to be larger than `request.max_size_patch`
  // because we don't know in advance how many supports will be discarded to
  // form the convex-hulls of the shapes supports which will serve as the
  // input of the Sutherland-Hodgman algorithm.
  size_t num_preallocated_supports = default_num_preallocated_supports;
  if (num_preallocated_supports < 2 * request.getNumSamplesCurvedShapes()) {
    num_preallocated_supports = 2 * request.getNumSamplesCurvedShapes();
  }

  // Used for support set computation of shape1 and for the first iterate of the
  // Sutherland-Hodgman algo.
  this->support_set_shape1.points().reserve(num_preallocated_supports);
  this->support_set_shape1.direction = SupportSetDirection::DEFAULT;

  // Used for computing the next iterate of the Sutherland-Hodgman algo.
  this->support_set_buffer.points().reserve(num_preallocated_supports);

  // Used for support set computation of shape2 and acts as the "clipper" set in
  // the Sutherland-Hodgman algo.
  this->support_set_shape2.points().reserve(num_preallocated_supports);
  this->support_set_shape2.direction = SupportSetDirection::INVERTED;

  this->num_samples_curved_shapes = request.getNumSamplesCurvedShapes();
  this->patch_tolerance = request.getPatchTolerance();
}

// ============================================================================
template <typename ShapeType1, typename ShapeType2>
void ContactPatchSolver::computePatch(const ShapeType1& s1,
                                      const Transform3s& tf1,
                                      const ShapeType2& s2,
                                      const Transform3s& tf2,
                                      const Contact& contact,
                                      ContactPatch& contact_patch) const {
  COAL_TRACY_ZONE_SCOPED_N("coal::ContactPatchSolver::computePatch");
  // Note: `ContactPatch` is an alias for `SupportSet`.
  // Step 1
  constructContactPatchFrameFromContact(contact, contact_patch);
  contact_patch.points().clear();
  if ((bool)(shape_traits<ShapeType1>::IsStrictlyConvex) ||
      (bool)(shape_traits<ShapeType2>::IsStrictlyConvex)) {
    // If a shape is strictly convex, the support set in any direction is
    // reduced to a single point. Thus, the contact point `contact.pos` is the
    // only point belonging to the contact patch, and it has already been
    // computed.
    // TODO(louis): even for strictly convex shapes, we can sample the support
    // function around the normal and return a pseudo support set. This would
    // allow spheres and ellipsoids to have a contact surface, which does make
    // sense in certain physics simulation cases.
    // Do the same for strictly convex regions of non-strictly convex shapes
    // like the ends of capsules.
    contact_patch.addPoint(contact.pos);
    return;
  }

  // Step 2 - Compute support set of each shape, in the direction of
  // the contact's normal.
  // The first shape's support set is called "current"; it will be the first
  // iterate of the Sutherland-Hodgman algorithm. The second shape's support set
  // is called "clipper"; it will be used to clip "current". The support set
  // computation step computes a convex polygon; its vertices are ordered
  // counter-clockwise. This is important as the Sutherland-Hodgman algorithm
  // expects points to be ranked counter-clockwise.
  this->reset(s1, tf1, s2, tf2, contact_patch);
  assert(this->num_samples_curved_shapes > 3);

  this->supportFuncShape1(&s1, this->support_set_shape1, this->support_guess[0],
                          this->supports_data[0],
                          this->num_samples_curved_shapes,
                          this->patch_tolerance);

  this->supportFuncShape2(&s2, this->support_set_shape2, this->support_guess[1],
                          this->supports_data[1],
                          this->num_samples_curved_shapes,
                          this->patch_tolerance);

  // We can immediatly return if one of the support set has only
  // one point.
  if (this->support_set_shape1.size() <= 1 ||
      this->support_set_shape2.size() <= 1) {
    contact_patch.addPoint(contact.pos);
    return;
  }

  // `eps` is be used to check strict positivity of determinants.
  const Scalar eps = Eigen::NumTraits<Scalar>::dummy_precision();
  using Polygon = SupportSet::Polygon;

  if ((this->support_set_shape1.size() == 2) &&
      (this->support_set_shape2.size() == 2)) {
    // Segment-Segment case
    // We compute the determinant; if it is non-zero, the intersection
    // has already been computed: it's `Contact::pos`.
    const Polygon& pts1 = this->support_set_shape1.points();
    const Vec2s& a = pts1[0];
    const Vec2s& b = pts1[1];

    const Polygon& pts2 = this->support_set_shape2.points();
    const Vec2s& c = pts2[0];
    const Vec2s& d = pts2[1];

    const Scalar det =
        (b(0) - a(0)) * (d(1) - c(1)) >= (b(1) - a(1)) * (d(0) - c(0));
    if ((std::abs(det) > eps) || ((c - d).squaredNorm() < eps) ||
        ((b - a).squaredNorm() < eps)) {
      contact_patch.addPoint(contact.pos);
      return;
    }

    const Vec2s cd = (d - c);
    const Scalar l = cd.squaredNorm();
    Polygon& patch = contact_patch.points();

    // Project a onto [c, d]
    Scalar t1 = (a - c).dot(cd);
    t1 = (t1 >= l) ? 1.0 : ((t1 <= 0) ? 0.0 : (t1 / l));
    const Vec2s p1 = c + t1 * cd;
    patch.emplace_back(p1);

    // Project b onto [c, d]
    Scalar t2 = (b - c).dot(cd);
    t2 = (t2 >= l) ? 1.0 : ((t2 <= 0) ? 0.0 : (t2 / l));
    const Vec2s p2 = c + t2 * cd;
    if ((p1 - p2).squaredNorm() >= eps) {
      patch.emplace_back(p2);
    }
    return;
  }

  //
  // Step 3 - Main loop of the algorithm: use the "clipper" polygon to clip the
  // "current" polygon. The resulting intersection is the contact patch of the
  // contact between s1 and s2. "clipper" and "current" are the support sets of
  // shape1 and shape2 (they can be swapped, i.e. clipper can be assigned to
  // shape1 and current to shape2, depending on which case we are). Currently,
  // to clip one polygon with the other, we use the Sutherland-Hodgman
  // algorithm:
  // https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
  // In the general case, Sutherland-Hodgman clips one polygon of size >=3 using
  // another polygon of size >=3. However, it can be easily extended to handle
  // the segment-polygon case.
  //
  // The maximum size of the output of the Sutherland-Hodgman algorithm is n1 +
  // n2 where n1 and n2 are the sizes of the first and second polygon.
  const size_t max_result_size =
      this->support_set_shape1.size() + this->support_set_shape2.size();
  if (this->added_to_patch.size() < max_result_size) {
    this->added_to_patch.assign(max_result_size, false);
  }

  const Polygon* clipper_ptr = nullptr;
  Polygon* current_ptr = nullptr;
  Polygon* previous_ptr = &(this->support_set_buffer.points());

  // Let the clipper set be the one with the most vertices, to make sure it is
  // at least a triangle.
  if (this->support_set_shape1.size() < this->support_set_shape2.size()) {
    current_ptr = &(this->support_set_shape1.points());
    clipper_ptr = &(this->support_set_shape2.points());
  } else {
    current_ptr = &(this->support_set_shape2.points());
    clipper_ptr = &(this->support_set_shape1.points());
  }

  const Polygon& clipper = *(clipper_ptr);
  const size_t clipper_size = clipper.size();
  for (size_t i = 0; i < clipper_size; ++i) {
    // Swap `current` and `previous`.
    // `previous` tracks the last iteration of the algorithm; `current` is
    // filled by clipping `current` using `clipper`.
    Polygon* tmp_ptr = previous_ptr;
    previous_ptr = current_ptr;
    current_ptr = tmp_ptr;

    const Polygon& previous = *(previous_ptr);
    Polygon& current = *(current_ptr);
    current.clear();

    const Vec2s& a = clipper[i];
    const Vec2s& b = clipper[(i + 1) % clipper_size];
    const Vec2s ab = b - a;

    if (previous.size() == 2) {
      //
      // Segment-Polygon case
      //
      const Vec2s& p1 = previous[0];
      const Vec2s& p2 = previous[1];

      const Vec2s ap1 = p1 - a;
      const Vec2s ap2 = p2 - a;

      const Scalar det1 = ab(0) * ap1(1) - ab(1) * ap1(0);
      const Scalar det2 = ab(0) * ap2(1) - ab(1) * ap2(0);

      if (det1 < 0 && det2 < 0) {
        // Both p1 and p2 are outside the clipping polygon, i.e. there is no
        // intersection. The algorithm can stop.
        break;
      }

      if (det1 >= 0 && det2 >= 0) {
        // Both p1 and p2 are inside the clipping polygon, there is nothing to
        // do; move to the next iteration.
        current = previous;
        continue;
      }

      // Compute the intersection between the line (a, b) and the segment
      // [p1, p2].
      if (det1 >= 0) {
        if (det1 > eps) {
          const Vec2s p = computeLineSegmentIntersection(a, b, p1, p2);
          current.emplace_back(p1);
          current.emplace_back(p);
          continue;
        } else {
          // p1 is the only point of current which is also a point of the
          // clipper. We can exit.
          current.emplace_back(p1);
          break;
        }
      } else {
        if (det2 > eps) {
          const Vec2s p = computeLineSegmentIntersection(a, b, p1, p2);
          current.emplace_back(p2);
          current.emplace_back(p);
          continue;
        } else {
          // p2 is the only point of current which is also a point of the
          // clipper. We can exit.
          current.emplace_back(p2);
          break;
        }
      }
    } else {
      //
      // Polygon-Polygon case.
      //
      std::fill(this->added_to_patch.begin(),  //
                this->added_to_patch.end(),    //
                false);

      const size_t previous_size = previous.size();
      for (size_t j = 0; j < previous_size; ++j) {
        const Vec2s& p1 = previous[j];
        const Vec2s& p2 = previous[(j + 1) % previous_size];

        const Vec2s ap1 = p1 - a;
        const Vec2s ap2 = p2 - a;

        const Scalar det1 = ab(0) * ap1(1) - ab(1) * ap1(0);
        const Scalar det2 = ab(0) * ap2(1) - ab(1) * ap2(0);

        if (det1 < 0 && det2 < 0) {
          // No intersection. Continue to next segment of previous.
          continue;
        }

        if (det1 >= 0 && det2 >= 0) {
          // Both p1 and p2 are inside the clipping polygon, add p1 to current
          // (only if it has not already been added).
          if (!this->added_to_patch[j]) {
            current.emplace_back(p1);
            this->added_to_patch[j] = true;
          }
          // Continue to next segment of previous.
          continue;
        }

        if (det1 >= 0) {
          if (det1 > eps) {
            if (!this->added_to_patch[j]) {
              current.emplace_back(p1);
              this->added_to_patch[j] = true;
            }
            const Vec2s p = computeLineSegmentIntersection(a, b, p1, p2);
            current.emplace_back(p);
          } else {
            // a, b and p1 are colinear; we add only p1.
            if (!this->added_to_patch[j]) {
              current.emplace_back(p1);
              this->added_to_patch[j] = true;
            }
          }
        } else {
          if (det2 > eps) {
            const Vec2s p = computeLineSegmentIntersection(a, b, p1, p2);
            current.emplace_back(p);
          } else {
            if (!this->added_to_patch[(j + 1) % previous.size()]) {
              current.emplace_back(p2);
              this->added_to_patch[(j + 1) % previous.size()] = true;
            }
          }
        }
      }
    }
    //
    // End of iteration i of Sutherland-Hodgman.
    if (current.size() <= 1) {
      // No intersection or one point found, the algo can early stop.
      break;
    }
  }

  // Transfer the result of the Sutherland-Hodgman algorithm to the contact
  // patch.
  this->getResult(contact, current_ptr, contact_patch);
}

// ============================================================================
inline void ContactPatchSolver::getResult(
    const Contact& contact, const ContactPatch::Polygon* result_ptr,
    ContactPatch& contact_patch) const {
  if (result_ptr->size() <= 1) {
    contact_patch.addPoint(contact.pos);
    return;
  }

  const ContactPatch::Polygon& result = *(result_ptr);
  ContactPatch::Polygon& patch = contact_patch.points();
  patch = result;
}

// ============================================================================
template <typename ShapeType1, typename ShapeType2>
inline void ContactPatchSolver::reset(const ShapeType1& shape1,
                                      const Transform3s& tf1,
                                      const ShapeType2& shape2,
                                      const Transform3s& tf2,
                                      const ContactPatch& contact_patch) const {
  // Reset internal quantities
  this->support_set_shape1.clear();
  this->support_set_shape2.clear();
  this->support_set_buffer.clear();

  // Get the support function of each shape
  const Transform3s& tfc = contact_patch.tf;

  this->support_set_shape1.direction = SupportSetDirection::DEFAULT;
  // Set the reference frame of the support set of the first shape to be the
  // local frame of shape 1.
  Transform3s& tf1c = this->support_set_shape1.tf;
  tf1c.rotation().noalias() = tf1.rotation().transpose() * tfc.rotation();
  tf1c.translation().noalias() =
      tf1.rotation().transpose() * (tfc.translation() - tf1.translation());
  this->supportFuncShape1 =
      this->makeSupportSetFunction(&shape1, this->supports_data[0]);

  this->support_set_shape2.direction = SupportSetDirection::INVERTED;
  // Set the reference frame of the support set of the second shape to be the
  // local frame of shape 2.
  Transform3s& tf2c = this->support_set_shape2.tf;
  tf2c.rotation().noalias() = tf2.rotation().transpose() * tfc.rotation();
  tf2c.translation().noalias() =
      tf2.rotation().transpose() * (tfc.translation() - tf2.translation());
  this->supportFuncShape2 =
      this->makeSupportSetFunction(&shape2, this->supports_data[1]);
}

// ==========================================================================
inline Vec2s ContactPatchSolver::computeLineSegmentIntersection(
    const Vec2s& a, const Vec2s& b, const Vec2s& c, const Vec2s& d) {
  const Vec2s ab = b - a;
  const Vec2s n(-ab(1), ab(0));
  const Scalar denominator = n.dot(c - d);
  if (std::abs(denominator) < std::numeric_limits<Scalar>::epsilon()) {
    return d;
  }
  const Scalar nominator = n.dot(a - d);
  Scalar alpha = nominator / denominator;
  alpha = std::min<Scalar>(1.0, std::max<Scalar>(0.0, alpha));
  return alpha * c + (1 - alpha) * d;
}

}  // namespace coal

#endif  // COAL_CONTACT_PATCH_SOLVER_HXX
