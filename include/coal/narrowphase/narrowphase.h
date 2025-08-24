/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2018-2019, Centre National de la Recherche Scientifique
 *  All rights reserved.
 *  Copyright (c) 2021-2024, INRIA
 *  All rights reserved.
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

/** \author Jia Pan, Florent Lamiraux */

#ifndef COAL_NARROWPHASE_H
#define COAL_NARROWPHASE_H

#include <limits>

#include "coal/narrowphase/gjk.h"
#include "coal/collision_data.h"
#include "coal/narrowphase/narrowphase_defaults.h"
#include "coal/logging.h"

namespace coal {

/// @brief collision and distance solver based on the GJK and EPA algorithms.
/// Originally, GJK and EPA were implemented in fcl which itself took
/// inspiration from the code of the GJK in bullet. Since then, both GJK and EPA
/// have been largely modified to be faster and more robust to numerical
/// accuracy and edge cases.
struct COAL_DLLAPI GJKSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief GJK algorithm
  mutable details::GJK gjk;

  /// @brief maximum number of iterations of GJK
  size_t gjk_max_iterations;

  /// @brief tolerance of GJK
  Scalar gjk_tolerance;

  /// @brief which warm start to use for GJK
  GJKInitialGuess gjk_initial_guess;

  /// @brief Whether smart guess can be provided
  /// @Deprecated Use gjk_initial_guess instead
  COAL_DEPRECATED_MESSAGE(Use gjk_initial_guess instead)
  bool enable_cached_guess;

  /// @brief smart guess
  mutable Vec3s cached_guess;

  /// @brief smart guess for the support function
  mutable support_func_guess_t support_func_cached_guess;

  /// @brief If GJK can guarantee that the distance between the shapes is
  /// greater than this value, it will early stop.
  Scalar distance_upper_bound;

  /// @brief Variant of the GJK algorithm (Default, Nesterov or Polyak).
  GJKVariant gjk_variant;

  /// @brief Convergence criterion for GJK
  GJKConvergenceCriterion gjk_convergence_criterion;

  /// @brief Absolute or relative convergence criterion for GJK
  GJKConvergenceCriterionType gjk_convergence_criterion_type;

  /// @brief EPA algorithm
  mutable details::EPA epa;

  /// @brief maximum number of iterations of EPA
  size_t epa_max_iterations;

  /// @brief tolerance of EPA
  Scalar epa_tolerance;

  /// @brief Minkowski difference used by GJK and EPA algorithms
  mutable details::MinkowskiDiff minkowski_difference;

 private:
  // Used internally for assertion checks.
  static constexpr Scalar m_dummy_precision = Scalar(1e-6);

 public:
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  /// @brief Default constructor for GJK algorithm
  /// By default, we don't want EPA to allocate memory because
  /// certain functions of the `GJKSolver` class have specializations
  /// which don't use EPA (and/or GJK).
  /// So we give EPA's constructor a max number of iterations of zero.
  /// Only the functions that need EPA will reset the algorithm and allocate
  /// memory if needed.
  GJKSolver()
      : gjk(GJK_DEFAULT_MAX_ITERATIONS, GJK_DEFAULT_TOLERANCE),
        gjk_max_iterations(GJK_DEFAULT_MAX_ITERATIONS),
        gjk_tolerance(GJK_DEFAULT_TOLERANCE),
        gjk_initial_guess(GJKInitialGuess::DefaultGuess),
        enable_cached_guess(false),  // Use gjk_initial_guess instead
        cached_guess(Vec3s(1, 0, 0)),
        support_func_cached_guess(support_func_guess_t::Zero()),
        distance_upper_bound((std::numeric_limits<Scalar>::max)()),
        gjk_variant(GJKVariant::DefaultGJK),
        gjk_convergence_criterion(GJKConvergenceCriterion::Default),
        gjk_convergence_criterion_type(GJKConvergenceCriterionType::Absolute),
        epa(0, EPA_DEFAULT_TOLERANCE),
        epa_max_iterations(EPA_DEFAULT_MAX_ITERATIONS),
        epa_tolerance(EPA_DEFAULT_TOLERANCE) {}

  /// @brief Constructor from a DistanceRequest
  ///
  /// \param[in] request DistanceRequest input
  ///
  /// See the default constructor; by default, we don't want
  /// EPA to allocate memory so we call EPA's constructor with 0 max
  /// number of iterations.
  /// However, the `set` method stores the actual values of the request.
  /// EPA will thus allocate memory only if needed.
  explicit GJKSolver(const DistanceRequest& request)
      : gjk(request.gjk_max_iterations, request.gjk_tolerance),
        epa(0, request.epa_tolerance) {
    this->cached_guess = Vec3s(1, 0, 0);
    this->support_func_cached_guess = support_func_guess_t::Zero();

    set(request);
  }

  /// @brief setter from a DistanceRequest
  ///
  /// \param[in] request DistanceRequest input
  ///
  void set(const DistanceRequest& request) {
    // ---------------------
    // GJK settings
    this->gjk_initial_guess = request.gjk_initial_guess;
    this->enable_cached_guess = request.enable_cached_gjk_guess;
    if (this->gjk_initial_guess == GJKInitialGuess::CachedGuess ||
        this->enable_cached_guess) {
      this->cached_guess = request.cached_gjk_guess;
      this->support_func_cached_guess = request.cached_support_func_guess;
    }
    this->gjk_max_iterations = request.gjk_max_iterations;
    this->gjk_tolerance = request.gjk_tolerance;
    // For distance computation, we don't want GJK to early stop
    this->distance_upper_bound = (std::numeric_limits<Scalar>::max)();
    this->gjk_variant = request.gjk_variant;
    this->gjk_convergence_criterion = request.gjk_convergence_criterion;
    this->gjk_convergence_criterion_type =
        request.gjk_convergence_criterion_type;

    // ---------------------
    // EPA settings
    this->epa_max_iterations = request.epa_max_iterations;
    this->epa_tolerance = request.epa_tolerance;

    // ---------------------
    // Reset GJK and EPA status
    this->epa.status = details::EPA::Status::DidNotRun;
    this->gjk.status = details::GJK::Status::DidNotRun;
  }

  /// @brief Constructor from a CollisionRequest
  ///
  /// \param[in] request CollisionRequest input
  ///
  /// See the default constructor; by default, we don't want
  /// EPA to allocate memory so we call EPA's constructor with 0 max
  /// number of iterations.
  /// However, the `set` method stores the actual values of the request.
  /// EPA will thus allocate memory only if needed.
  explicit GJKSolver(const CollisionRequest& request)
      : gjk(request.gjk_max_iterations, request.gjk_tolerance),
        epa(0, request.epa_tolerance) {
    this->cached_guess = Vec3s(1, 0, 0);
    this->support_func_cached_guess = support_func_guess_t::Zero();

    set(request);
  }

  /// @brief setter from a CollisionRequest
  ///
  /// \param[in] request CollisionRequest input
  ///
  void set(const CollisionRequest& request) {
    // ---------------------
    // GJK settings
    this->gjk_initial_guess = request.gjk_initial_guess;
    this->enable_cached_guess = request.enable_cached_gjk_guess;
    if (this->gjk_initial_guess == GJKInitialGuess::CachedGuess ||
        this->enable_cached_guess) {
      this->cached_guess = request.cached_gjk_guess;
      this->support_func_cached_guess = request.cached_support_func_guess;
    }
    this->gjk_tolerance = request.gjk_tolerance;
    this->gjk_max_iterations = request.gjk_max_iterations;
    // The distance upper bound should be at least greater to the requested
    // security margin. Otherwise, we will likely miss some collisions.
    this->distance_upper_bound =
        (std::max)(Scalar(0), (std::max)(request.distance_upper_bound,
                                         request.security_margin));
    this->gjk_variant = request.gjk_variant;
    this->gjk_convergence_criterion = request.gjk_convergence_criterion;
    this->gjk_convergence_criterion_type =
        request.gjk_convergence_criterion_type;

    // ---------------------
    // EPA settings
    this->epa_max_iterations = request.epa_max_iterations;
    this->epa_tolerance = request.epa_tolerance;

    // ---------------------
    // Reset GJK and EPA status
    this->gjk.status = details::GJK::Status::DidNotRun;
    this->epa.status = details::EPA::Status::DidNotRun;
  }

  /// @brief Copy constructor
  GJKSolver(const GJKSolver& other) = default;

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  bool operator==(const GJKSolver& other) const {
    return this->enable_cached_guess ==
               other.enable_cached_guess &&  // use gjk_initial_guess instead
           this->cached_guess == other.cached_guess &&
           this->support_func_cached_guess == other.support_func_cached_guess &&
           this->gjk_max_iterations == other.gjk_max_iterations &&
           this->gjk_tolerance == other.gjk_tolerance &&
           this->distance_upper_bound == other.distance_upper_bound &&
           this->gjk_variant == other.gjk_variant &&
           this->gjk_convergence_criterion == other.gjk_convergence_criterion &&
           this->gjk_convergence_criterion_type ==
               other.gjk_convergence_criterion_type &&
           this->gjk_initial_guess == other.gjk_initial_guess &&
           this->epa_max_iterations == other.epa_max_iterations &&
           this->epa_tolerance == other.epa_tolerance;
  }
  COAL_COMPILER_DIAGNOSTIC_POP

  bool operator!=(const GJKSolver& other) const { return !(*this == other); }

  /// @brief Helper to return the precision of the solver on the distance
  /// estimate, depending on whether or not `compute_penetration` is true.
  Scalar getDistancePrecision(const bool compute_penetration) const {
    return compute_penetration
               ? (std::max)(this->gjk_tolerance, this->epa_tolerance)
               : this->gjk_tolerance;
  }

  /// @brief Uses GJK and EPA to compute the distance between two shapes.
  /// @param `s1` the first shape.
  /// @param `tf1` the transformation of the first shape.
  /// @param `s2` the second shape.
  /// @param `tf2` the transformation of the second shape.
  /// @param `compute_penetration` if true and GJK finds the shape in collision,
  /// the EPA algorithm is also ran to compute penetration information.
  /// @param[out] `p1` the witness point on the first shape.
  /// @param[out] `p2` the witness point on the second shape.
  /// @param[out] `normal` the normal of the collision, pointing from the first
  /// to the second shape.
  /// @return the estimate of the distance between the two shapes.
  ///
  /// @note: if `this->distance_upper_bound` is set to a positive value, GJK
  /// will early stop if it finds the distance to be above this value. The
  /// distance returned by `this->shapeDistance` will be a lower bound on the
  /// distance between the two shapes.
  ///
  /// @note: the variables `this->gjk.status` and `this->epa.status` can be used
  /// to examine the status of GJK and EPA.
  ///
  /// @note: GJK and EPA give an estimate of the distance between the two
  /// shapes. This estimate is precise up to the tolerance of the algorithms:
  ///   - If `compute_penetration` is false, the distance is precise up to
  ///     `gjk_tolerance`.
  ///   - If `compute_penetration` is true, the distance is precise up to
  ///     `std::max(gjk_tolerance, epa_tolerance)`
  /// It's up to the user to decide whether the shapes are in collision or not,
  /// based on that estimate.
  template <typename S1, typename S2>
  Scalar shapeDistance(const S1& s1, const Transform3s& tf1, const S2& s2,
                       const Transform3s& tf2, const bool compute_penetration,
                       Vec3s& p1, Vec3s& p2, Vec3s& normal) const {
    constexpr bool relative_transformation_already_computed = false;
    Scalar distance;
    this->runGJKAndEPA(s1, tf1, s2, tf2, compute_penetration, distance, p1, p2,
                       normal, relative_transformation_already_computed);
    return distance;
  }

  /// @brief Partial specialization of `shapeDistance` for the case where the
  /// second shape is a triangle. It is more efficient to pre-compute the
  /// relative transformation between the two shapes before calling GJK/EPA.
  template <typename S1>
  Scalar shapeDistance(const S1& s1, const Transform3s& tf1,
                       const TriangleP& s2, const Transform3s& tf2,
                       const bool compute_penetration, Vec3s& p1, Vec3s& p2,
                       Vec3s& normal) const {
    const Transform3s tf_1M2(tf1.inverseTimes(tf2));
    TriangleP tri(tf_1M2.transform(s2.a), tf_1M2.transform(s2.b),
                  tf_1M2.transform(s2.c));

    constexpr bool relative_transformation_already_computed = true;
    Scalar distance;
    this->runGJKAndEPA(s1, tf1, tri, tf_1M2, compute_penetration, distance, p1,
                       p2, normal, relative_transformation_already_computed);
    return distance;
  }

  /// @brief See other partial template specialization of shapeDistance above.
  template <typename S2>
  Scalar shapeDistance(const TriangleP& s1, const Transform3s& tf1,
                       const S2& s2, const Transform3s& tf2,
                       const bool compute_penetration, Vec3s& p1, Vec3s& p2,
                       Vec3s& normal) const {
    Scalar distance = this->shapeDistance<S2>(
        s2, tf2, s1, tf1, compute_penetration, p2, p1, normal);
    normal = -normal;
    return distance;
  }

 protected:
  /// @brief initialize GJK.
  /// This method assumes `minkowski_difference` has been set.
  template <typename S1, typename S2>
  void getGJKInitialGuess(const S1& s1, const S2& s2, Vec3s& guess,
                          support_func_guess_t& support_hint,
                          const Vec3s& default_guess = Vec3s(1, 0, 0)) const {
    // There is no reason not to warm-start the support function, so we always
    // do it.
    support_hint = this->support_func_cached_guess;
    // The following switch takes care of the GJK warm-start.
    switch (gjk_initial_guess) {
      case GJKInitialGuess::DefaultGuess:
        guess = default_guess;
        break;
      case GJKInitialGuess::CachedGuess:
        guess = this->cached_guess;
        break;
      case GJKInitialGuess::BoundingVolumeGuess:
        if (s1.aabb_local.volume() < 0 || s2.aabb_local.volume() < 0) {
          COAL_THROW_PRETTY(
              "computeLocalAABB must have been called on the shapes before "
              "using "
              "GJKInitialGuess::BoundingVolumeGuess.",
              std::logic_error);
        }
        guess.noalias() =
            s1.aabb_local.center() -
            (this->minkowski_difference.oR1 * s2.aabb_local.center() +
             this->minkowski_difference.ot1);
        break;
      default:
        COAL_THROW_PRETTY("Wrong initial guess for GJK.", std::logic_error);
    }
    // TODO: use gjk_initial_guess instead
    COAL_COMPILER_DIAGNOSTIC_PUSH
    COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    if (this->enable_cached_guess) {
      guess = this->cached_guess;
    }
    COAL_COMPILER_DIAGNOSTIC_POP
  }

  /// @brief Runs the GJK algorithm.
  /// @param `s1` the first shape.
  /// @param `tf1` the transformation of the first shape.
  /// @param `s2` the second shape.
  /// @param `tf2` the transformation of the second shape.
  /// @param `compute_penetration` if true and if the shapes are in found in
  /// collision, the EPA algorithm is also ran to compute penetration
  /// information.
  /// @param[out] `distance` the distance between the two shapes.
  /// @param[out] `p1` the witness point on the first shape.
  /// @param[out] `p2` the witness point on the second shape.
  /// @param[out] `normal` the normal of the collision, pointing from the first
  /// to the second shape.
  /// @param `relative_transformation_already_computed` whether the relative
  /// transformation between the two shapes has already been computed.
  /// @tparam SupportOptions, see `MinkowskiDiff::set`. Whether the support
  /// computations should take into account the shapes' swept-sphere radii
  /// during the iterations of GJK and EPA. Please leave this default value to
  /// `false` unless you know what you are doing. This template parameter is
  /// only used for debugging/testing purposes. In short, there is no need to
  /// take into account the swept sphere radius when computing supports in the
  /// iterations of GJK and EPA. GJK and EPA will correct the solution once they
  /// have converged.
  /// @return the estimate of the distance between the two shapes.
  ///
  /// @note: The variables `this->gjk.status` and `this->epa.status` can be used
  /// to examine the status of GJK and EPA.
  template <typename S1, typename S2,
            int _SupportOptions = details::SupportOptions::NoSweptSphere>
  void runGJKAndEPA(
      const S1& s1, const Transform3s& tf1, const S2& s2,
      const Transform3s& tf2, const bool compute_penetration, Scalar& distance,
      Vec3s& p1, Vec3s& p2, Vec3s& normal,
      const bool relative_transformation_already_computed = false) const {
    // Reset internal state of GJK algorithm
    if (relative_transformation_already_computed)
      this->minkowski_difference.set<_SupportOptions>(&s1, &s2);
    else
      this->minkowski_difference.set<_SupportOptions>(&s1, &s2, tf1, tf2);
    this->gjk.reset(this->gjk_max_iterations, this->gjk_tolerance);
    this->gjk.setDistanceEarlyBreak(this->distance_upper_bound);
    this->gjk.gjk_variant = this->gjk_variant;
    this->gjk.convergence_criterion = this->gjk_convergence_criterion;
    this->gjk.convergence_criterion_type = this->gjk_convergence_criterion_type;
    this->epa.status = details::EPA::Status::DidNotRun;

    // Get initial guess for GJK: default, cached or bounding volume guess
    Vec3s guess;
    support_func_guess_t support_hint;
    getGJKInitialGuess(*(this->minkowski_difference.shapes[0]),
                       *(this->minkowski_difference.shapes[1]), guess,
                       support_hint);

    this->gjk.evaluate(this->minkowski_difference, guess.cast<SolverScalar>(),
                       support_hint);

    switch (this->gjk.status) {
      case details::GJK::DidNotRun:
        COAL_ASSERT(false, "GJK did not run. It should have!",
                    std::logic_error);
        this->cached_guess = Vec3s(1, 0, 0);
        this->support_func_cached_guess.setZero();
        distance = -(std::numeric_limits<Scalar>::max)();
        p1 = p2 = normal =
            Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
        break;
      case details::GJK::Failed:
        //
        // GJK ran out of iterations.
        COAL_LOG_WARNING("GJK ran out of iterations.");
        GJKExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
        break;
      case details::GJK::NoCollisionEarlyStopped:
        //
        // Case where GJK early stopped because the distance was found to be
        // above the `distance_upper_bound`.
        // The two witness points have no meaning.
        GJKEarlyStopExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                  normal);
        COAL_ASSERT(distance >= this->gjk.distance_upper_bound -
                                    this->m_dummy_precision,
                    "The distance should be bigger than GJK's "
                    "`distance_upper_bound`.",
                    std::logic_error);
        break;
      case details::GJK::NoCollision:
        //
        // Case where GJK converged and proved that the shapes are not in
        // collision, i.e their distance is above GJK's tolerance (default
        // 1e-6).
        GJKExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
        COAL_ASSERT(std::abs((p1 - p2).norm() - distance) <=
                        this->gjk.getTolerance() + this->m_dummy_precision,
                    "The distance found by GJK should coincide with the "
                    "distance between the closest points.",
                    std::logic_error);
        break;
      //
      // Next are the cases where GJK found the shapes to be in collision, i.e.
      // their distance is below GJK's tolerance (default 1e-6).
      case details::GJK::CollisionWithPenetrationInformation:
        GJKExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
        COAL_ASSERT(
            distance <= this->gjk.getTolerance() + this->m_dummy_precision,
            "The distance found by GJK should be negative or at "
            "least below GJK's tolerance.",
            std::logic_error);
        break;
      case details::GJK::Collision:
        if (!compute_penetration) {
          // Skip EPA and set the witness points and the normal to nans.
          GJKCollisionExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                    normal);
        } else {
          //
          // GJK was not enough to recover the penetration information.
          // We need to run the EPA algorithm to find the witness points,
          // penetration depth and the normal.

          // Reset EPA algorithm. Potentially allocate memory if
          // `epa_max_face_num` or `epa_max_vertex_num` are bigger than EPA's
          // current storage.
          this->epa.reset(this->epa_max_iterations, this->epa_tolerance);

          // TODO: understand why EPA's performance is so bad on cylinders and
          // cones.
          this->epa.evaluate(this->gjk, (-guess).cast<SolverScalar>());

          switch (epa.status) {
            //
            // In the following switch cases, until the "Valid" case,
            // EPA either ran out of iterations, of faces or of vertices.
            // The depth, witness points and the normal are still valid,
            // simply not at the precision of EPA's tolerance.
            // The flag `COAL_ENABLE_LOGGING` enables feebdack on these
            // cases.
            //
            // TODO: Remove OutOfFaces and OutOfVertices statuses and simply
            // compute the upper bound on max faces and max vertices as a
            // function of the number of iterations.
            case details::EPA::OutOfFaces:
              COAL_LOG_WARNING("EPA ran out of faces.");
              EPAExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
              break;
            case details::EPA::OutOfVertices:
              COAL_LOG_WARNING("EPA ran out of vertices.");
              EPAExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
              break;
            case details::EPA::Failed:
              COAL_LOG_WARNING("EPA ran out of iterations.");
              EPAExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
              break;
            case details::EPA::Valid:
            case details::EPA::AccuracyReached:
              COAL_ASSERT(
                  -epa.depth <= epa.getTolerance() + this->m_dummy_precision,
                  "EPA's penetration distance should be negative (or "
                  "at least below EPA's tolerance).",
                  std::logic_error);
              EPAExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
              break;
            case details::EPA::Degenerated:
              COAL_LOG_WARNING(
                  "EPA warning: created a polytope with a degenerated face.");
              EPAExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
              break;
            case details::EPA::NonConvex:
              COAL_LOG_WARNING(
                  "EPA warning: EPA got called onto non-convex shapes.");
              EPAExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
              break;
            case details::EPA::InvalidHull:
              COAL_LOG_WARNING("EPA warning: created an invalid polytope.");
              EPAExtractWitnessPointsAndNormal(tf1, distance, p1, p2, normal);
              break;
            case details::EPA::DidNotRun:
              COAL_ASSERT(false, "EPA did not run. It should have!",
                          std::logic_error);
              COAL_LOG_ERROR("EPA error: did not run. It should have.");
              EPAFailedExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                     normal);
              break;
            case details::EPA::FallBack:
              COAL_ASSERT(
                  false,
                  "EPA went into fallback mode. It should never do that.",
                  std::logic_error);
              COAL_LOG_ERROR("EPA error: FallBack.");
              EPAFailedExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                     normal);
              break;
          }
        }
        break;  // End of case details::GJK::Collision
    }
  }

  void GJKEarlyStopExtractWitnessPointsAndNormal(const Transform3s& tf1,
                                                 Scalar& distance, Vec3s& p1,
                                                 Vec3s& p2,
                                                 Vec3s& normal) const {
    COAL_UNUSED_VARIABLE(tf1);
    // Cache gjk result for potential future call to this GJKSolver.
    this->cached_guess = this->gjk.ray.cast<Scalar>();
    this->support_func_cached_guess = this->gjk.support_hint;

    distance = Scalar(this->gjk.distance);
    p1 = p2 = normal =
        Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
    // If we absolutely want to return some witness points, we could use
    // the following code (or simply merge the early stopped case with the
    // valid case below):
    // gjk.getWitnessPointsAndNormal(minkowski_difference, p1, p2, normal);
    // p1 = tf1.transform(p1);
    // p2 = tf1.transform(p2);
    // normal = tf1.getRotation() * normal;
  }

  void GJKExtractWitnessPointsAndNormal(const Transform3s& tf1,
                                        Scalar& distance, Vec3s& p1, Vec3s& p2,
                                        Vec3s& normal) const {
    // Apart from early stopping, there are two cases where GJK says there is no
    // collision:
    // 1. GJK proved the distance is above its tolerance (default 1e-6).
    // 2. GJK ran out of iterations.
    // In any case, `gjk.ray`'s norm is bigger than GJK's tolerance and thus
    // it can safely be normalized.
    COAL_ASSERT(this->gjk.ray.norm() >
                    this->gjk.getTolerance() - this->m_dummy_precision,
                "The norm of GJK's ray should be bigger than GJK's tolerance.",
                std::logic_error);
    // Cache gjk result for potential future call to this GJKSolver.
    this->cached_guess = this->gjk.ray.cast<Scalar>();
    this->support_func_cached_guess = this->gjk.support_hint;

    distance = Scalar(this->gjk.distance);
    // TODO: On degenerated case, the closest points may be non-unique.
    // (i.e. an object face normal is colinear to `gjk.ray`)
    Vec3ps p1_, p2_, normal_;
    gjk.getWitnessPointsAndNormal(this->minkowski_difference, p1_, p2_,
                                  normal_);
    p1 = p1_.cast<Scalar>();
    p2 = p2_.cast<Scalar>();
    normal = normal_.cast<Scalar>();
    Vec3s p = tf1.transform(0.5 * (p1 + p2));
    normal = tf1.getRotation() * normal;
    p1.noalias() = p - 0.5 * distance * normal;
    p2.noalias() = p + 0.5 * distance * normal;
  }

  void GJKCollisionExtractWitnessPointsAndNormal(const Transform3s& tf1,
                                                 Scalar& distance, Vec3s& p1,
                                                 Vec3s& p2,
                                                 Vec3s& normal) const {
    COAL_UNUSED_VARIABLE(tf1);
    COAL_ASSERT(this->gjk.distance <=
                    this->gjk.getTolerance() + this->m_dummy_precision,
                "The distance should be lower than GJK's tolerance.",
                std::logic_error);
    // Because GJK has returned the `Collision` status and EPA has not run,
    // we purposefully do not cache the result of GJK, because ray is zero.
    // However, we can cache the support function hint.
    // this->cached_guess = this->gjk.ray;
    this->support_func_cached_guess = this->gjk.support_hint;

    distance = Scalar(this->gjk.distance);
    p1 = p2 = normal =
        Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
  }

  void EPAExtractWitnessPointsAndNormal(const Transform3s& tf1,
                                        Scalar& distance, Vec3s& p1, Vec3s& p2,
                                        Vec3s& normal) const {
    // Cache EPA result for potential future call to this GJKSolver.
    // This caching allows to warm-start the next GJK call.
    this->cached_guess = -(this->epa.depth * this->epa.normal).cast<Scalar>();
    this->support_func_cached_guess = this->epa.support_hint;
    distance = (std::min)(Scalar(0), -Scalar(this->epa.depth));
    Vec3ps p1_, p2_, normal_;
    this->epa.getWitnessPointsAndNormal(this->minkowski_difference, p1_, p2_,
                                        normal_);
    p1 = p1_.cast<Scalar>();
    p2 = p2_.cast<Scalar>();
    normal = normal_.cast<Scalar>();
    // The following is very important to understand why EPA can sometimes
    // return a normal that is not colinear to the vector $p_1 - p_2$ when
    // working with tolerances like $\epsilon = 10^{-3}$.
    // It can be resumed with a simple idea:
    //     EPA is an algorithm meant to find the penetration depth and the
    //     normal. It is not meant to find the closest points.
    // Again, the issue here is **not** the normal, it's $p_1$ and $p_2$.
    //
    // More details:
    // We'll denote $S_1$ and $S_2$ the two shapes, $n$ the normal and $p_1$ and
    // $p_2$ the witness points. In theory, when EPA converges to $\epsilon =
    // 0$, the normal and witness points verify the following property (P):
    //   - $p_1 \in \partial \sigma_{S_1}(n)$,
    //   - $p_2 \in \partial \sigma_{S_2}(-n),
    // where $\sigma_{S_1}$ and $\sigma_{S_2}$ are the support functions of
    // $S_1$ and $S_2$. The $\partial \sigma(n)$ simply denotes the support set
    // of the support function in the direction $n$. (Note: I am leaving out the
    // details of frame choice for the support function, to avoid making the
    // mathematical notation too heavy.)
    // --> In practice, EPA converges to $\epsilon > 0$.
    // On polytopes and the likes, this does not change much and the property
    // given above is still valid.
    // --> However, this is very different on curved surfaces, such as
    // ellipsoids, cylinders, cones, capsules etc. For these shapes, converging
    // at $\epsilon = 10^{-6}$ or to $\epsilon = 10^{-3}$ does not change the
    // normal much, but the property (P) given above is no longer valid, which
    // means that the points $p_1$ and $p_2$ do not necessarily belong to the
    // support sets in the direction of $n$ and thus $n$ and $p_1 - p_2$ are not
    // colinear.
    //
    // Do not panic! This is fine.
    // Although the property above is not verified, it's almost verified,
    // meaning that $p_1$ and $p_2$ belong to support sets in directions that
    // are very close to $n$.
    //
    // Solution to compute better $p_1$ and $p_2$:
    // We compute the middle points of the current $p_1$ and $p_2$ and we use
    // the normal and the distance given by EPA to compute the new $p_1$ and
    // $p_2$.
    Vec3s p = tf1.transform(0.5 * (p1 + p2));
    normal = tf1.getRotation() * normal;
    p1.noalias() = p - 0.5 * distance * normal;
    p2.noalias() = p + 0.5 * distance * normal;
  }

  void EPAFailedExtractWitnessPointsAndNormal(const Transform3s& tf1,
                                              Scalar& distance, Vec3s& p1,
                                              Vec3s& p2, Vec3s& normal) const {
    this->cached_guess = Vec3s(1, 0, 0);
    this->support_func_cached_guess.setZero();

    COAL_UNUSED_VARIABLE(tf1);
    distance = -(std::numeric_limits<Scalar>::max)();
    p1 = p2 = normal =
        Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
  }
};

}  // namespace coal

#endif
