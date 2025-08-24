/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

/** \author Jia Pan */

#ifndef COAL_COLLISION_DATA_H
#define COAL_COLLISION_DATA_H

#include <vector>
#include <array>
#include <set>
#include <limits>
#include <numeric>

#include "coal/collision_object.h"
#include "coal/config.hh"
#include "coal/data_types.h"
#include "coal/timings.h"
#include "coal/narrowphase/narrowphase_defaults.h"
#include "coal/logging.h"

namespace coal {

/// @brief Contact information returned by collision
struct COAL_DLLAPI Contact {
  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief contact primitive in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief contact primitive in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;

  /// @brief contact normal, pointing from o1 to o2.
  /// The normal defined as the normalized separation vector:
  /// normal = (p2 - p1) / dist(o1, o2), where p1 = nearest_points[0]
  /// belongs to o1 and p2 = nearest_points[1] belongs to o2 and dist(o1, o2) is
  /// the **signed** distance between o1 and o2. The normal always points from
  /// o1 to o2.
  /// @note The separation vector is the smallest vector such that if o1 is
  /// translated by it, o1 and o2 are in touching contact (they share at least
  /// one contact point but have a zero intersection volume). If the shapes
  /// overlap, dist(o1, o2) = -((p2-p1).norm()). Otherwise, dist(o1, o2) =
  /// (p2-p1).norm().
  Vec3s normal;

  /// @brief nearest points associated to this contact.
  /// @note Also referred as "witness points" in other collision libraries.
  /// The points p1 = nearest_points[0] and p2 = nearest_points[1] verify the
  /// property that dist(o1, o2) * (p1 - p2) is the separation vector between o1
  /// and o2, with dist(o1, o2) being the **signed** distance separating o1 from
  /// o2. See \ref DistanceResult::normal for the definition of the separation
  /// vector. If o1 and o2 have multiple contacts, the nearest_points are
  /// associated with the contact which has the greatest penetration depth.
  /// TODO (louis): rename `nearest_points` to `witness_points`.
  std::array<Vec3s, 2> nearest_points;

  /// @brief contact position, in world space
  Vec3s pos;

  /// @brief penetration depth
  Scalar penetration_depth;

  /// @brief invalid contact primitive information
  static const int NONE = -1;

  /// @brief Default constructor
  Contact() : o1(NULL), o2(NULL), b1(NONE), b2(NONE) {
    penetration_depth = (std::numeric_limits<Scalar>::max)();
    nearest_points[0] = nearest_points[1] = normal = pos =
        Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
  }

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_,
          int b2_)
      : o1(o1_), o2(o2_), b1(b1_), b2(b2_) {
    penetration_depth = (std::numeric_limits<Scalar>::max)();
    nearest_points[0] = nearest_points[1] = normal = pos =
        Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
  }

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_,
          int b2_, const Vec3s& pos_, const Vec3s& normal_, Scalar depth_)
      : o1(o1_),
        o2(o2_),
        b1(b1_),
        b2(b2_),
        normal(normal_),
        nearest_points{pos_ - (depth_ * normal_ / 2),
                       pos_ + (depth_ * normal_ / 2)},
        pos(pos_),
        penetration_depth(depth_) {}

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_,
          int b2_, const Vec3s& p1, const Vec3s& p2, const Vec3s& normal_,
          Scalar depth_)
      : o1(o1_),
        o2(o2_),
        b1(b1_),
        b2(b2_),
        normal(normal_),
        nearest_points{p1, p2},
        pos((p1 + p2) / 2),
        penetration_depth(depth_) {}

  bool operator<(const Contact& other) const {
    if (b1 == other.b1) return b2 < other.b2;
    return b1 < other.b1;
  }

  bool operator==(const Contact& other) const {
    return o1 == other.o1 && o2 == other.o2 && b1 == other.b1 &&
           b2 == other.b2 && normal == other.normal && pos == other.pos &&
           nearest_points[0] == other.nearest_points[0] &&
           nearest_points[1] == other.nearest_points[1] &&
           penetration_depth == other.penetration_depth;
  }

  bool operator!=(const Contact& other) const { return !(*this == other); }

  Scalar getDistanceToCollision(const CollisionRequest& request) const;
};

struct QueryResult;

/// @brief base class for all query requests
struct COAL_DLLAPI QueryRequest {
  // @brief Initial guess to use for the GJK algorithm
  GJKInitialGuess gjk_initial_guess;

  /// @brief whether enable gjk initial guess
  /// @Deprecated Use gjk_initial_guess instead
  COAL_DEPRECATED_MESSAGE(Use gjk_initial_guess instead)
  bool enable_cached_gjk_guess;

  /// @brief the gjk initial guess set by user
  mutable Vec3s cached_gjk_guess;

  /// @brief the support function initial guess set by user
  mutable support_func_guess_t cached_support_func_guess;

  /// @brief maximum iteration for the GJK algorithm
  size_t gjk_max_iterations;

  /// @brief tolerance for the GJK algorithm.
  /// Note: This tolerance determines the precision on the estimated distance
  /// between two geometries which are not in collision.
  /// It is recommended to not set this tolerance to less than 1e-6.
  Scalar gjk_tolerance;

  /// @brief whether to enable the Nesterov accleration of GJK
  GJKVariant gjk_variant;

  /// @brief convergence criterion used to stop GJK
  GJKConvergenceCriterion gjk_convergence_criterion;

  /// @brief convergence criterion used to stop GJK
  GJKConvergenceCriterionType gjk_convergence_criterion_type;

  /// @brief max number of iterations for EPA
  size_t epa_max_iterations;

  /// @brief tolerance for EPA.
  /// Note: This tolerance determines the precision on the estimated distance
  /// between two geometries which are in collision.
  /// It is recommended to not set this tolerance to less than 1e-6.
  /// Also, setting EPA's tolerance to less than GJK's is not recommended.
  Scalar epa_tolerance;

  /// @brief enable timings when performing collision/distance request
  bool enable_timings;

  /// @brief threshold below which a collision is considered.
  Scalar collision_distance_threshold;

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  /// @brief Default constructor.
  QueryRequest()
      : gjk_initial_guess(GJKInitialGuess::DefaultGuess),
        enable_cached_gjk_guess(false),
        cached_gjk_guess(1, 0, 0),
        cached_support_func_guess(support_func_guess_t::Zero()),
        gjk_max_iterations(GJK_DEFAULT_MAX_ITERATIONS),
        gjk_tolerance(GJK_DEFAULT_TOLERANCE),
        gjk_variant(GJKVariant::DefaultGJK),
        gjk_convergence_criterion(GJKConvergenceCriterion::Default),
        gjk_convergence_criterion_type(GJKConvergenceCriterionType::Relative),
        epa_max_iterations(EPA_DEFAULT_MAX_ITERATIONS),
        epa_tolerance(EPA_DEFAULT_TOLERANCE),
        enable_timings(false),
        collision_distance_threshold(
            Eigen::NumTraits<Scalar>::dummy_precision()) {}

  /// @brief Copy  constructor.
  QueryRequest(const QueryRequest& other) = default;

  /// @brief Copy  assignment operator.
  QueryRequest& operator=(const QueryRequest& other) = default;
  COAL_COMPILER_DIAGNOSTIC_POP

  /// @brief Updates the guess for the internal GJK algorithm in order to
  /// warm-start it when reusing this collision request on the same collision
  /// pair.
  /// @note The option `gjk_initial_guess` must be set to
  /// `GJKInitialGuess::CachedGuess` for this to work.
  void updateGuess(const QueryResult& result) const;

  /// @brief whether two QueryRequest are the same or not
  inline bool operator==(const QueryRequest& other) const {
    COAL_COMPILER_DIAGNOSTIC_PUSH
    COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    return gjk_initial_guess == other.gjk_initial_guess &&
           enable_cached_gjk_guess == other.enable_cached_gjk_guess &&
           gjk_variant == other.gjk_variant &&
           gjk_convergence_criterion == other.gjk_convergence_criterion &&
           gjk_convergence_criterion_type ==
               other.gjk_convergence_criterion_type &&
           gjk_tolerance == other.gjk_tolerance &&
           gjk_max_iterations == other.gjk_max_iterations &&
           cached_gjk_guess == other.cached_gjk_guess &&
           cached_support_func_guess == other.cached_support_func_guess &&
           epa_max_iterations == other.epa_max_iterations &&
           epa_tolerance == other.epa_tolerance &&
           enable_timings == other.enable_timings &&
           collision_distance_threshold == other.collision_distance_threshold;
    COAL_COMPILER_DIAGNOSTIC_POP
  }
};

/// @brief base class for all query results
struct COAL_DLLAPI QueryResult {
  /// @brief stores the last GJK ray when relevant.
  Vec3s cached_gjk_guess;

  /// @brief stores the last support function vertex index, when relevant.
  support_func_guess_t cached_support_func_guess;

  /// @brief timings for the given request
  CPUTimes timings;

  QueryResult()
      : cached_gjk_guess(Vec3s::Zero()),
        cached_support_func_guess(support_func_guess_t::Constant(-1)) {}
};

inline void QueryRequest::updateGuess(const QueryResult& result) const {
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  if (gjk_initial_guess == GJKInitialGuess::CachedGuess ||
      enable_cached_gjk_guess) {
    cached_gjk_guess = result.cached_gjk_guess;
    cached_support_func_guess = result.cached_support_func_guess;
  }
  COAL_COMPILER_DIAGNOSTIC_POP
}

struct CollisionResult;

/// @brief flag declaration for specifying required params in CollisionResult
enum CollisionRequestFlag {
  CONTACT = 0x00001,
  DISTANCE_LOWER_BOUND = 0x00002,
  NO_REQUEST = 0x01000
};

/// @brief request to the collision algorithm
struct COAL_DLLAPI CollisionRequest : QueryRequest {
  /// @brief The maximum number of contacts that can be returned
  size_t num_max_contacts;

  /// @brief whether the contact information (normal, penetration depth and
  /// contact position) will return.
  bool enable_contact;

  /// Whether a lower bound on distance is returned when objects are disjoint
  COAL_DEPRECATED_MESSAGE(A lower bound on distance is always computed.)
  bool enable_distance_lower_bound;

  /// @brief Distance below which objects are considered in collision.
  /// See \ref coal_collision_and_distance_lower_bound_computation
  /// @note If set to -inf, the objects tested for collision are considered
  ///       as collision free and no test is actually performed by functions
  ///       coal::collide of class coal::ComputeCollision.
  Scalar security_margin;

  /// @brief Distance below which bounding volumes are broken down.
  /// See \ref coal_collision_and_distance_lower_bound_computation
  Scalar break_distance;

  /// @brief Distance above which GJK solver makes an early stopping.
  /// GJK stops searching for the closest points when it proves that the
  /// distance between two geometries is above this threshold.
  ///
  /// @remarks Consequently, the closest points might be incorrect, but allows
  /// to save computational resources.
  Scalar distance_upper_bound;

  /// @brief Constructor from a flag and a maximal number of contacts.
  ///
  /// @param[in] flag Collision request flag
  /// @param[in] num_max_contacts  Maximal number of allowed contacts
  ///
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  CollisionRequest(const CollisionRequestFlag flag, size_t num_max_contacts_)
      : num_max_contacts(num_max_contacts_),
        enable_contact(flag & CONTACT),
        enable_distance_lower_bound(flag & DISTANCE_LOWER_BOUND),
        security_margin(0),
        break_distance(Scalar(1e-3)),
        distance_upper_bound((std::numeric_limits<Scalar>::max)()) {}

  /// @brief Default constructor.
  CollisionRequest()
      : num_max_contacts(1),
        enable_contact(true),
        enable_distance_lower_bound(false),
        security_margin(0),
        break_distance(Scalar(1e-3)),
        distance_upper_bound((std::numeric_limits<Scalar>::max)()) {}
  COAL_COMPILER_DIAGNOSTIC_POP

  bool isSatisfied(const CollisionResult& result) const;

  /// @brief whether two CollisionRequest are the same or not
  inline bool operator==(const CollisionRequest& other) const {
    COAL_COMPILER_DIAGNOSTIC_PUSH
    COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    return QueryRequest::operator==(other) &&
           num_max_contacts == other.num_max_contacts &&
           enable_contact == other.enable_contact &&
           enable_distance_lower_bound == other.enable_distance_lower_bound &&
           security_margin == other.security_margin &&
           break_distance == other.break_distance &&
           distance_upper_bound == other.distance_upper_bound;
    COAL_COMPILER_DIAGNOSTIC_POP
  }
};

inline Scalar Contact::getDistanceToCollision(
    const CollisionRequest& request) const {
  return penetration_depth - request.security_margin;
}

/// @brief collision result
struct COAL_DLLAPI CollisionResult : QueryResult {
 private:
  /// @brief contact information
  std::vector<Contact> contacts;

 public:
  /// Lower bound on distance between objects if they are disjoint.
  /// See \ref coal_collision_and_distance_lower_bound_computation
  /// @note Always computed. If \ref CollisionRequest::distance_upper_bound is
  /// set to infinity, distance_lower_bound is the actual distance between the
  /// shapes.
  Scalar distance_lower_bound;

  /// @brief normal associated to nearest_points.
  /// Same as `CollisionResult::nearest_points` but for the normal.
  Vec3s normal;

  /// @brief nearest points.
  /// A `CollisionResult` can have multiple contacts.
  /// The nearest points in `CollisionResults` correspond to the witness points
  /// associated with the smallest distance i.e the `distance_lower_bound`.
  /// For bounding volumes and BVHs, these nearest points are available
  /// only when distance_lower_bound is inferior to
  /// CollisionRequest::break_distance.
  std::array<Vec3s, 2> nearest_points;

 public:
  CollisionResult()
      : distance_lower_bound((std::numeric_limits<Scalar>::max)()) {
    nearest_points[0] = nearest_points[1] = normal =
        Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
  }

  /// @brief Update the lower bound only if the distance is inferior.
  inline void updateDistanceLowerBound(const Scalar& distance_lower_bound_) {
    if (distance_lower_bound_ < distance_lower_bound)
      distance_lower_bound = distance_lower_bound_;
  }

  /// @brief add one contact into result structure
  inline void addContact(const Contact& c) { contacts.push_back(c); }

  /// @brief whether two CollisionResult are the same or not
  inline bool operator==(const CollisionResult& other) const {
    return contacts == other.contacts &&
           distance_lower_bound == other.distance_lower_bound &&
           nearest_points[0] == other.nearest_points[0] &&
           nearest_points[1] == other.nearest_points[1] &&
           normal == other.normal;
  }

  /// @brief return binary collision result
  bool isCollision() const { return contacts.size() > 0; }

  /// @brief number of contacts found
  size_t numContacts() const { return contacts.size(); }

  /// @brief get the i-th contact calculated
  const Contact& getContact(size_t i) const {
    if (contacts.size() == 0)
      COAL_THROW_PRETTY(
          "The number of contacts is zero. No Contact can be returned.",
          std::invalid_argument);

    if (i < contacts.size())
      return contacts[i];
    else
      return contacts.back();
  }

  /// @brief set the i-th contact calculated
  void setContact(size_t i, const Contact& c) {
    if (contacts.size() == 0)
      COAL_THROW_PRETTY(
          "The number of contacts is zero. No Contact can be returned.",
          std::invalid_argument);

    if (i < contacts.size())
      contacts[i] = c;
    else
      contacts.back() = c;
  }

  /// @brief get all the contacts
  void getContacts(std::vector<Contact>& contacts_) const {
    contacts_.resize(contacts.size());
    std::copy(contacts.begin(), contacts.end(), contacts_.begin());
  }

  const std::vector<Contact>& getContacts() const { return contacts; }

  /// @brief clear the results obtained
  void clear() {
    distance_lower_bound = (std::numeric_limits<Scalar>::max)();
    contacts.clear();
    timings.clear();
    nearest_points[0] = nearest_points[1] = normal =
        Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN());
  }

  /// @brief reposition Contact objects when fcl inverts them
  /// during their construction.
  void swapObjects();
};

/// @brief This structure allows to encode contact patches.
/// A contact patch is defined by a set of points belonging to a subset of a
/// plane passing by `p` and supported by `n`, where `n = Contact::normal` and
/// `p = Contact::pos`. If we denote by P this plane and by S1 and S2 the first
/// and second shape of a collision pair, a contact patch is represented as a
/// polytope which vertices all belong to `P & S1 & S2`, where `&` denotes the
/// set-intersection. Since a contact patch is a subset of a plane supported by
/// `n`, it has a preferred direction. In Coal, the `Contact::normal` points
/// from S1 to S2. In the same way, a contact patch points by default from S1
/// to S2.
///
/// @note For now (April 2024), a `ContactPatch` is a polygon (2D polytope),
/// so the points of the set, forming the convex-hull of the polytope, are
/// stored in a counter-clockwise fashion.
/// @note If needed, the internal algorithms of Coal can easily be extended
/// to compute a contact volume instead of a contact patch.
struct COAL_DLLAPI ContactPatch {
 public:
  using Polygon = std::vector<Vec2s>;

  /// @brief Frame of the set, expressed in the world coordinates.
  /// The z-axis of the frame's rotation is the contact patch normal.
  Transform3s tf;

  /// @brief Direction of ContactPatch.
  /// When doing collision detection, the convention of Coal is that the
  /// normal always points from the first to the second shape of the collision
  /// pair i.e. from shape1 to shape2 when calling `collide(shape1, shape2)`.
  /// The PatchDirection enum allows to identify if the patch points from
  /// shape 1 to shape 2 (Default type) or from shape 2 to shape 1 (Inverted
  /// type). The Inverted type should only be used for internal Coal
  /// computations (it allows to properly define two separate contact patches in
  /// the same frame).
  enum PatchDirection { DEFAULT = 0, INVERTED = 1 };

  /// @brief Direction of this contact patch.
  PatchDirection direction;

  /// @brief Penetration depth of the contact patch. This value corresponds to
  /// the signed distance `d` between the shapes.
  /// @note For each contact point `p` in the patch of normal `n`, `p1 = p -
  /// 0.5*d*n` and `p2 = p + 0.5*d*n` define a pair of witness points. `p1`
  /// belongs to the surface of the first shape and `p2` belongs to the surface
  /// of the second shape. For any pair of witness points, we always have `p2 -
  /// p1 = d * n`. The vector `d * n` is called a minimum separation vector:
  /// if S1 is translated by it, S1 and S2 are not in collision anymore.
  /// @note Although there may exist multiple minimum separation vectors between
  /// two shapes, the term "minimum" comes from the fact that it's impossible to
  /// find a different separation vector which has a smaller norm than `d * n`.
  Scalar penetration_depth;

  /// @brief Default maximum size of the polygon representing the contact patch.
  /// Used to pre-allocate memory for the patch.
  static constexpr size_t default_preallocated_size = 12;

 protected:
  /// @brief Container for the vertices of the set.
  Polygon m_points;

 public:
  /// @brief Default constructor.
  /// Note: the preallocated size does not determine the maximum number of
  /// points in the patch, it only serves as preallocation if the maximum size
  /// of the patch is known in advance. Coal will automatically expand/shrink
  /// the contact patch if needed.
  explicit ContactPatch(size_t preallocated_size = default_preallocated_size)
      : tf(Transform3s::Identity()),
        direction(PatchDirection::DEFAULT),
        penetration_depth(0) {
    this->m_points.reserve(preallocated_size);
  }

  /// @brief Normal of the contact patch, expressed in the WORLD frame.
  Vec3s getNormal() const {
    if (this->direction == PatchDirection::INVERTED) {
      return -this->tf.rotation().col(2);
    }
    return this->tf.rotation().col(2);
  }

  /// @brief Returns the number of points in the contact patch.
  size_t size() const { return this->m_points.size(); }

  /// @brief Add a 3D point to the set, expressed in the world frame.
  /// @note This function takes a 3D point and expresses it in the local frame
  /// of the set. It then takes only the x and y components of the vector,
  /// effectively doing a projection onto the plane to which the set belongs.
  /// TODO(louis): if necessary, we can store the offset to the plane (x, y).
  void addPoint(const Vec3s& point_3d) {
    const Vec3s point = this->tf.inverseTransform(point_3d);
    this->m_points.emplace_back(point.template head<2>());
  }

  /// @brief Get the i-th point of the set, expressed in the 3D world frame.
  Vec3s getPoint(const size_t i) const {
    Vec3s point(0, 0, 0);
    point.head<2>() = this->point(i);
    point = tf.transform(point);
    return point;
  }

  /// @brief Get the i-th point of the contact patch, projected back onto the
  /// first shape of the collision pair. This point is expressed in the 3D
  /// world frame.
  Vec3s getPointShape1(const size_t i) const {
    Vec3s point = this->getPoint(i);
    point -= (this->penetration_depth / 2) * this->getNormal();
    return point;
  }

  /// @brief Get the i-th point of the contact patch, projected back onto the
  /// first shape of the collision pair. This 3D point is expressed in the world
  /// frame.
  Vec3s getPointShape2(const size_t i) const {
    Vec3s point = this->getPoint(i);
    point += (this->penetration_depth / 2) * this->getNormal();
    return point;
  }

  /// @brief Getter for the 2D points in the set.
  Polygon& points() { return this->m_points; }

  /// @brief Const getter for the 2D points in the set.
  const Polygon& points() const { return this->m_points; }

  /// @brief Getter for the i-th 2D point in the set.
  Vec2s& point(const size_t i) {
    COAL_ASSERT(this->m_points.size() > 0, "Patch is empty.", std::logic_error);
    if (i < this->m_points.size()) {
      return this->m_points[i];
    }
    return this->m_points.back();
  }

  /// @brief Const getter for the i-th 2D point in the set.
  const Vec2s& point(const size_t i) const {
    COAL_ASSERT(this->m_points.size() > 0, "Patch is empty.", std::logic_error);
    if (i < this->m_points.size()) {
      return this->m_points[i];
    }
    return this->m_points.back();
  }

  /// @brief Clear the set.
  void clear() {
    this->m_points.clear();
    this->tf.setIdentity();
    this->penetration_depth = 0;
  }

  /// @brief Whether two contact patches are the same or not.
  /// @note This compares the two sets terms by terms.
  /// However, two contact patches can be identical, but have a different
  /// order for their points. Use `isEqual` in this case.
  bool operator==(const ContactPatch& other) const {
    return this->tf == other.tf && this->direction == other.direction &&
           this->penetration_depth == other.penetration_depth &&
           this->points() == other.points();
  }

  /// @brief Whether two contact patches are the same or not.
  /// Checks for different order of the points.
  bool isSame(
      const ContactPatch& other,
      const Scalar tol = Eigen::NumTraits<Scalar>::dummy_precision()) const {
    // The x and y axis of the set are arbitrary, but the z axis is
    // always the normal. The position of the origin of the frame is also
    // arbitrary. So we only check if the normals are the same.
    if (!this->getNormal().isApprox(other.getNormal(), tol)) {
      return false;
    }

    if (std::abs(this->penetration_depth - other.penetration_depth) > tol) {
      return false;
    }

    if (this->direction != other.direction) {
      return false;
    }

    if (this->size() != other.size()) {
      return false;
    }

    // Check all points of the contact patch.
    for (size_t i = 0; i < this->size(); ++i) {
      bool found = false;
      const Vec3s pi = this->getPoint(i);
      for (size_t j = 0; j < other.size(); ++j) {
        const Vec3s other_pj = other.getPoint(j);
        if (pi.isApprox(other_pj, tol)) {
          found = true;
        }
      }
      if (!found) {
        return false;
      }
    }

    return true;
  }
};

/// @brief Construct a frame from a `Contact`'s position and normal.
/// Because both `Contact`'s position and normal are expressed in the world
/// frame, this frame is also expressed w.r.t the world frame.
/// The origin of the frame is `contact.pos` and the z-axis of the frame is
/// `contact.normal`.
inline void constructContactPatchFrameFromContact(const Contact& contact,
                                                  ContactPatch& contact_patch) {
  contact_patch.penetration_depth = contact.penetration_depth;
  contact_patch.tf.translation() = contact.pos;
  contact_patch.tf.rotation() =
      constructOrthonormalBasisFromVector(contact.normal);
  contact_patch.direction = ContactPatch::PatchDirection::DEFAULT;
}

/// @brief Structure used for internal computations. A support set and a
/// contact patch can be represented by the same structure. In fact, a contact
/// patch is the intersection of two support sets, one with
/// `PatchDirection::DEFAULT` and one with `PatchDirection::INVERTED`.
/// @note A support set with `DEFAULT` direction is the support set of a shape
/// in the direction given by `+n`, where `n` is the z-axis of the frame's
/// patch rotation. An `INVERTED` support set is the support set of a shape in
/// the direction `-n`.
using SupportSet = ContactPatch;

/// @brief Request for a contact patch computation.
struct COAL_DLLAPI ContactPatchRequest {
  /// @brief Maximum number of contact patches that will be computed.
  size_t max_num_patch;

 protected:
  /// @brief Maximum samples to compute the support sets of curved shapes,
  /// i.e. when the normal is perpendicular to the base of a cylinder. For
  /// now, only relevant for Cone and Cylinder. In the future this might be
  /// extended to Sphere and Ellipsoid.
  size_t m_num_samples_curved_shapes;

  /// @brief Tolerance below which points are added to a contact patch.
  /// In details, given two shapes S1 and S2, a contact patch is the triple
  /// intersection between the separating plane (P) (passing by `Contact::pos`
  /// and supported by `Contact::normal`), S1 and S2; i.e. a contact patch is
  /// `P & S1 & S2` if we denote `&` the set intersection operator. If a point
  /// p1 of S1 is at a distance below `patch_tolerance` from the separating
  /// plane, it is taken into account in the computation of the contact patch.
  /// Otherwise, it is not used for the computation.
  /// @note Needs to be positive.
  Scalar m_patch_tolerance;

 public:
  /// @brief Default constructor.
  /// @param max_num_patch maximum number of contact patches per collision pair.
  /// @param max_sub_patch_size maximum size of each sub contact patch. Each
  /// contact patch contains an internal representation for an inscribed sub
  /// contact patch. This allows physics simulation to always work with a
  /// predetermined maximum size for each contact patch. A sub contact patch is
  /// simply a subset of the vertices of a contact patch.
  /// @param num_samples_curved_shapes for shapes like cones and cylinders,
  /// which have smooth basis (circles in this case), we need to sample a
  /// certain amount of point of this basis.
  /// @param patch_tolerance the tolerance below which a point of a shape is
  /// considered to belong to the support set of this shape in the direction of
  /// the normal. Said otherwise, `patch_tolerance` determines the "thickness"
  /// of the separating plane between shapes of a collision pair.
  explicit ContactPatchRequest(size_t max_num_patch = 1,
                               size_t num_samples_curved_shapes =
                                   ContactPatch::default_preallocated_size,
                               Scalar patch_tolerance = Scalar(1e-3))
      : max_num_patch(max_num_patch) {
    this->setNumSamplesCurvedShapes(num_samples_curved_shapes);
    this->setPatchTolerance(patch_tolerance);
  }

  /// @brief Construct a contact patch request from a collision request.
  explicit ContactPatchRequest(const CollisionRequest& collision_request,
                               size_t num_samples_curved_shapes =
                                   ContactPatch::default_preallocated_size,
                               Scalar patch_tolerance = Scalar(1e-3))
      : max_num_patch(collision_request.num_max_contacts) {
    this->setNumSamplesCurvedShapes(num_samples_curved_shapes);
    this->setPatchTolerance(patch_tolerance);
  }

  /// @copydoc m_num_samples_curved_shapes
  void setNumSamplesCurvedShapes(const size_t num_samples_curved_shapes) {
    if (num_samples_curved_shapes < 3) {
      COAL_LOG_WARNING(
          "`num_samples_curved_shapes` cannot be lower than 3. Setting it to "
          "3 to prevent bugs.");
      this->m_num_samples_curved_shapes = 3;
    } else {
      this->m_num_samples_curved_shapes = num_samples_curved_shapes;
    }
  }

  /// @copydoc m_num_samples_curved_shapes
  size_t getNumSamplesCurvedShapes() const {
    return this->m_num_samples_curved_shapes;
  }

  /// @copydoc m_patch_tolerance
  void setPatchTolerance(const Scalar patch_tolerance) {
    if (patch_tolerance < 0) {
      COAL_LOG_WARNING(
          "`patch_tolerance` cannot be negative. Setting it to 0 to prevent "
          "bugs.");
      this->m_patch_tolerance = Eigen::NumTraits<Scalar>::dummy_precision();
    } else {
      this->m_patch_tolerance = patch_tolerance;
    }
  }

  /// @copydoc m_patch_tolerance
  Scalar getPatchTolerance() const { return this->m_patch_tolerance; }

  /// @brief Whether two ContactPatchRequest are identical or not.
  bool operator==(const ContactPatchRequest& other) const {
    return this->max_num_patch == other.max_num_patch &&
           this->getNumSamplesCurvedShapes() ==
               other.getNumSamplesCurvedShapes() &&
           this->getPatchTolerance() == other.getPatchTolerance();
  }
};

/// @brief Result for a contact patch computation.
struct COAL_DLLAPI ContactPatchResult {
  using ContactPatchVector = std::vector<ContactPatch>;
  using ContactPatchRef = std::reference_wrapper<ContactPatch>;
  using ContactPatchRefVector = std::vector<ContactPatchRef>;

 protected:
  /// @brief Data container for the vector of contact patches.
  /// @note Contrary to `CollisionResult` or `DistanceResult`, which have a
  /// very small memory footprint, contact patches can contain relatively
  /// large polytopes. In order to reuse a `ContactPatchResult` while avoiding
  /// successive mallocs, we have a data container and a vector which points
  /// to the currently active patches in this data container.
  ContactPatchVector m_contact_patches_data;

  /// @brief Contact patches in `m_contact_patches_data` can have two
  /// statuses: used or unused. This index tracks the first unused patch in
  /// the `m_contact_patches_data` vector.
  size_t m_id_available_patch;

  /// @brief Vector of contact patches of the result.
  ContactPatchRefVector m_contact_patches;

 public:
  /// @brief Default constructor.
  ContactPatchResult() : m_id_available_patch(0) {
    const size_t max_num_patch = 1;
    const ContactPatchRequest request(max_num_patch);
    this->set(request);
  }

  /// @brief Constructor using a `ContactPatchRequest`.
  explicit ContactPatchResult(const ContactPatchRequest& request)
      : m_id_available_patch(0) {
    this->set(request);
  };

  /// @brief Number of contact patches in the result.
  size_t numContactPatches() const { return this->m_contact_patches.size(); }

  /// @brief Returns a new unused contact patch from the internal data vector.
  ContactPatchRef getUnusedContactPatch() {
    if (this->m_id_available_patch >= this->m_contact_patches_data.size()) {
      COAL_LOG_WARNING(
          "Trying to get an unused contact patch but all contact patches are "
          "used. Increasing size of contact patches vector, at the cost of a "
          "copy. You should increase `max_num_patch` in the "
          "`ContactPatchRequest`.");
      this->m_contact_patches_data.emplace_back(
          this->m_contact_patches_data.back());
      this->m_contact_patches_data.back().clear();
    }
    ContactPatch& contact_patch =
        this->m_contact_patches_data[this->m_id_available_patch];
    contact_patch.clear();
    this->m_contact_patches.emplace_back(contact_patch);
    ++(this->m_id_available_patch);
    return this->m_contact_patches.back();
  }

  /// @brief Const getter for the i-th contact patch of the result.
  const ContactPatch& getContactPatch(const size_t i) const {
    if (this->m_contact_patches.empty()) {
      COAL_THROW_PRETTY(
          "The number of contact patches is zero. No ContactPatch can be "
          "returned.",
          std::invalid_argument);
    }
    if (i < this->m_contact_patches.size()) {
      return this->m_contact_patches[i];
    }
    return this->m_contact_patches.back();
  }

  /// @brief Getter for the i-th contact patch of the result.
  ContactPatch& contactPatch(const size_t i) {
    if (this->m_contact_patches.empty()) {
      COAL_THROW_PRETTY(
          "The number of contact patches is zero. No ContactPatch can be "
          "returned.",
          std::invalid_argument);
    }
    if (i < this->m_contact_patches.size()) {
      return this->m_contact_patches[i];
    }
    return this->m_contact_patches.back();
  }

  /// @brief Clears the contact patch result.
  void clear() {
    this->m_contact_patches.clear();
    this->m_id_available_patch = 0;
    for (ContactPatch& patch : this->m_contact_patches_data) {
      patch.clear();
    }
  }

  /// @brief Set up a `ContactPatchResult` from a `ContactPatchRequest`
  void set(const ContactPatchRequest& request) {
    if (this->m_contact_patches_data.size() < request.max_num_patch) {
      this->m_contact_patches_data.resize(request.max_num_patch);
    }
    for (ContactPatch& patch : this->m_contact_patches_data) {
      patch.points().reserve(request.getNumSamplesCurvedShapes());
    }
    this->clear();
  }

  /// @brief Return true if this `ContactPatchResult` is aligned with the
  /// `ContactPatchRequest` given as input.
  bool check(const ContactPatchRequest& request) const {
    assert(this->m_contact_patches_data.size() >= request.max_num_patch);
    if (this->m_contact_patches_data.size() < request.max_num_patch) {
      return false;
    }

    for (const ContactPatch& patch : this->m_contact_patches_data) {
      if (patch.points().capacity() < request.getNumSamplesCurvedShapes()) {
        assert(patch.points().capacity() >=
               request.getNumSamplesCurvedShapes());
        return false;
      }
    }
    return true;
  }

  /// @brief Whether two ContactPatchResult are identical or not.
  bool operator==(const ContactPatchResult& other) const {
    if (this->numContactPatches() != other.numContactPatches()) {
      return false;
    }

    for (size_t i = 0; i < this->numContactPatches(); ++i) {
      const ContactPatch& patch = this->getContactPatch(i);
      const ContactPatch& other_patch = other.getContactPatch(i);
      if (!(patch == other_patch)) {
        return false;
      }
    }

    return true;
  }

  /// @brief Repositions the ContactPatch when they get inverted during their
  /// construction.
  void swapObjects() {
    // Create new transform: it's the reflection of `tf` along the z-axis.
    // This corresponds to doing a PI rotation along the y-axis, i.e. the x-axis
    // becomes -x-axis, y-axis stays the same, z-axis becomes -z-axis.
    for (size_t i = 0; i < this->numContactPatches(); ++i) {
      ContactPatch& patch = this->contactPatch(i);
      patch.tf.rotation().col(0) *= -1.0;
      patch.tf.rotation().col(2) *= -1.0;

      for (size_t j = 0; j < patch.size(); ++j) {
        patch.point(i)(0) *= Scalar(-1);  // only invert the x-axis
      }
    }
  }
};

struct DistanceResult;

/// @brief request to the distance computation
struct COAL_DLLAPI DistanceRequest : QueryRequest {
  /// @brief whether to return the nearest points.
  /// Nearest points are always computed and are the points of the shapes that
  /// achieve a distance of `DistanceResult::min_distance`.
  COAL_DEPRECATED_MESSAGE(
      Nearest points are always computed : they are the points of the shapes
          that achieve a distance of
      `DistanceResult::min_distance`
              .\n Use `enable_signed_distance` if you want to compute a signed
                  minimum distance(and thus its corresponding nearest points)
              .)
  bool enable_nearest_points;

  /// @brief whether to compute the penetration depth when objects are in
  /// collision.
  /// Turning this off can save computation time if only the distance
  /// when objects are disjoint is needed.
  /// @note The minimum distance between the shapes is stored in
  /// `DistanceResult::min_distance`.
  /// If `enable_signed_distance` is off, `DistanceResult::min_distance`
  /// is always positive.
  /// If `enable_signed_distance` is on, `DistanceResult::min_distance`
  /// can be positive or negative.
  /// The nearest points are the points of the shapes that achieve
  /// a distance of `DistanceResult::min_distance`.
  bool enable_signed_distance;

  /// @brief error threshold for approximate distance
  Scalar rel_err;  // relative error, between 0 and 1
  Scalar abs_err;  // absolute error

  /// \param enable_nearest_points_ enables the nearest points computation.
  /// \param enable_signed_distance_ allows to compute the penetration depth
  /// \param rel_err_
  /// \param abs_err_
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  DistanceRequest(bool enable_nearest_points_ = true,
                  bool enable_signed_distance_ = true, Scalar rel_err_ = 0.0,
                  Scalar abs_err_ = 0.0)
      : enable_nearest_points(enable_nearest_points_),
        enable_signed_distance(enable_signed_distance_),
        rel_err(rel_err_),
        abs_err(abs_err_) {}
  COAL_COMPILER_DIAGNOSTIC_POP

  bool isSatisfied(const DistanceResult& result) const;

  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  DistanceRequest& operator=(const DistanceRequest& other) = default;
  COAL_COMPILER_DIAGNOSTIC_POP

  /// @brief whether two DistanceRequest are the same or not
  inline bool operator==(const DistanceRequest& other) const {
    COAL_COMPILER_DIAGNOSTIC_PUSH
    COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    return QueryRequest::operator==(other) &&
           enable_nearest_points == other.enable_nearest_points &&
           enable_signed_distance == other.enable_signed_distance &&
           rel_err == other.rel_err && abs_err == other.abs_err;
    COAL_COMPILER_DIAGNOSTIC_POP
  }
};

/// @brief distance result
struct COAL_DLLAPI DistanceResult : QueryResult {
 public:
  /// @brief minimum distance between two objects.
  /// If two objects are in collision and
  /// DistanceRequest::enable_signed_distance is activated, min_distance <= 0.
  /// @note The nearest points are the points of the shapes that achieve a
  /// distance of `DistanceResult::min_distance`.
  Scalar min_distance;

  /// @brief normal.
  Vec3s normal;

  /// @brief nearest points.
  /// See CollisionResult::nearest_points.
  std::array<Vec3s, 2> nearest_points;

  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief information about the nearest point in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief information about the nearest point in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;

  /// @brief invalid contact primitive information
  static const int NONE = -1;

  DistanceResult(Scalar min_distance_ = (std::numeric_limits<Scalar>::max)())
      : min_distance(min_distance_), o1(NULL), o2(NULL), b1(NONE), b2(NONE) {
    const Vec3s nan(Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN()));
    nearest_points[0] = nearest_points[1] = normal = nan;
  }

  /// @brief add distance information into the result
  void update(Scalar distance, const CollisionGeometry* o1_,
              const CollisionGeometry* o2_, int b1_, int b2_) {
    if (min_distance > distance) {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
    }
  }

  /// @brief add distance information into the result
  void update(Scalar distance, const CollisionGeometry* o1_,
              const CollisionGeometry* o2_, int b1_, int b2_, const Vec3s& p1,
              const Vec3s& p2, const Vec3s& normal_) {
    if (min_distance > distance) {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
      nearest_points[0] = p1;
      nearest_points[1] = p2;
      normal = normal_;
    }
  }

  /// @brief add distance information into the result
  void update(const DistanceResult& other_result) {
    if (min_distance > other_result.min_distance) {
      min_distance = other_result.min_distance;
      o1 = other_result.o1;
      o2 = other_result.o2;
      b1 = other_result.b1;
      b2 = other_result.b2;
      nearest_points[0] = other_result.nearest_points[0];
      nearest_points[1] = other_result.nearest_points[1];
      normal = other_result.normal;
    }
  }

  /// @brief clear the result
  void clear() {
    const Vec3s nan(Vec3s::Constant(std::numeric_limits<Scalar>::quiet_NaN()));
    min_distance = (std::numeric_limits<Scalar>::max)();
    o1 = NULL;
    o2 = NULL;
    b1 = NONE;
    b2 = NONE;
    nearest_points[0] = nearest_points[1] = normal = nan;
    timings.clear();
  }

  /// @brief whether two DistanceResult are the same or not
  inline bool operator==(const DistanceResult& other) const {
    bool is_same = min_distance == other.min_distance &&
                   nearest_points[0] == other.nearest_points[0] &&
                   nearest_points[1] == other.nearest_points[1] &&
                   normal == other.normal && o1 == other.o1 && o2 == other.o2 &&
                   b1 == other.b1 && b2 == other.b2;

    // TODO: check also that two GeometryObject are indeed equal.
    if ((o1 != NULL) ^ (other.o1 != NULL)) return false;
    is_same &= (o1 == other.o1);
    //    else if (o1 != NULL and other.o1 != NULL) is_same &= *o1 ==
    //    *other.o1;

    if ((o2 != NULL) ^ (other.o2 != NULL)) return false;
    is_same &= (o2 == other.o2);
    //    else if (o2 != NULL and other.o2 != NULL) is_same &= *o2 ==
    //    *other.o2;

    return is_same;
  }
};

namespace internal {
inline void updateDistanceLowerBoundFromBV(const CollisionRequest& /*req*/,
                                           CollisionResult& res,
                                           const Scalar sqrDistLowerBound) {
  // BV cannot find negative distance.
  if (res.distance_lower_bound <= 0) return;
  Scalar new_dlb = std::sqrt(sqrDistLowerBound);  // - req.security_margin;
  if (new_dlb < res.distance_lower_bound) res.distance_lower_bound = new_dlb;
}

inline void updateDistanceLowerBoundFromLeaf(const CollisionRequest&,
                                             CollisionResult& res,
                                             const Scalar& distance,
                                             const Vec3s& p0, const Vec3s& p1,
                                             const Vec3s& normal) {
  if (distance < res.distance_lower_bound) {
    res.distance_lower_bound = distance;
    res.nearest_points[0] = p0;
    res.nearest_points[1] = p1;
    res.normal = normal;
  }
}
}  // namespace internal

inline CollisionRequestFlag operator~(CollisionRequestFlag a) {
  return static_cast<CollisionRequestFlag>(~static_cast<int>(a));
}

inline CollisionRequestFlag operator|(CollisionRequestFlag a,
                                      CollisionRequestFlag b) {
  return static_cast<CollisionRequestFlag>(static_cast<int>(a) |
                                           static_cast<int>(b));
}

inline CollisionRequestFlag operator&(CollisionRequestFlag a,
                                      CollisionRequestFlag b) {
  return static_cast<CollisionRequestFlag>(static_cast<int>(a) &
                                           static_cast<int>(b));
}

inline CollisionRequestFlag operator^(CollisionRequestFlag a,
                                      CollisionRequestFlag b) {
  return static_cast<CollisionRequestFlag>(static_cast<int>(a) ^
                                           static_cast<int>(b));
}

inline CollisionRequestFlag& operator|=(CollisionRequestFlag& a,
                                        CollisionRequestFlag b) {
  return (CollisionRequestFlag&)((int&)(a) |= static_cast<int>(b));
}

inline CollisionRequestFlag& operator&=(CollisionRequestFlag& a,
                                        CollisionRequestFlag b) {
  return (CollisionRequestFlag&)((int&)(a) &= static_cast<int>(b));
}

inline CollisionRequestFlag& operator^=(CollisionRequestFlag& a,
                                        CollisionRequestFlag b) {
  return (CollisionRequestFlag&)((int&)(a) ^= static_cast<int>(b));
}

}  // namespace coal

#endif
