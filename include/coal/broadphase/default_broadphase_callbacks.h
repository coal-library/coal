/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Toyota Research Institute
 *  Copyright (c) 2022-2023, INRIA
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
 *   * Neither the name of the copyright holder nor the names of its
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

/** @author Sean Curtis (sean@tri.global) */
/** @author Justin Carpentier (justin.carpentier@inria.fr) */

#ifndef COAL_BROADPHASE_DEFAULT_BROADPHASE_CALLBACKS_H
#define COAL_BROADPHASE_DEFAULT_BROADPHASE_CALLBACKS_H

#include "coal/broadphase/broadphase_callbacks.h"
#include "coal/collision.h"
#include "coal/distance.h"
// #include "coal/narrowphase/continuous_collision.h"
// #include "coal/narrowphase/continuous_collision_request.h"
// #include "coal/narrowphase/continuous_collision_result.h"
// #include "coal/narrowphase/distance_request.h"
// #include "coal/narrowphase/distance_result.h"

namespace coal {

/// @brief Collision data stores the collision request and the result given by
/// collision algorithm.
struct CollisionData {
  CollisionData() { done = false; }

  /// @brief Collision request
  CollisionRequest request;

  /// @brief Collision result
  CollisionResult result;

  /// @brief Whether the collision iteration can stop
  bool done;

  /// @brief Clears the CollisionData
  void clear() {
    result.clear();
    done = false;
  }
};

/// @brief Distance data stores the distance request and the result given by
/// distance algorithm.
struct DistanceData {
  DistanceData() { done = false; }

  /// @brief Distance request
  DistanceRequest request;

  /// @brief Distance result
  DistanceResult result;

  /// @brief Whether the distance iteration can stop
  bool done;

  /// @brief Clears the DistanceData
  void clear() {
    result.clear();
    done = false;
  }
};

/// @brief Provides a simple callback for the collision query in the
/// BroadPhaseCollisionManager. It assumes the `data` parameter is non-null and
/// points to an instance of CollisionData. It simply invokes the
/// `collide()` method on the culled pair of geometries and stores the results
/// in the data's CollisionResult instance.
///
/// This callback will cause the broadphase evaluation to stop if:
///   - the collision requests _disables_ cost _and_
///   - the collide() reports a collision for the culled pair, _and_
///   - we've reported the number of contacts requested in the CollisionRequest.
///
/// For a given instance of CollisionData, if broadphase evaluation has
/// already terminated (i.e., defaultCollisionFunction() returned `true`),
/// subsequent invocations with the same instance of CollisionData will
/// return immediately, requesting termination of broadphase evaluation (i.e.,
/// return `true`).
///
/// @param o1   The first object in the culled pair.
/// @param o2   The second object in the culled pair.
/// @param data A non-null pointer to a CollisionData instance.
/// @return `true` if the broadphase evaluation should stop.
/// @tparam S   The scalar type with which the computation will be performed.
bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2,
                              void* data);

/// @brief Collision data for use with the DefaultContinuousCollisionFunction.
/// It stores the collision request and the result given by the collision
/// algorithm (and stores the conclusion of whether further evaluation of the
/// broadphase collision manager has been deemed unnecessary).
// struct DefaultContinuousCollisionData {
//   ContinuousCollisionRequest request;
//   ContinuousCollisionResult result;
//
//   /// If `true`, requests that the broadphase evaluation stop.
//   bool done{false};
// };

/// @brief Provides a simple callback for the continuous collision query in the
/// BroadPhaseCollisionManager. It assumes the `data` parameter is non-null and
/// points to an instance of DefaultContinuousCollisionData. It simply invokes
/// the `collide()` method on the culled pair of geometries and stores the
/// results in the data's ContinuousCollisionResult instance.
///
/// This callback will never cause the broadphase evaluation to terminate early.
/// However, if the `done` member of the DefaultContinuousCollisionData is set
/// to true, this method will simply return without doing any computation.
///
/// For a given instance of DefaultContinuousCollisionData, if broadphase
/// evaluation has already terminated (i.e.,
/// DefaultContinuousCollisionFunction() returned `true`), subsequent
/// invocations with the same instance of CollisionData will return
/// immediately, requesting termination of broadphase evaluation (i.e., return
/// `true`).
///
/// @param o1   The first object in the culled pair.
/// @param o2   The second object in the culled pair.
/// @param data A non-null pointer to a CollisionData instance.
/// @return True if the broadphase evaluation should stop.
/// @tparam S   The scalar type with which the computation will be performed.
// bool DefaultContinuousCollisionFunction(ContinuousCollisionObject* o1,
//                                         ContinuousCollisionObject* o2,
//                                         void* data) {
//   assert(data != nullptr);
//   auto* cdata = static_cast<DefaultContinuousCollisionData*>(data);
//
//   if (cdata->done) return true;
//
//   const ContinuousCollisionRequest& request = cdata->request;
//   ContinuousCollisionResult& result = cdata->result;
//   collide(o1, o2, request, result);
//
//   return cdata->done;
// }

/// @brief Provides a simple callback for the distance query in the
/// BroadPhaseCollisionManager. It assumes the `data` parameter is non-null and
/// points to an instance of DistanceData. It simply invokes the
/// `distance()` method on the culled pair of geometries and stores the results
/// in the data's DistanceResult instance.
///
/// This callback will cause the broadphase evaluation to stop if:
///   - The distance is less than or equal to zero (i.e., the pair is in
///     contact).
///
/// For a given instance of DistanceData, if broadphase evaluation has
/// already terminated (i.e., defaultDistanceFunction() returned `true`),
/// subsequent invocations with the same instance of DistanceData will
/// simply report the previously reported minimum distance and request
/// termination of broadphase evaluation (i.e., return `true`).
///
/// @param o1     The first object in the culled pair.
/// @param o2     The second object in the culled pair.
/// @param data   A non-null pointer to a DistanceData instance.
/// @param dist   The distance computed by distance().
/// @return `true` if the broadphase evaluation should stop.
/// @tparam S   The scalar type with which the computation will be performed.
bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2,
                             void* data, Scalar& dist);

/// @brief Default collision callback to check collision between collision
/// objects.
struct COAL_DLLAPI CollisionCallBackDefault : CollisionCallBackBase {
  /// @brief Initialize the callback.
  /// Clears the collision result and sets the done boolean to false.
  void init() { data.clear(); }

  bool collide(CollisionObject* o1, CollisionObject* o2);

  CollisionData data;

  virtual ~CollisionCallBackDefault() {};
};

/// @brief Default distance callback to check collision between collision
/// objects.
struct COAL_DLLAPI DistanceCallBackDefault : DistanceCallBackBase {
  /// @brief Initialize the callback.
  /// Clears the distance result and sets the done boolean to false.
  void init() { data.clear(); }

  bool distance(CollisionObject* o1, CollisionObject* o2, Scalar& dist);

  DistanceData data;

  virtual ~DistanceCallBackDefault() {};
};

/// @brief Collision callback to collect collision pairs potentially in contacts
struct COAL_DLLAPI CollisionCallBackCollect : CollisionCallBackBase {
  typedef std::pair<CollisionObject*, CollisionObject*> CollisionPair;

  /// @brief Default constructor.
  CollisionCallBackCollect(const size_t max_size);

  bool collide(CollisionObject* o1, CollisionObject* o2);

  /// @brief Returns the number of registered collision pairs
  size_t numCollisionPairs() const;

  /// @brief Returns a const reference to the active collision_pairs to check
  const std::vector<CollisionPair>& getCollisionPairs() const;

  /// @brief Reset the callback
  void init();

  /// @brief Check whether a collision pair exists
  bool exist(const CollisionPair& pair) const;

  /// @brief Check whether a collision pair exists
  bool exist(CollisionObject* o1, CollisionObject* o2) const;

  virtual ~CollisionCallBackCollect() {};

 protected:
  std::vector<CollisionPair> collision_pairs;
  size_t max_size;
};

}  // namespace coal

#endif  // COAL_BROADPHASE_DEFAULT_BROADPHASE_CALLBACKS_H
