/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, INRIA
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

/** @author Justin Carpentier (justin.carpentier@inria.fr) */

#ifndef COAL_BROADPHASE_BROAD_PHASE_CALLBACKS_H
#define COAL_BROADPHASE_BROAD_PHASE_CALLBACKS_H

#include "coal/fwd.hh"
#include "coal/data_types.h"

namespace coal {

/// @brief Base callback class for collision queries.
/// This class can be supersed by child classes to provide desired behaviors
/// according to the application (e.g, only listing the potential
/// CollisionObjects in collision).
struct COAL_DLLAPI CollisionCallBackBase {
  /// @brief Initialization of the callback before running the collision
  /// broadphase manager.
  virtual void init() {};

  /// @brief Collision evaluation between two objects in collision.
  ///        This callback will cause the broadphase evaluation to stop if it
  ///        returns true.
  ///
  /// @param[in] o1 Collision object #1.
  /// @param[in] o2 Collision object #2.
  virtual bool collide(CollisionObject* o1, CollisionObject* o2) = 0;

  /// @brief Functor call associated to the collide operation.
  virtual bool operator()(CollisionObject* o1, CollisionObject* o2) {
    return collide(o1, o2);
  }
};

/// @brief Base callback class for distance queries.
/// This class can be supersed by child classes to provide desired behaviors
/// according to the application (e.g, only listing the potential
/// CollisionObjects in collision).
struct COAL_DLLAPI DistanceCallBackBase {
  /// @brief Initialization of the callback before running the collision
  /// broadphase manager.
  virtual void init() {};

  /// @brief Distance evaluation between two objects in collision.
  ///        This callback will cause the broadphase evaluation to stop if it
  ///        returns true.
  ///
  /// @param[in] o1 Collision object #1.
  /// @param[in] o2 Collision object #2.
  /// @param[out] dist Distance between the two collision geometries.
  virtual bool distance(CollisionObject* o1, CollisionObject* o2,
                        Scalar& dist) = 0;

  /// @brief Functor call associated to the distance operation.
  virtual bool operator()(CollisionObject* o1, CollisionObject* o2,
                          Scalar& dist) {
    return distance(o1, o2, dist);
  }
};

}  // namespace coal

#endif  // COAL_BROADPHASE_BROAD_PHASE_CALLBACKS_H
