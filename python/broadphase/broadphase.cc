//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2022 INRIA
//  Author: Justin Carpentier
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of CNRS-LAAS. nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#include "coal/fwd.hh"
#include "../coal.hh"
#include "../utils/std-pair.hh"

#include "coal/broadphase/broadphase_dynamic_AABB_tree.h"
#include "coal/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "coal/broadphase/broadphase_bruteforce.h"
#include "coal/broadphase/broadphase_SaP.h"
#include "coal/broadphase/broadphase_SSaP.h"
#include "coal/broadphase/broadphase_interval_tree.h"
#include "coal/broadphase/broadphase_spatialhash.h"

COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
COAL_COMPILER_DIAGNOSTIC_POP
#include "doxygen_autodoc/coal/broadphase/default_broadphase_callbacks.h"
// #include "doxygen_autodoc/coal/broadphase/broadphase_dynamic_AABB_tree.h"
// #include
//"doxygen_autodoc/coal/broadphase/broadphase_dynamic_AABB_tree_array.h"
// #include "doxygen_autodoc/coal/broadphase/broadphase_bruteforce.h"
// #include "doxygen_autodoc/coal/broadphase/broadphase_SaP.h"
// #include "doxygen_autodoc/coal/broadphase/broadphase_SSaP.h"
// #include "doxygen_autodoc/coal/broadphase/broadphase_interval_tree.h"
// #include "doxygen_autodoc/coal/broadphase/broadphase_spatialhash.h"
#endif

#include "broadphase_callbacks.hh"
#include "broadphase_collision_manager.hh"

using namespace coal;

COAL_COMPILER_DIAGNOSTIC_PUSH
COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
void exposeBroadPhase() {
  CollisionCallBackBaseWrapper::expose();
  DistanceCallBackBaseWrapper::expose();

  // CollisionCallBackDefault
  bp::class_<CollisionCallBackDefault, bp::bases<CollisionCallBackBase>>(
      "CollisionCallBackDefault", bp::no_init)
      .def(dv::init<CollisionCallBackDefault>())
      .DEF_RW_CLASS_ATTRIB(CollisionCallBackDefault, data);

  // DistanceCallBackDefault
  bp::class_<DistanceCallBackDefault, bp::bases<DistanceCallBackBase>>(
      "DistanceCallBackDefault", bp::no_init)
      .def(dv::init<DistanceCallBackDefault>())
      .DEF_RW_CLASS_ATTRIB(DistanceCallBackDefault, data);

  // CollisionCallBackCollect
  bp::class_<CollisionCallBackCollect, bp::bases<CollisionCallBackBase>>(
      "CollisionCallBackCollect", bp::no_init)
      .def(dv::init<CollisionCallBackCollect, const size_t>())
      .DEF_CLASS_FUNC(CollisionCallBackCollect, numCollisionPairs)
      .DEF_CLASS_FUNC2(CollisionCallBackCollect, getCollisionPairs,
                       bp::return_value_policy<bp::copy_const_reference>())
      .def(dv::member_func(
          "exist", (bool (CollisionCallBackCollect::*)(
                       const CollisionCallBackCollect::CollisionPair &) const) &
                       CollisionCallBackCollect::exist))
      .def(dv::member_func("exist",
                           (bool (CollisionCallBackCollect::*)(
                               CollisionObject *, CollisionObject *) const) &
                               CollisionCallBackCollect::exist));

  StdPairConverter<CollisionCallBackCollect::CollisionPair>::registration();

  bp::class_<CollisionData>("CollisionData", bp::no_init)
      .def(dv::init<CollisionData>())
      .DEF_RW_CLASS_ATTRIB(CollisionData, request)
      .DEF_RW_CLASS_ATTRIB(CollisionData, result)
      .DEF_RW_CLASS_ATTRIB(CollisionData, done)
      .DEF_CLASS_FUNC(CollisionData, clear);

  bp::class_<DistanceData>("DistanceData", bp::no_init)
      .def(dv::init<DistanceData>())
      .DEF_RW_CLASS_ATTRIB(DistanceData, request)
      .DEF_RW_CLASS_ATTRIB(DistanceData, result)
      .DEF_RW_CLASS_ATTRIB(DistanceData, done)
      .DEF_CLASS_FUNC(DistanceData, clear);

  BroadPhaseCollisionManagerWrapper::expose();

  BroadPhaseCollisionManagerWrapper::exposeDerived<
      DynamicAABBTreeCollisionManager>();
  BroadPhaseCollisionManagerWrapper::exposeDerived<
      DynamicAABBTreeArrayCollisionManager>();
  BroadPhaseCollisionManagerWrapper::exposeDerived<
      IntervalTreeCollisionManager>();
  BroadPhaseCollisionManagerWrapper::exposeDerived<SSaPCollisionManager>();
  BroadPhaseCollisionManagerWrapper::exposeDerived<SaPCollisionManager>();
  BroadPhaseCollisionManagerWrapper::exposeDerived<NaiveCollisionManager>();

  // Specific case of SpatialHashingCollisionManager
  {
    typedef detail::SimpleHashTable<AABB, CollisionObject *,
                                    detail::SpatialHash>
        HashTable;
    typedef SpatialHashingCollisionManager<HashTable> Derived;
    bp::class_<Derived, bp::bases<BroadPhaseCollisionManager>>(
        "SpatialHashingCollisionManager", bp::no_init)
        .def(dv::init<Derived, Scalar, const Vec3s &, const Vec3s &,
                      bp::optional<unsigned int>>());
  }
}
COAL_COMPILER_DIAGNOSTIC_POP
