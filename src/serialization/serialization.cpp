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

/** \author Justin Carpentier */

#include "coal/serialization/fwd.h"

using namespace coal;

#include "coal/serialization/transform.h"
#include "coal/serialization/collision_data.h"
#include "coal/serialization/geometric_shapes.h"
#include "coal/serialization/convex.h"
#include "coal/serialization/hfield.h"
#include "coal/serialization/BVH_model.h"
#ifdef COAL_HAS_OCTOMAP
#include "coal/serialization/octree.h"
#endif

COAL_SERIALIZATION_DEFINE_EXPORT(CollisionRequest)
COAL_SERIALIZATION_DEFINE_EXPORT(CollisionResult)
COAL_SERIALIZATION_DEFINE_EXPORT(DistanceRequest)
COAL_SERIALIZATION_DEFINE_EXPORT(DistanceResult)

COAL_SERIALIZATION_DEFINE_EXPORT(ShapeBase)
COAL_SERIALIZATION_DEFINE_EXPORT(CollisionGeometry)
COAL_SERIALIZATION_DEFINE_EXPORT(TriangleP)
COAL_SERIALIZATION_DEFINE_EXPORT(Box)
COAL_SERIALIZATION_DEFINE_EXPORT(Sphere)
COAL_SERIALIZATION_DEFINE_EXPORT(Ellipsoid)
COAL_SERIALIZATION_DEFINE_EXPORT(Capsule)
COAL_SERIALIZATION_DEFINE_EXPORT(Cone)
COAL_SERIALIZATION_DEFINE_EXPORT(Cylinder)
COAL_SERIALIZATION_DEFINE_EXPORT(Halfspace)
COAL_SERIALIZATION_DEFINE_EXPORT(Plane)

#define EXPORT_AND_CAST(Derived, Base)            \
  COAL_SERIALIZATION_CAST_REGISTER(Derived, Base) \
  COAL_SERIALIZATION_DEFINE_EXPORT(Derived)       \
  /**/

COAL_SERIALIZATION_CAST_REGISTER(ConvexBase16, CollisionGeometry)
COAL_SERIALIZATION_CAST_REGISTER(ConvexBase32, CollisionGeometry)
EXPORT_AND_CAST(ConvexTpl<Triangle16>, ConvexBase16)
EXPORT_AND_CAST(ConvexTpl<Triangle32>, ConvexBase32)
EXPORT_AND_CAST(ConvexTpl<Quadrilateral16>, ConvexBase16)
EXPORT_AND_CAST(ConvexTpl<Quadrilateral32>, ConvexBase32)

COAL_SERIALIZATION_DEFINE_EXPORT(HeightField<AABB>)
COAL_SERIALIZATION_DEFINE_EXPORT(HeightField<OBB>)
COAL_SERIALIZATION_DEFINE_EXPORT(HeightField<OBBRSS>)

COAL_SERIALIZATION_CAST_REGISTER(BVHModelBase, CollisionGeometry)

EXPORT_AND_CAST(BVHModel<AABB>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<OBB>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<RSS>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<OBBRSS>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<kIOS>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<KDOP<16>>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<KDOP<18>>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<KDOP<24>>, BVHModelBase)

#ifdef COAL_HAS_OCTOMAP
COAL_SERIALIZATION_DEFINE_EXPORT(OcTree)
#endif
