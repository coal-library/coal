/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
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

/** \authors Jia Pan, Florent Lamiraux, Josef Mirabel, Louis Montaut */

#ifndef COAL_SUPPORT_DATA_H
#define COAL_SUPPORT_DATA_H

#include "coal/data_types.h"
#include "coal/collision_data.h"

namespace coal {
namespace details {

/// @brief Options for the computation of support points.
/// `NoSweptSphere` option is used when the support function is called
/// by GJK or EPA. In this case, the swept sphere radius is not taken into
/// account in the support function. It is used by GJK and EPA after they have
/// converged to correct the solution.
/// `WithSweptSphere` option is used when the support function is called
/// directly by the user. In this case, the swept sphere radius is taken into
/// account in the support function.
enum SupportOptions {
  NoSweptSphere = 0,
  WithSweptSphere = 0x1,
};

/// @brief Stores temporary data for the computation of support points.
struct ShapeSupportData {
  // @brief Tracks which points have been visited in a ConvexBase.
  std::vector<int8_t> visited;

  // @brief Tracks the last support direction used on this shape; used to
  // warm-start the ConvexBase support function.
  Vec3s last_dir = Vec3s::Zero();

  // @brief Temporary set used to compute the convex-hull of a support set.
  // Only used for ConvexBase and Box.
  SupportSet::Polygon polygon;
};

}  // namespace details
}  // namespace coal

#endif  // COAL_SUPPORT_DATA_H
