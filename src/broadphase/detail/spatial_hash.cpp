/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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

/** @author Jia Pan */

#ifndef COAL_BROADPHASE_SPATIALHASH_INL_H
#define COAL_BROADPHASE_SPATIALHASH_INL_H

#include "coal/broadphase/detail/spatial_hash.h"
#include <algorithm>

namespace coal {
namespace detail {

//==============================================================================
SpatialHash::SpatialHash(const AABB& scene_limit_, Scalar cell_size_)
    : cell_size(cell_size_), scene_limit(scene_limit_) {
  width[0] =
      static_cast<unsigned int>(std::ceil(scene_limit.width() / cell_size));
  width[1] =
      static_cast<unsigned int>(std::ceil(scene_limit.height() / cell_size));
  width[2] =
      static_cast<unsigned int>(std::ceil(scene_limit.depth() / cell_size));
}

//==============================================================================
std::vector<unsigned int> SpatialHash::operator()(const AABB& aabb) const {
  unsigned int min_x = static_cast<unsigned int>(
      std::floor((aabb.min_[0] - scene_limit.min_[0]) / cell_size));
  unsigned int max_x = static_cast<unsigned int>(
      std::ceil((aabb.max_[0] - scene_limit.min_[0]) / cell_size));
  unsigned int min_y = static_cast<unsigned int>(
      std::floor((aabb.min_[1] - scene_limit.min_[1]) / cell_size));
  unsigned int max_y = static_cast<unsigned int>(
      std::ceil((aabb.max_[1] - scene_limit.min_[1]) / cell_size));
  unsigned int min_z = static_cast<unsigned int>(
      std::floor((aabb.min_[2] - scene_limit.min_[2]) / cell_size));
  unsigned int max_z = static_cast<unsigned int>(
      std::ceil((aabb.max_[2] - scene_limit.min_[2]) / cell_size));

  std::vector<unsigned int> keys(
      static_cast<size_t>((max_x - min_x) * (max_y - min_y) * (max_z - min_z)));
  size_t id = 0;
  for (unsigned int x = min_x; x < max_x; ++x) {
    for (unsigned int y = min_y; y < max_y; ++y) {
      for (unsigned int z = min_z; z < max_z; ++z) {
        keys[id++] = x + y * width[0] + z * width[0] * width[1];
      }
    }
  }
  return keys;
}

}  // namespace detail
}  // namespace coal

#endif
