/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Jelle Feringa
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

#ifndef COAL_TRAVERSAL_STATS_H
#define COAL_TRAVERSAL_STATS_H

#include "coal/fwd.hh"

#include <cstdint>

namespace coal {

/// @brief Per-thread totals of the mesh-against-shape traversal work.
///
/// Counted only when Coal is built with the CMake option
/// `COAL_ENABLE_TRAVERSAL_STATS` (which defines the macro of the same name for
/// Coal and its dependents); otherwise the totals stay zero and the traversal
/// carries no counting code. Counted independently of the traversal nodes'
/// `enable_statistics`, in the calling thread's storage.
struct TraversalStats {
  /// One per narrowphase call on one triangle against one shape: the leaf
  /// tests of the collision and distance traversals, and the distance
  /// traversal's preprocess call on the first triangle.
  std::uint64_t leaf_tests{0};
  /// One per bounding-volume test of a mesh BV against the shape's BV:
  /// `BVDisjoints` (collision) and `BVDistanceLowerBound` (distance).
  std::uint64_t box_tests{0};
};

/// @brief The calling thread's totals.
///
/// The storage is one thread_local object defined in the library, so every
/// module linking it shares one copy per thread.
COAL_DLLAPI TraversalStats& traversalStats();

/// @brief Set the calling thread's totals to zero.
COAL_DLLAPI void resetTraversalStats();

}  // namespace coal

#ifdef COAL_ENABLE_TRAVERSAL_STATS
#define COAL_TRAVERSAL_STATS_INC(field) (++::coal::traversalStats().field)
#else
#define COAL_TRAVERSAL_STATS_INC(field) ((void)0)
#endif

#endif  // COAL_TRAVERSAL_STATS_H
