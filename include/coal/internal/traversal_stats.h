#ifndef COAL_INTERNAL_TRAVERSAL_STATS_H
#define COAL_INTERNAL_TRAVERSAL_STATS_H

#include "coal/fwd.hh"

#include <cstdint>

namespace coal {

/// @brief Per-thread totals of the mesh-against-shape traversal work.
///
/// Counted unconditionally (independent of the traversal nodes'
/// `enable_statistics`), in the calling thread's storage.
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

#endif  // COAL_INTERNAL_TRAVERSAL_STATS_H
