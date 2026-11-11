#include "coal/internal/traversal_stats.h"

namespace coal {

namespace {
// Defined here, in the library, and nowhere else: a thread_local in an inline
// header function would give each shared library its own copy on Mach-O.
thread_local TraversalStats traversal_stats;
}  // namespace

TraversalStats& traversalStats() { return traversal_stats; }

void resetTraversalStats() { traversal_stats = TraversalStats(); }

}  // namespace coal
