//
// Copyright (c) 2026 Jelle Feringa
//

#include "coal.hh"

#include "coal/traversal_stats.h"

using namespace coal;

namespace {
// By value: the C++ function returns a reference to the calling thread's
// storage, which must not outlive that thread on the Python side.
TraversalStats traversalStatsCopy() { return traversalStats(); }
}  // namespace

void exposeTraversalStats() {
  namespace bp = boost::python;

  bp::class_<TraversalStats>(
      "TraversalStats",
      "Per-thread totals of the mesh-against-shape traversal work. Counted "
      "only when Coal is built with COAL_ENABLE_TRAVERSAL_STATS.",
      bp::no_init)
      .def_readonly("leaf_tests", &TraversalStats::leaf_tests,
                    "Narrow-phase calls on one triangle against one shape.")
      .def_readonly(
          "bv_tests", &TraversalStats::bv_tests,
          "Bounding-volume tests of a mesh BV against the shape's BV.");

  bp::def("traversalStats", &traversalStatsCopy,
          "A copy of the calling thread's totals.");

  bp::def("resetTraversalStats", &resetTraversalStats,
          "Set the calling thread's totals to zero.");

  bp::def("traversalStatsEnabled", &traversalStatsEnabled,
          "Whether the library was built with COAL_ENABLE_TRAVERSAL_STATS. "
          "When False the totals stay zero.");
}
