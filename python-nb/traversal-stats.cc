/// Copyright 2026 Jelle Feringa

#include "coal/traversal_stats.h"

#include "copyable.hh"

#include "fwd.h"

using namespace coal;

void exposeTraversalStats(nb::module_& m) {
  nb::class_<TraversalStats>(m, "TraversalStats")
      .def_ro("leaf_tests", &TraversalStats::leaf_tests,
              "Narrow-phase calls on one triangle against one shape.")
      .def_ro("bv_tests", &TraversalStats::bv_tests,
              "Bounding-volume tests of a mesh BV against the shape's BV.")
      .def(python::v2::CopyableVisitor<TraversalStats>());

  // By value: the C++ function returns a reference to the calling thread's
  // storage, which must not outlive that thread on the Python side.
  m.def(
      "traversalStats", []() -> TraversalStats { return traversalStats(); },
      "A copy of the calling thread's totals.");

  m.def("resetTraversalStats", &resetTraversalStats,
        "Set the calling thread's totals to zero.");

  m.def("traversalStatsEnabled", &traversalStatsEnabled,
        "Whether the library was built with COAL_ENABLE_TRAVERSAL_STATS. "
        "When False the totals stay zero.");
}
