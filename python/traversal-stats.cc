//
// Copyright (c) 2026 Jelle Feringa
//

#include <eigenpy/eigenpy.hpp>

#include "coal.hh"

#include "coal/traversal_stats.h"

#ifdef COAL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/coal/traversal_stats.h"
#endif

using namespace boost::python;
using namespace coal;

namespace {
// By value: the C++ function returns a reference to the calling thread's
// storage, which must not outlive that thread on the Python side.
TraversalStats traversalStatsCopy() { return traversalStats(); }
}  // namespace

void exposeTraversalStats() {
  if (!eigenpy::register_symbolic_link_to_registered_type<TraversalStats>()) {
    class_<TraversalStats>("TraversalStats",
                           doxygen::class_doc<TraversalStats>(), no_init)
        .DEF_RO_CLASS_ATTRIB(TraversalStats, leaf_tests)
        .DEF_RO_CLASS_ATTRIB(TraversalStats, bv_tests);
  }

  def("traversalStats", &traversalStatsCopy,
      doxygen::member_func_doc(&traversalStats));
  def("resetTraversalStats", &resetTraversalStats,
      doxygen::member_func_doc(&resetTraversalStats));
  def("traversalStatsEnabled", &traversalStatsEnabled,
      doxygen::member_func_doc(&traversalStatsEnabled));
}
