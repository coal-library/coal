//
// Copyright (c) 2026 INRIA
//

#ifndef COAL_CONTACT_PATCH_POLYGON_CONVEX_HULL_H
#define COAL_CONTACT_PATCH_POLYGON_CONVEX_HULL_H

#include <vector>

#include "coal/config.hh"
#include "coal/data_types.h"

namespace coal {

// -----------------------------------------------------------
// Collection of various utilities useful in the Coal library.
// -----------------------------------------------------------

/// @brief Compute the convex hull of a polygon.
/// @note This function internally uses alloca to avoid memory allocation on the
/// heap. Hence, this function is meant to be used on point clouds with a small
/// amount of points. If you encounter problems with this method, feel free to
/// raise an issue.
COAL_DLLAPI void computePolygonConvexHull(const std::vector<Vec2s>& cloud,
                                          std::vector<Vec2s>& cvx_hull);

}  // namespace coal

#endif  // COAL_CONTACT_PATCH_POLYGON_CONVEX_HULL_H
