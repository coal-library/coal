#include "coal/math/transform.h"
#include "coal/shape/geometric_shapes.h"

#include "coal/internal/shape_shape_func.h"
#include "../narrowphase/details.h"

#include "coal/tracy.hh"

namespace coal {
struct GJKSolver;

namespace internal {
template <>
Scalar ShapeShapeDistance<Box, Capsule>(const CollisionGeometry* o1,
                                        const Transform3s& tf1,
                                        const CollisionGeometry* o2,
                                        const Transform3s& tf2,
                                        const GJKSolver*, const bool, Vec3s& p1,
                                        Vec3s& p2, Vec3s& normal) {
  COAL_TRACY_ZONE_SCOPED_N("coal::internal::ShapeShapeDistance<Box, Capsule>");
  const Box& s1 = static_cast<const Box&>(*o1);
  const Capsule& s2 = static_cast<const Capsule&>(*o2);
  return details::boxCapsuleDistance(s1, tf1, s2, tf2, p1, p2, normal);
}

template <>
Scalar ShapeShapeDistance<Capsule, Box>(const CollisionGeometry* o1,
                                        const Transform3s& tf1,
                                        const CollisionGeometry* o2,
                                        const Transform3s& tf2,
                                        const GJKSolver*, const bool, Vec3s& p1,
                                        Vec3s& p2, Vec3s& normal) {
  COAL_TRACY_ZONE_SCOPED_N("coal::internal::ShapeShapeDistance<Capsule, Box>");
  const Capsule& s1 = static_cast<const Capsule&>(*o1);
  const Box& s2 = static_cast<const Box&>(*o2);
  const Scalar distance =
      details::boxCapsuleDistance(s2, tf2, s1, tf1, p2, p1, normal);
  normal = -normal;
  return distance;
}
}  // namespace internal

}  // namespace coal
