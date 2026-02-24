#include "coal/contact_patch/polygon_convex_hull.h"
#include "coal/alloca.h"

#include <algorithm>

namespace coal {
void computePolygonConvexHull(const std::vector<Vec2s>& cloud,
                              std::vector<Vec2s>& cvx_hull) {
  cvx_hull.clear();

  if (cloud.size() <= 2) {
    // Point or segment, nothing to do.
    for (const Vec2s& point : cloud) {
      cvx_hull.emplace_back(point);
    }
    return;
  }

  // temporary copy of cloud to work on
  COAL_MAKE_ALLOCA_BOOST_SPAN(Vec2s, cloud_, cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    cloud_[i] = cloud[i];
  }

  if (cloud_.size() == 3) {
    // We have a triangle, we only need to arrange it in a counter clockwise
    // fashion.
    //
    // Put the vector which has the lowest y coordinate first.
    if (cloud_[0](1) > cloud_[1](1)) {
      std::swap(cloud_[0], cloud_[1]);
    }
    if (cloud_[0](1) > cloud_[2](1)) {
      std::swap(cloud_[0], cloud_[2]);
    }
    const Vec2s& a = cloud_[0];
    const Vec2s& b = cloud_[1];
    const Vec2s& c = cloud_[2];
    const Scalar det =
        (b(0) - a(0)) * (c(1) - a(1)) - (b(1) - a(1)) * (c(0) - a(0));
    if (det < 0) {
      std::swap(cloud_[1], cloud_[2]);
    }

    for (const Vec2s& point : cloud_) {
      cvx_hull.emplace_back(point);
    }
    return;
  }

  // The following is an implementation of the O(nlog(n)) graham scan
  // algorithm, used to compute convex-hulls in 2D.
  // See https://en.wikipedia.org/wiki/Graham_scan
  //
  // Step 1 - Compute first element of the convex-hull by computing the support
  // in the direction (0, -1) (take the element of the set which has the lowest
  // y coordinate).
  size_t support_idx = 0;
  Scalar support_val = cloud_[0](1);
  for (size_t i = 1; i < cloud_.size(); ++i) {
    const Scalar val = cloud_[i](1);
    if (val < support_val) {
      support_val = val;
      support_idx = i;
    }
  }
  Vec2s tmp = cloud_[0];
  cloud_[0] = cloud_[support_idx];
  cloud_[support_idx] = tmp;
  cvx_hull.clear();
  cvx_hull.emplace_back(cloud_[0]);
  const Vec2s v = cvx_hull[0];

  // Step 2 - Sort the rest of the point cloud_ according to the angle made with
  // v. Note: we use stable_sort instead of sort because sort can fail if two
  // values are identical.
  std::sort(
      cloud_.begin() + 1, cloud_.end(), [&v](const Vec2s& p1, const Vec2s& p2) {
        // p1 is "smaller" than p2 if det(p1 - v, p2 - v) >= 0
        const Scalar det =
            (p1(0) - v(0)) * (p2(1) - v(1)) - (p1(1) - v(1)) * (p2(0) - v(0));
        if (std::abs(det) <= Eigen::NumTraits<Scalar>::dummy_precision()) {
          // If two points are identical or (v, p1, p2) are colinear, p1 is
          // "smaller" if it is closer to v.
          return ((p1 - v).squaredNorm() <= (p2 - v).squaredNorm());
        }
        return det > 0;
      });

  // Step 3 - We iterate over the now ordered point of cloud_ and add the points
  // to the cvx-hull if they successively form "left turns" only. A left turn
  // is: considering the last three points of the cvx-hull, if they form a
  // right-hand basis (determinant > 0) then they make a left turn.
  auto isRightSided = [](const Vec2s& p1, const Vec2s& p2, const Vec2s& p3) {
    // Checks if (p2 - p1, p3 - p1) forms a right-sided base based on
    // det(p2 - p1, p3 - p1)
    const Scalar det =
        (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(1)) * (p3(0) - p1(0));
    // Note: we set a dummy precision threshold so that identical points or
    // colinear pionts are not added to the cvx-hull.
    return det > Eigen::NumTraits<Scalar>::dummy_precision();
  };

  // We initialize the cvx-hull algo by adding the first three
  // (distinct) points of the set.
  // These three points are guaranteed, due to the previous sorting,
  // to form a right sided basis, hence to form a left turn.
  size_t cloud_beginning_idx = 1;
  while (cvx_hull.size() < 3) {
    const Vec2s& vec = cloud_[cloud_beginning_idx];
    if ((cvx_hull.back() - vec).squaredNorm() >
        Eigen::NumTraits<Scalar>::epsilon()) {
      cvx_hull.emplace_back(vec);
    }
    ++cloud_beginning_idx;
  }
  // The convex-hull should wrap counter-clockwise, i.e. three successive
  // points should always form a right-sided basis. Every time we do a turn
  // in the wrong direction, we remove the last point of the convex-hull.
  // When we do a turn in the correct direction, we add a point to the
  // convex-hull.
  for (size_t i = cloud_beginning_idx; i < cloud_.size(); ++i) {
    const Vec2s& vec = cloud_[i];
    while (cvx_hull.size() > 1 &&
           !isRightSided(cvx_hull[cvx_hull.size() - 2],
                         cvx_hull[cvx_hull.size() - 1], vec)) {
      cvx_hull.pop_back();
    }
    cvx_hull.emplace_back(vec);
  }
}
}  // namespace coal
