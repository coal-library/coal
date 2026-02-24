/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, INRIA
 *  All rights reserved.
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
 *   * Neither the name of INRIA nor the names of its
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

/** \author Louis Montaut */

#include "coal/contact_patch/contact_patch_simplifier.h"
#include "coal/alloca.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace coal {

void ContactPatchSimplifierNaive::compute(const ContactPatch& patch_in,
                                          std::size_t target_vertices,
                                          ContactPatch& patch_out) {
  const auto& pts = patch_in.points();
  const std::size_t n = pts.size();
  simplified_buffer_.clear();
  simplified_buffer_.reserve(std::min(target_vertices, n));

  if (n == 0 || target_vertices == 0 || target_vertices >= n) {
    simplified_buffer_.assign(pts.begin(), pts.end());
  } else if (target_vertices == 1) {
    Vec2s barycenter = Vec2s::Zero();
    for (std::size_t i = 0; i < n; ++i) {
      barycenter += pts[i];
    }
    barycenter /= static_cast<Scalar>(n);
    simplified_buffer_.push_back(barycenter);
  } else if (target_vertices == 2) {
    const Vec2s& first = pts[0];
    std::size_t farthest_idx = 0;
    Scalar max_dist_sq = Scalar(0);
    for (std::size_t i = 1; i < n; ++i) {
      const Scalar dist_sq = (pts[i] - first).squaredNorm();
      if (dist_sq > max_dist_sq) {
        max_dist_sq = dist_sq;
        farthest_idx = i;
      }
    }
    simplified_buffer_.push_back(first);
    simplified_buffer_.push_back(pts[farthest_idx]);
  } else {
    // Brute-force: enumerate all C(n, k) subsets of vertices (preserving CCW
    // order) and keep the one that maximises the polygon area.
    const std::size_t k = target_vertices;
    std::vector<std::size_t> indices(k);
    std::iota(indices.begin(), indices.end(), std::size_t(0));

    // Shoelace formula for the polygon formed by the selected indices.
    const auto compute_area = [&](const std::vector<std::size_t>& idx) {
      Scalar area = Scalar(0);
      for (std::size_t i = 0; i < k; ++i) {
        const std::size_t j = (i + 1) % k;
        area += pts[idx[i]](0) * pts[idx[j]](1);
        area -= pts[idx[j]](0) * pts[idx[i]](1);
      }
      return std::abs(area) * Scalar(0.5);
    };

    Scalar best_area = Scalar(-1);
    std::vector<std::size_t> best_indices(indices);

    while (true) {
      const Scalar area = compute_area(indices);
      if (area > best_area) {
        best_area = area;
        best_indices = indices;
      }

      // Advance to the next combination in lexicographic order.
      // Find the rightmost index that can still be incremented.
      int i = static_cast<int>(k) - 1;
      while (i >= 0 && indices[static_cast<std::size_t>(i)] ==
                           n - k + static_cast<std::size_t>(i)) {
        --i;
      }
      if (i < 0) break;

      ++indices[static_cast<std::size_t>(i)];
      for (std::size_t j = static_cast<std::size_t>(i) + 1; j < k; ++j) {
        indices[j] = indices[j - 1] + 1;
      }
    }

    for (std::size_t i = 0; i < k; ++i) {
      simplified_buffer_.push_back(pts[best_indices[i]]);
    }
  }

  if (&patch_in != &patch_out) {
    patch_out = patch_in;
  }
  auto& polygon = patch_out.points();
  polygon.assign(simplified_buffer_.begin(), simplified_buffer_.end());
}

void ContactPatchSimplifierNaive::simplify(ContactPatch& patch,
                                           std::size_t target_vertices) {
  compute(patch, target_vertices, patch);
}

namespace {
// -----------------------------------------------
// Utils for non-trivial contact patch simplifiers
// -----------------------------------------------
using Index = std::size_t;

template <typename T>
inline T clamp(T val, T min_val, T max_val) {
  if (val < min_val) {
    return min_val;
  }
  if (val > max_val) {
    return max_val;
  }
  return val;
}

inline Scalar double_triangle_area(const Vec2s& a, const Vec2s& b,
                                   const Vec2s& c) {
  const Vec2s ab = b - a;
  const Vec2s ac = c - a;
  return ab.x() * ac.y() - ab.y() * ac.x();
}

Scalar compute_triangle_area(const std::vector<Vec2s>& pts,
                             const std::vector<int>& prev,
                             const std::vector<int>& next, int vertex) {
  const int prev_idx = prev[static_cast<Index>(vertex)];
  const int next_idx = next[static_cast<Index>(vertex)];
  if (prev_idx == vertex || next_idx == vertex) {
    return Scalar(0);
  }
  const Vec2s& a = pts[static_cast<Index>(prev_idx)];
  const Vec2s& b = pts[static_cast<Index>(vertex)];
  const Vec2s& c = pts[static_cast<Index>(next_idx)];
  return std::abs(double_triangle_area(a, b, c)) * Scalar(0.5);
}

Scalar compute_triangle_area_kgon(const std::vector<Vec2s>& pts,  //
                                  int i, int j, int k) {
  int n = static_cast<int>(pts.size());
  i %= n;
  j %= n;
  k %= n;
  if (i == j || j == k || i == k) return Scalar(0);
  const Vec2s& a = pts[static_cast<Index>(i)];
  const Vec2s& b = pts[static_cast<Index>(j)];
  const Vec2s& c = pts[static_cast<Index>(k)];
  return std::abs(double_triangle_area(a, b, c)) * Scalar(0.5);
}

Scalar compute_rooted_kgon(const std::vector<Vec2s>& pts, int root, int k,
                           boost::span<const int> left_c,
                           boost::span<const int> right_c,
                           boost::span<int> best_v, boost::span<Scalar> dp_area,
                           boost::span<int> dp_prev) {
  int n = static_cast<int>(pts.size());

  for (int m = 1; m < k; ++m) {
    int len = right_c[Index(m - 1)] - left_c[Index(m - 1)] + 1;
    for (int i = 0; i < len; ++i) {
      dp_area[Index(m * 2 * n + i)] = Scalar(-1.0);
      dp_prev[Index(m * 2 * n + i)] = -1;
    }
  }

  for (int i = left_c[0]; i <= right_c[0]; ++i) {
    dp_area[Index(1 * 2 * n + (i - left_c[0]))] = Scalar(0.0);
    dp_prev[Index(1 * 2 * n + (i - left_c[0]))] = root;
  }

  for (int m = 2; m < k; ++m) {
    for (int i = left_c[Index(m - 1)]; i <= right_c[Index(m - 1)]; ++i) {
      Scalar best_area = Scalar(-1.0);
      int best_j = -1;

      int j_start = left_c[Index(m - 2)];
      int j_end = std::min(right_c[Index(m - 2)], i - 1);

      for (int j = j_start; j <= j_end; ++j) {
        Scalar prev_val =
            dp_area[Index((m - 1) * 2 * n + (j - left_c[Index(m - 2)]))];
        if (prev_val < Scalar(0.0)) continue;

        Scalar area = prev_val + compute_triangle_area_kgon(pts, root, j, i);
        if (area > best_area) {
          best_area = area;
          best_j = j;
        }
      }
      dp_area[Index(m * 2 * n + (i - left_c[Index(m - 1)]))] = best_area;
      dp_prev[Index(m * 2 * n + (i - left_c[Index(m - 1)]))] = best_j;
    }
  }

  Scalar max_total_area = Scalar(-1.0);
  int best_end = -1;
  for (int i = left_c[Index(k - 2)]; i <= right_c[Index(k - 2)]; ++i) {
    Scalar val = dp_area[Index((k - 1) * 2 * n + (i - left_c[Index(k - 2)]))];
    if (val > max_total_area) {
      max_total_area = val;
      best_end = i;
    }
  }

  std::fill(best_v.begin(), best_v.end(), 0);
  best_v[0] = root;
  if (best_end != -1) {
    int curr = best_end;
    for (int m = k - 1; m >= 1; --m) {
      best_v[Index(m)] = curr;
      curr = dp_prev[Index(m * 2 * n + (curr - left_c[Index(m - 1)]))];
    }
  }
  return max_total_area;
}

void solve_recursive(const std::vector<Vec2s>& pts, int k, int root_start,
                     int root_end, boost::span<const int> left_bound,
                     boost::span<const int> right_bound,
                     Scalar& global_max_area, boost::span<int> global_best_v,
                     boost::span<Scalar> dp_area, boost::span<int> dp_prev) {
  if (root_start > root_end) return;

  int mid_root = root_start + (root_end - root_start) / 2;

  COAL_MAKE_ALLOCA_BOOST_SPAN(int, mid_v, static_cast<Index>(k));
  Scalar mid_area = compute_rooted_kgon(pts, mid_root, k, left_bound,
                                        right_bound, mid_v, dp_area, dp_prev);

  if (mid_area > global_max_area) {
    global_max_area = mid_area;
    for (int i = 0; i < k; ++i) {
      global_best_v[static_cast<Index>(i)] = mid_v[static_cast<Index>(i)];
    }
  }

  if (root_start < mid_root) {
    COAL_MAKE_ALLOCA_BOOST_SPAN(int, new_right, static_cast<Index>(k - 1));
    for (int m = 1; m < k; ++m)
      new_right[static_cast<Index>(m - 1)] = mid_v[static_cast<Index>(m)];
    solve_recursive(pts, k, root_start, mid_root - 1, left_bound, new_right,
                    global_max_area, global_best_v, dp_area, dp_prev);
  }

  if (mid_root < root_end) {
    COAL_MAKE_ALLOCA_BOOST_SPAN(int, new_left, static_cast<Index>(k - 1));
    for (int m = 1; m < k; ++m)
      new_left[static_cast<Index>(m - 1)] = mid_v[static_cast<Index>(m)];
    solve_recursive(pts, k, mid_root + 1, root_end, new_left, right_bound,
                    global_max_area, global_best_v, dp_area, dp_prev);
  }
}

}  // namespace

// Let's start by a remark: if you start at vertex p0 of a 2D polygon,
// (p0 is any vertex in the polygon, then p1, p2... are the counter-clockwise
// sequence of vertices forming the polygon), you can subdivide this polygon
// by the sequence of triangles ((p0, p1, p2), (p1, p2, p3), ... (pi, pi+1,
// pi+2)...).
// These triangles don't overlap and the sum of their area is the area of the
// polygon. In short, you can view a polygon as a "fan" of triangles starting at
// p0. As said, this vertex p0 is arbitrary and we could use any vertex of the
// polygon to produce a new fan.
// What we want to do is select a vertex p0 and a sequence of vertices such that
// the fan starting at p0 and produced by the sequence has the maximum possible
// area.
//
// A naive way to do that would be to test all possible combinations of
// (starting vertex, sequence of vertices).
// Instead, we use dynamics programming to find this combination.
//
// - We want to find a sequence of size d inside our polygon of size n.
// - We start at an anchor vertex p0 (a vertex of the polygon).
// - We construct dp[i][k] using DP. dp[i][k] is a table that iteratively
//   computes the highest possible area of the sequence of k vertices starting
//   at p0 and ending at pi. It is updated as: dp[0][1] = 0 (anchor with only
//   itself as sequence has a 0 area). Then for i+1 >= k (and k <= n): dp[i][k]
//   = max(dp[j][k-1] + area(p0, pj, pi), for j in [k-2, i-1])
// - Once dp has been constructed for anchor p0, we search dp[i][n] (for i >=
//   n-1) to find the best terminating pi and recover the associated sequence of
//   vertices.
// - This produces the "best" fan for anchor p0. We compare it to the previous
//   "best" (anchor, sequence) and keep the best of the two.
// - We repeat this procedure with the next anchor.
//
// Note: this implementation is heavily inspired by CGAL's 2D inscribed k-gon
// method. See here for more info:
// https://doc.cgal.org/latest/Inscribed_areas/group__PkgInscribedAreasRef.html
void ContactPatchSimplifierMaxArea::compute(const ContactPatch& patch_in,
                                            Index target_vertices,
                                            ContactPatch& patch_out) {
  const auto& pts = patch_in.points();
  const Index n = pts.size();
  simplified_buffer_.clear();
  simplified_buffer_.reserve(std::min(target_vertices, n));

  if (n == 0 || target_vertices == 0 || target_vertices >= n) {
    simplified_buffer_.assign(pts.begin(), pts.end());
  } else if (target_vertices == 1) {
    Vec2s barycenter = Vec2s::Zero();
    for (Index i = 0; i < n; ++i) {
      barycenter += pts[i];
    }
    barycenter /= static_cast<Scalar>(n);
    simplified_buffer_.push_back(barycenter);
  } else if (target_vertices == 2) {
    const Vec2s& first = pts[0];
    Index farthest_idx = 0;
    Scalar max_dist_sq = Scalar(0);
    for (Index i = 1; i < n; ++i) {
      const Scalar dist_sq = (pts[i] - first).squaredNorm();
      if (dist_sq > max_dist_sq) {
        max_dist_sq = dist_sq;
        farthest_idx = i;
      }
    }
    simplified_buffer_.push_back(first);
    simplified_buffer_.push_back(pts[farthest_idx]);
  } else {
    const std::size_t desired = clamp<Index>(target_vertices, 3, n);

    COAL_MAKE_ALLOCA_BOOST_SPAN(Scalar, dp_area,
                                static_cast<std::size_t>(desired * 2 * n));
    COAL_MAKE_ALLOCA_BOOST_SPAN(int, dp_prev,
                                static_cast<std::size_t>(desired * 2 * n));

    COAL_MAKE_ALLOCA_BOOST_SPAN(int, left_bound, desired - 1);
    COAL_MAKE_ALLOCA_BOOST_SPAN(int, right_bound, desired - 1);
    for (std::size_t m = 1; m < desired; ++m) {
      left_bound[m - 1] = int(m);
      right_bound[m - 1] = int(n - desired + m);
    }

    COAL_MAKE_ALLOCA_BOOST_SPAN(int, P0, desired);
    Scalar area0 = compute_rooted_kgon(pts, 0, int(desired), left_bound,
                                       right_bound, P0, dp_area, dp_prev);

    COAL_MAKE_ALLOCA_BOOST_SPAN(int, P0_ext, desired + 1);
    for (std::size_t i = 0; i < desired; ++i) {
      P0_ext[i] = P0[i];
    }
    P0_ext[desired] = int(n);

    COAL_MAKE_ALLOCA_BOOST_SPAN(int, P1, desired);
    COAL_MAKE_ALLOCA_BOOST_SPAN(int, left_bound1, desired - 1);
    COAL_MAKE_ALLOCA_BOOST_SPAN(int, right_bound1, desired - 1);
    for (std::size_t m = 1; m < desired; ++m) {
      left_bound1[m - 1] = P0_ext[m];
      right_bound1[m - 1] = P0_ext[m + 1];
    }

    Scalar area1 = compute_rooted_kgon(pts, P0[1], int(desired), left_bound1,
                                       right_bound1, P1, dp_area, dp_prev);

    Scalar global_max_area = std::max(area0, area1);
    COAL_MAKE_ALLOCA_BOOST_SPAN(int, global_best_v, desired);
    if (area0 > area1) {
      for (std::size_t i = 0; i < desired; ++i) {
        global_best_v[i] = P0[i];
      }
    } else {
      for (std::size_t i = 0; i < desired; ++i) {
        global_best_v[i] = P1[i];
      }
    }

    if (P0[1] - P0[0] > 1) {
      COAL_MAKE_ALLOCA_BOOST_SPAN(int, arg_left_bound, desired - 1);
      COAL_MAKE_ALLOCA_BOOST_SPAN(int, arg_right_bound, desired - 1);
      for (std::size_t i = 0; i < desired - 1; ++i) {
        arg_left_bound[i] = P0_ext[i + 1];
        arg_right_bound[i] = P1[i + 1];
      }

      solve_recursive(pts, int(desired), P0[0] + 1, P0[1] - 1, arg_left_bound,
                      arg_right_bound, global_max_area, global_best_v, dp_area,
                      dp_prev);
    }

    for (std::size_t i = 0; i < desired; ++i) {
      int v = global_best_v[i];
      simplified_buffer_.push_back(pts[static_cast<std::size_t>(v % int(n))]);
    }
  }

  if (&patch_in != &patch_out) {
    patch_out = patch_in;
  }

  auto& polygon = patch_out.points();
  polygon.assign(simplified_buffer_.begin(), simplified_buffer_.end());
}

void ContactPatchSimplifierMaxArea::simplify(ContactPatch& patch,
                                             Index target_vertices) {
  compute(patch, target_vertices, patch);
}

// This greedy version is based on the Visvalingam–Whyatt rule.
// Basically, instead of constructing a sequence of indices, we instead look
// at which vertices can be removed without reducing the overall area too much
// compared to the original polygon.
//
// First we compute the area of all the triangles (pi, pi+1, pi+2) and put them
// in a heap.
// Note that we can associate a triangle to each vertex and vice versa.
// Then, until we reach the desired number of points in the sub-polygon,
// we remove the smallest triangle, remove the associated vertex (if not already
// removed), compute the area of the newly formed triangles using the vertex's
// neighbors, and put these new triangles in the heap.
void ContactPatchSimplifierGreedy::compute(const ContactPatch& patch_in,
                                           Index target_vertices,
                                           ContactPatch& patch_out) {
  const auto& pts = patch_in.points();
  const Index n = pts.size();
  simplified_buffer_.clear();
  simplified_buffer_.reserve(std::min(target_vertices, static_cast<size_t>(n)));

  if (n == 0 || target_vertices == 0 || target_vertices >= n) {
    simplified_buffer_.assign(pts.begin(), pts.end());
  } else if (target_vertices == 1) {
    // Return the barycenter of the patch
    Vec2s barycenter = Vec2s::Zero();
    for (Index i = 0; i < n; ++i) {
      barycenter += pts[i];
    }
    barycenter /= static_cast<Scalar>(n);
    simplified_buffer_.push_back(barycenter);
  } else if (target_vertices == 2) {
    // Return the first point and the point farthest from it
    const Vec2s& first = pts[0];
    Index farthest_idx = 0;
    Scalar max_dist_sq = Scalar(0);
    for (Index i = 1; i < n; ++i) {
      const Scalar dist_sq = (pts[i] - first).squaredNorm();
      if (dist_sq > max_dist_sq) {
        max_dist_sq = dist_sq;
        farthest_idx = i;
      }
    }
    simplified_buffer_.push_back(first);
    simplified_buffer_.push_back(pts[farthest_idx]);
  } else {
    const Index desired = clamp<Index>(target_vertices, 3, n);

    prev_.resize(n);
    next_.resize(n);
    removed_.resize(n);
    // note: versions_ is used to only compare up to data nodes.
    // That way we don't have to pop outdated nodes from the heap.
    versions_.resize(n);
    heap_storage_.clear();
    heap_storage_.reserve(n);

    for (Index i = 0; i < n; ++i) {
      prev_[i] = static_cast<int>((i + n - 1) % n);
      next_[i] = static_cast<int>((i + 1) % n);
      removed_[i] = false;
      versions_[i] = 0;

      const Scalar area =
          compute_triangle_area(pts, prev_, next_, static_cast<int>(i));
      heap_storage_.push_back({i, area, versions_[i]});
    }

    const auto comp = [](const HeapEntry& lhs, const HeapEntry& rhs) {
      return lhs.area > rhs.area;
    };

    std::make_heap(heap_storage_.begin(), heap_storage_.end(), comp);

    Index remaining = n;
    while (remaining > desired && !heap_storage_.empty()) {
      std::pop_heap(heap_storage_.begin(), heap_storage_.end(), comp);
      HeapEntry node = heap_storage_.back();
      heap_storage_.pop_back();

      if (removed_[node.idx]) {
        continue;
      }
      if (node.version != versions_[node.idx]) {
        continue;
      }

      const Index prev_idx = static_cast<Index>(prev_[node.idx]);
      const Index next_idx = static_cast<Index>(next_[node.idx]);
      if (prev_idx == node.idx || next_idx == node.idx) {
        break;
      }

      removed_[node.idx] = true;
      --remaining;

      next_[prev_idx] = static_cast<int>(next_idx);
      prev_[next_idx] = static_cast<int>(prev_idx);

      if (!removed_[prev_idx]) {
        ++versions_[prev_idx];
        const Scalar area_prev = compute_triangle_area(
            pts, prev_, next_, static_cast<int>(prev_idx));
        heap_storage_.push_back({prev_idx, area_prev, versions_[prev_idx]});
        std::push_heap(heap_storage_.begin(), heap_storage_.end(), comp);
      }

      if (!removed_[next_idx]) {
        ++versions_[next_idx];
        const Scalar area_next = compute_triangle_area(
            pts, prev_, next_, static_cast<int>(next_idx));
        heap_storage_.push_back({next_idx, area_next, versions_[next_idx]});
        std::push_heap(heap_storage_.begin(), heap_storage_.end(), comp);
      }
    }

    for (Index i = 0; i < n; ++i) {
      if (!removed_[i]) {
        simplified_buffer_.push_back(pts[i]);
      }
    }
  }

  if (&patch_in != &patch_out) {
    patch_out = patch_in;
  }

  auto& polygon = patch_out.points();
  polygon.assign(simplified_buffer_.begin(), simplified_buffer_.end());
}

void ContactPatchSimplifierGreedy::simplify(ContactPatch& patch,
                                            Index target_vertices) {
  compute(patch, target_vertices, patch);
}

}  // namespace coal
