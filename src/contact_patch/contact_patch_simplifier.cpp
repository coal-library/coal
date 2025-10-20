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

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace coal {

namespace {
using Index = typename ContactPatchSimplifierMaxArea::Index;

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

Scalar compute_triangle_area(const ContactPatch::Polygon& pts,
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
void ContactPatchSimplifierMaxArea::compute(const ContactPatch& patch_in,
                                            Index target_vertices,
                                            ContactPatch& patch_out) {
  const auto& pts = patch_in.points();
  const Index n = pts.size();
  simplified_buffer_.clear();
  simplified_buffer_.reserve(std::min(target_vertices, n));

  if (n == 0 || target_vertices == 0 || target_vertices >= n) {
    simplified_buffer_.assign(pts.begin(), pts.end());
  } else {
    const Index desired = clamp<Index>(target_vertices, 1, n);

    ordered_indices_.resize(n);
    const Index dp_width = desired + 1;
    // note: we could use an Eigen::Matrix but using a std::vector<Scalar> for
    // dp_area_ is mirroring dp_prev_ (the "best" sequence of indices so far).
    dp_area_.assign(n * dp_width, -std::numeric_limits<Scalar>::infinity());
    dp_prev_.assign(n * dp_width, -1);
    selection_indices_tmp_.clear();
    selection_indices_tmp_.reserve(desired);
    best_indices_.clear();
    best_indices_.reserve(desired);

    const Scalar invalid_area = -std::numeric_limits<Scalar>::infinity();
    Scalar best_area = invalid_area;

    const auto dp_index = [dp_width](Index i, Index k) {
      return static_cast<Index>(i * dp_width + k);
    };

    for (Index anchor = 0; anchor < n; ++anchor) {
      for (Index i = 0; i < n; ++i) {
        ordered_indices_[i] = (anchor + i) % n;
      }

      std::fill(dp_area_.begin(), dp_area_.end(), invalid_area);
      std::fill(dp_prev_.begin(), dp_prev_.end(), -1);
      selection_indices_tmp_.clear();

      dp_area_[dp_index(0, 1)] = Scalar(0);
      dp_prev_[dp_index(0, 1)] = -1;

      Scalar anchor_best_area = invalid_area;
      int anchor_best_end = -1;

      if (desired == 1) {
        anchor_best_area = Scalar(0);
        anchor_best_end = 0;
      } else {
        for (Index i = 1; i < n; ++i) {
          const Index max_k = std::min(desired, i + 1);
          for (Index k = 2; k <= max_k; ++k) {
            for (Index j = k - 2; j < i; ++j) {
              const Scalar prev_area = dp_area_[dp_index(j, k - 1)];
              if (prev_area == invalid_area) {
                continue;
              }

              const Scalar tri_area = std::abs(double_triangle_area(
                  pts[ordered_indices_[0]], pts[ordered_indices_[j]],
                  pts[ordered_indices_[i]]));
              const Scalar candidate_area = prev_area + tri_area;
              const Index cur_idx = dp_index(i, k);
              if (candidate_area > dp_area_[cur_idx]) {
                dp_area_[cur_idx] = candidate_area;
                dp_prev_[cur_idx] = static_cast<int>(j);
              }
            }
          }
        }

        for (Index i = desired - 1; i < n; ++i) {
          const Scalar area = dp_area_[dp_index(i, desired)];
          if (area > anchor_best_area) {
            anchor_best_area = area;
            anchor_best_end = static_cast<int>(i);
          }
        }
      }

      if (anchor_best_area == invalid_area || anchor_best_end < 0) {
        continue;
      }

      selection_indices_tmp_.clear();

      Index current = static_cast<Index>(anchor_best_end);
      Index current_k = desired;
      selection_indices_tmp_.push_back(ordered_indices_[current]);

      while (current_k > 1) {
        const int predecessor = dp_prev_[dp_index(current, current_k)];
        if (predecessor < 0) {
          anchor_best_area = invalid_area;
          break;
        }
        current = static_cast<Index>(predecessor);
        --current_k;
        selection_indices_tmp_.push_back(ordered_indices_[current]);
      }

      if (anchor_best_area == invalid_area) {
        continue;
      }

      std::reverse(selection_indices_tmp_.begin(),
                   selection_indices_tmp_.end());

      if (anchor_best_area > best_area) {
        best_area = anchor_best_area;
        best_indices_ = selection_indices_tmp_;
      }
    }

    if (best_indices_.empty()) {
      best_indices_.clear();
      for (Index i = 0; i < desired; ++i) {
        best_indices_.push_back(i);
      }
    }

    keep_.resize(n);
    std::fill(keep_.begin(), keep_.end(), uint8_t(0));
    for (Index idx : best_indices_) {
      if (idx < n) {
        keep_[idx] = uint8_t(1);
      }
    }

    for (Index i = 0; i < n; ++i) {
      if (keep_[i]) {
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
  } else {
    const Index desired = clamp<Index>(target_vertices, 1, n);

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
